// Qualcomm QCM2290 camera subsystem: clock rates by sensor mode
// https://github.com/usbarmory/tamago
//
// Use of this source code is governed by the license
// that can be found in the LICENSE file.

package qcm2290

import (
	"fmt"

	"github.com/usbarmory/tamago/internal/reg"
)

// Three CAMSS roots scale with the sensor mode (camss-spec §2.2.1): the
// CSI PHY timer, CPHY RX and CSID with the link frequency (minimum link /
// 4), the TFE core with the pixel rate (minimum, for the raw path, pixel
// rate x bits per pixel / 64). Each runs at the lowest offered rate
// strictly above its minimum x 105/100, from the rates bmx can reach with
// the PLLs running under it (GPLL0, GPLL6).

// camssRate is one rate a root offers and the CFG selecting it.
type camssRate struct {
	hz  uint64
	cfg uint32
}

// The rates offered per root, lowest first, limited to sources that run
// under bmx (§2.2, §2.2.1). Left out: the timer's 268.8 MHz (GPLL4, which
// must be running), CPHY RX's 341.33/384 MHz (select value not recorded),
// CSID's 192 MHz (GPLL6 main /2: §2.2 lists it, but bmx has always run
// the CSID at the 240 MHz alternative and keeps doing so), CSID's 426.4
// MHz (source not recorded) and the TFE core's GPLL10 rates (GPLL10 is off
// under bmx).
var (
	camssTimerRates = []camssRate{
		{100_000_000, 2<<8 | 5}, // GPLL0 aux2 300 MHz / 3, CFG 0x205 (§2.2)
		{200_000_000, 2<<8 | 2}, // GPLL0 aux2 300 MHz / 1.5, CFG 0x202 (§2.2.1)
	}

	camssCPHYRXRates = []camssRate{
		{240_000_000, 1<<8 | 4}, // GPLL0 early 600 MHz / 2.5, CFG 0x104 (§2.2)
	}

	camssCSIDRates = []camssRate{
		{240_000_000, 1<<8 | 4}, // GPLL0 early / 2.5, CFG 0x104 (§2.2 alternative)
		{384_000_000, 4<<8 | 0}, // GPLL6 main 384 MHz, CFG 0x400 (§2.2.1)
	}

	// The TFE core's 200 and 240 MHz are the specification's bmx route
	// (§2.2.1): GPLL0 early, inside Linux's range but not in its list,
	// flagged for verification on the hardware.
	camssTFERates = []camssRate{
		{19_200_000, 0},         // crystal (§2.2)
		{200_000_000, 1<<8 | 5}, // GPLL0 early / 3, CFG 0x105
		{240_000_000, 1<<8 | 4}, // GPLL0 early / 2.5, CFG 0x104
	}
)

// pickRate returns the lowest rate strictly above min x 105/100.
func pickRate(name string, rates []camssRate, min uint64) (camssRate, error) {
	need := min * 105 / 100

	for _, r := range rates {
		if r.hz > need {
			return r, nil
		}
	}

	return camssRate{}, fmt.Errorf("%s: no rate above %d Hz (needed %d x 1.05) from the PLLs running", name, need, min)
}

// CAMSSRates are the rates a capture runs its scaling roots at.
type CAMSSRates struct {
	TimerHz, CPHYRXHz, CSIDHz, TFEHz uint64
}

// camssRoot is a scaling root and its rate table.
type camssRoot struct {
	name  string
	cmd   uint32
	rates []camssRate
}

var (
	camssRootTimer  = camssRoot{"csiphy timer (0x45000)", gccCSIPHYTimerCmdRCGR, camssTimerRates}
	camssRootTimer1 = camssRoot{"csiphy1 timer (0x4501c)", gccCSIPHY1TimerCmdRCGR, camssTimerRates} // §3a: PHY 0's table
	camssRootCPHYRX = camssRoot{"cphy rx (0x52064)", gccCPHYRXCmdRCGR, camssCPHYRXRates}
	camssRootCSID   = camssRoot{"tfe0 csid (0x52094)", gccTFE0CSIDCmdRCGR, camssCSIDRates}
	camssRootTFE    = camssRoot{"tfe0 core (0x52004)", gccTFE0CmdRCGR, camssTFERates}
)

// rcgCmdRootOff is CMD bit 31: the root is not running.
const rcgCmdRootOff = 1 << 31

// rcgCfgSelDiv are the CFG fields this code programs: source select
// [10:8] and divider [4:0].
const rcgCfgSelDiv = 0x71f

// setRoot programs a root's CFG through the update handshake and checks
// that it took: the update bit cleared, the CFG reads back with the
// source and divider written, and the root reports running.
func setRoot(r camssRoot, rate camssRate) error {
	if err := rcgUpdate(GCC_BASE, r.cmd, rate.cfg); err != nil {
		return fmt.Errorf("%s at %d Hz: %v", r.name, rate.hz, err)
	}

	cfg := reg.Read(GCC_BASE + r.cmd + rcgCfg)

	if cfg&rcgCfgSelDiv != rate.cfg {
		return fmt.Errorf("%s at %d Hz: CFG reads %#x after writing %#x", r.name, rate.hz, cfg, rate.cfg)
	}

	if !pollTime(GCC_BASE+r.cmd, rcgCmdRootOff, 0, clkPollTime) {
		return fmt.Errorf("%s at %d Hz: root off after the update (CMD %#x)", r.name, rate.hz, reg.Read(GCC_BASE+r.cmd))
	}

	return nil
}

// SetCAMSSRates programs the scaling roots for a link frequency (Hz, the
// CSI-2 clock lane), a pixel rate (pixels per second as the receiver
// takes them; 0: unknown, the TFE core keeps the §2.2 rate) and bits per
// pixel. With the minimums of an OV5647 mode it sets the §2.2 rates.
// EnableCAMSSClocks must have been called (the branches are on).
func SetCAMSSRates(linkHz, pixelRate uint64, bpp int) (CAMSSRates, error) {
	return setCAMSSRates(0, linkHz, pixelRate, bpp, 0)
}

// setCAMSSRates programs the roots for a capture on PHY phy: that PHY's
// timer, and the shared CPHY RX, CSID and TFE core roots.
func setCAMSSRates(phy int, linkHz, pixelRate uint64, bpp int, tfeFloor uint64) (CAMSSRates, error) {
	var got CAMSSRates

	linkMin := linkHz / 4

	set := func(r camssRoot, min uint64, out *uint64) error {
		rate, err := pickRate(r.name, r.rates, min)

		if err != nil {
			return err
		}

		if err := setRoot(r, rate); err != nil {
			return err
		}

		*out = rate.hz

		return nil
	}

	timer := camssRootTimer

	if phy == 1 {
		timer = camssRootTimer1
	}

	if err := set(timer, linkMin, &got.TimerHz); err != nil {
		return got, err
	}

	if err := set(camssRootCPHYRX, linkMin, &got.CPHYRXHz); err != nil {
		return got, err
	}

	if err := set(camssRootCSID, linkMin, &got.CSIDHz); err != nil {
		return got, err
	}

	tfeMin := uint64(0)

	if pixelRate > 0 && bpp > 0 {
		tfeMin = pixelRate * uint64(bpp) / 64
	}

	// a floor asked for by the caller: the rate picked is above it
	// whatever the margin
	if f := tfeFloor * 100 / 105; f > tfeMin {
		tfeMin = f
	}

	if err := set(camssRootTFE, tfeMin, &got.TFEHz); err != nil {
		return got, err
	}

	return got, nil
}

// CAMSSRootStatus reads back each scaling root: its CMD and CFG, and the
// rate the CFG selects when it is one of the rates offered here (0 when
// it is not).
type CAMSSRootStatus struct {
	Name     string
	CMD, CFG uint32
	Hz       uint64
}

// CAMSSRoots reads the scaling roots back from the GCC.
func CAMSSRoots() (s []CAMSSRootStatus) {
	for _, r := range []camssRoot{camssRootTimer, camssRootTimer1, camssRootCPHYRX, camssRootCSID, camssRootTFE} {
		st := CAMSSRootStatus{
			Name: r.name,
			CMD:  reg.Read(GCC_BASE + r.cmd),
			CFG:  reg.Read(GCC_BASE + r.cmd + rcgCfg),
		}

		for _, rate := range r.rates {
			if st.CFG&rcgCfgSelDiv == rate.cfg {
				st.Hz = rate.hz
			}
		}

		s = append(s, st)
	}

	return
}
