// Analogix ANX7625 MIPI DSI to DisplayPort bridge (USB-C video), polled
// https://github.com/usbarmory/tamago
//
// Copyright (c) the qcm2290 package authors
//
// Use of this source code is governed by the license
// that can be found in the LICENSE file.

package qcm2290

import (
	"errors"
	"fmt"
	"time"
)

// On the UNO Q the ANX7625 takes the DSI controller's output and drives
// DisplayPort over the USB-C port; it is also the port's Type-C / Power
// Delivery controller. Its on-chip firmware (OCM) runs the DisplayPort
// side; the host configures the MIPI receiver, reads the sink's EDID
// over the AUX channel and powers the sink. It is permanently powered,
// so no enable or reset pin is driven, and it is polled (its interrupt
// line is ignored).
//
// EXPERIMENTAL: the bridge locks to the DSI stream and trains the
// DisplayPort link (the sink reports itself synchronised), but the
// USB-C to HDMI adapter tested produces no HDMI picture yet; see
// display-spec.md §12.6 for what is verified and what is open.
//
// Written from bmx's docs/display-spec.md §12; see
// docs/display-provenance.md.

// ANX7625 register pages: one 7-bit I2C address each (§12.1)
const (
	ANXTCPC = 0x2c // Type-C port controller
	ANXTXP0 = 0x38 // DP transmitter
	ANXTXP1 = 0x3d // DP transmitter PHY
	ANXTXP2 = 0x39 // DP transmitter: HPD timer, audio
	ANXRXP0 = 0x3f // firmware interface: status, AUX, AV status
	ANXRXP1 = 0x42 // MIPI receiver PHY and PLL
	ANXRXP2 = 0x2a // MIPI receiver video timing
)

// anxPageDummy is the reserved register of each page written with 0x00
// before the first access after a page change (§12.1).
var anxPageDummy = map[uint8]uint8{
	ANXTCPC: 0x00,
	ANXTXP0: 0xd1,
	ANXTXP1: 0x60,
	ANXTXP2: 0x00,
	ANXRXP0: 0x39,
	ANXRXP1: 0x7f,
	ANXRXP2: 0x00,
}

// RX_P0 registers (§12.2)
const (
	anxXTALSel     = 0x3f
	anxFWStatus    = 0x05
	anxFWVersion   = 0x31
	anxDSCCtrl     = 0x40
	anxIntrCtrl    = 0x43
	anxIntrStatus  = 0x44
	anxSysStatus   = 0x45
	anxHPDCfg      = 0x49
	anxAUXAddr     = 0x11
	anxAUXCtrl     = 0x14
	anxAUXBuf      = 0x15
	anxPixelClk    = 0x25
	anxAUXCmd      = 0x27
	anxAVStatus    = 0x28
	anxTCPCAlert   = 0xcc
	anxFWRunning   = 1 << 7
	anxHPDCfgDone  = 1 << 6
	anxXTAL27MHz   = 0x80
	anxAUXStart    = 1 << 4
	anxAUXAddrOnly = 1 << 5
	anxAUXErr      = 0x0f

	// AP_AV_STATUS bits
	ANXAVMIPIMute   = 1 << 4
	ANXAVMIPIEnable = 1 << 5

	// system status bits
	ANXSysHPD   = 1 << 7
	ANXSysVBUS  = 1 << 3
	ANXSysVCONN = 1 << 2

	// DP AUX operations (§12.2)
	auxNativeWrite = 8
	auxNativeRead  = 9
	auxI2CWrite    = 0
	auxI2CRead     = 1
	auxI2CMOT      = 4

	// EDID I2C address over AUX
	edidAddr = 0x50

	auxPoll    = 2 * time.Millisecond
	auxTimeout = 300 * time.Millisecond
)

// ANX7625 represents the bridge on a GENI I2C bus (I2C1 on the UNO Q,
// which must be initialized at 100 kHz).
type ANX7625 struct {
	Bus *GENII2C

	// page is the address of the page accessed last
	page uint8
}

// switchPage performs the dummy write the page-switch rule requires
// when p differs from the page accessed last.
func (a *ANX7625) switchPage(p uint8) error {
	if p == a.page {
		return nil
	}

	r, ok := anxPageDummy[p]

	if !ok {
		return fmt.Errorf("unknown ANX7625 page %#x", p)
	}

	if err := a.Bus.Write(p, []byte{r, 0}, true); err != nil {
		a.page = 0
		return fmt.Errorf("page %#x switch: %v", p, err)
	}

	a.page = p

	return nil
}

// ReadReg reads register r of page p.
func (a *ANX7625) ReadReg(p, r uint8) (uint8, error) {
	if err := a.switchPage(p); err != nil {
		return 0, err
	}

	var b [1]byte

	if err := a.Bus.ReadReg(p, r, b[:]); err != nil {
		return 0, fmt.Errorf("read %#x[%#x]: %v", p, r, err)
	}

	return b[0], nil
}

// WriteReg writes register r of page p.
func (a *ANX7625) WriteReg(p, r, v uint8) error {
	if err := a.switchPage(p); err != nil {
		return err
	}

	if err := a.Bus.Write(p, []byte{r, v}, true); err != nil {
		return fmt.Errorf("write %#x[%#x]: %v", p, r, err)
	}

	return nil
}

// UpdateReg clears the clr bits and sets the set bits of register r of
// page p.
func (a *ANX7625) UpdateReg(p, r, clr, set uint8) error {
	v, err := a.ReadReg(p, r)

	if err != nil {
		return err
	}

	return a.WriteReg(p, r, v&^clr|set)
}

// writeRegs writes consecutive (register, value) pairs of page p.
func (a *ANX7625) writeRegs(p uint8, rv ...uint8) error {
	for i := 0; i+1 < len(rv); i += 2 {
		if err := a.WriteReg(p, rv[i], rv[i+1]); err != nil {
			return err
		}
	}

	return nil
}

// PowerOn selects the 27 MHz crystal, waits for the bridge firmware,
// waits for the HPD source configuration, sets the HPD detect window
// and returns the firmware version (§12.3 steps 1-4).
func (a *ANX7625) PowerOn() (version [2]byte, err error) {
	// the bus may have been used by others: force a page switch
	a.page = 0
	running := false

	for try := 0; try < 3 && !running; try++ {
		if err = a.WriteReg(ANXRXP0, anxXTALSel, anxXTAL27MHz); err != nil {
			return
		}

		for i := 0; i < 10; i++ {
			v, e := a.ReadReg(ANXRXP0, anxFWStatus)

			if e == nil && v&anxFWRunning != 0 {
				running = true
				break
			}

			time.Sleep(time.Millisecond)
		}
	}

	if !running {
		return version, errors.New("bridge firmware not running")
	}

	// HPD source configuration: a timeout here is harmless (§12.3 step 3)
	for start := time.Now(); time.Since(start) < 40*time.Millisecond; {
		if v, e := a.ReadReg(ANXRXP0, anxHPDCfg); e == nil && v&anxHPDCfgDone != 0 {
			break
		}

		time.Sleep(2 * time.Millisecond)
	}

	// 2 ms HPD detect window (54000 = 0x00d2f0)
	if err = a.writeRegs(ANXTXP2, 0xea, 0xf0, 0xeb, 0xd2, 0xec, 0x00); err != nil {
		return
	}

	if version[0], err = a.ReadReg(ANXRXP0, anxFWVersion); err != nil {
		return
	}

	version[1], err = a.ReadReg(ANXRXP0, anxFWVersion+1)

	return
}

// Setup clears the bridge's interrupts and Type-C alerts and programs
// the DP lane swing (§12.3 steps 6-8). It returns the system status
// (bit 7 HPD).
func (a *ANX7625) Setup() (sys uint8, err error) {
	if err = a.WriteReg(ANXRXP0, anxIntrCtrl, 0); err != nil {
		return
	}

	if sys, err = a.ReadReg(ANXRXP0, anxSysStatus); err != nil {
		return
	}

	if err = a.WriteReg(ANXTCPC, anxTCPCAlert, 0xff); err != nil {
		return
	}

	if _, err = a.ReadReg(ANXRXP0, anxIntrStatus); err != nil {
		return
	}

	if err = a.WriteReg(ANXRXP0, anxIntrStatus, 0); err != nil {
		return
	}

	if err = a.writeRegs(ANXRXP1, 0xee, 0x00, 0xec, 0x10, 0xff, 0x01); err != nil {
		return
	}

	// the board's per-lane swing settings, lanes 0 and 1
	swing := []uint8{0x14, 0x54, 0x64, 0x74}

	for i, v := range swing {
		if err = a.WriteReg(ANXTXP1, uint8(i), v); err != nil {
			return
		}

		if err = a.WriteReg(ANXTXP1, 0x14+uint8(i), v); err != nil {
			return
		}
	}

	return
}

// SystemStatus returns RX_P0 system status: bit 7 HPD, bit 5 data role,
// bit 4 CC, bit 3 VBUS, bit 2 VCONN.
func (a *ANX7625) SystemStatus() (uint8, error) {
	return a.ReadReg(ANXRXP0, anxSysStatus)
}

// AUX performs one AUX channel transaction (§12.2): op is the DP AUX
// operation (with the I2C middle-of-transaction flag where needed), addr
// the 20-bit address; wdata is written, then n bytes (up to 16) are
// read back from the buffer. The transaction length is len(wdata) for a
// write, n for a read.
func (a *ANX7625) AUX(op uint8, addr uint32, wdata []byte, n int) ([]byte, error) {
	length := len(wdata)

	if n > 0 {
		length = n
	}

	if length < 1 || length > 16 {
		return nil, errors.New("AUX length must be 1-16")
	}

	cmd := uint8(length-1)<<4 | op&0xf

	if err := a.writeRegs(ANXRXP0,
		anxAUXCmd, cmd,
		anxAUXAddr, uint8(addr),
		anxAUXAddr+1, uint8(addr>>8),
		anxAUXAddr+2, uint8(addr>>16)&0xf); err != nil {
		return nil, err
	}

	for i, b := range wdata {
		if err := a.WriteReg(ANXRXP0, anxAUXBuf+uint8(i), b); err != nil {
			return nil, err
		}
	}

	if err := a.UpdateReg(ANXRXP0, anxAUXCtrl, anxAUXAddrOnly, anxAUXStart); err != nil {
		return nil, err
	}

	var ctrl uint8

	for start := time.Now(); ; {
		time.Sleep(auxPoll)

		v, err := a.ReadReg(ANXRXP0, anxAUXCtrl)

		if err != nil {
			return nil, err
		}

		if ctrl = v; ctrl&anxAUXStart == 0 {
			break
		}

		if time.Since(start) > auxTimeout {
			return nil, fmt.Errorf("AUX timeout (ctrl %#x)", ctrl)
		}
	}

	if ctrl&anxAUXErr != 0 {
		return nil, fmt.Errorf("AUX error %#x (cmd %#x addr %#x)", ctrl&anxAUXErr, cmd, addr)
	}

	if n == 0 {
		return nil, nil
	}

	buf := make([]byte, n)

	for i := range buf {
		v, err := a.ReadReg(ANXRXP0, anxAUXBuf+uint8(i))

		if err != nil {
			return nil, err
		}

		buf[i] = v
	}

	return buf, nil
}

// ReadEDID reads the sink's 128-byte base EDID block, as I2C over AUX
// at address 0x50, 16 bytes at a time (§12.2). HPD must be high.
func (a *ANX7625) ReadEDID() ([]byte, error) {
	edid := make([]byte, 0, 128)

	for off := 0; off < 128; off += 16 {
		if _, err := a.AUX(auxI2CWrite|auxI2CMOT, edidAddr, []byte{byte(off)}, 0); err != nil {
			return nil, fmt.Errorf("EDID offset %d: %v", off, err)
		}

		b, err := a.AUX(auxI2CRead, edidAddr, nil, 16)

		if err != nil {
			return nil, fmt.Errorf("EDID read %d: %v", off, err)
		}

		edid = append(edid, b...)
	}

	return edid, nil
}

// MIPIPLL returns the MIPI receiver PLL post divider and M/N values for
// a pixel clock (§12.4), with the 27 MHz crystal.
func MIPIPLL(pixelHz uint64) (d, m, n uint32, err error) {
	const xtal = 27_000_000

	for _, min := range []uint64{520_000_000, 300_000_000} {
		for div := uint64(1); div <= 16; div++ {
			if pixelHz*div >= min {
				d = uint32(div)
				break
			}
		}

		if d != 0 {
			break
		}
	}

	switch d {
	case 0:
		return 0, 0, 0, errors.New("pixel clock out of range")
	case 7:
		d = 8
	case 11:
		d = 12
	case 13, 14:
		d = 15
	}

	// M / N = p / (27 MHz / d) = p x d / 27 MHz, in lowest terms
	mm, nn := pixelHz*uint64(d), uint64(xtal)
	g := gcd(mm, nn)
	mm, nn = mm/g, nn/g

	const limit = 1 << 24

	if mm >= limit || nn >= limit {
		s := max(mm, nn)/limit + 1
		mm, nn = mm/s, nn/s
	}

	for mm < limit && nn < limit {
		mm, nn = mm*2, nn*2
	}

	return d, uint32(mm / 2), uint32(nn / 2), nil
}

func gcd(a, b uint64) uint64 {
	for b != 0 {
		a, b = b, a%b
	}

	return a
}

// ConfigureMIPI powers the sink and configures the bridge's MIPI
// receiver for the mode (§12.3 steps 11-21). The DSI controller must
// already be running video mode, idling in blanking; the DPU is started
// after this returns, so that pixels flow into a receiver already
// locked to the DSI clock. ivo selects the PLL setting for sinks whose
// EDID manufacturer is IVO.
func (a *ANX7625) ConfigureMIPI(mode *DisplayMode, ivo bool) error {
	// 11. sink power: DPCD 0x600 = 1 (D0)
	if _, err := a.AUX(auxNativeWrite, 0x600, []byte{1}, 0); err != nil {
		return fmt.Errorf("sink power: %v", err)
	}

	steps := []struct {
		p, r, clr, set uint8
	}{
		{ANXRXP1, 0xee, 0x60, 0},    // 12.
		{ANXRXP0, anxDSCCtrl, 1, 0}, // 13. DSC off
		{ANXRXP1, 0x4a, 0, 0x10},    // 14. swap DSI lane 3
	}

	for _, s := range steps {
		if err := a.UpdateReg(s.p, s.r, s.clr, s.set); err != nil {
			return err
		}
	}

	// 15. MIPI PHY: clock lane manual power-down enables, HS prepare,
	// DSI input
	if err := a.writeRegs(ANXRXP1, 0x03, 0x18, 0x08, 0x10); err != nil {
		return err
	}

	if err := a.UpdateReg(ANXRXP1, 0x3d, 0, 0x20); err != nil {
		return err
	}

	// 16. video timing
	mhz := uint16(mode.PixelHz / 1_000_000)

	if err := a.writeRegs(ANXRXP0, anxPixelClk, uint8(mhz), anxPixelClk+1, uint8(mhz>>8)); err != nil {
		return err
	}

	if err := a.UpdateReg(ANXRXP1, 0x05, 0x03, uint8(mode.Lanes-1)&3); err != nil {
		return err
	}

	le := func(r uint8, v int) []uint8 { return []uint8{r, uint8(v), r + 1, uint8(v >> 8)} }

	var t []uint8
	t = append(t, le(0x19, mode.HTotal())...)
	t = append(t, le(0x1b, mode.Width)...)
	t = append(t, le(0x1d, mode.HFP)...)
	t = append(t, le(0x1f, mode.HSync)...)
	t = append(t, le(0x21, mode.HBP)...)
	t = append(t, le(0x14, mode.Height)...)
	t = append(t, 0x16, uint8(mode.VFP), 0x17, uint8(mode.VSync), 0x18, uint8(mode.VBP))

	if err := a.writeRegs(ANXRXP2, t...); err != nil {
		return err
	}

	// 17. MIPI PLL M/N
	d, m, n, err := MIPIPLL(mode.PixelHz)

	if err != nil {
		return err
	}

	if err := a.writeRegs(ANXRXP1,
		0x1e, uint8(m>>16), 0x1f, uint8(m>>8), 0x20, uint8(m),
		0x21, uint8(n>>16), 0x22, uint8(n>>8), 0x23, uint8(n)); err != nil {
		return err
	}

	pll := uint8(0x3d)

	if ivo {
		pll = 0x3b
	}

	if err := a.WriteReg(ANXRXP1, 0x1b, pll); err != nil {
		return err
	}

	// 18. PLL output divider, VCO tune, PLL reset pulse
	post := []struct {
		r, clr, set uint8
	}{
		{0x3b, 0, 0x02},               // 27 MHz reference
		{0x33, 0xf0, uint8(d-1) << 4}, // post divider
		{0x2b, 0x30, 0},               // VCO tune bits
		{0x2b, 0x02, 0},               // PLL reset
		{0x2b, 0, 0x02},               // and release
		{0x2a, 0x18, 0},               // 19. M/N load
	}

	for _, s := range post {
		if err := a.UpdateReg(ANXRXP1, s.r, s.clr, s.set); err != nil {
			return err
		}
	}

	time.Sleep(time.Millisecond)

	if err := a.UpdateReg(ANXRXP1, 0x2a, 0, 0x18); err != nil {
		return err
	}

	// 20. video stable count; lane control 10 toggle latches onto the
	// running DSI stream
	if err := a.writeRegs(ANXRXP1, 0x0a, 0x02, 0x0f, 0x00, 0x0f, 0x80); err != nil {
		return err
	}

	// 21. MIPI receiver enable, unmute
	return a.UpdateReg(ANXRXP0, anxAVStatus, ANXAVMIPIMute, ANXAVMIPIEnable)
}

// ANXStatus is a bridge snapshot for diagnostics.
type ANXStatus struct {
	FWStatus  uint8 // bit 7 firmware running
	Version   [2]uint8
	Sys       uint8 // bit 7 HPD
	HPDCfg    uint8
	Intr      uint8
	AV        uint8 // AP_AV_STATUS
	PixelMHz  uint16
	DSCCtrl   uint8
	MIPIError uint8 // RX_P1 0x31, bit 7 HS sync checksum error
}

// Status reads the bridge's firmware, HPD, AV and MIPI receiver status
// without changing anything.
func (a *ANX7625) Status() (s ANXStatus, err error) {
	rd := func(p, r uint8, dst *uint8) {
		if err == nil {
			*dst, err = a.ReadReg(p, r)
		}
	}

	var lo, hi uint8

	rd(ANXRXP0, anxFWStatus, &s.FWStatus)
	rd(ANXRXP0, anxFWVersion, &s.Version[0])
	rd(ANXRXP0, anxFWVersion+1, &s.Version[1])
	rd(ANXRXP0, anxSysStatus, &s.Sys)
	rd(ANXRXP0, anxHPDCfg, &s.HPDCfg)
	rd(ANXRXP0, anxIntrStatus, &s.Intr)
	rd(ANXRXP0, anxAVStatus, &s.AV)
	rd(ANXRXP0, anxDSCCtrl, &s.DSCCtrl)
	rd(ANXRXP0, anxPixelClk, &lo)
	rd(ANXRXP0, anxPixelClk+1, &hi)
	rd(ANXRXP1, 0x31, &s.MIPIError)

	s.PixelMHz = uint16(hi)<<8 | uint16(lo)

	return
}

// Dump reads n consecutive registers of page p from r, one at a time.
func (a *ANX7625) Dump(p, r uint8, n int) ([]byte, error) {
	buf := make([]byte, n)

	for i := range buf {
		v, err := a.ReadReg(p, r+uint8(i))

		if err != nil {
			return buf[:i], err
		}

		buf[i] = v
	}

	return buf, nil
}

// EDIDInfo is what bmx reports from a base EDID block: the
// manufacturer, product and the preferred (first detailed) timing.
type EDIDInfo struct {
	Manufacturer string
	Product      uint16
	PixelHz      uint64
	HActive      int
	HBlank       int
	HFP, HSync   int
	VActive      int
	VBlank       int
	VFP, VSync   int
}

// ParseEDID decodes the manufacturer ID and preferred timing of a base
// EDID block (VESA E-EDID 1.3/1.4 layout), after checking its header
// and checksum.
func ParseEDID(b []byte) (e EDIDInfo, err error) {
	if len(b) < 128 {
		return e, errors.New("short EDID")
	}

	header := []byte{0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0}

	for i, v := range header {
		if b[i] != v {
			return e, fmt.Errorf("bad EDID header % x", b[:8])
		}
	}

	var sum byte

	for _, v := range b[:128] {
		sum += v
	}

	if sum != 0 {
		return e, fmt.Errorf("EDID checksum %#x", sum)
	}

	id := uint16(b[8])<<8 | uint16(b[9])
	e.Manufacturer = string([]byte{
		byte('A' - 1 + id>>10&0x1f),
		byte('A' - 1 + id>>5&0x1f),
		byte('A' - 1 + id&0x1f),
	})
	e.Product = uint16(b[11])<<8 | uint16(b[10])

	// first detailed timing descriptor: the preferred timing
	d := b[54:72]
	e.PixelHz = uint64(uint16(d[1])<<8|uint16(d[0])) * 10_000
	e.HActive = int(d[2]) | int(d[4]>>4)<<8
	e.HBlank = int(d[3]) | int(d[4]&0xf)<<8
	e.VActive = int(d[5]) | int(d[7]>>4)<<8
	e.VBlank = int(d[6]) | int(d[7]&0xf)<<8
	e.HFP = int(d[8]) | int(d[11]>>6&3)<<8
	e.HSync = int(d[9]) | int(d[11]>>4&3)<<8
	e.VFP = int(d[10]>>4) | int(d[11]>>2&3)<<4
	e.VSync = int(d[10]&0xf) | int(d[11]&3)<<4

	return
}
