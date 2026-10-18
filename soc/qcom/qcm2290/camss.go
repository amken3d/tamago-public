// Qualcomm QCM2290 camera subsystem (CAMSS): raw CSI-2 capture (polled)
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

	"github.com/usbarmory/tamago/internal/reg"
)

// The raw-dump capture path of CAMERA0: CSI PHY 0 (D-PHY receiver) feeds
// CSID 0 (CSI-2 decoder), whose RDI0 output feeds TFE 0 (thin front
// end), which writes the packets to memory through bus write master 7.
// No image processing: memory receives the CSI-2 payload as sent.
//
// Written from bmx's docs/camss-spec.md (section references below
// point at it); see docs/camss-provenance.md for why it is the only
// hardware documentation this driver was written from.
const (
	CSIPHY0_BASE = 0x05c52000
	CSID0_BASE   = 0x05c6e000
	TFE0_BASE    = 0x05c6f000
)

// GCC camera clocks (§2.2): branch CBCRs and the RCG CMD registers of
// their roots
const (
	gccCAMSSAXICmdRCGR = 0x5802c
	gccCAMSSAXICBCR    = 0x58044
	gccCAMSSRTAXICBCR  = 0x5805c
	gccCAMSSNRTAXICBCR = 0x58054

	gccCSIPHYTimerCmdRCGR = 0x45000
	gccCSIPHY0TimerCBCR   = 0x45018

	gccCPHYRXCmdRCGR  = 0x52064 // shared by CPHY 0 and TFE 0 CPHY RX
	gccCPHY0CBCR      = 0x52088
	gccTFE0CPHYRXCBCR = 0x5207c

	gccTFE0CSIDCmdRCGR = 0x52094
	gccTFE0CSIDCBCR    = 0x520ac

	gccTFE0CmdRCGR = 0x52004
	gccTFE0CBCR    = 0x5201c

	// CFG_RCGR values (§2.2): source select [10:8], 2 x divider - 1
	// [4:0]
	cfgGPLL0EarlyDiv2p5 = 1<<8 | 4 // 600 MHz / 2.5 = 240 MHz
	cfgGPLL0Aux2Div3    = 2<<8 | 5 // 300 MHz / 3 = 100 MHz
	cfgXOBypass         = 0        // 19.2 MHz

	// CSIPHYTimerHz is the PHY timer rate set by EnableCAMSSClocks
	// (§2.2). Start may raise it for a fast link (§2.2.1) and counts the
	// settle count in the rate it set (CAMSS.Rates).
	CSIPHYTimerHz = 100_000_000
)

// EnableCAMSSClocks configures and enables the GCC clocks of the raw
// capture path (§2.2). EnableCCIPower must have been called first: it
// powers the CAMSS top GDSC and clocks the register interface (camera
// AHB/XO, CAMSS top AHB). Every CAMSS register access below CCI depends
// on this call: an access to an unclocked block hangs the bus.
//
// The CSID root runs at 240 MHz from GPLL0 early (the specification's
// alternative to Linux's 192 MHz from GPLL6), so that nothing depends
// on a PLL the boot chain may have left off.
func EnableCAMSSClocks() error {
	roots := []struct {
		name string
		cmd  uint32
		cfg  uint32
	}{
		{"camss axi", gccCAMSSAXICmdRCGR, cfgXOBypass},
		{"csiphy timer", gccCSIPHYTimerCmdRCGR, cfgGPLL0Aux2Div3},
		{"cphy rx", gccCPHYRXCmdRCGR, cfgGPLL0EarlyDiv2p5},
		{"tfe0 csid", gccTFE0CSIDCmdRCGR, cfgGPLL0EarlyDiv2p5},
		{"tfe0", gccTFE0CmdRCGR, cfgXOBypass},
	}

	for _, r := range roots {
		if err := rcgUpdate(GCC_BASE, r.cmd, r.cfg); err != nil {
			return fmt.Errorf("%s rcg: %v", r.name, err)
		}
	}

	branches := []struct {
		name string
		cbcr uint32
	}{
		{"camss axi", gccCAMSSAXICBCR},
		{"camss rt axi", gccCAMSSRTAXICBCR},
		{"camss nrt axi", gccCAMSSNRTAXICBCR},
		{"csiphy0 timer", gccCSIPHY0TimerCBCR},
		{"cphy0", gccCPHY0CBCR},
		{"tfe0 cphy rx", gccTFE0CPHYRXCBCR},
		{"tfe0 csid", gccTFE0CSIDCBCR},
		{"tfe0", gccTFE0CBCR},
	}

	for _, b := range branches {
		if err := dispClkOn(GCC_BASE, b.cbcr); err != nil {
			return fmt.Errorf("%s: %v", b.name, err)
		}
	}

	// the camera NoC's priorities on the system NoC (usb-spec §7.1)
	if err := SetSNOCQoS(SNOCPortCamNRT, SNOCQoSCamNRT); err != nil {
		return err
	}

	return SetSNOCQoS(SNOCPortCamRT, SNOCQoSCamRT)
}

// CAMSSClockStatus returns the CBCR of every capture-path branch, in
// EnableCAMSSClocks order, for diagnostics (bit 0 enable, bit 31 off).
func CAMSSClockStatus() []uint32 {
	var v []uint32

	for _, cbcr := range []uint32{
		gccCAMSSAXICBCR, gccCAMSSRTAXICBCR, gccCAMSSNRTAXICBCR,
		gccCSIPHY0TimerCBCR, gccCPHY0CBCR, gccTFE0CPHYRXCBCR,
		gccTFE0CSIDCBCR, gccTFE0CBCR,
	} {
		v = append(v, reg.Read(GCC_BASE+cbcr))
	}

	return v
}

// Camera SMMU streams (§2.3), each matched exactly (mask 0). They get
// their own context bank and stream match groups, so that a camera
// fault is told apart from a display one.
var cameraStreamIDs = [4]uint32{0x400, 0x800, 0x820, 0x840}

const (
	smmuIDR0 = 0x020

	// camCB is the camera's context bank; camSMR the first of four
	// consecutive stream match groups. The boot chain uses groups 0-3
	// and banks 0-2, the display group and bank 8.
	camCB  = 9
	camSMR = 9

	// S2CR TYPE [17:16] and CBNDX [7:0]
	s2crTypeCBMask = 0x300ff
)

// SMMUMapCameraIdentity routes the camera streams (§2.3) through context
// bank 9 with stage-1 translation on, over the same DDR identity table
// as SMMUMapMDSSIdentity (pt may be that very page: the content is
// identical), so that CAMSS device addresses equal physical ones.
func SMMUMapCameraIdentity(pt uint32) error {
	idr0 := reg.Read(SMMU_BASE + smmuIDR0)
	idr1 := reg.Read(SMMU_BASE + smmuIDR1)

	if n := idr0 & 0xff; camSMR+len(cameraStreamIDs) > int(n) {
		return fmt.Errorf("SMMU has %d stream match groups", n)
	}

	if n := idr1 & 0xff; camCB >= n {
		return fmt.Errorf("SMMU has %d context banks", n)
	}

	// refuse a bank another valid stream match group already routes to
	for n := uint32(0); n < idr0&0xff; n++ {
		if n >= camSMR && n < camSMR+uint32(len(cameraStreamIDs)) {
			continue
		}

		smr := reg.Read(SMMU_BASE + smmuSMR0 + n*4)
		s2cr := reg.Read(SMMU_BASE + smmuS2CR0 + n*4)

		if smr&smrValid != 0 && s2cr&s2crTypeCBMask == s2crTypeTrans|camCB {
			return fmt.Errorf("context bank %d in use by group %d (SMR %#x)", camCB, n, smr)
		}
	}

	// refuse to take groups somebody else owns; ours (from an earlier
	// call) are reprogrammed
	for i := range cameraStreamIDs {
		n := uint32(camSMR + i)
		smr := reg.Read(SMMU_BASE + smmuSMR0 + n*4)
		s2cr := reg.Read(SMMU_BASE + smmuS2CR0 + n*4)

		if smr&smrValid != 0 && s2cr&s2crTypeCBMask != s2crTypeTrans|camCB {
			return fmt.Errorf("stream match group %d in use (SMR %#x S2CR %#x)", n, smr, s2cr)
		}

		reg.Write(SMMU_BASE+smmuSMR0+n*4, 0)
	}

	if err := identityBank(camCB, pt); err != nil {
		return err
	}

	for i, id := range cameraStreamIDs {
		n := uint32(camSMR + i)

		reg.Write(SMMU_BASE+smmuS2CR0+n*4, s2crTypeTrans|camCB)
		reg.Write(SMMU_BASE+smmuSMR0+n*4, smrValid|id)
	}

	return nil
}

// SMMUCameraFault returns the camera context bank's fault status,
// faulting address and syndrome.
func SMMUCameraFault() (fsr uint32, far uint64, fsynr0 uint32) {
	cb := contextBankBase(camCB)

	fsr = reg.Read(cb + cbFSR)
	far = reg.Read64(uint64(cb + cbFAR))
	fsynr0 = reg.Read(cb + cbFSYNR0)

	return
}

// CSI PHY registers (§3.1): lane groups at L x 0x100 (data lanes 0, 2,
// 4, 6; clock lane 7), common block at +0x800
const (
	phyCommonCtrl0 = 0x800 // bit 0: software reset
	phyCommonCtrl5 = 0x814 // lane enables
	phyCommonCtrl6 = 0x818 // bit 0: powered, bit 1: show revision
	phyCommonCtrl7 = 0x81c

	phyCommonCtrl11 = 0x82c // interrupt masks, through COMMON_CTRL21
	phyCommonCtrl21 = 0x854

	phyCommonStatus0 = 0x8b0

	phyClockLaneGroup = 7
	phyClockLaneBit   = 1 << 7

	// PHY reset pulse width (§3.2: 5-8 ms)
	phyResetPulse = 6 * time.Millisecond
)

// phyLaneWrite is one lane register write; settle marks CFG3, whose
// value is the computed settle count.
type phyLaneWrite struct {
	off    uint32
	val    uint32
	settle bool
}

// Lane group writes, in the order §3.2 step 7 gives them.
var phyDataLane = []phyLaneWrite{
	{0x30, 0x02, false}, {0x2c, 0x01, false}, {0x34, 0x01, false},
	{0x3c, 0xb8, false}, {0x1c, 0x0a, false}, {0x00, 0xd7, false},
	{0x04, 0x08, false}, {0x20, 0x00, false}, {0x08, 0, true},
	{0x10, 0x50, false}, {0x38, 0x01, false}, {0x60, 0x00, false},
	{0x64, 0x3f, false},
}

var phyClockLane = []phyLaneWrite{
	{0x30, 0x02, false}, {0x2c, 0x01, false}, {0x34, 0x01, false},
	{0x28, 0x04, false}, {0x3c, 0xb8, false}, {0x1c, 0x0a, false},
	{0x00, 0xc0, false}, {0x04, 0x08, false}, {0x20, 0x00, false},
	{0x08, 0, true}, {0x0c, 0xff, false}, {0x10, 0x50, false},
	{0x38, 0x01, false}, {0x60, 0x00, false}, {0x64, 0x3f, false},
}

// CSIPHYSettleCount returns the HS settle count (§3.3) for a CSI-2
// clock lane frequency linkHz and a PHY timer rate timerHz:
// floor((85 ns + 6 UI) x timerHz) - 6, with UI = 1 / (2 x linkHz).
//
// 85 ns + 6 UI is the minimum of the D-PHY HS settle window, and on
// hardware it proved marginal: for the OV5647 640x480 mode (208.33 MHz
// link) it gives 3, which lost line packets and frame ends -- buffers
// 40-80% written, frames missing -- while 4 to 18 all captured cleanly
// (2026-10-16). Start therefore uses CSIPHYSettleCountMid.
func CSIPHYSettleCount(linkHz, timerHz uint64) uint32 {
	return settleCount(85_000, 6, linkHz, timerHz)
}

// CSIPHYSettleCountMid returns the settle count for the middle of the
// D-PHY HS settle window, which runs from 85 ns + 6 UI to 145 ns + 10 UI
// (MIPI D-PHY): 115 ns + 8 UI. It is 7 for the OV5647 640x480 mode, in
// the middle of the observed working range.
func CSIPHYSettleCountMid(linkHz, timerHz uint64) uint32 {
	return settleCount(115_000, 8, linkHz, timerHz)
}

// settleCount computes floor((ps + nUI x UI) x timerHz) - 6, in
// picoseconds to stay in integers.
func settleCount(ps, nUI, linkHz, timerHz uint64) uint32 {
	ui := 1_000_000_000_000 / (2 * linkHz)
	t := ps + nUI*ui
	n := t * timerHz / 1_000_000_000_000

	if n < 6 {
		return 0
	}

	return uint32(n - 6)
}

// CSID registers (§4.1)
const (
	csidRstStrobes = 0x010
	csidIRQStatus  = 0x070
	csidIRQMask    = 0x074
	csidIRQClear   = 0x078
	csidIRQCmd     = 0x080
	csidRXCfg0     = 0x100
	csidRXCfg1     = 0x104

	csidResetDone = 1 << 0

	// IRQ, front-end, PHY and CSID clock domains; registers preserved
	csidRstStrobesAll = 0x1e

	rdiEnable     = 1 << 31
	rdiDecodeNone = 0xf << 12
)

func csidRDICfg0(n int) uint32 { return uint32(0x300 + 0x100*n) }
func csidRDICtrl(n int) uint32 { return uint32(0x308 + 0x100*n) }

// TFE registers (§5.1)
const (
	tfeGlobalResetCmd = 0x014
	tfeRegUpdateCmd   = 0x02c
	tfeIRQCmd         = 0x030
	tfeIRQMask0       = 0x034
	tfeIRQClear0      = 0x040
	tfeIRQStatus0     = 0x04c

	tfeIRQResetDone = 1 << 0
	tfeIRQBusWrite  = 1 << 1

	tfeBusIRQMask0       = 0xa18
	tfeBusIRQClear0      = 0xa20
	tfeBusIRQStatus0     = 0xa28
	tfeBusIRQCmd         = 0xa30
	tfeBusStatusClear    = 0xa60
	tfeBusOverflowStatus = 0xa68

	// register-update done for every interface, buffer done for every
	// subgroup, and the three violations
	tfeBusIRQMaskAll = 0xd000ff0f

	// write master 7 (RDI0)
	tfeWM7          = 0x1300
	wmClientCfg     = 0x00
	wmImageAddr     = 0x04
	wmFrameIncr     = 0x08
	wmImageCfg0     = 0x0c
	wmImageCfg1     = 0x10
	wmImageCfg2     = 0x14
	wmPackerCfg     = 0x18
	wmSubsampleCfg0 = 0x30
	wmSubsampleCfg1 = 0x34
	wmFramedropCfg0 = 0x38
	wmFramedropCfg1 = 0x3c

	wmEnable      = 1 << 0
	wmFrameBased  = 1 << 16
	wmPackPlain64 = 0xa

	// REG_UPDATE_CMD bit of RDI0
	tfeRegUpdateRDI0 = 1 << 1

	// camssPollSpins bounds reset-done polling
	camssPollSpins = 1000000
)

// Bus status bits (BUS_IRQ_STATUS_0, §5.1)
const (
	CAMSSBusRegUpdateRDI0 = 1 << 1
	CAMSSBusBufDoneRDI0   = 1 << 13 // buffer done, subgroup 5 (RDI0)
	CAMSSBusConfigViol    = 1 << 28
	CAMSSBusInputViol     = 1 << 30
	CAMSSBusImageSizeViol = 1 << 31

	CAMSSBusErrors = CAMSSBusConfigViol | CAMSSBusInputViol | CAMSSBusImageSizeViol

	// write master 7 in BUS_OVERFLOW_STATUS
	CAMSSOverflowWM7 = 1 << 7
)

// CaptureConfig describes the stream to capture on CAMERA0.
type CaptureConfig struct {
	// Lanes is the number of data lanes, on PHY lane positions 0 to
	// Lanes-1 (1-4) unless PHYLanes says otherwise.
	Lanes int
	// PHYLanes, when not nil, names the PHY lane position (0-3) feeding
	// each CSID lane, in CSID lane order: the CSID's lane input select
	// (one nibble per CSID lane, §4.1 CSI2_RX_CFG0 bits 17:4) and the
	// PHY's lane enables (COMMON_CTRL5, data lane at position p = bit 2p,
	// §3.1). {1, 0} swaps the two lanes of a 2-lane sensor.
	PHYLanes []int
	// LinkHz is the CSI-2 clock lane frequency, for the settle count.
	LinkHz uint64
	// Settle, when not zero, overrides the settle count computed from
	// LinkHz (CSIPHYSettleCountMid).
	Settle uint32
	// DT is the CSI-2 data type (RAW10 = 0x2b).
	DT uint8
	// Stride and Height give the bytes per frame (Stride x Height); a
	// RAW10 line is Width x 10 / 8 bytes, and must be a multiple of 4
	// (§6: for RAW10, a width that is a multiple of 16 pixels).
	Stride int
	Height int
	// PixelRate is the rate pixels reach the receiver, per second, and
	// BPP their bits each: the TFE core's minimum rate (§2.2.1). 0 keeps
	// the TFE core at its §2.2 rate.
	PixelRate uint64
	BPP       int
	// TFEMinHz, when not zero, raises the TFE core's minimum to at least
	// this rate (bench diagnostics: run the core faster than the rule
	// asks).
	TFEMinHz uint64
}

// CAMSS represents the raw capture path of one camera port.
type CAMSS struct {
	PHY  uint32
	CSID uint32
	TFE  uint32

	settle uint32
	rates  CAMSSRates
}

// Camera0 is CAMERA0's path: PHY 0, CSID 0, TFE 0.
var Camera0 = &CAMSS{
	PHY:  CSIPHY0_BASE,
	CSID: CSID0_BASE,
	TFE:  TFE0_BASE,
}

// Reset performs the power-up part of each block's sequence: the PHY
// software reset (§3.2 steps 1-2), the CSID reset (§4.2 steps 1-3) and
// the TFE core reset (§5.2 steps 1-3). EnableCAMSSClocks must have been
// called.
func (c *CAMSS) Reset() error {
	c.ResetPHY()

	if err := c.ResetCSID(); err != nil {
		return err
	}

	return c.ResetTFE()
}

// ResetPHY pulses the PHY software reset and exposes its revision.
func (c *CAMSS) ResetPHY() {
	reg.Write(c.PHY+phyCommonCtrl0, 1)
	time.Sleep(phyResetPulse)
	reg.Write(c.PHY+phyCommonCtrl0, 0)
	reg.Write(c.PHY+phyCommonCtrl6, 2)
}

// ResetCSID resets the CSID's IRQ, front-end, PHY and CSID clock
// domains, keeping its registers.
func (c *CAMSS) ResetCSID() error {
	reg.Write(c.CSID+csidIRQMask, csidResetDone)
	reg.Write(c.CSID+csidIRQClear, csidResetDone)
	reg.Write(c.CSID+csidIRQCmd, 1)
	reg.Write(c.CSID+csidRstStrobes, csidRstStrobesAll)

	if !c.pollBit(c.CSID+csidIRQStatus, csidResetDone) {
		return fmt.Errorf("CSID reset timeout (status %#x)", reg.Read(c.CSID+csidIRQStatus))
	}

	reg.Write(c.CSID+csidIRQClear, csidResetDone)
	reg.Write(c.CSID+csidIRQCmd, 1)

	return nil
}

// ResetTFE resets the TFE core.
func (c *CAMSS) ResetTFE() error {
	reg.Write(c.TFE+tfeIRQMask0, tfeIRQResetDone)
	reg.Write(c.TFE+tfeGlobalResetCmd, 1)

	if !c.pollBit(c.TFE+tfeIRQStatus0, tfeIRQResetDone) {
		return fmt.Errorf("TFE reset timeout (status %#x)", reg.Read(c.TFE+tfeIRQStatus0))
	}

	reg.Write(c.TFE+tfeIRQClear0, tfeIRQResetDone)
	reg.Write(c.TFE+tfeIRQCmd, 1)

	return nil
}

func (c *CAMSS) pollBit(addr, mask uint32) bool {
	for i := 0; i < camssPollSpins; i++ {
		if reg.Read(addr)&mask != 0 {
			return true
		}
	}

	return false
}

// PHYRevision returns the PHY revision bytes (COMMON_STATUS12..15),
// readable after Reset.
func (c *CAMSS) PHYRevision() (rev [4]uint32) {
	for i := range rev {
		rev[i] = reg.Read(c.PHY + phyCommonStatus0 + uint32(12+i)*4)
	}

	return
}

// Start configures the path for cfg and starts it in the traced order
// (§5.2): TFE write master 7 with buf0 and buf1 queued, CSID RDI0 on,
// then the PHY lanes. The sensor is started by the caller afterwards.
// Buffers are device addresses (equal to physical ones with
// SMMUMapCameraIdentity).
func (c *CAMSS) Start(cfg CaptureConfig, buf0, buf1 uint32) error {
	if cfg.Lanes < 1 || cfg.Lanes > 4 {
		return errors.New("lanes must be 1-4")
	}

	if cfg.Stride <= 0 || cfg.Height <= 0 {
		return errors.New("invalid frame geometry")
	}

	// A line that is not a whole number of 32-bit words is written short
	// (§6, hardware observation): refuse it rather than deliver frames
	// that end early.
	if cfg.Stride%4 != 0 {
		return fmt.Errorf("a %d-byte line is not a whole number of 32-bit words: the write master ends such frames early (camss-spec §6)", cfg.Stride)
	}

	phyLanes := cfg.PHYLanes

	if phyLanes == nil {
		for l := 0; l < cfg.Lanes; l++ {
			phyLanes = append(phyLanes, l)
		}
	}

	if len(phyLanes) != cfg.Lanes {
		return fmt.Errorf("%d PHY lanes named for %d data lanes", len(phyLanes), cfg.Lanes)
	}

	for _, p := range phyLanes {
		if p < 0 || p > 3 {
			return fmt.Errorf("PHY lane %d (0-3)", p)
		}
	}

	// the scaling roots for this mode (§2.2.1), before any block is
	// configured; the settle count is counted in the timer rate set
	rates, err := setCAMSSRates(cfg.LinkHz, cfg.PixelRate, cfg.BPP, cfg.TFEMinHz)

	if err != nil {
		return err
	}

	c.rates = rates
	c.settle = cfg.Settle

	if c.settle == 0 {
		c.settle = CSIPHYSettleCountMid(cfg.LinkHz, rates.TimerHz)
	}

	// TFE: write master 7 (§5.2 steps 4-10)
	wm := c.TFE + tfeWM7

	reg.Write(wm+wmImageCfg0, 0xffff)
	reg.Write(wm+wmImageCfg1, 0)
	reg.Write(wm+wmImageCfg2, 0xffff)
	reg.Write(wm+wmFrameIncr, uint32(cfg.Stride*cfg.Height))
	reg.Write(wm+wmPackerCfg, wmPackPlain64)
	reg.Write(wm+wmFramedropCfg0, 0)
	reg.Write(wm+wmFramedropCfg1, 1)
	reg.Write(wm+wmSubsampleCfg0, 0)
	reg.Write(wm+wmSubsampleCfg1, 1)

	reg.Write(c.TFE+tfeIRQMask0, tfeIRQResetDone|tfeIRQBusWrite)
	reg.Write(c.TFE+tfeBusIRQMask0, tfeBusIRQMaskAll)

	reg.Write(wm+wmClientCfg, wmFrameBased|wmEnable)

	c.Queue(buf0)
	c.Queue(buf1)

	// CSID: RDI0 from the PHY lanes (§4.2 steps 4-7)
	var laneSel uint32

	for l, p := range phyLanes {
		laneSel |= uint32(p) << (4 * l)
	}

	reg.Write(c.CSID+csidRXCfg0, uint32(cfg.Lanes-1)|laneSel<<4|(0+1)<<20)
	reg.Write(c.CSID+csidRXCfg1, 1)
	reg.Write(c.CSID+csidRDICfg0(0), rdiEnable|uint32(cfg.DT&0x3f)<<16|rdiDecodeNone)
	reg.Write(c.CSID+csidRDICtrl(0), 1)

	// PHY: lane enable (§3.2 steps 3-8)
	lanes := uint32(phyClockLaneBit)

	for _, p := range phyLanes {
		lanes |= 1 << (2 * p)
	}

	reg.Write(c.PHY+phyCommonCtrl5, lanes)
	reg.Write(c.PHY+phyCommonCtrl6, 1)
	reg.Write(c.PHY+phyCommonCtrl7, 2)
	reg.Write(c.PHY+phyCommonCtrl0, 0)

	c.laneGroup(0, phyDataLane)
	c.laneGroup(phyClockLaneGroup, phyClockLane)

	for _, g := range []uint32{2, 4, 6} {
		c.laneGroup(g, phyDataLane)
	}

	for off := uint32(phyCommonCtrl11); off <= phyCommonCtrl21; off += 4 {
		reg.Write(c.PHY+off, 0)
	}

	return nil
}

func (c *CAMSS) laneGroup(g uint32, writes []phyLaneWrite) {
	base := c.PHY + g*0x100

	for _, w := range writes {
		val := w.val

		if w.settle {
			val = c.settle & 0xff
		}

		reg.Write(base+w.off, val)
	}
}

// Rates returns the scaling roots' rates set by the last Start.
func (c *CAMSS) Rates() CAMSSRates {
	return c.rates
}

// Settle returns the settle count programmed by the last Start.
func (c *CAMSS) Settle() uint32 {
	return c.settle
}

// Queue gives write master 7 its next buffer and latches it with an
// RDI0 register update (§5.2 steps 10 and 13).
func (c *CAMSS) Queue(addr uint32) {
	reg.Write(c.TFE+tfeWM7+wmImageAddr, addr)
	reg.Write(c.TFE+tfeRegUpdateCmd, tfeRegUpdateRDI0)
}

// CAMSSEvent is what one Poll found and cleared.
type CAMSSEvent struct {
	IRQ      uint32 // TFE IRQ_STATUS_0
	Bus      uint32 // BUS_IRQ_STATUS_0
	Overflow uint32 // BUS_OVERFLOW_STATUS
}

// BufDone reports a completed frame in the current RDI0 buffer.
func (e CAMSSEvent) BufDone() bool { return e.Bus&CAMSSBusBufDoneRDI0 != 0 }

// Err returns the violations and write master 7 overflow seen, if any.
func (e CAMSSEvent) Err() error {
	if e.Bus&CAMSSBusErrors == 0 && e.Overflow == 0 {
		return nil
	}

	var s string

	if e.Bus&CAMSSBusConfigViol != 0 {
		s += " config"
	}

	if e.Bus&CAMSSBusInputViol != 0 {
		s += " input"
	}

	if e.Bus&CAMSSBusImageSizeViol != 0 {
		s += " image-size"
	}

	if s != "" {
		s = "violation:" + s
	}

	if e.Overflow != 0 {
		if s != "" {
			s += ", "
		}

		s += fmt.Sprintf("overflow %#x", e.Overflow)
	}

	return errors.New(s)
}

// Poll reads and clears the TFE interrupt and bus status (§5.2 step
// 11), and the overflow status. It returns a zero event when nothing is
// pending.
func (c *CAMSS) Poll() (e CAMSSEvent) {
	e.IRQ = reg.Read(c.TFE + tfeIRQStatus0)

	if e.IRQ != 0 {
		reg.Write(c.TFE+tfeIRQClear0, e.IRQ)
		reg.Write(c.TFE+tfeIRQCmd, 1)
	}

	e.Bus = reg.Read(c.TFE + tfeBusIRQStatus0)

	if e.Bus != 0 {
		reg.Write(c.TFE+tfeBusIRQClear0, e.Bus)
		reg.Write(c.TFE+tfeBusIRQCmd, 1)
	}

	e.Overflow = reg.Read(c.TFE + tfeBusOverflowStatus)

	if e.Overflow != 0 {
		reg.Write(c.TFE+tfeBusStatusClear, 1)
	}

	return
}

// Stop halts the path in reverse order: CSID RDI0 off (§4.2), write
// master 7 off (§5.2), PHY lanes off (§3.2), then a TFE reset, which
// leaves the core as Reset did.
func (c *CAMSS) Stop() error {
	cfg := reg.Read(c.CSID + csidRDICfg0(0))
	reg.Write(c.CSID+csidRDICfg0(0), cfg&^rdiEnable)
	reg.Write(c.CSID+csidRDICtrl(0), 0)

	reg.Write(c.TFE+tfeWM7+wmClientCfg, 0)

	reg.Write(c.PHY+phyCommonCtrl5, 0)
	reg.Write(c.PHY+phyCommonCtrl6, 0)

	return c.ResetTFE()
}

// CAMSSStatus is a raw register snapshot of the capture path, read
// without clearing anything.
type CAMSSStatus struct {
	TFEIRQ, TFEIRQMask  uint32
	Bus, BusMask        uint32
	Overflow            uint32
	WMClientCfg         uint32
	WMImageAddr         uint32
	CSIDIRQ             uint32
	CSIDRXCfg0, RDICfg0 uint32
	RDICtrl             uint32
	PHYLanes, PHYCtrl6  uint32
	PHYStatus           [12]uint32 // COMMON_STATUS0..11
}

// Status reads the capture path's state for diagnostics.
func (c *CAMSS) Status() (s CAMSSStatus) {
	s.TFEIRQ = reg.Read(c.TFE + tfeIRQStatus0)
	s.TFEIRQMask = reg.Read(c.TFE + tfeIRQMask0)
	s.Bus = reg.Read(c.TFE + tfeBusIRQStatus0)
	s.BusMask = reg.Read(c.TFE + tfeBusIRQMask0)
	s.Overflow = reg.Read(c.TFE + tfeBusOverflowStatus)
	s.WMClientCfg = reg.Read(c.TFE + tfeWM7 + wmClientCfg)
	s.WMImageAddr = reg.Read(c.TFE + tfeWM7 + wmImageAddr)
	s.CSIDIRQ = reg.Read(c.CSID + csidIRQStatus)
	s.CSIDRXCfg0 = reg.Read(c.CSID + csidRXCfg0)
	s.RDICfg0 = reg.Read(c.CSID + csidRDICfg0(0))
	s.RDICtrl = reg.Read(c.CSID + csidRDICtrl(0))
	s.PHYLanes = reg.Read(c.PHY + phyCommonCtrl5)
	s.PHYCtrl6 = reg.Read(c.PHY + phyCommonCtrl6)

	for i := range s.PHYStatus {
		s.PHYStatus[i] = reg.Read(c.PHY + phyCommonStatus0 + uint32(i)*4)
	}

	return
}
