// Qualcomm QCM2290 USB host bring-up (GCC, QUSB2 PHY, DWC3 wrapper and core)
// https://github.com/usbarmory/tamago
//
// Copyright (c) the qcm2290 package authors
//
// Use of this source code is governed by the license
// that can be found in the LICENSE file.

package qcm2290

import (
	"fmt"
	"time"

	"github.com/usbarmory/tamago/internal/reg"
)

// The USB-C port as a USB 2.0 high-speed host: the USB30_PRIM power
// domain and clocks, the QUSB2 high-speed PHY, the controller's Qualcomm
// wrapper (UTMI clock as pipe clock, VBUS/session overrides), the DWC3
// global registers for host mode, and the controller's SMMU stream.
// Above that the controller is a standard xHCI 1.1 host at USB_BASE,
// driven by the caller.
//
// Every write goes through the display trace (disptrace.go), so that
// 'dptrace' captures a USB bring-up as well.
//
// Written from bmx's docs/usb-spec.md (section references below point
// at it); see docs/usb-provenance.md for why it is the only hardware
// documentation this driver was written from.

// Block addresses (§1)
const (
	// USB_BASE is the DWC3 controller; its xHCI registers start here.
	USB_BASE = 0x04e00000

	// USB_QSCRATCH_BASE is the controller's Qualcomm wrapper.
	USB_QSCRATCH_BASE = 0x04ef8800

	// QUSB2PHY_BASE is the high-speed PHY.
	QUSB2PHY_BASE = 0x01613000

	// QFPROM_BASE is the fuse block's corrected read window.
	QFPROM_BASE = 0x01b44000

	// USB_IRQ is the controller interrupt (SPI 255), level high.
	USB_IRQ = 287
)

// GCC USB30_PRIM power domain, roots, branches and resets (§2)
const (
	gccUSB30PrimGDSC = 0x1a004

	gccUSB30MasterCmdRCGR   = 0x1a01c
	gccUSB30MockUTMICmdRCGR = 0x1a034
	gccUSB30MockUTMIPostDiv = 0x1a04c

	gccCfgNocUSB3PrimAXICBCR = 0x1a084
	gccSysNocUSB3PrimAXICBCR = 0x1a080
	gccUSB30PrimMasterCBCR   = 0x1a010
	gccUSB30PrimSleepCBCR    = 0x1a014
	gccUSB30PrimMockUTMICBCR = 0x1a018
	gccUSB3PrimClkrefCBCR    = 0x9f000
	gccAHB2PHYUSBCBCR        = 0x1d008

	gccUSB30PrimBCR    = 0x1a000
	gccQUSB2PHYPrimBCR = 0x1c000
	gccAHB2PHYUSBBCR   = 0x1d000

	bcrAssert = 1 << 0

	// RCG CFG fields (§2): source select [10:8], 2 x divider - 1 [4:0]
	rcgCfgFields = 0x7<<8 | 0x1f

	// master root: GPLL0 early / 4.5 = 133.33 MHz (observed CFG 0x108)
	usbMasterCfg = 1<<8 | 8
	// mock UTMI root: crystal, divide 1
	usbMockUTMICfg = 0
)

// QUSB2 PHY registers, 8 bits wide at 32-bit spacing (§3)
const (
	qusb2PLLTest        = 0x04
	qusb2PLLTune        = 0x08
	qusb2PLLUserCtl1    = 0x0c
	qusb2PLLUserCtl2    = 0x10
	qusb2PLLPwrCtrl     = 0x18
	qusb2PLLAutopgmCtl1 = 0x1c
	qusb2PLLStatus      = 0x38
	qusb2PortTune1      = 0x80
	qusb2PortTune2      = 0x84
	qusb2PortTune3      = 0x88
	qusb2PortTune4      = 0x8c
	qusb2PortTest2      = 0x9c
	qusb2PortPowerdown  = 0xb4

	qusb2PLLTestClkRefSel = 1 << 7 // single-ended reference clock
	qusb2PLLLocked        = 1 << 5

	qusb2ClampNEn  = 1 << 5
	qusb2FreezioN  = 1 << 1
	qusb2PowerDown = 1 << 0

	// fused high-speed TX trim: byte 3 of this QFPROM word, bits 4:1
	qfpromUSBTrimWord = 0x258
)

// qusb2Tuning is the tuning table, in the order it is written (§3 step
// 5).
var qusb2Tuning = [...]struct{ off, val uint32 }{
	{qusb2PortTune1, 0xf8},
	{qusb2PortTune2, 0x53},
	{qusb2PortTune3, 0x81},
	{qusb2PortTune4, 0x17},
	{qusb2PLLTune, 0x30},
	{qusb2PLLUserCtl1, 0x79},
	{qusb2PLLUserCtl2, 0x21},
	{qusb2PortTest2, 0x14},
	{qusb2PLLAutopgmCtl1, 0x9f},
	{qusb2PLLPwrCtrl, 0x00},
}

// Wrapper (QSCRATCH) registers (§4)
const (
	qscratchGeneralCfg = 0x08
	qscratchHSPHYCtrl  = 0x10
	qscratchSSPHYCtrl  = 0x30

	pipeUTMIClkSel    = 1 << 0
	pipe3PHYStatusSW  = 1 << 3
	pipeUTMIClkDis    = 1 << 8
	utmiOTGVbusValid  = 1 << 20
	swSessVldSel      = 1 << 28
	lane0PwrPresent   = 1 << 24
	qscratchClkSettle = 500 * time.Microsecond
)

// DWC3 global registers (§5), offsets from USB_BASE
const (
	dwc3GSBUSCFG0    = 0xc100
	dwc3GSBUSCFG1    = 0xc104
	dwc3GCTL         = 0xc110
	dwc3GSTS         = 0xc118
	dwc3GUCTL1       = 0xc11c
	dwc3GSNPSID      = 0xc120
	dwc3GUCTL        = 0xc12c
	dwc3GUCTL2       = 0xc19c
	dwc3GUSB2PHYCFG0 = 0xc200
	dwc3GUSB3PIPECTL = 0xc2c0
	dwc3GFLADJ       = 0xc630

	gctlPrtCapMask = 3 << 12
	gctlPrtCapHost = 1 << 12

	guctl1ParkModeDisableSS = 1 << 17
	guctl2RstActBitLater    = 1 << 14

	// SUSPHY and ENBLSLPM clear, 16-bit UTMI+ turnaround 9 (§5 step 2)
	gusb2PHYCfgHost = 0x00002400
)

// dwc3Agreed holds the global registers whose value the boot chain and
// Linux's host mode agree on (§5 table, "same"). The block reset may
// return them to hardware defaults; USBHostInit puts back the observed
// value of any that changed.
var dwc3Agreed = [...]struct{ off, val uint32 }{
	{dwc3GSBUSCFG0, 0x2222000e},
	{dwc3GSBUSCFG1, 0x00001700},
	{dwc3GUCTL, 0x0d00c010},
	{dwc3GFLADJ, 0x8c80c8a0},
}

// USB SMMU stream (§6)
const (
	usbStreamID = 0x120

	// usbCB is the controller's context bank and usbSMR the stream
	// match group claimed when no valid group already matches the
	// stream: the boot chain owns banks 0-2 and groups 0-3, the display
	// bank 8 / group 8, the camera bank 9 / groups 9-12.
	usbCB  = 10
	usbSMR = 13
)

// usbHostState records the last bring-up, for USBHostStatus.
var usbHostState struct {
	mapped   bool   // SMMUMapUSBIdentity succeeded
	smr      uint32 // stream match group in use
	reused   bool   // the group was an existing match for the stream
	trim     uint32 // fused TX trim applied (0 = none)
	restored []uint32
}

// usbDelay waits at least d without the scheduler.
func usbDelay(d time.Duration) {
	for start := time.Now(); time.Since(start) < d; {
	}
}

// usbRMW replaces the masked bits of a register.
func usbRMW(addr, mask, val uint32) {
	traceWrite(addr, reg.Read(addr)&^mask|val&mask)
}

// phyWrite writes an 8-bit QUSB2 register.
func phyWrite(off, v uint32) {
	traceWrite(QUSB2PHY_BASE+off, v&0xff)
}

// phyRead reads an 8-bit QUSB2 register.
func phyRead(off uint32) uint32 {
	return reg.Read(QUSB2PHY_BASE+off) & 0xff
}

// usbRCG programs a root clock generator's source and divider fields,
// keeping the CFG register's other bits, and commits it (§2).
func usbRCG(cmd, cfg uint32) error {
	addr := uint32(GCC_BASE + cmd + rcgCfg)

	return rcgUpdate(GCC_BASE, cmd, reg.Read(addr)&^rcgCfgFields|cfg&rcgCfgFields)
}

// usbClocks powers the USB30_PRIM domain and starts its roots and
// branches (§2).
func usbClocks() error {
	if err := gdscOn(GCC_BASE, gccUSB30PrimGDSC); err != nil {
		return fmt.Errorf("usb30 prim gdsc: %v", err)
	}

	if err := usbRCG(gccUSB30MasterCmdRCGR, usbMasterCfg); err != nil {
		return fmt.Errorf("master root: %v", err)
	}

	if err := usbRCG(gccUSB30MockUTMICmdRCGR, usbMockUTMICfg); err != nil {
		return fmt.Errorf("mock utmi root: %v", err)
	}

	// mock UTMI post divider: divide by 1
	usbRMW(GCC_BASE+gccUSB30MockUTMIPostDiv, 0x3, 0)

	branches := []struct {
		name string
		cbcr uint32
	}{
		{"cfg noc usb3 axi", gccCfgNocUSB3PrimAXICBCR},
		{"sys noc usb3 axi", gccSysNocUSB3PrimAXICBCR},
		{"usb30 master", gccUSB30PrimMasterCBCR},
		{"usb30 sleep", gccUSB30PrimSleepCBCR},
		{"usb30 mock utmi", gccUSB30PrimMockUTMICBCR},
		{"usb3 clkref", gccUSB3PrimClkrefCBCR},
	}

	for _, b := range branches {
		if err := dispClkOn(GCC_BASE, b.cbcr); err != nil {
			return fmt.Errorf("%s: %v", b.name, err)
		}
	}

	// its "off" bit reads set while the PHY registers are accessible
	// (§2): not waited on
	clkOnAdvisory(GCC_BASE, gccAHB2PHYUSBCBCR, cbcrEnable)

	// the PHY register access path must not be held in reset
	if reg.Read(GCC_BASE+gccAHB2PHYUSBBCR)&bcrAssert != 0 {
		traceWrite(GCC_BASE+gccAHB2PHYUSBBCR, 0)
	}

	return nil
}

// usbControllerReset pulses the controller's block reset (§2), which
// also stops whatever the boot chain left it doing as a device.
func usbControllerReset() {
	traceWrite(GCC_BASE+gccUSB30PrimBCR, bcrAssert)
	usbDelay(100 * time.Microsecond)
	traceWrite(GCC_BASE+gccUSB30PrimBCR, 0)
	usbDelay(100 * time.Microsecond)
}

// usbFuseTrim returns the fused high-speed TX trim (§3 step 6), 0 when
// unfused.
func usbFuseTrim() uint32 {
	b := reg.Read(QFPROM_BASE+qfpromUSBTrimWord) >> 24 & 0xff
	return b >> 1 & 0xf
}

// usbPHYInit runs the QUSB2 initialisation (§3 steps 2-10). The PHY's
// clocks are on (usbClocks).
func usbPHYInit() error {
	traceWrite(GCC_BASE+gccQUSB2PHYPrimBCR, bcrAssert)
	usbDelay(150 * time.Microsecond)
	traceWrite(GCC_BASE+gccQUSB2PHYPrimBCR, 0)
	usbDelay(10 * time.Microsecond)

	// disable
	phyWrite(qusb2PortPowerdown, phyRead(qusb2PortPowerdown)|qusb2ClampNEn|qusb2FreezioN|qusb2PowerDown)

	pllTest := phyRead(qusb2PLLTest)

	for _, t := range qusb2Tuning {
		phyWrite(t.off, t.val)
	}

	usbHostState.trim = usbFuseTrim()

	if trim := usbHostState.trim; trim != 0 {
		phyWrite(qusb2PortTune2, phyRead(qusb2PortTune2)&0x0f|trim<<4)
	}

	// enable: clamp and freeze bits stay set
	phyWrite(qusb2PortPowerdown, phyRead(qusb2PortPowerdown)&^qusb2PowerDown)
	usbDelay(150 * time.Microsecond)

	// single-ended reference clock
	phyWrite(qusb2PLLTest, pllTest|qusb2PLLTestClkRefSel)
	_ = phyRead(qusb2PLLTest)

	usbDelay(100 * time.Microsecond)

	// §3 step 10 checks once after 100 µs; a little more time is given
	// before calling it failed
	if !pollTime(QUSB2PHY_BASE+qusb2PLLStatus, qusb2PLLLocked, qusb2PLLLocked, 5*time.Millisecond) {
		return fmt.Errorf("QUSB2 PLL not locked (PLL_STATUS %#02x PLL_TEST %#02x POWERDOWN %#02x)",
			phyRead(qusb2PLLStatus), phyRead(qusb2PLLTest), phyRead(qusb2PortPowerdown))
	}

	return nil
}

// usbWrapperUTMIClock selects the UTMI clock as the pipe clock (§4), for
// high-speed operation without the USB 3 PHY.
func usbWrapperUTMIClock() {
	cfg := uint32(USB_QSCRATCH_BASE + qscratchGeneralCfg)

	traceWrite(cfg, reg.Read(cfg)|pipeUTMIClkDis)
	usbDelay(qscratchClkSettle)

	traceWrite(cfg, reg.Read(cfg)|pipeUTMIClkSel|pipe3PHYStatusSW)
	usbDelay(qscratchClkSettle)

	traceWrite(cfg, reg.Read(cfg)&^pipeUTMIClkDis)
}

// usbWrapperVbus sets the VBUS valid and session overrides (§4).
func usbWrapperVbus() {
	hs := uint32(USB_QSCRATCH_BASE + qscratchHSPHYCtrl)
	ss := uint32(USB_QSCRATCH_BASE + qscratchSSPHYCtrl)

	traceWrite(hs, reg.Read(hs)|utmiOTGVbusValid|swSessVldSel)
	traceWrite(ss, reg.Read(ss)|lane0PwrPresent)
}

// usbCoreHost configures the DWC3 global registers for host mode (§5
// steps 2-4).
func usbCoreHost() {
	b := uint32(USB_BASE)

	usbHostState.restored = usbHostState.restored[:0]

	for _, r := range dwc3Agreed {
		if reg.Read(b+r.off) != r.val {
			usbHostState.restored = append(usbHostState.restored, r.off)
			traceWrite(b+r.off, r.val)
		}
	}

	traceWrite(b+dwc3GUSB2PHYCFG0, gusb2PHYCfgHost)
	usbRMW(b+dwc3GCTL, gctlPrtCapMask, gctlPrtCapHost)
	traceWrite(b+dwc3GUCTL1, reg.Read(b+dwc3GUCTL1)|guctl1ParkModeDisableSS)
	traceWrite(b+dwc3GUCTL2, reg.Read(b+dwc3GUCTL2)|guctl2RstActBitLater)
}

// smrMatches reports whether a valid stream match register value
// matches stream id.
func smrMatches(smr, id uint32) bool {
	if smr&smrValid == 0 {
		return false
	}

	mask := smr >> 16 & 0x7fff

	return (smr^id)&0xffff&^mask == 0
}

// SMMUMapUSBIdentity routes the controller's stream (ID 0x120, mask 0)
// through context bank 10 over the DDR identity table at pt (§6;
// display-spec §8), so that xHCI DMA addresses equal physical ones. A
// valid stream match group that already matches the stream (the boot
// chain runs the port as a device) is reprogrammed; otherwise group 13
// is claimed. pt is filled by this call; it may be a page of its own or
// the display's (the content is identical).
func SMMUMapUSBIdentity(pt uint32) error {
	idr0 := reg.Read(SMMU_BASE + smmuIDR0)
	idr1 := reg.Read(SMMU_BASE + smmuIDR1)
	groups := idr0 & 0xff

	if n := idr1 & 0xff; usbCB >= n {
		return fmt.Errorf("SMMU has %d context banks", n)
	}

	smr := uint32(usbSMR)
	reused := false

	for n := uint32(0); n < groups; n++ {
		v := reg.Read(SMMU_BASE + smmuSMR0 + n*4)

		if smrMatches(v, usbStreamID) {
			smr, reused = n, true
			break
		}
	}

	if smr >= groups {
		return fmt.Errorf("SMMU has %d stream match groups", groups)
	}

	// refuse a bank another valid group already routes to
	for n := uint32(0); n < groups; n++ {
		if n == smr {
			continue
		}

		v := reg.Read(SMMU_BASE + smmuSMR0 + n*4)
		s2cr := reg.Read(SMMU_BASE + smmuS2CR0 + n*4)

		if v&smrValid != 0 && s2cr&s2crTypeCBMask == s2crTypeTrans|usbCB {
			return fmt.Errorf("context bank %d in use by group %d (SMR %#x)", usbCB, n, v)
		}
	}

	// a claimed (not matching) group must be free or ours
	if !reused {
		v := reg.Read(SMMU_BASE + smmuSMR0 + smr*4)
		s2cr := reg.Read(SMMU_BASE + smmuS2CR0 + smr*4)

		if v&smrValid != 0 && s2cr&s2crTypeCBMask != s2crTypeTrans|usbCB {
			return fmt.Errorf("stream match group %d in use (SMR %#x S2CR %#x)", smr, v, s2cr)
		}
	}

	traceWrite(SMMU_BASE+smmuSMR0+smr*4, 0)

	if err := identityBank(usbCB, pt); err != nil {
		return err
	}

	traceWrite(SMMU_BASE+smmuS2CR0+smr*4, s2crTypeTrans|usbCB)
	traceWrite(SMMU_BASE+smmuSMR0+smr*4, smrValid|usbStreamID)

	usbHostState.smr, usbHostState.reused = smr, reused
	usbHostState.mapped = true

	return nil
}

// SMMUUSBFault returns the USB context bank's fault status, faulting
// address and syndrome.
func SMMUUSBFault() (fsr uint32, far uint64, fsynr0 uint32) {
	cb := contextBankBase(usbCB)

	fsr = reg.Read(cb + cbFSR)
	far = reg.Read64(uint64(cb + cbFAR))
	fsynr0 = reg.Read(cb + cbFSYNR0)

	return
}

// USBHostInit brings the USB-C port up as a USB 2.0 high-speed host
// controller: power domain and clocks (§2), controller block reset, SMMU
// stream over the identity table at pt (§6), QUSB2 PHY (§3), wrapper
// clock selection (§4), DWC3 host configuration (§5 steps 2-4) and the
// VBUS/session overrides (§5 step 5). It returns the xHCI register base;
// the caller then runs the controller as a standard xHCI host (§5 step
// 6). pt is one 4 KB aligned page this call fills.
func USBHostInit(pt uint32) (xhciBase uint32, err error) {
	// A block reset left asserted (USBHostStop) holds the controller's
	// branch clocks reporting "off", so usbClocks would time out waiting
	// on them: release it first. usbControllerReset pulses it again
	// once the clocks run.
	if reg.Read(GCC_BASE+gccUSB30PrimBCR)&bcrAssert != 0 {
		traceWrite(GCC_BASE+gccUSB30PrimBCR, 0)
		usbDelay(100 * time.Microsecond)
	}

	if err = usbClocks(); err != nil {
		return 0, err
	}

	usbControllerReset()

	if err = SMMUMapUSBIdentity(pt); err != nil {
		return 0, fmt.Errorf("smmu: %v", err)
	}

	if err = usbPHYInit(); err != nil {
		return 0, err
	}

	usbWrapperUTMIClock()
	usbCoreHost()
	usbWrapperVbus()

	// the USB master's default priority on the system NoC (§7.1): at
	// the boot chain's 0 a live panel scanout costs the controller
	// isochronous service intervals
	if err = SetSNOCQoS(SNOCPortUSB3, SNOCQoSUSB); err != nil {
		return 0, err
	}

	if id := reg.Read(USB_BASE + dwc3GSNPSID); id>>16 != 0x5533 {
		return 0, fmt.Errorf("no DWC3 core answering (GSNPSID %#08x)", id)
	}

	// GSTS bits 1:0 report the current mode (1 = host)
	if !pollTime(USB_BASE+dwc3GSTS, 3, 1, 10*time.Millisecond) {
		return 0, fmt.Errorf("controller not in host mode (GSTS %#08x, GCTL %#08x)",
			reg.Read(USB_BASE+dwc3GSTS), reg.Read(USB_BASE+dwc3GCTL))
	}

	return USB_BASE, nil
}

// USBHostStop holds the controller in its block reset and powers the
// high-speed PHY down. The caller has halted the xHCI first. Clocks and
// the power domain stay on; USBHostInit starts over from here (it
// releases the held reset before waiting on the branch clocks, which
// read "off" while the block is in reset).
func USBHostStop() {
	traceWrite(GCC_BASE+gccUSB30PrimBCR, bcrAssert)
	phyWrite(qusb2PortPowerdown, phyRead(qusb2PortPowerdown)|qusb2ClampNEn|qusb2FreezioN|qusb2PowerDown)
}

// USBRegs is a snapshot of the USB host bring-up registers.
type USBRegs struct {
	GDSC, MasterCBCR, MockUTMICBCR, AHB2PHYCBCR, ControllerBCR uint32

	// SNOC QoS of the USB and display master ports (noc.go, §7.1)
	SNOCUSB, SNOCDisplay uint32

	GCTL, GSTS, GUCTL1, GUCTL2, GSNPSID, GUSB2PHYCFG, GUSB3PIPECTL uint32
	GSBUSCFG0, GFLADJ                                              uint32

	GeneralCfg, HSPHYCtrl, SSPHYCtrl uint32

	PLLStatus, PLLTest, PortTune2, PortPowerdown uint32

	// Fuse is the fused TX trim; Restored lists the DWC3 global
	// registers the last USBHostInit put back to their agreed value.
	Fuse     uint32
	Restored []uint32

	// Mapped reports that the USB stream was routed; SMR is the stream
	// match group in use, Reused whether it was an existing match for
	// the stream; SMRValue and S2CR its registers.
	Mapped              bool
	SMR, SMRValue, S2CR uint32
	Reused              bool
}

// USBHostStatus reads the USB host bring-up registers. The DWC3 and
// wrapper fields are only meaningful (and only read) while the domain
// is powered and the controller out of reset.
func USBHostStatus() (r USBRegs) {
	r.GDSC = reg.Read(GCC_BASE + gccUSB30PrimGDSC)
	r.MasterCBCR = reg.Read(GCC_BASE + gccUSB30PrimMasterCBCR)
	r.MockUTMICBCR = reg.Read(GCC_BASE + gccUSB30PrimMockUTMICBCR)
	r.AHB2PHYCBCR = reg.Read(GCC_BASE + gccAHB2PHYUSBCBCR)
	r.ControllerBCR = reg.Read(GCC_BASE + gccUSB30PrimBCR)
	r.SNOCUSB, _ = SNOCQoS(SNOCPortUSB3)
	r.SNOCDisplay, _ = SNOCQoS(SNOCPortMDP0)

	r.Fuse = usbFuseTrim()
	r.Restored = usbHostState.restored
	r.Mapped = usbHostState.mapped
	r.SMR, r.Reused = usbHostState.smr, usbHostState.reused

	if r.Mapped {
		r.SMRValue = reg.Read(SMMU_BASE + smmuSMR0 + r.SMR*4)
		r.S2CR = reg.Read(SMMU_BASE + smmuS2CR0 + r.SMR*4)
	}

	// an access to an unpowered or unclocked block hangs the bus
	if r.GDSC&gdscPwrOn == 0 || r.MasterCBCR&cbcrOff != 0 {
		return
	}

	// PHY registers need the AHB2PHY branch enabled
	if r.AHB2PHYCBCR&cbcrEnable == 0 {
		return
	}

	r.PLLStatus = phyRead(qusb2PLLStatus)
	r.PLLTest = phyRead(qusb2PLLTest)
	r.PortTune2 = phyRead(qusb2PortTune2)
	r.PortPowerdown = phyRead(qusb2PortPowerdown)

	if r.ControllerBCR&bcrAssert != 0 {
		return
	}

	b := uint32(USB_BASE)

	r.GCTL = reg.Read(b + dwc3GCTL)
	r.GSTS = reg.Read(b + dwc3GSTS)
	r.GUCTL1 = reg.Read(b + dwc3GUCTL1)
	r.GUCTL2 = reg.Read(b + dwc3GUCTL2)
	r.GSNPSID = reg.Read(b + dwc3GSNPSID)
	r.GUSB2PHYCFG = reg.Read(b + dwc3GUSB2PHYCFG0)
	r.GUSB3PIPECTL = reg.Read(b + dwc3GUSB3PIPECTL)
	r.GSBUSCFG0 = reg.Read(b + dwc3GSBUSCFG0)
	r.GFLADJ = reg.Read(b + dwc3GFLADJ)

	r.GeneralCfg = reg.Read(USB_QSCRATCH_BASE + qscratchGeneralCfg)
	r.HSPHYCtrl = reg.Read(USB_QSCRATCH_BASE + qscratchHSPHYCtrl)
	r.SSPHYCtrl = reg.Read(USB_QSCRATCH_BASE + qscratchSSPHYCtrl)

	return
}

// USBHostAccessible reports whether the controller's registers may be
// read (domain powered, master clock running, out of reset): reading
// them otherwise hangs the bus.
func USBHostAccessible() bool {
	return reg.Read(GCC_BASE+gccUSB30PrimGDSC)&gdscPwrOn != 0 &&
		reg.Read(GCC_BASE+gccUSB30PrimMasterCBCR)&cbcrOff == 0 &&
		reg.Read(GCC_BASE+gccUSB30PrimBCR)&bcrAssert == 0
}
