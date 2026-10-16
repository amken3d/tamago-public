// Qualcomm QCM2290 display clocks and power (GCC display branches, DISPCC)
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

// Written from bmx's docs/display-spec.md (section references below
// point at it); see docs/display-provenance.md for why it is the only
// hardware documentation this driver was written from.

// DISPCC_BASE is the display clock controller (§1).
const DISPCC_BASE = 0x05f00000

// Clock and power primitives (§2)
const (
	cbcrEnable = 1 << 0
	cbcrOff    = 1 << 31

	rcgCmdUpdate = 1 << 0
	rcgCfg       = 0x4 // CFG register, 4 bytes after CMD

	gdscCollapse = 1 << 0
	gdscPwrOn    = 1 << 31

	// clkPollTime bounds every branch, RCG and GDSC poll
	clkPollTime = 10 * time.Millisecond
)

// pollTime polls addr until (value & mask) == want or the timeout
// expires, without the scheduler (polled drivers, runtime may idle).
func pollTime(addr, mask, want uint32, timeout time.Duration) bool {
	start := time.Now()

	for {
		if reg.Read(addr)&mask == want {
			return true
		}

		if time.Since(start) > timeout {
			return false
		}
	}
}

// rcgUpdate programs a root clock generator's CFG (source select bits
// 10:8, divider 2 x div - 1 in bits 4:0) and commits it through the CMD
// register, waiting for the hardware to clear the update bit (§2).
func rcgUpdate(base, cmd, cfg uint32) error {
	reg.Write(base+cmd+rcgCfg, cfg)
	reg.Write(base+cmd, reg.Read(base+cmd)|rcgCmdUpdate)

	if !pollTime(base+cmd, rcgCmdUpdate, 0, clkPollTime) {
		return fmt.Errorf("RCG %#x update timeout (cmd %#x)", base+cmd, reg.Read(base+cmd))
	}

	return nil
}

// dispClkOn enables a branch clock and waits for its off status bit to
// clear (§2).
func dispClkOn(base, cbcr uint32) error {
	reg.Write(base+cbcr, reg.Read(base+cbcr)|cbcrEnable)

	if !pollTime(base+cbcr, cbcrOff, 0, clkPollTime) {
		return fmt.Errorf("branch %#x stuck off (%#x)", base+cbcr, reg.Read(base+cbcr))
	}

	return nil
}

// clkOnAdvisory enables a branch whose status is unreliable under
// dynamic gating (§2, §5.1): the enable is written and the status is
// not waited on.
func clkOnAdvisory(base, cbcr, set uint32) {
	reg.Write(base+cbcr, reg.Read(base+cbcr)|set)
}

// gdscOn powers a GDSC power domain: software collapse off, then wait
// for power on (§2).
func gdscOn(base, gdsc uint32) error {
	reg.Write(base+gdsc, reg.Read(base+gdsc)&^gdscCollapse)

	if !pollTime(base+gdsc, gdscPwrOn, gdscPwrOn, clkPollTime) {
		return fmt.Errorf("GDSC %#x power-on timeout (%#x)", base+gdsc, reg.Read(base+gdsc))
	}

	return nil
}

// GCC display branches and DISPCC registers (§5)
const (
	gccDispAHBCBCR   = 0x1700c
	gccDispHFAXICBCR = 0x17020
	gccDispXOCBCR    = 0x1702c

	// enable plus retention bits (§5.1)
	gccDispAHBEnable = 0xb

	dispccAHBCmdRCGR    = 0x2154
	dispccNonGDSCAHB    = 0x4004
	dispccMDSSGDSC      = 0x3000
	dispccMDSSAHBCBCR   = 0x2044
	dispccByteCmdRCGR   = 0x20a4
	dispccByteIntfDiv   = 0x20bc
	dispccPixelCmdRCGR  = 0x205c
	dispccEscCmdRCGR    = 0x20c0
	dispccByteCBCR      = 0x201c
	dispccByteIntfCBCR  = 0x2020
	dispccPixelCBCR     = 0x2004
	dispccEscCBCR       = 0x2024
	dispccMDPCmdRCGR    = 0x2074
	dispccMDPCBCR       = 0x2008
	dispccMDPLUTCBCR    = 0x2010
	dispccVsyncCBCR     = 0x2018
	dispccVsyncCmdRCGR  = 0x208c
	dispccCfgXO         = 0x001 // crystal, divide by 1
	dispccPLL0Mode      = 0x00
	dispccPLL0L         = 0x04
	dispccPLL0ConfigCtl = 0x10
	dispccPLL0ConfigHi  = 0x14
	dispccPLL0UserCtl   = 0x18

	// PLL0 mode bits (§5.3)
	pll0OutputOn     = 1 << 0
	pll0BypassOff    = 1 << 1
	pll0ResetRelease = 1 << 2
	pll0Lock         = 1 << 31
)

// EnableDisplayPower clocks the display register interface and powers
// the MDSS domain (§5.1). It must precede any MDSS, DPU, DSI or DSI PHY
// register access: an access to the unclocked blocks hangs the bus.
func EnableDisplayPower() error {
	// these three may report "off" while running: not waited on
	clkOnAdvisory(GCC_BASE, gccDispAHBCBCR, gccDispAHBEnable)
	clkOnAdvisory(GCC_BASE, gccDispHFAXICBCR, cbcrEnable)
	clkOnAdvisory(GCC_BASE, gccDispXOCBCR, cbcrEnable)

	if err := rcgUpdate(DISPCC_BASE, dispccAHBCmdRCGR, dispccCfgXO); err != nil {
		return fmt.Errorf("dispcc ahb: %v", err)
	}

	if err := dispClkOn(DISPCC_BASE, dispccNonGDSCAHB); err != nil {
		return fmt.Errorf("dispcc non-gdsc ahb: %v", err)
	}

	if err := gdscOn(DISPCC_BASE, dispccMDSSGDSC); err != nil {
		return fmt.Errorf("mdss gdsc: %v", err)
	}

	if err := dispClkOn(DISPCC_BASE, dispccMDSSAHBCBCR); err != nil {
		return fmt.Errorf("mdss ahb: %v", err)
	}

	return nil
}

// EnableDPUClock starts DISPCC PLL0 (768 MHz) and runs the MDP core
// root from it with the mode's configuration (§5.3).
func EnableDPUClock(m *DisplayMode) error {
	pll := uint32(DISPCC_BASE)

	if reg.Read(pll+dispccPLL0Mode)&pll0Lock == 0 {
		reg.Write(pll+dispccPLL0L, 0x28)
		reg.Write(pll+dispccPLL0ConfigCtl, 0x00200001)
		reg.Write(pll+dispccPLL0ConfigHi, 0x00000004)
		reg.Write(pll+dispccPLL0UserCtl, 0x4001055b)

		mode := reg.Read(pll + dispccPLL0Mode)
		mode |= pll0BypassOff
		reg.Write(pll+dispccPLL0Mode, mode)
		time.Sleep(10 * time.Microsecond)

		mode |= pll0ResetRelease
		reg.Write(pll+dispccPLL0Mode, mode)
		time.Sleep(50 * time.Microsecond)

		if !pollTime(pll+dispccPLL0Mode, pll0Lock, pll0Lock, clkPollTime) {
			return fmt.Errorf("dispcc PLL0 lock timeout (mode %#x)", reg.Read(pll+dispccPLL0Mode))
		}
	}

	reg.Write(pll+dispccPLL0Mode, reg.Read(pll+dispccPLL0Mode)|pll0OutputOn)

	if err := rcgUpdate(DISPCC_BASE, dispccMDPCmdRCGR, m.MDPCfg); err != nil {
		return fmt.Errorf("mdp root: %v", err)
	}

	for _, b := range []uint32{dispccMDPCBCR, dispccMDPLUTCBCR, dispccVsyncCBCR} {
		if err := dispClkOn(DISPCC_BASE, b); err != nil {
			return err
		}
	}

	if err := rcgUpdate(DISPCC_BASE, dispccVsyncCmdRCGR, dispccCfgXO); err != nil {
		return fmt.Errorf("vsync root: %v", err)
	}

	return nil
}

// EnableDSILinkClocks runs the DSI byte, byte interface, pixel and
// escape clocks from the DSI PHY PLL outputs (§5.2). The PLL must have
// locked (InitDSIPHY): an RCG cannot switch to a stopped source.
func EnableDSILinkClocks(m *DisplayMode) error {
	if err := rcgUpdate(DISPCC_BASE, dispccByteCmdRCGR, m.ByteCfg); err != nil {
		return fmt.Errorf("byte root: %v", err)
	}

	reg.Write(DISPCC_BASE+dispccByteIntfDiv, m.ByteIntfDiv)

	if err := rcgUpdate(DISPCC_BASE, dispccPixelCmdRCGR, m.PixelCfg); err != nil {
		return fmt.Errorf("pixel root: %v", err)
	}

	if err := rcgUpdate(DISPCC_BASE, dispccEscCmdRCGR, dispccCfgXO); err != nil {
		return fmt.Errorf("escape root: %v", err)
	}

	branches := []struct {
		name string
		cbcr uint32
	}{
		{"byte", dispccByteCBCR},
		{"byte intf", dispccByteIntfCBCR},
		{"pixel", dispccPixelCBCR},
		{"escape", dispccEscCBCR},
	}

	for _, b := range branches {
		if err := dispClkOn(DISPCC_BASE, b.cbcr); err != nil {
			return fmt.Errorf("%s: %v", b.name, err)
		}
	}

	return nil
}

// DisplayClockStatus returns the DISPCC branch CBCRs (MDSS AHB, MDP,
// byte, byte interface, pixel, escape) for diagnostics (bit 31 off).
func DisplayClockStatus() []uint32 {
	var v []uint32

	for _, cbcr := range []uint32{
		dispccMDSSAHBCBCR, dispccMDPCBCR, dispccByteCBCR,
		dispccByteIntfCBCR, dispccPixelCBCR, dispccEscCBCR,
	} {
		v = append(v, reg.Read(DISPCC_BASE+cbcr))
	}

	return v
}
