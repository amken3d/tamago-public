// Qualcomm QCM2290 display clock controller support
// https://github.com/usbarmory/tamago
//
// Copyright (c) the qcm2290 package authors
//
// Use of this source code is governed by the license
// that can be found in the LICENSE file.

package qcm2290

import (
	"errors"

	"github.com/usbarmory/tamago/internal/reg"
)

// Display clock controller and GCC display-domain registers
// (drivers/clk/qcom/dispcc-qcm2290.c, gcc-qcm2290.c). Only the clocks
// the DSI link needs are driven here: the MDP/DPU clocks are D3.
const (
	DISPCC_BASE = 0x05f00000

	// MDSS GDSC power domain
	dispccGDSCR   = 0x3000
	gdscSWCollapse = 0
	gdscPowerOn    = 31

	// RCGs (cmd_rcgr; CFG at +4)
	dispccPclk0CmdRCGR = 0x205c
	dispccByte0CmdRCGR = 0x20a4
	dispccByte0Div     = 0x20bc
	dispccEsc0CmdRCGR  = 0x20c0
	dispccAHBCmdRCGR   = 0x2154

	// branch CBCRs
	dispccPclk0CBCR      = 0x2004
	dispccByte0CBCR      = 0x201c
	dispccByte0IntfCBCR  = 0x2020
	dispccEsc0CBCR       = 0x2024
	dispccAHBCBCR        = 0x2044
	dispccNonGDSCAHBCBCR = 0x4004

	// CFG_RCGR source selects (dispcc parent maps): the DSI0 PHY PLL
	// byte/pixel outputs are source 1 on their RCGs; XO is source 0
	cfgSrcDSI0PLL = 1 << 8
	cfgSrcXODiv1  = 0<<8 | 1

	// GCC display-domain branches
	gccDispAHBCBCR   = 0x1700c
	gccDispHFAXICBCR = 0x17020
	gccDispXOCBCR    = 0x1702c

	branchEnable = 0
)

// dispClkOn enables a simple clock branch and waits for it to run.
func dispClkOn(base, cbcr uint32) error {
	reg.Set(base+cbcr, branchEnable)

	for i := 0; i < clkPollSpins; i++ {
		if !reg.Get(base+cbcr, CLK_OFF) {
			return nil
		}
	}

	return errors.New("branch stuck off")
}

// rcgUpdate commits a CFG_RCGR configuration.
func rcgUpdate(base, cmdRCGR, cfg uint32) error {
	reg.Write(base+cmdRCGR+0x4, cfg)
	reg.Set(base+cmdRCGR, CMD_UPDATE)

	for i := 0; i < clkPollSpins; i++ {
		if !reg.Get(base+cmdRCGR, CMD_UPDATE) {
			return nil
		}
	}

	return errors.New("RCG update timeout")
}

// EnableDisplayPower brings up everything needed to touch the DSI
// controller and PHY registers: the GCC display-domain branches, the
// MDSS GDSC power domain, and the dispcc AHB interface clock.
//
// The GCC display AHB branch reports CLK_OFF even when enabled and
// serving a running display (dynamic root gating; observed live under
// Linux), so the GCC-side branches are enabled without treating a
// stuck halt bit as fatal.
func EnableDisplayPower() error {
	// GCC side: AHB access (with retention bits, as Linux leaves it),
	// AXI, and XO to the display subsystem
	reg.SetN(GCC_BASE+gccDispAHBCBCR, 0, 0xf, 0xb)
	dispClkOn(GCC_BASE, gccDispHFAXICBCR)
	dispClkOn(GCC_BASE, gccDispXOCBCR)

	// dispcc AHB clock source: XO/1 (runs the register interface)
	if err := rcgUpdate(DISPCC_BASE, dispccAHBCmdRCGR, cfgSrcXODiv1); err != nil {
		return errors.New("dispcc ahb rcg: " + err.Error())
	}

	if err := dispClkOn(DISPCC_BASE, dispccNonGDSCAHBCBCR); err != nil {
		return errors.New("dispcc non-gdsc ahb: " + err.Error())
	}

	// MDSS GDSC: clear the software collapse bit, wait for power-on
	reg.Clear(DISPCC_BASE+dispccGDSCR, gdscSWCollapse)

	for i := 0; ; i++ {
		if reg.Get(DISPCC_BASE+dispccGDSCR, gdscPowerOn) {
			break
		}

		if i >= clkPollSpins {
			return errors.New("MDSS GDSC stuck off")
		}
	}

	return dispClkOn(DISPCC_BASE, dispccAHBCBCR)
}

// EnableDSIClocks routes the DSI PHY PLL outputs into the display
// clock tree: byte0 and pclk0 from the PLL (which must be locked), and
// the escape clock from XO. Call after the PLL reports lock.
func EnableDSIClocks(m *DisplayMode) error {
	if err := rcgUpdate(DISPCC_BASE, dispccByte0CmdRCGR, cfgSrcDSI0PLL); err != nil {
		return errors.New("byte0 rcg: " + err.Error())
	}

	// byte0_intf divider, as the live system runs the mode (/2 at
	// 1080p, /1 for the 800x480 panel)
	reg.Write(DISPCC_BASE+dispccByte0Div, m.byte0IntfDiv)

	if err := rcgUpdate(DISPCC_BASE, dispccPclk0CmdRCGR, cfgSrcDSI0PLL); err != nil {
		return errors.New("pclk0 rcg: " + err.Error())
	}

	if err := rcgUpdate(DISPCC_BASE, dispccEsc0CmdRCGR, cfgSrcXODiv1); err != nil {
		return errors.New("esc0 rcg: " + err.Error())
	}

	for _, c := range []struct {
		name string
		cbcr uint32
	}{
		{"byte0", dispccByte0CBCR},
		{"byte0 intf", dispccByte0IntfCBCR},
		{"pclk0", dispccPclk0CBCR},
		{"esc0", dispccEsc0CBCR},
	} {
		if err := dispClkOn(DISPCC_BASE, c.cbcr); err != nil {
			return errors.New(c.name + ": " + err.Error())
		}
	}

	return nil
}
