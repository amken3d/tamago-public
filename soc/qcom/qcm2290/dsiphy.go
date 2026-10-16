// Qualcomm QCM2290 DSI PHY and PLL (14 nm PHY)
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

// Written from bmx's docs/display-spec.md §6; see
// docs/display-provenance.md.

// DSI PHY bases (§1)
const (
	DSIPHY_BASE  = 0x05e94400 // common block
	DSILANE_BASE = 0x05e94500
	DSIPLL_BASE  = 0x05e94800
)

// PHY common registers used by name, and the PLL status (§6)
const (
	phyClkCfg       = 0x10
	phyGlblTestCtrl = 0x18
	phyCtrl0        = 0x1c
	phyCtrl1        = 0x20
	phyPLLCntrl     = 0x48
	phyLDOCntrl     = 0x4c

	pllStatus       = 0x0cc
	pllStatusLocked = 1 << 5
	pllStatusReady  = 1 << 0

	pllLockTimeout = 100 * time.Millisecond
)

// writeTable writes a register table at a block base, in order.
func writeTable(base uint32, t []RegVal) {
	for _, r := range t {
		reg.Write(base+r.Off, r.Val)
	}
}

// InitDSIPHY powers the DSI PHY, programs its lanes and PLL for the
// mode and starts the PLL, waiting for lock and ready (§6). The DSI
// link clocks can be enabled once it returns (EnableDSILinkClocks).
// EnableDisplayPower must have been called.
func InitDSIPHY(m *DisplayMode) error {
	// 1. controller PHY-interface reset
	reg.Write(DSI0_BASE+dsiPHYReset, 0x01000001)
	time.Sleep(100 * time.Microsecond)
	reg.Write(DSI0_BASE+dsiPHYReset, 0)

	// 2. LDO and global test control
	reg.Write(DSIPHY_BASE+phyLDOCntrl, 0x1c)
	reg.Write(DSIPHY_BASE+phyGlblTestCtrl, 0x01)

	// 3. lanes
	writeTable(DSILANE_BASE, m.PHYLanes)

	// 4. digital reset with the PLL stopped
	reg.Write(DSIPHY_BASE+phyPLLCntrl, 0)
	reg.Write(DSIPHY_BASE+phyCtrl1, 0x80)
	time.Sleep(100 * time.Microsecond)
	reg.Write(DSIPHY_BASE+phyCtrl1, 0)

	// 5. PLL and all lanes powered
	reg.Write(DSIPHY_BASE+phyCtrl0, 0xff)

	// 6. PLL frequency, spread spectrum, trims
	writeTable(DSIPLL_BASE, m.PLL)

	// 7. PLL software reset
	reg.Write(DSIPHY_BASE+phyPLLCntrl, 0)
	reg.Write(DSIPHY_BASE+phyCtrl1, 0x20)
	time.Sleep(10 * time.Microsecond)
	reg.Write(DSIPHY_BASE+phyCtrl1, 0)

	// 8. common registers, never PLL_CNTRL from a table
	for _, r := range m.PHYCommon {
		if r.Off != phyPLLCntrl {
			reg.Write(DSIPHY_BASE+r.Off, r.Val)
		}
	}

	// 9. start the PLL
	reg.Write(DSIPHY_BASE+phyPLLCntrl, 1)

	if !pollTime(DSIPLL_BASE+pllStatus, pllStatusLocked, pllStatusLocked, pllLockTimeout) {
		return fmt.Errorf("DSI PLL lock timeout (status %#x)", reg.Read(DSIPLL_BASE+pllStatus))
	}

	if !pollTime(DSIPLL_BASE+pllStatus, pllStatusReady, pllStatusReady, pllLockTimeout) {
		return fmt.Errorf("DSI PLL ready timeout (status %#x)", reg.Read(DSIPLL_BASE+pllStatus))
	}

	return nil
}

// DSIPLLStatus returns the PLL status register (bit 5 locked, bit 0
// ready) and the PHY clock configuration.
func DSIPLLStatus() (status, clkCfg uint32) {
	return reg.Read(DSIPLL_BASE + pllStatus), reg.Read(DSIPHY_BASE + phyClkCfg)
}
