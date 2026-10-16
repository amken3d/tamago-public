// Qualcomm DSI 6G v2.4 controller driver (video mode + test pattern)
// https://github.com/usbarmory/tamago
//
// Copyright (c) the qcm2290 package authors
//
// Use of this source code is governed by the license
// that can be found in the LICENSE file.

package qcm2290

import (
	"github.com/usbarmory/tamago/internal/reg"
)

// DSI controller registers, physical offsets from the controller base
// (dsi_host.c with the 6G +4 shift applied; matches techpack
// dsi_ctrl_reg.h). Timing values replay this hardware's live 1080p60
// snapshot: sync-pulse traffic with HSA_HE, RGB888, 4 lanes.
const (
	DSI_CTRL_BASE = 0x05e94000

	dsiCTRL            = 0x004
	dsiSTATUS0         = 0x008
	dsiVIDCFG0         = 0x010
	dsiHSTimer         = 0x014
	dsiVIDCFG1         = 0x020
	dsiActiveH         = 0x024
	dsiActiveV         = 0x028
	dsiTotal           = 0x02c
	dsiActiveHSync     = 0x030
	dsiVSyncHPos       = 0x034
	dsiVSyncVPos       = 0x038
	dsiCmdDMACtrl      = 0x03c
	dsiTrigCtrl        = 0x084
	dsiLaneCtrl        = 0x0ac
	dsiLaneSwapCtrl    = 0x0b0
	dsiClkoutTiming    = 0x0c4
	dsiEOTPacketCtrl   = 0x0cc
	dsiErrIntMask0     = 0x10c
	dsiIntrCtrl        = 0x110
	dsiReset           = 0x118
	dsiClkCtrl         = 0x11c
	dsiClkStatus       = 0x120
	dsiTPGCtrl         = 0x15c
	dsiTPGVideoInitVal = 0x164
	dsiTPGMainControl  = 0x19c
	dsiTPGVideoConfig  = 0x1a4
	dsiTimingDBMode    = 0x1e8

	dsiClkCtrlAllOn = 0x23f
)

// spinDelay busy-waits by repeated MMIO reads (~0.3 us each).
func spinDelay(reads int) {
	for i := 0; i < reads; i++ {
		reg.Read(DSI_CTRL_BASE + dsiSTATUS0)
	}
}

// dsiCtrlSnap replays every programmed controller register from the
// live 1080p60 snapshot (RO and order-critical registers excluded:
// version, status, CTRL, reset, clock status). Notably it includes
// the sync-pulse HSYNC/VSYNC counts (0x40/0x44) and per-lane timing
// groups (0x7c/0x80) that the mainline programming recipe omits.
var dsiCtrlSnap = []regVal{
	{0x00c, 0x00001010}, {0x010, 0x10009030}, {0x014, 0x31211101}, {0x018, 0x3e2e1e0e},
	{0x01c, 0x00001900}, {0x024, 0x084000c0}, {0x028, 0x04610029}, {0x02c, 0x04640897},
	{0x030, 0x002c0000}, {0x038, 0x00050000}, {0x03c, 0x14000000}, {0x040, 0x06100006},
	{0x044, 0x00003c2c}, {0x054, 0x00000900}, {0x07c, 0x22211211}, {0x080, 0x001c1a02},
	{0x084, 0x80001004}, {0x0a8, 0x00001f00}, {0x0b4, 0x00088888},
	{0x0b8, 0xffffffff}, {0x0bc, 0x0000ffff}, {0x0c0, 0x00000001}, {0x0c4, 0x00000d30},
	{0x0c8, 0x010f0f08}, {0x0cc, 0x00000001}, {0x10c, 0x13ff3be0}, {0x110, 0xaa21aa02},
	{0x11c, 0x0000023f}, {0x134, 0xffffffff}, {0x138, 0xffffffff}, {0x13c, 0xffffffff},
	{0x140, 0xffffffff}, {0x148, 0x0000ffff}, {0x14c, 0x0000ffff}, {0x150, 0x0000ffff},
	{0x154, 0x0000ffff}, {0x15c, 0x00000004}, {0x1a4, 0x00000001}, {0x1a8, 0x00ff0000},
	{0x1ac, 0x00400040}, {0x1b0, 0x000000ff}, {0x1b4, 0x00000024}, {0x1b8, 0x00000006},
	{0x1c8, 0xffffffff}, {0x1d0, 0x00290000}, {0x1f4, 0x03000104}, {0x200, 0x80000000},
	{0x2a0, 0x00000b00}, {0x2a8, 0x39003900}, {0x2b8, 0x3e2e0600}, {0x2bc, 0x0000f000},
	{0x2c8, 0x00000004}, {0x300, 0x0000ffff}, {0x310, 0x00008421}, {0x314, 0x0013ffff},
	{0x318, 0x0002a300}, {0x31c, 0x00000141}, {0x320, 0x10ffffff}, {0x324, 0x0000b81f},
	{0x32c, 0x0000000d}, {0x330, 0x33533000}, {0x334, 0xffffffff}, {0x338, 0xffffffff},
	{0x33c, 0xffffffff},
}

// InitDSIVideo programs the DSI controller for the mode's RGB888 video
// timing and lane count and starts the video engine. Display power, the
// DSI PHY/PLL, and the dispcc byte/pixel clocks must already be running.
func InitDSIVideo(m *DisplayMode) {
	b := uint32(DSI_CTRL_BASE)

	// soft reset with interface clocks forced on, then replay the
	// full live configuration and enable
	reg.Write(b+dsiClkCtrl, dsiClkCtrlAllOn)
	reg.Write(b+dsiReset, 1)
	spinDelay(60000) // ~20 ms
	reg.Write(b+dsiReset, 0)

	replay(b, m.dsiCtrl)

	// clock lane + data lanes, enable, then video mode
	ctrl := 0x100 | m.dsiLaneMask() | 1
	reg.Write(b+dsiCTRL, ctrl)
	reg.Write(b+dsiCTRL, ctrl|2)

	// clock-lane HS force LAST (Linux dsi_op_mode_config order):
	// forcing it before the controller enable starts the clock lane
	// dirty and bit-slips every packet (live checksum errors on the
	// bridge until this ordering was found)
	reg.Write(b+dsiLaneCtrl, 0x11000000)
}

// EnableTPG turns on the controller's video test pattern generator
// (no DPU needed): the checkered-rectangle pattern free-runs at the
// video timing.
func EnableTPG() {
	b := uint32(DSI_CTRL_BASE)

	reg.Write(b+dsiTPGVideoInitVal, 0xff)
	reg.Write(b+dsiTPGMainControl, 0x100) // checkered rectangles
	reg.Write(b+dsiTPGVideoConfig, 0x5)   // 24bpp RGB
	reg.Write(b+dsiTPGCtrl, 0x30)         // pattern sel 3 (general)
	reg.Write(b+dsiTPGCtrl, 0x31)         // + enable
}

// DisableTPG stops the test pattern generator: pixel data then comes
// from the MDP interface (DPU).
func DisableTPG() {
	reg.Write(DSI_CTRL_BASE+dsiTPGCtrl, 0)
}

// DSIStatus returns engine status, interrupt status, and clock status
// for diagnostics.
func DSIStatus() (status0, intr, clkStatus uint32) {
	status0 = reg.Read(DSI_CTRL_BASE + dsiSTATUS0)
	intr = reg.Read(DSI_CTRL_BASE + dsiIntrCtrl)
	clkStatus = reg.Read(DSI_CTRL_BASE + dsiClkStatus)

	return
}
