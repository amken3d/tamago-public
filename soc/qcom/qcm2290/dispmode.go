// Qualcomm QCM2290 display modes
// https://github.com/usbarmory/tamago
//
// Copyright (c) the qcm2290 package authors
//
// Use of this source code is governed by the license
// that can be found in the LICENSE file.

package qcm2290

// DisplayMode is one complete display pipeline configuration: the DSI
// controller, PHY and PLL, the dispcc byte clock divider, and the DPU
// scanout path (VIG0 -> LM0 -> PP0 -> INTF1), each replayed from a live
// Linux register snapshot of that mode.
type DisplayMode struct {
	Name string

	// Width and Height are the active pixels; Stride is the framebuffer
	// line pitch in bytes the VIG0 fetch is configured for.
	Width, Height, Stride int

	// Lanes is the number of DSI data lanes in use (lane 0 upwards).
	Lanes int

	// byte0IntfDiv is the dispcc byte0_intf divider register value
	// (0 = /1, 1 = /2).
	byte0IntfDiv uint32

	dsiCtrl, phyCMN, phyLane, phyPLL                   []regVal
	dpuTop, dpuCTL0, dpuVIG0, dpuLM0, dpuPP0, dpuINTF1 []regVal
}

// Mode1080p60 drives the ANX7625 DSI-to-DP bridge (USB-C DisplayPort):
// 1920x1080 at 60 Hz, 4 lanes at 891 Mbps.
var Mode1080p60 = &DisplayMode{
	Name:   "1080p60 (ANX7625 USB-C DP)",
	Width:  1920,
	Height: 1080,
	Stride: 1920 * 4,
	Lanes:  4,

	byte0IntfDiv: 1,

	dsiCtrl: dsiCtrlSnap,
	phyCMN:  phyCMNSnap,
	phyLane: phyLaneSnap,
	phyPLL:  phyPLLSnap,

	dpuTop:   dpuTopSnap,
	dpuCTL0:  dpuCTL0Snap,
	dpuVIG0:  dpuVIG0Snap,
	dpuLM0:   dpuLM0Snap,
	dpuPP0:   dpuPP0Snap,
	dpuINTF1: dpuINTF1Snap,
}

// ModePanel800x480 drives a Raspberry Pi style 800x480 DSI panel with a
// self-configuring bridge (Waveshare 4.3"/5" class) on the UNO Media
// Carrier DISPLAY connector: 1 lane at 666.6 Mbps, 27.777 MHz pixel
// clock, 800x480 in a 911x510 total. Its power and backlight are
// switched by the panel's attiny over CCI, not by the DSI link.
//
// Defined as the 1080p tables plus every register that differs in the
// live Linux snapshot of this panel (panel-simple
// "waveshare,4-3-inch-dsi", 2026-10-15), so the two modes stay
// comparable line by line. Status registers (DSI FIFO/lane status,
// INTF frame and line counters) are left out of the patch.
var ModePanel800x480 = &DisplayMode{
	Name:   "800x480 (Media Carrier DSI panel)",
	Width:  800,
	Height: 480,
	Stride: 3328, // 800 * 4 rounded up to 256 bytes, as Linux allocates
	Lanes:  1,

	byte0IntfDiv: 0,

	// sync-event traffic, 800x480 active in 911x510 total, 2-pixel
	// HSYNC / 2-line VSYNC, clock-lane timing for 666 Mbps
	dsiCtrl: patched(dsiCtrlSnap,
		regVal{0x010, 0x00009130}, regVal{0x024, 0x034f002f}, regVal{0x028, 0x01f80018}, regVal{0x02c, 0x01fe0389},
		regVal{0x030, 0x00020000}, regVal{0x038, 0x00020000}, regVal{0x0c4, 0x00000c27},
	),

	// PHY clock divider for the lower bit clock (Linux also shows
	// PLL_CNTRL 0x048 = 1, which EnableDSIPHY writes last itself)
	phyCMN: patched(phyCMNSnap,
		regVal{0x010, 0xc2},
	),

	// per-lane D-PHY timings (HS prepare/zero/trail etc.) at 666 Mbps
	phyLane: patched(phyLaneSnap,
		regVal{0x018, 0x21}, regVal{0x01c, 0x1d}, regVal{0x020, 0x06}, regVal{0x024, 0x07},
		regVal{0x028, 0x04}, regVal{0x098, 0x21}, regVal{0x09c, 0x1d}, regVal{0x0a0, 0x06},
		regVal{0x0a4, 0x07}, regVal{0x0a8, 0x04}, regVal{0x118, 0x21}, regVal{0x11c, 0x1d},
		regVal{0x120, 0x06}, regVal{0x124, 0x07}, regVal{0x128, 0x04}, regVal{0x198, 0x21},
		regVal{0x19c, 0x1d}, regVal{0x1a0, 0x06}, regVal{0x1a4, 0x07}, regVal{0x1a8, 0x04},
		regVal{0x218, 0x21}, regVal{0x21c, 0x14}, regVal{0x220, 0x06}, regVal{0x224, 0x07},
		regVal{0x228, 0x04},
	),

	// VCO 1.333 GHz dividers and SSC step, bank 1 SSC mirrored from
	// bank 0 as in the 1080p table
	phyPLL: patched(phyPLLSnap,
		regVal{0x074, 0x9a}, regVal{0x078, 0x02}, regVal{0x07c, 0xf1}, regVal{0x080, 0x06},
		regVal{0x090, 0x45}, regVal{0x0a8, 0x31}, regVal{0x0ac, 0xb1}, regVal{0x0bc, 0x07},
		regVal{0x0d0, 0x0b}, regVal{0x2a8, 0x31}, regVal{0x2ac, 0xb1},
	),

	dpuTop: patched(dpuTopSnap,
		regVal{0x2b0, 0x00000000}, regVal{0x300, 0x00000000}, regVal{0x304, 0x00000000}, regVal{0x308, 0x00000000},
	),

	dpuCTL0: dpuCTL0Snap,

	// 800x480 source/destination, 3328-byte stride, QoS for the size
	dpuVIG0: patched(dpuVIG0Snap,
		regVal{0x000, 0x01e00320}, regVal{0x00c, 0x01e00320}, regVal{0x024, 0x00000d00}, regVal{0x0d0, 0x00001180},
		regVal{0x0e0, 0x000009cf}, regVal{0x0f8, 0x000c000c}, regVal{0x108, 0x01e00320}, regVal{0x118, 0x01e00320},
		regVal{0x128, 0x01e00320},
	),

	dpuLM0: patched(dpuLM0Snap,
		regVal{0x004, 0x01e00320},
	),

	dpuPP0: dpuPP0Snap,

	// 911x510 timing
	dpuINTF1: patched(dpuINTF1Snap,
		regVal{0x008, 0x038a0002}, regVal{0x00c, 0x00071076}, regVal{0x014, 0x00000714}, regVal{0x01c, 0x000054f0},
		regVal{0x024, 0x0006f7af}, regVal{0x03c, 0x034e002f}, regVal{0x064, 0x034e002f},
	),
}

// patched returns a copy of a snapshot with the given registers
// replaced, and appended when the base does not program them.
func patched(base []regVal, changes ...regVal) []regVal {
	out := append([]regVal(nil), base...)

next:
	for _, c := range changes {
		for i := range out {
			if out[i].off == c.off {
				out[i].val = c.val
				continue next
			}
		}

		out = append(out, c)
	}

	return out
}

// dsiLaneMask returns the CTRL data-lane enable bits for the mode.
func (m *DisplayMode) dsiLaneMask() uint32 {
	return uint32((1<<m.Lanes)-1) << 4
}
