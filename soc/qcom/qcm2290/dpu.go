// Qualcomm QCM2290 display processing unit (DPU): framebuffer scanout
// https://github.com/usbarmory/tamago
//
// Copyright (c) the qcm2290 package authors
//
// Use of this source code is governed by the license
// that can be found in the LICENSE file.

package qcm2290

import (
	"time"

	"github.com/usbarmory/tamago/internal/reg"
)

// One fixed video-mode pipeline: source pipe VIG0 fetches an XRGB8888
// framebuffer, layer mixer LM0 stages it, pingpong PP0 passes it on and
// INTF1's timing generator feeds the DSI controller.
//
// Written from bmx's docs/display-spec.md §9; see
// docs/display-provenance.md.

// DPU and VBIF bases (§1, §9.1)
const (
	MDSS_BASE = 0x05e00000
	DPU_BASE  = 0x05e01000
	VBIF_BASE = 0x05eb0000

	dpuTop   = DPU_BASE + 0x0
	dpuCTL0  = DPU_BASE + 0x1000
	dpuVIG0  = DPU_BASE + 0x4000
	dpuLM0   = DPU_BASE + 0x44000
	dpuINTF1 = DPU_BASE + 0x6a800
	dpuPP0   = DPU_BASE + 0x70000
)

// DPU registers (§9.1)
const (
	topIntrStatus = 0x014
	topIntrClear  = 0x018

	ctlTop        = 0x014
	ctlFlush      = 0x018
	ctlIntfFlush  = 0x110
	ctlIntfActive = 0x0f4

	vigSrc0Addr = 0x014
	vigSrcAddr2 = 0x0a4

	intfTimingEngineEn = 0x000
	intfUnderflowColor = 0x048
	intfPanelFormat    = 0x090
	intfLineCountEn    = 0x0a8
	intfLineCount      = 0x0b0

	// RGB888, 8 bits per component: the low bits are write-only and a
	// running display reads back 0x2100 (§9.1)
	intfPanelFormatRGB888 = 0x213f

	// INTF1 selected in CTL0
	ctlIntf1 = 0x2

	// VIG0, LM0, CTL, INTF (§9.3); without INTF for later updates
	ctlFlushStart  = 0x80020041
	ctlFlushUpdate = 0x00020041
)

// DPU health bits (§9.1, §11)
const (
	DPUIntrIntf1Vsync    = 1 << 27
	DPUIntrIntf1Underrun = 1 << 26
)

// dpuZeroWrites are the writes a register snapshot cannot capture:
// zeros over non-zero reset values and registers that do not read back
// (§9.2 step 5).
var dpuZeroWrites = []RegVal{
	// TOP: spare, split display off
	{dpuTop + 0x028, 0}, {dpuTop + 0x2f4, 0}, {dpuTop + 0x2f8, 0}, {dpuTop + 0x3f0, 0},
	// CTL0: CTL_TOP 0 in video mode, layer extensions, merge, DSC,
	// writeback, concurrent writeback inactive
	{dpuCTL0 + ctlTop, 0}, {dpuCTL0 + 0x040, 0}, {dpuCTL0 + 0x070, 0}, {dpuCTL0 + 0x0a0, 0},
	{dpuCTL0 + 0x0e4, 0}, {dpuCTL0 + 0x0e8, 0}, {dpuCTL0 + 0x0ec, 0}, {dpuCTL0 + 0x0f0, 0},
	{dpuCTL0 + 0x0fc, 0},
	// INTF1: active window, border colour, skew, polarity unused
	{dpuINTF1 + 0x02c, 0}, {dpuINTF1 + 0x034, 0}, {dpuINTF1 + 0x040, 0}, {dpuINTF1 + 0x044, 0},
	{dpuINTF1 + 0x04c, 0}, {dpuINTF1 + 0x050, 0}, {dpuINTF1 + 0x068, 0},
	{dpuINTF1 + intfLineCountEn, 3},
	// PP0: dither off
	{dpuPP0 + 0x30e0, 0},
}

// InitDPU configures the scanout pipeline for the mode with the
// framebuffer at device address fb (§9.2), without starting it.
// EnableDPUClock and SMMUMapMDSSIdentity must have been called.
func InitDPU(m *DisplayMode, fb uint32) {
	// 1. real-time read QoS for the display fetch, and the DPU's
	// priority on the system NoC (usb-spec §7.1)
	writeTable(VBIF_BASE, m.VBIF)
	SetSNOCQoS(SNOCPortMDP0, SNOCQoSDisplay)

	// 2. TOP, CTL0, VIG0
	writeTable(dpuTop, m.Top)
	writeTable(dpuCTL0, m.CTL)
	writeTable(dpuVIG0, m.VIG)

	// 3. framebuffer address, both address registers
	traceWrite(dpuVIG0+vigSrc0Addr, fb)
	traceWrite(dpuVIG0+vigSrcAddr2, fb)

	// 4. LM0, PP0, INTF1
	writeTable(dpuLM0, m.LM)
	writeTable(dpuPP0, m.PP)
	writeTable(dpuINTF1, m.INTF)

	// the panel format must be written in full whatever a table holds
	traceWrite(dpuINTF1+intfPanelFormat, intfPanelFormatRGB888)

	// 5. what a snapshot cannot capture
	for _, w := range dpuZeroWrites {
		traceWrite(w.Off, w.Val)
	}
}

// StartDPU flushes the configuration and starts INTF1's timing engine
// (§9.3). Video mode has no CTL_START.
func StartDPU() {
	traceWrite(dpuCTL0+ctlIntfFlush, ctlIntf1)
	traceWrite(dpuCTL0+ctlFlush, ctlFlushStart)
	traceWrite(dpuINTF1+intfTimingEngineEn, 1)
}

// StopDPU stops INTF1's timing engine and waits, up to about 100 ms,
// for its line counter to settle; it reports whether it did. The
// specification gives no stop sequence: the disable is flushed like the
// start (CTL_INTF_FLUSH, CTL_FLUSH INTF bit), because a USB-C mode
// engine was seen to keep running after the bare disable write.
func StopDPU() bool {
	traceWrite(dpuINTF1+intfTimingEngineEn, 0)
	traceWrite(dpuCTL0+ctlIntfFlush, ctlIntf1)
	traceWrite(dpuCTL0+ctlFlush, 1<<31)

	last := reg.Read(dpuINTF1 + intfLineCount)

	for i := 0; i < 20; i++ {
		time.Sleep(5 * time.Millisecond)

		l := reg.Read(dpuINTF1 + intfLineCount)

		if l == last && i > 0 {
			return true
		}

		last = l
	}

	return false
}

// SetDPUFramebuffer points VIG0 at another framebuffer and flushes the
// update (without the INTF bit, §9.3).
func SetDPUFramebuffer(fb uint32) {
	traceWrite(dpuVIG0+vigSrc0Addr, fb)
	traceWrite(dpuVIG0+vigSrcAddr2, fb)
	traceWrite(dpuCTL0+ctlFlush, ctlFlushUpdate)
}

// SetDPUUnderflowColor sets the colour INTF1 shows on underrun (packing
// B[7:0], R[15:8], G[23:16]) and flushes it: if the screen changes
// colour, it shows underrun, not pixels (§11).
func SetDPUUnderflowColor(c uint32) {
	traceWrite(dpuINTF1+intfUnderflowColor, c)
	traceWrite(dpuCTL0+ctlIntfFlush, ctlIntf1)
	traceWrite(dpuCTL0+ctlFlush, ctlFlushStart)
}

// DPUStatus returns INTF1's line counter (advancing = scanning) and
// CTL0's flush register (0 once the hardware consumed a flush).
func DPUStatus() (line, flush uint32) {
	return reg.Read(dpuINTF1 + intfLineCount), reg.Read(dpuCTL0 + ctlFlush)
}

// DPUIntrStatus returns TOP INTR_STATUS (bit 27 INTF1 vsync, bit 26
// INTF1 underrun).
func DPUIntrStatus() uint32 {
	return reg.Read(dpuTop + topIntrStatus)
}

// ClearDPUIntr clears the latched TOP interrupt status, so that a later
// DPUIntrStatus shows only new events.
func ClearDPUIntr() {
	traceWrite(dpuTop+topIntrClear, reg.Read(dpuTop+topIntrStatus))
}

// DPUHealth is a DPU snapshot for diagnostics (§9.4).
type DPUHealth struct {
	Intr        uint32
	Flush       uint32
	IntfActive  uint32
	TimingEn    uint32
	PanelFormat uint32
	Lines       [3]uint32
}

// DPUHealthStatus samples the DPU's state; the line counter is sampled
// at irregular intervals (50 ms is exactly three frames, §11).
func DPUHealthStatus() (h DPUHealth) {
	h.Intr = DPUIntrStatus()
	h.Flush = reg.Read(dpuCTL0 + ctlFlush)
	h.IntfActive = reg.Read(dpuCTL0 + ctlIntfActive)
	h.TimingEn = reg.Read(dpuINTF1 + intfTimingEngineEn)
	h.PanelFormat = reg.Read(dpuINTF1 + intfPanelFormat)

	for i := range h.Lines {
		h.Lines[i] = reg.Read(dpuINTF1 + intfLineCount)

		if i < len(h.Lines)-1 {
			time.Sleep(time.Duration(7+6*i) * time.Millisecond)
		}
	}

	return
}
