// Qualcomm QCM2290 DSI controller: video mode, test pattern, command DMA
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

// Written from bmx's docs/display-spec.md §7; see
// docs/display-provenance.md. This controller version places every
// register 4 bytes above the older documented layout; the offsets here
// are physical.

// DSI0_BASE is the SoC's only DSI controller (§1).
const DSI0_BASE = 0x05e94000

// DSI controller registers (§7.1)
const (
	dsiHWVersion     = 0x000
	dsiCtrl          = 0x004
	dsiStatus0       = 0x008
	dsiFIFOStatus    = 0x00c
	dsiCmdDMACtrl    = 0x03c
	dsiDMABase       = 0x048
	dsiDMALen        = 0x04c
	dsiTrigCtrl      = 0x084
	dsiTrigDMA       = 0x090
	dsiLaneStatus    = 0x0a8
	dsiLaneCtrl      = 0x0ac
	dsiTimeoutStatus = 0x0c0
	dsiIntrCtrl      = 0x110
	dsiReset         = 0x118
	dsiClkCtrl       = 0x11c
	dsiClkStatus     = 0x120
	dsiPHYReset      = 0x12c
	dsiTPGCtrl       = 0x15c
	dsiTPGInitVal    = 0x164
	dsiTPGMainCtrl   = 0x19c
	dsiTPGVideoCfg   = 0x1a4

	dsiCtrlEnable    = 1 << 0
	dsiCtrlVideo     = 1 << 1
	dsiCtrlCmd       = 1 << 2
	dsiCtrlLane0     = 1 << 4
	dsiCtrlClockLane = 1 << 8

	dsiStatusCmdBusy   = 1 << 1
	dsiStatusVideoBusy = 1 << 3

	dsiDMACtrlLP = 1 << 26

	dsiIntrDMADone     = 1 << 0
	dsiIntrDMADoneMask = 1 << 1
	dsiIntrError       = 1 << 24

	dsiClkForceOn    = 0x23f
	dsiTrigSWDMA     = 0x80001004
	dsiLaneForceHSCk = 0x11000000

	dsiCmdTimeout = 200 * time.Millisecond
)

// DSI health bits (§9.4, §11)
const (
	DSIStatusVideoBusy  = dsiStatusVideoBusy
	DSIFIFOVideoUnder   = 1 << 3
	DSIFIFOLane0HSUnder = 1 << 19
	DSIIntrError        = dsiIntrError
	DSITimeoutHS        = 1 << 0
)

// StartDSIVideo resets the DSI controller, programs it for the mode and
// starts video mode (§7.2). The DSI PHY and link clocks must be running.
// The clock lane is forced to HS last: forcing it before the enable
// starts the clock lane in a bad state.
func StartDSIVideo(m *DisplayMode) {
	reg.Write(DSI0_BASE+dsiClkCtrl, dsiClkForceOn)
	reg.Write(DSI0_BASE+dsiReset, 1)
	time.Sleep(20 * time.Millisecond)
	reg.Write(DSI0_BASE+dsiReset, 0)

	writeTable(DSI0_BASE, m.DSI)

	ctrl := m.ctrlLanes() | dsiCtrlEnable
	reg.Write(DSI0_BASE+dsiCtrl, ctrl)
	reg.Write(DSI0_BASE+dsiCtrl, ctrl|dsiCtrlVideo)

	reg.Write(DSI0_BASE+dsiLaneCtrl, dsiLaneForceHSCk)
}

// StopDSI disables the controller (CTRL = 0) and releases the clock
// lane force.
func StopDSI() {
	reg.Write(DSI0_BASE+dsiLaneCtrl, 0)
	reg.Write(DSI0_BASE+dsiCtrl, 0)
}

// DSITestPattern turns the controller's own checkered test pattern on
// or off (§7.2); it replaces the DPU as the pixel source.
func DSITestPattern(on bool) {
	if !on {
		reg.Write(DSI0_BASE+dsiTPGCtrl, 0)
		return
	}

	reg.Write(DSI0_BASE+dsiTPGInitVal, 0xff)
	reg.Write(DSI0_BASE+dsiTPGMainCtrl, 0x100)
	reg.Write(DSI0_BASE+dsiTPGVideoCfg, 0x5)
	reg.Write(DSI0_BASE+dsiTPGCtrl, 0x30)
	reg.Write(DSI0_BASE+dsiTPGCtrl, 0x31)
}

// SendDSICommand transmits the command packets in a memory buffer by
// DMA (§7.3), in LP (escape mode) or HS. The buffer address is fetched
// through the SMMU (SMMUMapMDSSIdentity first) and its length must be a
// multiple of 4.
func SendDSICommand(addr uint32, length int, lp bool) error {
	if length <= 0 || length%4 != 0 {
		return errors.New("command buffer length must be a positive multiple of 4")
	}

	ctrl := reg.Read(DSI0_BASE + dsiCtrl)
	defer reg.Write(DSI0_BASE+dsiCtrl, ctrl)

	reg.Write(DSI0_BASE+dsiCtrl, ctrl|dsiCtrlEnable|dsiCtrlCmd)

	dma := reg.Read(DSI0_BASE + dsiCmdDMACtrl)

	if lp {
		dma |= dsiDMACtrlLP
	} else {
		dma &^= dsiDMACtrlLP
	}

	reg.Write(DSI0_BASE+dsiCmdDMACtrl, dma)
	reg.Write(DSI0_BASE+dsiTrigCtrl, dsiTrigSWDMA)

	// clear and unmask command DMA done
	reg.Write(DSI0_BASE+dsiIntrCtrl, reg.Read(DSI0_BASE+dsiIntrCtrl)|dsiIntrDMADone|dsiIntrDMADoneMask)

	reg.Write(DSI0_BASE+dsiDMABase, addr)
	reg.Write(DSI0_BASE+dsiDMALen, uint32(length))
	reg.Write(DSI0_BASE+dsiTrigDMA, 1)

	if !pollTime(DSI0_BASE+dsiIntrCtrl, dsiIntrDMADone, dsiIntrDMADone, dsiCmdTimeout) {
		return fmt.Errorf("command DMA timeout (status %#x intr %#x)",
			reg.Read(DSI0_BASE+dsiStatus0), reg.Read(DSI0_BASE+dsiIntrCtrl))
	}

	reg.Write(DSI0_BASE+dsiIntrCtrl, reg.Read(DSI0_BASE+dsiIntrCtrl)|dsiIntrDMADone)

	return nil
}

// DSIStatus returns STATUS0 (bit 3 video engine busy), INTR_CTRL (bit
// 24 error) and FIFO_STATUS (bits 3 and 19 underflow).
func DSIStatus() (status0, intr, fifo uint32) {
	return reg.Read(DSI0_BASE + dsiStatus0), reg.Read(DSI0_BASE + dsiIntrCtrl), reg.Read(DSI0_BASE + dsiFIFOStatus)
}

// DSIHealth is a DSI controller snapshot for diagnostics (§9.4, §11).
type DSIHealth struct {
	Version, Ctrl       uint32
	Status0, FIFO, Intr uint32
	Timeout             uint32
	ClkStatus           uint32
	// LaneStatus is instantaneous: sampled several times
	LaneStatus [4]uint32
}

// DSIHealthStatus samples the controller's state without clearing
// anything.
func DSIHealthStatus() (h DSIHealth) {
	h.Version = reg.Read(DSI0_BASE + dsiHWVersion)
	h.Ctrl = reg.Read(DSI0_BASE + dsiCtrl)
	h.Status0, h.Intr, h.FIFO = DSIStatus()
	h.Timeout = reg.Read(DSI0_BASE + dsiTimeoutStatus)
	h.ClkStatus = reg.Read(DSI0_BASE + dsiClkStatus)

	for i := range h.LaneStatus {
		h.LaneStatus[i] = reg.Read(DSI0_BASE + dsiLaneStatus)
		time.Sleep(time.Duration(3+4*i) * time.Millisecond)
	}

	return
}

// ClearDSIStatus clears the latched FIFO, timeout and interrupt status
// (write 1 to clear; INTR_CTRL's mask bits are written back unchanged),
// so that a later DSIHealthStatus shows only new events (the error bit
// was seen latched, not recurring, after a switch back from USB-C).
func ClearDSIStatus() {
	reg.Write(DSI0_BASE+dsiFIFOStatus, reg.Read(DSI0_BASE+dsiFIFOStatus))
	reg.Write(DSI0_BASE+dsiTimeoutStatus, reg.Read(DSI0_BASE+dsiTimeoutStatus))
	reg.Write(DSI0_BASE+dsiIntrCtrl, reg.Read(DSI0_BASE+dsiIntrCtrl))
}
