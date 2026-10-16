// Qualcomm DSI 6G v2.4 controller driver (command DMA transmit)
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

// DSI command DMA registers, physical offsets (dsi.xml + the 6G +4
// shift, as for every other register in dsictrl.go)
const (
	dsiDMABase = 0x048
	dsiDMALen  = 0x04c
	dsiTrigDMA = 0x090

	// CTRL
	dsiCtrlEnable    = 1 << 0
	dsiCtrlCmdModeEn = 1 << 2

	// CMD_DMA_CTRL bit: transmit in LP escape mode
	dsiCmdDMALowPower = 26

	// INTR_CTRL (status bits are write-1-to-clear, mask bits enable)
	dsiIntrCmdDMADone     = 1 << 0
	dsiIntrMaskCmdDMADone = 1 << 1

	// STATUS0
	dsiStatusCmdDMABusy = 1 << 1

	// dsiCmdPollSpins bounds the DMA-done wait at ~200 ms of MMIO
	// reads (~0.3 us each), Linux's dsi_cmd_dma_tx timeout: an LP
	// escape-mode byte takes ~2 us at a 19.2 MHz escape clock, so even
	// a maximal long packet completes in a few ms.
	dsiCmdPollSpins = 600000
)

// MIPI DSI data types (MIPI DSI v1.3 table 16)
const (
	DSIGenericShortWrite0 = 0x03
	DSIGenericShortWrite1 = 0x13
	DSIGenericShortWrite2 = 0x23
	DSIDCSShortWrite0     = 0x05
	DSIDCSShortWrite1     = 0x15
	DSIGenericLongWrite   = 0x29
	DSIDCSLongWrite       = 0x39
)

// DSIPacket encodes one DSI packet in the MSM command DMA buffer
// format (dsi_cmd_dma_add): the header bytes reordered as
// {h1, h2, h0, flags} with flags bit 7 = last packet and bit 6 = long
// packet, the payload of a long packet after it, and 0xff padding to
// a 32-bit boundary. The hardware computes ECC and checksum.
func DSIPacket(vc int, dt byte, payload []byte) []byte {
	h0 := dt | byte(vc&3)<<6
	long := dt == DSIGenericLongWrite || dt == DSIDCSLongWrite

	var h1, h2 byte
	flags := byte(1 << 7)

	if long {
		h1 = byte(len(payload))
		h2 = byte(len(payload) >> 8)
		flags |= 1 << 6
	} else {
		if len(payload) > 0 {
			h1 = payload[0]
		}

		if len(payload) > 1 {
			h2 = payload[1]
		}
	}

	pkt := []byte{h1, h2, h0, flags}

	if long {
		pkt = append(pkt, payload...)
	}

	for len(pkt)%4 != 0 {
		pkt = append(pkt, 0xff)
	}

	return pkt
}

// DCSPacket encodes a DCS command with its parameters, picking the
// short (0/1 parameter) or long data type as mipi_dsi_dcs_write does.
func DCSPacket(cmd byte, params ...byte) []byte {
	payload := append([]byte{cmd}, params...)

	switch len(params) {
	case 0:
		return DSIPacket(0, DSIDCSShortWrite0, payload)
	case 1:
		return DSIPacket(0, DSIDCSShortWrite1, payload)
	default:
		return DSIPacket(0, DSIDCSLongWrite, payload)
	}
}

// GenericPacket encodes a generic write, short for up to two bytes
// and long otherwise, as mipi_dsi_generic_write does.
func GenericPacket(data ...byte) []byte {
	switch len(data) {
	case 0:
		return DSIPacket(0, DSIGenericShortWrite0, nil)
	case 1:
		return DSIPacket(0, DSIGenericShortWrite1, data)
	case 2:
		return DSIPacket(0, DSIGenericShortWrite2, data)
	default:
		return DSIPacket(0, DSIGenericLongWrite, data)
	}
}

// InitDSICommand prepares the controller for command transmission
// before video starts (a panel's init sequence): soft reset, replay of
// the lane/timing configuration, then enable with command mode and
// all lanes, leaving the clock lane in LP (no HS force) for LP
// escape-mode commands. Display power, the PHY/PLL and the dispcc
// byte/pixel clocks must be running.
func InitDSICommand(m *DisplayMode) {
	b := uint32(DSI_CTRL_BASE)

	reg.Write(b+dsiClkCtrl, dsiClkCtrlAllOn)
	reg.Write(b+dsiReset, 1)
	spinDelay(60000) // ~20 ms
	reg.Write(b+dsiReset, 0)

	replay(b, m.dsiCtrl)

	// clock lane + the mode's data lanes, command mode, enable
	reg.Write(b+dsiCTRL, 0x100|m.dsiLaneMask()|dsiCtrlCmdModeEn|dsiCtrlEnable)
}

// DSICmdTx transmits the command packets held in a DMA buffer at
// physical address dma (len bytes, as built by DSIPacket, already
// cleaned from the data cache by the caller) and waits for the DMA
// done status. With lp set the packets go out in LP escape mode,
// otherwise in HS. Command mode is enabled for the transfer and the
// previous CTRL value restored afterwards, as msm_dsi_host_xfer_prepare
// and _restore do.
//
// The buffer is fetched through the MDSS SMMU stream: SMMUMapMDSS must
// have run, or the fetch faults.
func DSICmdTx(dma uint32, n int, lp bool) error {
	b := uint32(DSI_CTRL_BASE)

	if n <= 0 || n%4 != 0 {
		return errors.New("DMA length must be a positive multiple of 4")
	}

	ctrl := reg.Read(b + dsiCTRL)
	reg.Write(b+dsiCTRL, ctrl|dsiCtrlCmdModeEn|dsiCtrlEnable)

	defer reg.Write(b+dsiCTRL, ctrl)

	if lp {
		reg.Set(b+dsiCmdDMACtrl, dsiCmdDMALowPower)
	} else {
		reg.Clear(b+dsiCmdDMACtrl, dsiCmdDMALowPower)
	}

	// clear a stale done status, unmask done (the status bit latches
	// either way; Linux unmasks it for the transfer)
	intr := reg.Read(b + dsiIntrCtrl)
	reg.Write(b+dsiIntrCtrl, intr|dsiIntrCmdDMADone|dsiIntrMaskCmdDMADone)

	reg.Write(b+dsiDMABase, dma)
	reg.Write(b+dsiDMALen, uint32(n))
	reg.Write(b+dsiTrigDMA, 1)

	for i := 0; i < dsiCmdPollSpins; i++ {
		if reg.Read(b+dsiIntrCtrl)&dsiIntrCmdDMADone != 0 {
			reg.Write(b+dsiIntrCtrl, reg.Read(b+dsiIntrCtrl)|dsiIntrCmdDMADone)
			return nil
		}
	}

	if reg.Read(b+dsiSTATUS0)&dsiStatusCmdDMABusy != 0 {
		return errors.New("command DMA timeout (engine still busy)")
	}

	return errors.New("command DMA timeout (engine idle, no done status)")
}
