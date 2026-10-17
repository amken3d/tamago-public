// Qualcomm GENI serial engine UART with hardware flow control (SE2: the
// UNO Q's link to its STM32U585)
// https://github.com/usbarmory/tamago
//
// Copyright (c) the qcm2290 package authors
//
// Use of this source code is governed by the license
// that can be found in the LICENSE file.

package qcm2290

// Clean-room implementation from bmx's docs/mcu-link-spec.md (sections
// cited as §n); see bmx's docs/mcu-link-provenance.md. Unlike the console
// driver (geniuart.go), nothing here relies on the boot chain having set
// the engine up: the boot chain loads the engine's UART firmware and
// programs the clock root's frequency table, and everything else is done
// here.

import (
	"errors"
	"fmt"
	"sync"
	"time"

	"github.com/usbarmory/tamago/internal/reg"
)

// QUPv3 wrapper 0 and serial engine 2 (§1)
const (
	SE2_BASE = 0x04a88000

	// SE2 interrupt: SPI 0x149, GIC INTID 0x169, level high
	SE2_IRQ = 0x169

	QUPV3_WRAP0_BASE   = 0x04ac0000
	QUPV3_HW_VER       = 0x4
	QUPV3_HW_VER_OSR16 = 0x20050000 // from here on, 16x oversampling (§4.3)
)

// Serial engine registers added for SE2 (§4.1), offsets from the SE base
const (
	GENI_FORCE_DEFAULT_REG       = 0x020
	GENI_OUTPUT_CTRL             = 0x024
	SE_GENI_CGC_CTRL             = 0x028
	GENI_SER_S_CLK_CFG           = 0x04c
	SE_GENI_CLK_SEL              = 0x07c
	SE_UART_LOOPBACK_CFG         = 0x22c
	SE_UART_IO_MACRO_CTRL        = 0x240
	SE_UART_RX_STALE_CNT         = 0x294
	SE_UART_MANUAL_RFR           = 0x2ac
	SE_GENI_RX_WATERMARK_REG     = 0x810
	SE_GENI_RX_RFR_WATERMARK_REG = 0x814
	SE_GENI_IOS                  = 0x908
	SE_DMA_TX_IRQ_CLR            = 0xc44
	SE_DMA_RX_IRQ_CLR            = 0xd44
	SE_IRQ_EN                    = 0xe1c
	SE_HW_PARAM_0                = 0xe24
	SE_HW_PARAM_1                = 0xe28
	SE_DMA_GENERAL_CFG           = 0xe30

	GENI_CMD_CANCEL = 1 << 2

	S_CMD_DONE     = 1 << 0
	S_CMD_CANCEL   = 1 << 4 // by analogy with the M sequencer's abort bit 5
	S_CMD_ABORT    = 1 << 5
	S_RX_FIFO_WM   = 1 << 26
	S_RX_FIFO_LAST = 1 << 27

	IOS_CTS = 1 << 1 // CTS input level: 0 = asserted (the MCU is ready)

	HW_PARAM_FIFO_DEPTH_SHFT = 16
	HW_PARAM_FIFO_DEPTH_MASK = 0x3f
)

// GCC clocks of QUPv3 wrapper 0 and SE2 (§2)
const (
	GCC_QUP0_M_AHB_CBCR  = 0x1f004 // vote bit 6
	GCC_QUP0_S_AHB_CBCR  = 0x1f008 // vote bit 7
	GCC_QUP0_CORE_CBCR   = 0x1f00c // vote bit 8
	GCC_QUP0_CORE2X_CBCR = 0x1f014 // vote bit 9
	GCC_QUP0_S2_CBCR     = 0x1f3a4 // vote bit 12

	// SE2 clock root, DFS mode: level registers relative to it
	GCC_QUP0_S2_CMD_RCGR = 0x1f3a8
	dfsCurrentLevel      = 0x14
	dfsLevelCfg          = 0x1c
	dfsLevelM            = 0x5c
	dfsLevelN            = 0x9c
	dfsLevels            = 8

	xoHz = 19200000
)

// MCUUART pins: TLMM gpio6, gpio7, gpio71, gpio80, all function 1 (§1)
var mcuUARTPins = []int{6, 7, 71, 80}

// DFSTable is the set of rates the SE2 clock root is expected to offer
// (§2). A decoded level whose rate is not one of these is not used.
var DFSTable = []uint64{
	7372800, 14745600, 19200000, 29491200, 32000000, 48000000, 64000000,
	75000000, 80000000, 96000000, 100000000, 102400000, 112500000,
	117964800, 120000000, 128000000,
}

// dfsCandidateHz are the rates tried for a level's non-crystal source. The
// specification does not establish the source numbering or the GPLL0
// output rate (§2): a level is decoded under each candidate and accepted
// only if the result lands in DFSTable.
var dfsCandidateHz = []uint64{300000000, 600000000}

// dfsMatchPPM is how close a decoded rate must be to a table entry (the
// table's 117.96 MHz is a rounded 117.9648).
const dfsMatchPPM = 200

// MaxBaudErrorPPM is the largest divider error SetBaud accepts.
const MaxBaudErrorPPM = 10000

// DFSLevel is one decoded performance level of the SE2 clock root.
type DFSLevel struct {
	Index        int
	Cfg, M, NReg uint32 // raw registers

	Src    uint32 // source select (bits 10:8)
	PreDiv uint32 // pre-divider field (2 x divide - 1, 0 = bypass)
	MN     bool   // M/N in use (mode bits 13:12 non-zero)
	N      uint32 // decoded N

	SrcHz uint64 // the source rate assumed in decoding (0 = undecodable)
	Hz    uint64 // decoded rate
	Valid bool   // Hz is a table rate: the level may be used
	Note  string
}

func (l *DFSLevel) rate(src uint64) uint64 {
	r := src

	if l.PreDiv != 0 {
		r = src * 2 / uint64(l.PreDiv+1)
	}

	if l.MN {
		if l.N == 0 {
			return 0
		}

		r = r * uint64(l.M) / uint64(l.N)
	}

	return r
}

func inDFSTable(hz uint64) bool {
	for _, t := range DFSTable {
		d := int64(hz) - int64(t)
		if d < 0 {
			d = -d
		}

		if uint64(d)*1000000 <= t*dfsMatchPPM {
			return true
		}
	}

	return false
}

// DFS enable: bit 0 of the root's current-level register (observed on
// hardware, 2026-10-17: the boot chain leaves it clear for SE2, the
// register reading 0x8020, and the root then runs from its own CFG).
const dfsEnable = 1 << 0

// rootCfgOff is the RCG CFG register, 4 bytes after CMD (display-spec
// §2 conventions).
const rootCfgOff = 0x4

// DFSEnabled reports whether a current-level register value has DFS on.
func DFSEnabled(current uint32) bool { return current&dfsEnable != 0 }

// decodeRootCfg describes the root's single rate when DFS is off, from its
// CFG register. Only the crystal (source 0) with the pre-divider is
// decoded: the source numbering of GPLL0's outputs and the M/N/D register
// layout of this root are not established, so either is refused.
func decodeRootCfg(base uint32) DFSLevel {
	l := DFSLevel{Index: 0, Cfg: reg.Read(base + rootCfgOff)}

	l.Src = (l.Cfg >> 8) & 0x7
	l.PreDiv = l.Cfg & 0x1f
	l.MN = (l.Cfg>>12)&0x3 != 0

	switch {
	case l.Src != 0:
		l.Note = fmt.Sprintf("DFS off; root CFG selects source %d, whose rate is not established (only the 19.2 MHz crystal, source 0, is)", l.Src)
	case l.MN:
		l.Note = "DFS off; root CFG has M/N in use, whose register layout is not established"
	default:
		l.SrcHz = xoHz
		l.Hz = l.rate(xoHz)
		l.Valid = true
		l.Note = "DFS off: the root's own CFG (crystal)"
	}

	return l
}

// DecodeDFS reads and decodes the SE2 clock root's eight performance
// levels. It only reads GCC registers.
func DecodeDFS() (levels []DFSLevel, current uint32) {
	base := uint32(GCC_BASE + GCC_QUP0_S2_CMD_RCGR)

	current = reg.Read(base + dfsCurrentLevel)

	if current&dfsEnable == 0 {
		return []DFSLevel{decodeRootCfg(base)}, current
	}

	for i := 0; i < dfsLevels; i++ {
		l := DFSLevel{
			Index: i,
			Cfg:   reg.Read(base + dfsLevelCfg + uint32(4*i)),
			M:     reg.Read(base+dfsLevelM+uint32(4*i)) & 0xffff,
			NReg:  reg.Read(base + dfsLevelN + uint32(4*i)),
		}

		l.Src = (l.Cfg >> 8) & 0x7
		l.PreDiv = l.Cfg & 0x1f
		l.MN = (l.Cfg>>12)&0x3 != 0

		if l.MN {
			l.N = (^l.NReg&0xffff + l.M) & 0xffff
		}

		levels = append(levels, l)
	}

	// The crystal is source 0. Every other source number gets the
	// candidate rate under which the most of its levels decode to table
	// rates; a source no candidate explains is refused.
	srcHz := map[uint32]uint64{0: xoHz}

	for _, l := range levels {
		if _, done := srcHz[l.Src]; done {
			continue
		}

		best, bestN := uint64(0), 0

		for _, c := range dfsCandidateHz {
			n := 0

			for _, m := range levels {
				if m.Src == l.Src && inDFSTable(m.rate(c)) {
					n++
				}
			}

			if n > bestN {
				best, bestN = c, n
			}
		}

		srcHz[l.Src] = best
	}

	for i := range levels {
		l := &levels[i]
		l.SrcHz = srcHz[l.Src]

		switch {
		case l.SrcHz == 0:
			l.Note = fmt.Sprintf("source %d: no candidate rate explains it", l.Src)
		case l.MN && l.N == 0:
			l.Note = "M/N with N = 0"
		default:
			l.Hz = l.rate(l.SrcHz)
			l.Valid = inDFSTable(l.Hz)

			if !l.Valid {
				l.Note = "not a table rate"
			}
		}
	}

	return
}

// BaudPlan is how a baud rate would be produced.
type BaudPlan struct {
	Baud         int
	Level        int
	LevelHz      uint64
	Divider      uint32
	Oversampling uint32
	ActualBaud   float64
	ErrorPPM     int
}

// oversampling returns the UART oversampling factor for the wrapper
// version (§4.3). The wrapper's AHB clocks must be on.
func oversampling() (ovs uint32, ver uint32) {
	ver = reg.Read(QUPV3_WRAP0_BASE + QUPV3_HW_VER)

	if ver >= QUPV3_HW_VER_OSR16 {
		return 16, ver
	}

	return 32, ver
}

func planBaud(baud int, levels []DFSLevel, ovs uint32) (BaudPlan, error) {
	if baud < 300 || baud > 8000000 {
		return BaudPlan{}, fmt.Errorf("baud %d outside 300..8000000", baud)
	}

	req := uint64(baud) * uint64(ovs)

	best := BaudPlan{Level: -1}
	bestErr := int64(1) << 62

	for _, l := range levels {
		if !l.Valid {
			continue
		}

		div := (l.Hz + req - 1) / req

		if div < 1 {
			div = 1
		}

		if div > 4095 {
			continue
		}

		actual := float64(l.Hz) / float64(div*uint64(ovs))
		ppm := int64((actual - float64(baud)) / float64(baud) * 1e6)
		abs := ppm

		if abs < 0 {
			abs = -abs
		}

		if abs < bestErr {
			bestErr = abs
			best = BaudPlan{
				Baud: baud, Level: l.Index, LevelHz: l.Hz, Divider: uint32(div),
				Oversampling: ovs, ActualBaud: actual, ErrorPPM: int(ppm),
			}
		}
	}

	if best.Level < 0 {
		return best, fmt.Errorf("no usable clock level for %d baud (none of the decoded levels is a verified table rate)", baud)
	}

	if bestErr > MaxBaudErrorPPM {
		err := fmt.Errorf("%d baud: best is level %d (%d Hz) / %d = %.0f baud, %d ppm off (limit %d)",
			baud, best.Level, best.LevelHz, best.Divider, best.ActualBaud, best.ErrorPPM, MaxBaudErrorPPM)

		if len(levels) == 1 && levels[0].SrcHz == xoHz && levels[0].PreDiv == 0 {
			err = fmt.Errorf("%w; the SE2 clock root runs from the 19.2 MHz crystal (DFS off), so the reachable rates are %d / n "+
				"(e.g. %d, %d, %d, %d); faster or other rates need a GPLL0-fed root, not yet supported",
				err, xoHz/ovs, xoHz/ovs, xoHz/ovs/2, xoHz/ovs/3, xoHz/ovs/4)
		}

		return best, err
	}

	return best, nil
}

// MCUUARTDiag is a snapshot of the link UART for bring-up diagnostics.
type MCUUARTDiag struct {
	Initialized bool

	Proto          uint32
	WrapperVersion uint32
	Oversampling   uint32

	Levels       []DFSLevel
	CurrentLevel uint32 // raw DFS current-level register

	Plan BaudPlan // the rate in use

	TXDepth, RXDepth uint32 // FIFO depths, words

	IOS         uint32
	CTSAsserted bool // the MCU is ready to receive

	ClkSel, MClkCfg, SClkCfg uint32
	TxTransCfg, ManualRFR    uint32
	RFRWatermark             uint32
	RXFIFOStatus             uint32
	TXFIFOStatus             uint32
	MIRQ, SIRQ               uint32

	RxBytes, TxBytes uint64
	RingUsed, RingHW int // bytes held now / most ever held
	ReadRestarts     uint32
	CancelNote       string
}

// GENIFlowUART is a GENI serial engine in UART mode with RTS/CTS flow
// control, polled FIFO mode, set up from scratch.
//
// Received bytes are moved from the engine's FIFO into a software ring by
// every Read and Write (and by Poll). When nobody drains, the FIFO fills to
// the RFR watermark, the engine deasserts RTS and the MCU waits: bytes are
// delayed, never lost. When the ring itself is full the FIFO is left alone
// for the same effect.
type GENIFlowUART struct {
	Base uint32

	// SEVoteBit is the serial engine's clock vote bit in
	// GCC_APCS_BRANCH_ENA_VOTE, SECBCR its branch status register.
	SEVoteBit uint32
	SECBCR    uint32

	// Pins are the TLMM pins muxed to function 1.
	Pins []int

	mu    sync.Mutex
	ready bool
	diag  MCUUARTDiag

	txWM uint32

	ring       []byte
	head, tail int // head: next write; tail: next read (both unwrapped)
}

// MCUUART is the UNO Q's host-MCU link (SE2).
var MCUUART = &GENIFlowUART{
	Base:      SE2_BASE,
	SEVoteBit: 12,
	SECBCR:    GCC_QUP0_S2_CBCR,
	Pins:      mcuUARTPins,
}

const ringSize = 64 * 1024

// CBCR bit 1: hardware clock control (§2, "Hardware-gated branches").
const cbcrHWCtl = 1

func hwGated(cbcr uint32) bool { return reg.Get(GCC_BASE+cbcr, cbcrHWCtl) }

// branchOn reports whether a branch can be relied on: voted, and either
// hardware-gated (its off bit then means nothing) or reporting on.
func branchOn(cbcr uint32, vote int) bool {
	if !reg.Get(GCC_BASE+GCC_APCS_BRANCH_ENA_VOTE, vote) {
		return false
	}

	return hwGated(cbcr) || !reg.Get(GCC_BASE+cbcr, CLK_OFF)
}

// enableClocks votes on the wrapper and SE clocks and waits for each
// branch to report on (§2, §4.2 step 1).
func (u *GENIFlowUART) enableClocks() error {
	branches := []struct {
		vote uint32
		cbcr uint32
		name string
	}{
		{6, GCC_QUP0_M_AHB_CBCR, "QUPv3 wrapper 0 master AHB"},
		{7, GCC_QUP0_S_AHB_CBCR, "QUPv3 wrapper 0 slave AHB"},
		{8, GCC_QUP0_CORE_CBCR, "QUPv3 wrapper 0 core"},
		{9, GCC_QUP0_CORE2X_CBCR, "QUPv3 wrapper 0 core 2x"},
		{u.SEVoteBit, u.SECBCR, "serial engine"},
	}

	for _, b := range branches {
		reg.Set(GCC_BASE+GCC_APCS_BRANCH_ENA_VOTE, int(b.vote))
	}

	for _, b := range branches {
		deadline := time.Now().Add(10 * time.Millisecond)

		// a hardware-gated branch (status bit 1) reports bit 31 as the
		// hardware gates it, not as voted: voted is on (§2)
		if hwGated(b.cbcr) {
			continue
		}

		for reg.Get(GCC_BASE+b.cbcr, CLK_OFF) {
			if time.Now().After(deadline) {
				return fmt.Errorf("%s clock (GCC %#x, vote bit %d) still off: CBCR %#08x",
					b.name, b.cbcr, b.vote, reg.Read(GCC_BASE+b.cbcr))
			}
		}
	}

	return nil
}

func (u *GENIFlowUART) muxPins() error {
	for _, n := range u.Pins {
		g, err := NewGPIO(n)
		if err != nil {
			return err
		}

		g.SelectFunction(1)
		g.SetDrive(2)
		g.SetPull(PULL_NONE)
	}

	return nil
}

// stopRx cancels the secondary (receive) sequencer's command (§4.2 step
// 4), falling back to an abort. Neither status may appear when no command
// was running, which is normal; what happened is recorded.
func (u *GENIFlowUART) stopRx() string {
	reg.Write(u.Base+SE_GENI_S_CMD_CTRL_REG, GENI_CMD_CANCEL)

	note := ""

	if !u.waitS(S_CMD_DONE|S_CMD_CANCEL, 5*time.Millisecond) {
		reg.Write(u.Base+SE_GENI_S_CMD_CTRL_REG, GENI_CMD_ABORT)

		if u.waitS(S_CMD_DONE|S_CMD_ABORT, 5*time.Millisecond) {
			note = "receive command aborted (cancel not acknowledged)"
		} else {
			note = "no receive command acknowledged a cancel or abort (none running?)"
		}
	} else {
		note = "receive command cancelled"
	}

	reg.Write(u.Base+SE_GENI_S_IRQ_CLEAR, 0xffffffff)

	return note
}

func (u *GENIFlowUART) waitS(mask uint32, d time.Duration) bool {
	deadline := time.Now().Add(d)

	for reg.Read(u.Base+SE_GENI_S_IRQ_STATUS)&mask == 0 {
		if time.Now().After(deadline) {
			return false
		}
	}

	return true
}

func (u *GENIFlowUART) startRx() {
	reg.Write(u.Base+SE_GENI_S_CMD0, UART_START_READ<<M_OPCODE_SHFT)
}

// PlanBaud reports how a rate would be produced, or why it cannot be,
// without changing anything. Before Init the levels are decoded afresh
// (GCC registers are readable at any time) and the oversampling factor
// is read from the wrapper, whose clocks must then already be on.
func (u *GENIFlowUART) PlanBaud(baud int) (BaudPlan, error) {
	u.mu.Lock()
	ready, levels, ovs := u.ready, u.diag.Levels, u.diag.Oversampling
	u.mu.Unlock()

	if !ready {
		// the wrapper version register needs the wrapper's AHB clock
		if !branchOn(GCC_QUP0_S_AHB_CBCR, 7) {
			return BaudPlan{}, errors.New("QUPv3 wrapper 0 clocks are off (bring the link UART up first)")
		}

		levels, _ = DecodeDFS()
		ovs, _ = oversampling()
	}

	return planBaud(baud, levels, ovs)
}

// Init brings the serial engine up as an 8N1 UART at baud with CTS
// honoured and automatic RTS (§4.2).
func (u *GENIFlowUART) Init(baud int) error {
	u.mu.Lock()
	defer u.mu.Unlock()

	u.ready = false
	u.diag = MCUUARTDiag{}

	// 1. clocks
	if err := u.enableClocks(); err != nil {
		return fmt.Errorf("clocks: %w", err)
	}

	// 2. pins
	if err := u.muxPins(); err != nil {
		return fmt.Errorf("pins: %w", err)
	}

	// 3. firmware protocol (§3)
	u.diag.Proto = (reg.Read(u.Base+GENI_FW_REVISION_RO) >> FW_REV_PROTOCOL_SHFT) & FW_REV_PROTOCOL_MASK

	if u.diag.Proto != GENI_SE_PROTO_UART {
		return fmt.Errorf("serial engine at %#x runs protocol %d firmware, not UART (%d), FW_REVISION %#08x: "+
			"bmx has no serial engine firmware loader; the boot chain must load UART firmware",
			u.Base, u.diag.Proto, GENI_SE_PROTO_UART, reg.Read(u.Base+GENI_FW_REVISION_RO))
	}

	u.diag.Oversampling, u.diag.WrapperVersion = oversampling()
	u.diag.Levels, u.diag.CurrentLevel = DecodeDFS()

	plan, err := planBaud(baud, u.diag.Levels, u.diag.Oversampling)
	if err != nil {
		return err
	}

	// 4. stop any receive in progress
	u.diag.CancelNote = u.stopRx()

	// 5. common engine initialisation
	reg.Write(u.Base+SE_GSI_EVENT_EN, 0)
	reg.Write(u.Base+SE_GENI_M_IRQ_CLEAR, 0xffffffff)
	reg.Write(u.Base+SE_GENI_S_IRQ_CLEAR, 0xffffffff)
	reg.Write(u.Base+SE_DMA_TX_IRQ_CLR, 0xffffffff)
	reg.Write(u.Base+SE_DMA_RX_IRQ_CLR, 0xffffffff)
	reg.Write(u.Base+SE_IRQ_EN, 0xffffffff)

	reg.Write(u.Base+SE_GENI_CGC_CTRL, reg.Read(u.Base+SE_GENI_CGC_CTRL)|0x7f)
	reg.Write(u.Base+SE_DMA_GENERAL_CFG, reg.Read(u.Base+SE_DMA_GENERAL_CFG)|0xf)
	reg.Write(u.Base+GENI_OUTPUT_CTRL, 0x7f)
	reg.Write(u.Base+GENI_FORCE_DEFAULT_REG, 1)

	reg.Write(u.Base+SE_IRQ_EN, reg.Read(u.Base+SE_IRQ_EN)|0xf)
	reg.Clear(u.Base+SE_GENI_DMA_MODE_EN, 0)
	reg.Write(u.Base+SE_GSI_EVENT_EN, 0)

	u.diag.TXDepth = (reg.Read(u.Base+SE_HW_PARAM_0) >> HW_PARAM_FIFO_DEPTH_SHFT) & HW_PARAM_FIFO_DEPTH_MASK
	u.diag.RXDepth = (reg.Read(u.Base+SE_HW_PARAM_1) >> HW_PARAM_FIFO_DEPTH_SHFT) & HW_PARAM_FIFO_DEPTH_MASK

	if u.diag.TXDepth < 4 || u.diag.RXDepth < 4 {
		return fmt.Errorf("implausible FIFO depths: TX %d RX %d words (HW_PARAM_0 %#08x, HW_PARAM_1 %#08x)",
			u.diag.TXDepth, u.diag.RXDepth, reg.Read(u.Base+SE_HW_PARAM_0), reg.Read(u.Base+SE_HW_PARAM_1))
	}

	reg.Write(u.Base+SE_GENI_RX_WATERMARK_REG, 2)
	reg.Write(u.Base+SE_GENI_RX_RFR_WATERMARK_REG, u.diag.RXDepth-2)

	reg.Write(u.Base+SE_GENI_M_IRQ_EN, reg.Read(u.Base+SE_GENI_M_IRQ_EN)|
		0x7e|1<<21|1<<22|1<<23|1<<24|1<<25|1<<28|1<<29)
	reg.Write(u.Base+SE_GENI_S_IRQ_EN, reg.Read(u.Base+SE_GENI_S_IRQ_EN)|
		0x3e|0x3e00|1<<24|1<<25)

	// 6. packing: one byte per FIFO word, LSB first, both directions
	reg.Write(u.Base+SE_GENI_BYTE_GRAN, 0)
	reg.Write(u.Base+SE_GENI_TX_PACKING_CFG0, packing1x8)
	reg.Write(u.Base+SE_GENI_TX_PACKING_CFG1, 0)
	reg.Write(u.Base+SE_GENI_RX_PACKING_CFG0, packing1x8)
	reg.Write(u.Base+SE_GENI_RX_PACKING_CFG1, 0)

	// TX watermark: refill once the FIFO is down to two words
	u.txWM = 2
	reg.Write(u.Base+SE_GENI_TX_WATERMARK, u.txWM)

	// 7. stale count: 10 bits x 16
	reg.Write(u.Base+SE_UART_RX_STALE_CNT, 160)

	// 8. 8N1, CTS honoured, automatic RTS, no loopback, no swaps
	reg.Write(u.Base+SE_UART_TX_WORD_LEN, 8)
	reg.Write(u.Base+SE_UART_RX_WORD_LEN, 8)
	reg.Write(u.Base+SE_UART_TX_STOP_BIT_LEN, 0)
	reg.Write(u.Base+SE_UART_TX_PARITY_CFG, 0)
	reg.Write(u.Base+SE_UART_RX_PARITY_CFG, 0)
	reg.Write(u.Base+SE_UART_TX_TRANS_CFG, 0)
	reg.Write(u.Base+SE_UART_RX_TRANS_CFG, 0)
	reg.Write(u.Base+SE_UART_MANUAL_RFR, 0)
	reg.Write(u.Base+SE_UART_LOOPBACK_CFG, 0)
	reg.Write(u.Base+SE_UART_IO_MACRO_CTRL, 0)

	// 9. rate
	u.applyPlan(plan)

	// 10. receiver on
	if u.ring == nil {
		u.ring = make([]byte, ringSize)
	}

	u.head, u.tail = 0, 0
	u.startRx()

	u.ready = true

	return nil
}

func (u *GENIFlowUART) applyPlan(p BaudPlan) {
	clk := p.Divider<<CLK_DIV_SHFT | SER_CLK_EN

	reg.Write(u.Base+GENI_SER_M_CLK_CFG, clk)
	reg.Write(u.Base+GENI_SER_S_CLK_CFG, clk)
	reg.Write(u.Base+SE_GENI_CLK_SEL, uint32(p.Level)&0x7)

	u.diag.Plan = p
}

// SetBaud changes the rate. The transmitter is idle (Write is not running:
// the lock is held); the receiver is stopped around the change. Bytes
// already received stay in the ring for the caller to discard.
func (u *GENIFlowUART) SetBaud(baud int) error {
	u.mu.Lock()
	defer u.mu.Unlock()

	if !u.ready {
		return errors.New("link UART not initialized")
	}

	plan, err := planBaud(baud, u.diag.Levels, u.diag.Oversampling)
	if err != nil {
		return err
	}

	u.poll()
	u.stopRx()
	u.applyPlan(plan)
	u.startRx()

	return nil
}

// Baud returns the configured rate (0 before Init).
func (u *GENIFlowUART) Baud() int {
	u.mu.Lock()
	defer u.mu.Unlock()

	if !u.ready {
		return 0
	}

	return u.diag.Plan.Baud
}

// poll moves received words into the ring; the caller holds the lock.
func (u *GENIFlowUART) poll() {
	if s := reg.Read(u.Base + SE_GENI_S_IRQ_STATUS); s != 0 {
		// The read command runs forever; if it ever reports done, the
		// receiver has stopped and is restarted.
		if s&S_CMD_DONE != 0 {
			u.diag.ReadRestarts++
			reg.Write(u.Base+SE_GENI_S_IRQ_CLEAR, s)
			u.startRx()
		} else {
			reg.Write(u.Base+SE_GENI_S_IRQ_CLEAR, s&(S_RX_FIFO_WM|S_RX_FIFO_LAST))
		}
	}

	for {
		wc := reg.Read(u.Base+SE_GENI_RX_FIFO_STATUS) & RX_FIFO_WC_MASK

		if wc == 0 {
			return
		}

		for ; wc > 0; wc-- {
			if u.head-u.tail == len(u.ring) {
				// full: leave the rest in the FIFO, RTS holds the MCU
				return
			}

			u.ring[u.head%len(u.ring)] = byte(reg.Read(u.Base + SE_GENI_RX_FIFO))
			u.head++
			u.diag.RxBytes++
		}

		if used := u.head - u.tail; used > u.diag.RingHW {
			u.diag.RingHW = used
		}
	}
}

// Poll drains the receive FIFO into the ring.
func (u *GENIFlowUART) Poll() {
	u.mu.Lock()
	defer u.mu.Unlock()

	if u.ready {
		u.poll()
	}
}

// Read copies received bytes into buf, without blocking.
func (u *GENIFlowUART) Read(buf []byte) (n int, err error) {
	u.mu.Lock()
	defer u.mu.Unlock()

	if !u.ready {
		return 0, errors.New("link UART not initialized")
	}

	u.poll()

	for n < len(buf) && u.tail != u.head {
		buf[n] = u.ring[u.tail%len(u.ring)]
		u.tail++
		n++
	}

	return n, nil
}

// txTimeout bounds one transmit command: the wire time at the current rate
// four times over, plus a fixed allowance for the MCU holding CTS.
func (u *GENIFlowUART) txTimeout(n int) time.Duration {
	baud := u.diag.Plan.Baud
	if baud <= 0 {
		baud = 115200
	}

	return 4*time.Duration(n*10)*time.Second/time.Duration(baud) + 250*time.Millisecond
}

func (u *GENIFlowUART) abortTx() {
	reg.Write(u.Base+SE_GENI_M_CMD_CTRL_REG, GENI_CMD_ABORT)

	deadline := time.Now().Add(5 * time.Millisecond)

	for reg.Read(u.Base+SE_GENI_M_IRQ_STATUS)&M_CMD_ABORT_DONE == 0 && time.Now().Before(deadline) {
	}

	reg.Write(u.Base+SE_GENI_M_IRQ_CLEAR, 0xffffffff)
}

func (u *GENIFlowUART) txStall(what string, sent, n int) error {
	ios := reg.Read(u.Base + SE_GENI_IOS)
	cts := "asserted (MCU ready)"

	if ios&IOS_CTS != 0 {
		cts = "DEASSERTED (the MCU is holding the host off, or is not driving CTS)"
	}

	err := fmt.Errorf("transmit stalled waiting for %s after %d of %d bytes: CTS %s, M_IRQ %#08x, TX FIFO status %#08x",
		what, sent, n, cts, reg.Read(u.Base+SE_GENI_M_IRQ_STATUS), reg.Read(u.Base+SE_GENI_TX_FIFO_STATUS))

	u.abortTx()

	return err
}

// Write transmits buf. Received bytes keep being drained while it waits,
// so the two ends cannot hold each other off.
func (u *GENIFlowUART) Write(buf []byte) (int, error) {
	u.mu.Lock()
	defer u.mu.Unlock()

	if !u.ready {
		return 0, errors.New("link UART not initialized")
	}

	const maxCmd = 4096

	sent := 0

	for sent < len(buf) {
		chunk := buf[sent:]

		if len(chunk) > maxCmd {
			chunk = chunk[:maxCmd]
		}

		n := len(chunk)
		deadline := time.Now().Add(u.txTimeout(n))

		reg.Write(u.Base+SE_GENI_M_IRQ_CLEAR, M_CMD_DONE|M_TX_FIFO_WM)
		reg.Write(u.Base+SE_UART_TX_TRANS_LEN, uint32(n))
		reg.Write(u.Base+SE_GENI_M_CMD0, UART_START_TX<<M_OPCODE_SHFT)

		room := int(u.diag.TXDepth - u.txWM)

		for i := 0; i < n; {
			for reg.Read(u.Base+SE_GENI_M_IRQ_STATUS)&M_TX_FIFO_WM == 0 {
				u.poll()

				if time.Now().After(deadline) {
					return sent + i, u.txStall("TX FIFO space", sent+i, len(buf))
				}
			}

			for k := 0; k < room && i < n; k++ {
				reg.Write(u.Base+SE_GENI_TX_FIFO, uint32(chunk[i]))
				i++
			}

			reg.Write(u.Base+SE_GENI_M_IRQ_CLEAR, M_TX_FIFO_WM)
		}

		for reg.Read(u.Base+SE_GENI_M_IRQ_STATUS)&M_CMD_DONE == 0 {
			u.poll()

			if time.Now().After(deadline) {
				return sent, u.txStall("command done", sent+n, len(buf))
			}
		}

		reg.Write(u.Base+SE_GENI_M_IRQ_CLEAR, M_CMD_DONE|M_TX_FIFO_WM)

		sent += n
		u.diag.TxBytes += uint64(n)
	}

	return sent, nil
}

// Diag returns a snapshot of the engine's configuration and live state.
// Before Init it still decodes the clock levels.
func (u *GENIFlowUART) Diag() MCUUARTDiag {
	u.mu.Lock()
	defer u.mu.Unlock()

	d := u.diag
	d.Initialized = u.ready

	if !u.ready {
		d.Levels, d.CurrentLevel = DecodeDFS()

		if !branchOn(GCC_QUP0_S_AHB_CBCR, 7) || !branchOn(u.SECBCR, int(u.SEVoteBit)) {
			// engine registers are not accessible yet
			return d
		}
	}

	b := u.Base

	d.Proto = (reg.Read(b+GENI_FW_REVISION_RO) >> FW_REV_PROTOCOL_SHFT) & FW_REV_PROTOCOL_MASK
	d.IOS = reg.Read(b + SE_GENI_IOS)
	d.CTSAsserted = d.IOS&IOS_CTS == 0
	d.ClkSel = reg.Read(b + SE_GENI_CLK_SEL)
	d.MClkCfg = reg.Read(b + GENI_SER_M_CLK_CFG)
	d.SClkCfg = reg.Read(b + GENI_SER_S_CLK_CFG)
	d.TxTransCfg = reg.Read(b + SE_UART_TX_TRANS_CFG)
	d.ManualRFR = reg.Read(b + SE_UART_MANUAL_RFR)
	d.RFRWatermark = reg.Read(b + SE_GENI_RX_RFR_WATERMARK_REG)
	d.RXFIFOStatus = reg.Read(b + SE_GENI_RX_FIFO_STATUS)
	d.TXFIFOStatus = reg.Read(b + SE_GENI_TX_FIFO_STATUS)
	d.MIRQ = reg.Read(b + SE_GENI_M_IRQ_STATUS)
	d.SIRQ = reg.Read(b + SE_GENI_S_IRQ_STATUS)
	d.RingUsed = u.head - u.tail

	return d
}
