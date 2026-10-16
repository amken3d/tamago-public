// Qualcomm QCM2290 DPU (display processor) single-layer scanout
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

// DPU block offsets from the MDP base (dpu_6_5_qcm2290.h catalog).
// The static configuration replays a live register snapshot of this
// hardware scanning out 1080p60 XRGB8888 through VIG0 -> LM0 -> PP0 ->
// INTF1 (DSI); only the source addresses are repointed at the caller's
// framebuffer.
const (
	DPU_BASE  = 0x05e01000
	VBIF_BASE = 0x05eb0000

	dpuTop   = 0x0
	dpuCTL0  = 0x1000
	dpuVIG0  = 0x4000
	dpuLM0   = 0x44000
	dpuPP0   = 0x70000
	dpuINTF1 = 0x6a800

	// CTL kickoff registers (excluded from the snapshot)
	ctlFlush     = 0x018
	ctlStart     = 0x01c
	ctlIntfFlush = 0x110

	// CTL_FLUSH block bits (dpu_hw_ctl.c)
	ctlFlushVIG0 = 1 << 0
	ctlFlushLM0  = 1 << 6
	ctlFlushCTL  = 1 << 17
	ctlFlushINTF = 1 << 31

	// VIG0 source address registers (repointed)
	vigSrc0Addr = 0x014
	vigSrcAddrB = 0x0a4

	// INTF timing engine enable (written last)
	intfTimingEn = 0x000

	// INTF line counter; advances while the timing engine scans. Note
	// 0x0b4 reads back zero even on a live Linux scanout -- it is not
	// the counter, and reading it makes a working pipeline look dead.
	intfLineCount = 0x0b0

	// Display SMMU (MMU-500). The secure configuration on this part
	// permits neither escape hatch: sCR0.CLIENTPD is RAZ/WI (a write
	// of 1 reads back 0), and an S2CR of TYPE_BYPASS reads back as
	// TYPE_FAULT. Unmatched streams fault (sCR0.USFCFG is set), and
	// the boot chain leaves no entry for MDSS, so the first DPU fetch
	// faults and hangs the bus into a watchdog reset.
	//
	// The way through is a real context bank: a bank whose SCTLR.M is
	// clear has stage-1 translation disabled and forwards addresses
	// unmodified, which is an identity map for four register writes
	// and no page tables.
	SMMU_BASE = 0x0c600000

	smmuSCR0  = 0x000
	smmuIDR1  = 0x024
	smmuGFSR  = 0x048
	smmuSMR0  = 0x800
	smmuS2CR0 = 0xc00
	smmuCBAR0 = 0x1000

	// context bank register offsets
	cbSCTLR = 0x000
	cbFSR   = 0x058

	// MDSS stream, as the stock DT programs it:
	// iommus = <&apps_smmu 0x420 0x2>
	mdssStreamID   = 0x420
	mdssStreamMask = 0x002

	smrValid = 1 << 31

	// S2CR.TYPE == 0 routes a matched stream to context bank CBNDX.
	s2crTypeTrans = 0 << 16

	// CBAR.TYPE == 1 (S1_TRANS_S2_BYPASS) plus the memory attributes
	// the stock driver uses for its own MDSS bank.
	cbarMDSS = 0x0001f300

	// Banks 0-2 are already spoken for by the boot chain's mappings.
	mdssCB = 8
)

// dpuVBIFQoS is the VBIF read QoS configuration of the live Linux
// display (the DPU's memory bus arbiter, at VBIF_BASE): the client
// priority levels and the per-level QoS remap tables, which raise the
// real-time scanout clients' priority. Left at reset values, the VIG0
// fetch loses arbitration: the DSI video FIFO underflows (FIFO_STATUS
// VIDEO_MDP_FIFO_UNDERFLOW and lane 0 HS underflow) and a panel falls
// back to its own self test. Mode independent.
var dpuVBIFQoS = []regVal{
	{0x160, 0x33333333}, {0x164, 0x00333333}, {0x550, 0x00000003},
	{0x554, 0x00000000}, {0x558, 0x11111113}, {0x55c, 0x00111111},
	{0x560, 0x22222224}, {0x564, 0x00222222}, {0x568, 0x33333334},
	{0x56c, 0x00333333}, {0x570, 0x44444445}, {0x574, 0x00444444},
	{0x578, 0x55555555}, {0x57c, 0x00555555}, {0x580, 0x66666666},
	{0x584, 0x00666666}, {0x588, 0x77777776}, {0x58c, 0x00777777},
	{0x590, 0x00000003}, {0x594, 0x00000000}, {0x598, 0x11111113},
	{0x59c, 0x00111111}, {0x5a0, 0x22222224}, {0x5a4, 0x00222222},
	{0x5a8, 0x33333334}, {0x5ac, 0x00333333}, {0x5b0, 0x44444445},
	{0x5b4, 0x00444444}, {0x5b8, 0x55555555}, {0x5bc, 0x00555555},
	{0x5c0, 0x66666666}, {0x5c4, 0x00666666}, {0x5c8, 0x77777776},
	{0x5cc, 0x00777777},
}

var dpuTopSnap = []regVal{
	{0x00c, 0x00000200}, {0x014, 0x08000100}, {0x19c, 0x00000455},
	{0x1e8, 0x00000038}, {0x1ec, 0x00000001}, {0x2ac, 0x40000444},
	{0x2b0, 0x00000001}, {0x2b4, 0x40000444}, {0x2bc, 0x44444444},
	{0x2c4, 0x00004444}, {0x2e0, 0x00011111}, {0x300, 0x00010e00},
	{0x304, 0x0001008b}, {0x308, 0x00010944}, {0x364, 0x51500555},
	{0x374, 0x00000001}, {0x384, 0x00000002}, {0x394, 0x00000002},
	{0x3a8, 0x44444044}, {0x3ac, 0x00090080}, {0x3b0, 0x44444044},
	{0x3b8, 0x44444040}, {0x3d0, 0x00444000}, {0x3d8, 0x000000e4},
	{0x3dc, 0x000000e4}, {0x3e0, 0x00004000}, {0x3e4, 0x00004000},
	{0x3f8, 0x00000001}, {0x424, 0x00000002}, {0x434, 0x00000002},
	{0x444, 0x00000002}, {0x484, 0x00000002}, {0x488, 0x00000011},
}

var dpuCTL0Snap = []regVal{
	{0x000, 0x01000002}, {0x03c, 0x00000001}, {0x064, 0x00000001},
	{0x0d8, 0xc068e038}, {0x0dc, 0x1f971fc7}, {0x0e0, 0x00000001},
	{0x0f4, 0x00000002},
}

var dpuVIG0Snap = []regVal{
	{0x000, 0x04380780}, {0x00c, 0x04380780}, {0x024, 0x00001e00},
	{0x030, 0x000236ff}, {0x034, 0x03020001}, {0x038, 0x80000000},
	{0x044, 0xc0000101}, {0x048, 0x00000087}, {0x04c, 0x00000707},
	{0x060, 0x000000ff}, {0x064, 0x0000fff0}, {0x06c, 0x00000001},
	{0x074, 0x22335777}, {0x078, 0x00112222}, {0x0d0, 0x00054a80},
	{0x0e0, 0x00054527}, {0x0f0, 0x000f000f}, {0x0f4, 0x00010030},
	{0x0f8, 0x02d102d1}, {0x108, 0x04380780}, {0x118, 0x04380780},
	{0x128, 0x04380780}, {0x134, 0x00000009}, {0x13c, 0x0000000f},
	{0x17c, 0x80000000}, {0x194, 0x00001001}, {0x198, 0x00ff0000},
	{0x19c, 0x00400040}, {0x1a0, 0x000000ff}, {0x1a4, 0x00000024},
	{0x1a8, 0x00ffffff}, {0x1bc, 0x00000010}, {0x1f0, 0x00070502},
	{0x1f4, 0x00070502},
}

var dpuLM0Snap = []regVal{
	{0x000, 0x00000002}, {0x004, 0x04380780}, {0x020, 0x00000100},
	{0x024, 0x00ff0000},
}

var dpuPP0Snap = []regVal{
	{0x0ac, 0x0002c688}, {0x0c8, 0x0002c688},
}

// INTF_PANEL_FORMAT (0x090) is 0x213f -- RGB888 with 8 bits per
// component -- although a snapshot reads back 0x2100: the bit depth
// fields are write-only. Replayed as read, the INTF has no pixel
// format and underruns, showing INTF_UNDERFLOW_COLOR (blue).
var dpuINTF1Snap = []regVal{
	{0x004, 0x00800000}, {0x008, 0x0898002c}, {0x00c, 0x0025c3f8},
	{0x014, 0x00002af8}, {0x01c, 0x00016058}, {0x024, 0x0025a197},
	{0x03c, 0x083f00c0}, {0x048, 0x000000ff}, {0x060, 0x00000010},
	{0x064, 0x083f00c0}, {0x08c, 0x00000011}, {0x090, 0x0000213f},
	{0x094, 0x76543210}, {0x0ac, 0x0000554a}, {0x0b0, 0x0000033b},
	{0x0b8, 0x00000010}, {0x178, 0xffffffff}, {0x180, 0x01000000},
	{0x1c4, 0x00000a00}, {0x1d8, 0x00000002}, {0x1dc, 0x00020001},
	{0x1f0, 0xffffffff}, {0x250, 0xffffffff}, {0x25c, 0x000f0000},
	{0x268, 0xffffffff}, {0x26c, 0x00000001},
}

// SMMUBypass makes unmatched streams bypass translation (physical
// addressing) by clearing sCR0.USFCFG -- and ensures CLIENTPD stays
// clear: setting it TERMINATES all client transactions, and the DPU's
// aborted fetch escalates to a fatal NoC error (learned by ramdump).
// Returns the previous sCR0 value.
// contextBankBase returns the base address of SMMU context bank n. The
// bank array follows the global/implementation pages, whose count IDR1
// reports as 2^(NUMPAGENDXB+1) (64 here, so banks start at +0x40000).
func contextBankBase(n uint32) uint32 {
	idr1 := reg.Read(SMMU_BASE + smmuIDR1)
	numPage := uint32(1) << (((idr1 >> 28) & 0x7) + 1)

	return SMMU_BASE + (numPage+n)*0x1000
}

// SMMUMapMDSS routes the display stream through a pass-through context
// bank, so DPU scanout may fetch the framebuffer's physical address.
// It returns the prior sCR0 for diagnostics. See the register block
// above for why neither bypass mechanism works on this part.
func SMMUMapMDSS() uint32 {
	scr0 := reg.Read(SMMU_BASE + smmuSCR0)
	cb := contextBankBase(mdssCB)

	// stage-1 translation off: addresses pass through unmodified
	reg.Write(cb+cbSCTLR, 0)
	reg.Write(SMMU_BASE+smmuCBAR0+mdssCB*4, cbarMDSS)

	// configure the destination before making the match valid, so no
	// transaction can land on a half-programmed entry
	reg.Write(SMMU_BASE+smmuS2CR0+mdssCB*4, s2crTypeTrans|mdssCB)
	reg.Write(SMMU_BASE+smmuSMR0+mdssCB*4,
		smrValid|mdssStreamMask<<16|mdssStreamID)

	return scr0
}

// SMMUFaults returns the global fault status and the MDSS context
// bank's fault status. Both zero means scanout is being let through.
func SMMUFaults() (gfsr, cbfsr uint32) {
	gfsr = reg.Read(SMMU_BASE + smmuGFSR)
	cbfsr = reg.Read(contextBankBase(mdssCB) + cbFSR)

	return
}

// dpuTraceWrites are configuration writes the Linux driver makes that
// a register snapshot cannot capture: zeros over non-zero reset values
// (CTL_TOP resets to 0x1f00 and must be 0 in video mode, or CTL0 never
// routes to the INTF) and registers whose value does not read back.
// Taken from a trace of every DPU write during a live Linux bring-up
// of the 800x480 panel; mode independent.
var dpuTraceWrites = []struct {
	block uint32
	regVal
}{
	{dpuTop, regVal{0x028, 0}}, // SSPP_SPARE
	{dpuTop, regVal{0x2f4, 0}}, // SPLIT_DISPLAY_EN
	{dpuTop, regVal{0x2f8, 0}}, // SPLIT_DISPLAY_UPPER_PIPE_CTRL
	{dpuTop, regVal{0x3f0, 0}}, // SPLIT_DISPLAY_LOWER_PIPE_CTRL

	{dpuCTL0, regVal{0x014, 0}}, // CTL_TOP: video mode, no INTF select
	{dpuCTL0, regVal{0x040, 0}}, // CTL_LAYER_EXT(LM0)
	{dpuCTL0, regVal{0x070, 0}}, // CTL_LAYER_EXT2(LM0)
	{dpuCTL0, regVal{0x0a0, 0}}, // CTL_LAYER_EXT3(LM0)
	{dpuCTL0, regVal{0x0e4, 0}}, // CTL_MERGE_3D_ACTIVE
	{dpuCTL0, regVal{0x0e8, 0}}, // CTL_DSC_ACTIVE
	{dpuCTL0, regVal{0x0ec, 0}}, // CTL_WB_ACTIVE
	{dpuCTL0, regVal{0x0f0, 0}}, // CTL_CWB_ACTIVE
	{dpuCTL0, regVal{0x0fc, 0}}, // CTL_FETCH_PIPE_ACTIVE

	{dpuINTF1, regVal{0x02c, 0}}, // INTF_ACTIVE_V_START_F0
	{dpuINTF1, regVal{0x034, 0}}, // INTF_ACTIVE_V_END_F0
	{dpuINTF1, regVal{0x040, 0}}, // INTF_ACTIVE_HCTL
	{dpuINTF1, regVal{0x044, 0}}, // INTF_BORDER_COLOR
	{dpuINTF1, regVal{0x04c, 0}}, // INTF_HSYNC_SKEW
	{dpuINTF1, regVal{0x050, 0}}, // INTF_POLARITY_CTL
	{dpuINTF1, regVal{0x068, 0}}, // INTF_ACTIVE_DATA_HCTL
	{dpuINTF1, regVal{0x0a8, 3}}, // INTF_FRAME_LINE_COUNT_EN

	{dpuPP0, regVal{0x30e0, 0}}, // PP_DITHER_EN
}

// InitDPU replays the mode's scanout pipeline configuration with the
// VIG0 source repointed at the framebuffer (physical address, laid out
// with the mode's Stride; SMMUMapMDSS first). The display power domain
// must be up.
func InitDPU(m *DisplayMode, fb uint32) {
	replay(VBIF_BASE, dpuVBIFQoS)
	replay(DPU_BASE+dpuTop, m.dpuTop)
	replay(DPU_BASE+dpuCTL0, m.dpuCTL0)
	replay(DPU_BASE+dpuVIG0, m.dpuVIG0)

	reg.Write(DPU_BASE+dpuVIG0+vigSrc0Addr, fb)
	reg.Write(DPU_BASE+dpuVIG0+vigSrcAddrB, fb)

	replay(DPU_BASE+dpuLM0, m.dpuLM0)
	replay(DPU_BASE+dpuPP0, m.dpuPP0)
	replay(DPU_BASE+dpuINTF1, m.dpuINTF1)

	for _, w := range dpuTraceWrites {
		reg.Write(DPU_BASE+w.block+w.off, w.val)
	}
}

// StartDPU flushes the configuration and starts the interface timing
// engine: pixels begin streaming into the DSI controller.
func StartDPU() {
	// The order the Linux driver uses (traced from a live bring-up of
	// this panel): one flush of the pipeline blocks plus the CTL and
	// the INTF aggregate, then the timing engine enable. Video mode
	// takes no CTL_START; later frames re-flush without the INTF bit.
	//
	// The earlier two-kickoff, engine-first sequence here was a
	// workaround for INTF_PANEL_FORMAT missing its bit depth fields
	// (see dpuINTF1Snap): with no valid pixel format the INTF
	// underruns whatever the order.
	reg.Write(DPU_BASE+dpuCTL0+ctlIntfFlush, 1<<1) // INTF1
	reg.Write(DPU_BASE+dpuCTL0+ctlFlush,
		ctlFlushVIG0|ctlFlushLM0|ctlFlushCTL|ctlFlushINTF)
	reg.Write(DPU_BASE+dpuINTF1+intfTimingEn, 1)
}

// DPUStatus returns the INTF line count (advancing = scanning) and
// the CTL flush register (0 = flush consumed).
func DPUStatus() (line, flush uint32) {
	line = reg.Read(DPU_BASE + dpuINTF1 + intfLineCount)
	flush = reg.Read(DPU_BASE + dpuCTL0 + ctlFlush)

	return
}

// EnableMDPClocks brings up dispcc PLL0 (768 MHz) and the MDP core
// clock at 384 MHz plus its branches, required for DPU operation.
func EnableMDPClocks() error {
	pll := uint32(DISPCC_BASE) // PLL0 at dispcc base

	// L, user/config control (snapshot values; alpha = 0 -> 768 MHz
	// integer mode from the 19.2 MHz XO)
	reg.Write(pll+0x04, 0x28)
	reg.Write(pll+0x10, 0x00200001)
	reg.Write(pll+0x14, 0x00000004)
	reg.Write(pll+0x18, 0x4001055b)

	// alpha PLL enable: BYPASSNL -> RESET_N -> lock -> OUTCTRL
	reg.Set(pll+0x00, 1)
	udelaySpin(10)
	reg.Set(pll+0x00, 2)
	udelaySpin(50)

	locked := false

	for i := 0; i < 1000; i++ {
		if reg.Get(pll+0x00, 31) {
			locked = true
			break
		}

		udelaySpin(10)
	}

	if !locked {
		return errors.New("dispcc PLL0 lock timeout")
	}

	reg.Set(pll+0x00, 0)

	// MDP RCG: PLL0/2 = 384 MHz (src 1, div field 3), then branches
	if err := rcgUpdate(DISPCC_BASE, 0x2074, 1<<8|3); err != nil {
		return errors.New("mdp rcg: " + err.Error())
	}

	if err := dispClkOn(DISPCC_BASE, 0x2008); err != nil { // mdp
		return errors.New("mdp branch: " + err.Error())
	}

	dispClkOn(DISPCC_BASE, 0x2010) // mdp lut
	dispClkOn(DISPCC_BASE, 0x2018) // vsync

	if err := rcgUpdate(DISPCC_BASE, 0x208c, cfgSrcXODiv1); err != nil { // vsync rcg
		return errors.New("vsync rcg: " + err.Error())
	}

	return nil
}
