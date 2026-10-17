// Qualcomm QCM2290 interrupt routing
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

// ARM generic timer private peripheral interrupts, GIC INTID = 16 + the
// device tree PPI index (agatti.dtsi timer node, in interrupts order:
// secure-physical 1, non-secure-physical 2, virtual 3, hypervisor 0).
//
// The virtual timer is the one delivered to non-secure EL1 under the
// Qualcomm hypervisor (the physical timer PPI never signals; the stock
// kernel selects the virtual timer for the same reason).
const (
	TIMER_EL1_PHYS_IRQ = 16 + 2
	TIMER_EL1_VIRT_IRQ = 16 + 3
)

// defined in gic.s
func write_icc_igrpen1_el1(val uint64)
func write_icc_sre_el1(val uint64)
func write_icc_pmr_el1(val uint64)

// GICv3 register offsets used by the minimal non-secure init
const (
	gicdCTLR      = 0x000
	gicrWAKER     = 0x014
	gicrIGROUPR0  = 0x10000 + 0x080 // SGI_BASE frame, PPI/SGI groups
	gicrISENABLER = 0x10000 + 0x100 // SGI_BASE frame, PPI/SGI enables
	gicrISPENDR0  = 0x10000 + 0x200 // SGI_BASE frame, PPI/SGI pending
)

// InitGIC initializes the GICv3 for wake-up capable timer interrupts.
//
// The OS runs at non-secure EL1 under the Qualcomm hypervisor with
// GICD_CTLR.DS=0: non-secure interrupts are Group 1 and Group 0 state
// is not writable in the non-secure register views. This is a minimal
// Group 1 configuration so a pending timer PPI reaches the core as a
// WFI wake-up event (IRQs stay masked at the PE; no exception is
// taken). The generic gic package Init() is not used: its full
// distributor sweep and Group 0 CPU-interface writes reset the platform
// from non-secure EL1 (hypervisor/XPU intervention).
func InitGIC() {
	// Mark the CPU as online (reads back awake on this platform)
	reg.Clear(GICR_BASE+gicrWAKER, 1)

	// System register interface, unmasked priorities, Group 1 signaling
	write_icc_sre_el1(1)
	write_icc_pmr_el1(0xff)
	write_icc_igrpen1_el1(1)

	// Affinity routing for the non-secure state: without it the
	// redistributor SGI/PPI frame is not in effect (bit 4 = ARE_NS in
	// the non-secure GICD_CTLR view; groups must still be disabled at
	// this point)
	reg.Set(GICD_BASE+gicdCTLR, 4)

	// Distributor Group 1 forwarding (non-secure view of GICD_CTLR:
	// bit 0 EnableGrp1, bit 1 EnableGrp1A under affinity routing)
	reg.Set(GICD_BASE+gicdCTLR, 0)
	reg.Set(GICD_BASE+gicdCTLR, 1)

	// Timer PPIs: the virtual timer is the deliverable one; the
	// physical PPI is enabled for completeness
	reg.SetTo(GICR_BASE+gicrISENABLER, TIMER_EL1_PHYS_IRQ, true)
	reg.SetTo(GICR_BASE+gicrISENABLER, TIMER_EL1_VIRT_IRQ, true)
}

// GICState returns the non-secure views of GICD_CTLR and the
// redistributor PPI group/enable/pending registers, for bring-up
// diagnostics.
func GICState() (ctlr, igroupr0, isenabler0, ispendr0 uint32) {
	return reg.Read(GICD_BASE + gicdCTLR),
		reg.Read(GICR_BASE + gicrIGROUPR0),
		reg.Read(GICR_BASE + gicrISENABLER),
		reg.Read(GICR_BASE + gicrISPENDR0)
}

// GICv3 redistributor frame layout (ARM IHI 0069): each PE has an RD_base
// frame followed by an SGI_base frame, 64 KB each, plus two more 64 KB
// frames when the redistributor supports virtual LPIs (GICv4).
const (
	gicrTYPER      = 0x008
	gicrTYPER_VLPI = 1 << 1
	gicrTYPER_LAST = 1 << 4

	gicrFrameSize     = 0x20000
	gicrFrameSizeVLPI = 0x40000

	// bound on the frame walk: four Cortex-A53 cores on this SoC, with
	// headroom; the walk stops at the frame flagged Last regardless
	gicrMaxFrames = 8
)

// Redistributor returns the RD_base address of the redistributor serving
// the PE with the argument affinity (MPIDR_EL1 Aff3.Aff2.Aff1.Aff0 packed
// as GICR_TYPER[63:32] reports it), found by walking the redistributor
// frames from GICR_BASE until the one flagged Last.
func Redistributor(aff uint32) (rd uint32, ok bool) {
	rd = GICR_BASE

	for i := 0; i < gicrMaxFrames; i++ {
		lo := reg.Read(rd + gicrTYPER)
		hi := reg.Read(rd + gicrTYPER + 4)

		if hi == aff {
			return rd, true
		}

		if lo&gicrTYPER_LAST != 0 {
			break
		}

		if lo&gicrTYPER_VLPI != 0 {
			rd += gicrFrameSizeVLPI
		} else {
			rd += gicrFrameSize
		}
	}

	return 0, false
}

// RedistributorTyper returns GICR_TYPER of the redistributor at rd.
func RedistributorTyper(rd uint32) uint64 {
	return uint64(reg.Read(rd+gicrTYPER+4))<<32 | uint64(reg.Read(rd+gicrTYPER))
}

// InitRedistributor prepares the redistributor at rd (as returned by
// Redistributor) of a secondary PE for timer wake-ups, as InitGIC does
// for the boot PE's own: mark it awake and enable the timer PPIs. The
// PE's CPU interface (the ICC_* system registers) can only be set up by
// that PE itself.
func InitRedistributor(rd uint32) {
	reg.Clear(rd+gicrWAKER, 1)

	reg.SetTo(rd+gicrISENABLER, TIMER_EL1_PHYS_IRQ, true)
	reg.SetTo(rd+gicrISENABLER, TIMER_EL1_VIRT_IRQ, true)
}

// RedistributorState returns GICR_WAKER and the PPI/SGI group, enable and
// pending registers of the redistributor at rd, for bring-up diagnostics.
func RedistributorState(rd uint32) (waker, igroupr0, isenabler0, ispendr0 uint32) {
	return reg.Read(rd + gicrWAKER),
		reg.Read(rd + gicrIGROUPR0),
		reg.Read(rd + gicrISENABLER),
		reg.Read(rd + gicrISPENDR0)
}
