// Qualcomm QCM2290 apps SMMU (MMU-500): MDSS stage-1 identity mapping
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

// SMMUv2 registers beyond those in dpu.go (arm-smmu.h)
const (
	// global register space 1: per-bank attribute registers
	smmuCBA2R0 = 0x1800
	cba2rVA64  = 1 << 0

	// context bank, stage 1, AArch64 descriptor format
	cbTCR2      = 0x010
	cbTTBR0     = 0x020
	cbTCR       = 0x030
	cbMAIR0     = 0x038
	cbFAR       = 0x060
	cbFSYNR0    = 0x068
	cbTLBIASID  = 0x610
	cbTLBSYNC   = 0x7f0
	cbTLBSTATUS = 0x7f4

	// SCTLR: translation on, TEX remap (MAIR), access flag managed by
	// descriptors, context fault reporting -- the Linux MDSS bank's
	// 0x67 without the interrupt enable (polled here)
	sctlrM    = 1 << 0
	sctlrTRE  = 1 << 1
	sctlrAFE  = 1 << 2
	sctlrCFRE = 1 << 5

	// TCR: 4 GB input space (T0SZ 32, so the walk starts at level 1
	// with four 1 GB entries), 4 KB granule, non-cacheable
	// non-shareable walks (the table is written through device memory),
	// TTBR1 walks disabled
	tcrT0SZ32 = 32
	tcrEPD1   = 1 << 23

	// TCR2: 40-bit output addresses, upstream sign extension
	tcr2PASize40 = 2
	tcr2SEP      = 7 << 15

	// MAIR attribute 0: normal memory, inner/outer non-cacheable. The
	// CPU cleans the framebuffer to memory, so the DPU must not look
	// in a cache for it.
	mairNormalNC = 0x44

	// level 1 block descriptor: block, AttrIndx 0, AP = EL1/EL0
	// read-write, access flag set
	l1Block = 0b01
	l1APRW  = 1 << 6
	l1AF    = 1 << 10

	smmuPollSpins = 100000
)

// SMMUMapMDSSIdentity routes the display stream through context bank 8
// with stage-1 translation on, over a page table that maps DDR
// (0x40000000-0xffffffff, three 1 GB blocks) to itself as normal
// non-cacheable memory. Device addresses then equal physical ones as
// with SMMUMapMDSS, but every DPU access takes a real walk, as under
// Linux; the first gigabyte (peripherals) stays unmapped and faults.
//
// pt is the physical address of a 4 KB aligned page for the level 1
// table; only its first four entries are used.
func SMMUMapMDSSIdentity(pt uint32) error {
	if pt&0xfff != 0 {
		return errors.New("page table must be 4 KB aligned")
	}

	cb := contextBankBase(mdssCB)
	smr := uint32(SMMU_BASE + smmuSMR0 + mdssCB*4)

	// no transaction may match the bank while it is reprogrammed
	reg.Write(smr, 0)
	reg.Write(cb+cbSCTLR, 0)

	reg.Write64(uint64(pt), 0)

	for i := uint64(1); i < 4; i++ {
		reg.Write64(uint64(pt)+i*8, i<<30|l1AF|l1APRW|l1Block)
	}

	reg.Write(SMMU_BASE+smmuCBAR0+mdssCB*4, cbarMDSS)
	reg.Write(SMMU_BASE+smmuCBA2R0+mdssCB*4, cba2rVA64)

	reg.Write(cb+cbTCR2, tcr2SEP|tcr2PASize40)
	reg.Write(cb+cbMAIR0, mairNormalNC)
	reg.Write64(uint64(cb+cbTTBR0), uint64(pt))
	reg.Write(cb+cbTCR, tcrEPD1|tcrT0SZ32)

	// drop anything cached for ASID 0 from the pass-through era
	reg.Write(cb+cbTLBIASID, 0)
	reg.Write(cb+cbTLBSYNC, 0)

	for i := 0; ; i++ {
		if reg.Read(cb+cbTLBSTATUS)&1 == 0 {
			break
		}

		if i >= smmuPollSpins {
			return errors.New("SMMU TLB sync timeout")
		}
	}

	// clear stale faults, then enable translation
	reg.Write(cb+cbFSR, reg.Read(cb+cbFSR))
	reg.Write(cb+cbSCTLR, sctlrCFRE|sctlrAFE|sctlrTRE|sctlrM)

	reg.Write(SMMU_BASE+smmuS2CR0+mdssCB*4, s2crTypeTrans|mdssCB)
	reg.Write(smr, smrValid|mdssStreamMask<<16|mdssStreamID)

	return nil
}

// SMMUFaultDetail returns the MDSS context bank's fault status, faulting
// address and syndrome.
func SMMUFaultDetail() (fsr uint32, far uint64, fsynr0 uint32) {
	cb := contextBankBase(mdssCB)

	fsr = reg.Read(cb + cbFSR)
	far = reg.Read64(uint64(cb + cbFAR))
	fsynr0 = reg.Read(cb + cbFSYNR0)

	return
}
