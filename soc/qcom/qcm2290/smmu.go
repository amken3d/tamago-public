// Qualcomm QCM2290 apps SMMU (MMU-500): display stream identity mapping
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

// The display's memory masters (DPU fetch, DSI command DMA) reach DDR
// through the apps SMMU. Unmatched streams fault and bypass is not
// available on this part, so the display stream is routed to a context
// bank whose stage-1 tables map DDR one to one: device addresses equal
// physical addresses.
//
// Written from bmx's docs/display-spec.md §8; see
// docs/display-provenance.md.

// SMMU_BASE is the apps SMMU register base (§1).
const SMMU_BASE = 0x0c600000

// Global register space (SMMUv2 layout, §8)
const (
	smmuIDR1   = 0x024
	smmuGFSR   = 0x048
	smmuSMR0   = 0x800
	smmuS2CR0  = 0xc00
	smmuCBAR0  = 0x1000
	smmuCBA2R0 = 0x1800

	smrValid      = 1 << 31
	s2crTypeTrans = 0 << 16

	// stage 1 translate, stage 2 bypass, memory attributes
	cbarS1Bypass2 = 0x0001f300
	cba2rVA64     = 1 << 0
)

// Context bank registers (§8)
const (
	cbSCTLR     = 0x000
	cbTCR2      = 0x010
	cbTTBR0     = 0x020
	cbTCR       = 0x030
	cbMAIR0     = 0x038
	cbFSR       = 0x058
	cbFAR       = 0x060
	cbFSYNR0    = 0x068
	cbTLBIASID  = 0x610
	cbTLBSYNC   = 0x7f0
	cbTLBSTATUS = 0x7f4

	// upstream sign extension, 40-bit output
	cbTCR2Value = 7<<15 | 2
	// attribute 0: normal non-cacheable (the CPU writes reach memory)
	cbMAIR0Value = 0x44
	// TTBR1 walks off, 4 GB input, 4 KB granule, non-cacheable walks
	cbTCRValue = 1<<23 | 32
	// translate, TEX remap, access flag management, fault reporting
	cbSCTLRValue = 0x27

	// FSR format field reporting AArch64 descriptors: not a fault
	cbFSRFormat = 0x400
)

// Display stream (§8)
const (
	mdssStreamID   = 0x420
	mdssStreamMask = 0x2

	// mdssCB is the display's context bank and stream match group:
	// banks 0-2 and groups 0-3 belong to the boot chain
	mdssCB  = 8
	mdssSMR = 8
)

// contextBankBase returns the address of context bank n: the banks
// start P pages above the SMMU base, P = 2^(IDR1[30:28] + 1).
func contextBankBase(n uint32) uint32 {
	p := uint32(1) << ((reg.Read(SMMU_BASE+smmuIDR1)>>28)&7 + 1)
	return SMMU_BASE + (p+n)*0x1000
}

// identityBank programs context bank cb for stage-1 translation over a
// level-1 table at pt (one 4 KB aligned page, which this call fills):
// the first gigabyte (peripherals) unmapped, the next three as 1 GB
// read-write blocks of attribute 0 (§8 steps 1-6). The caller has
// invalidated the stream match groups that route to cb.
func identityBank(cb uint32, pt uint32) error {
	if pt&0xfff != 0 {
		return fmt.Errorf("page table %#x not 4 KB aligned", pt)
	}

	b := contextBankBase(cb)

	reg.Write(b+cbSCTLR, 0)

	// level-1 descriptors, written as aligned words: the table page
	// may be device memory to the CPU
	for i := uint32(0); i < 4; i++ {
		var d uint64

		if i > 0 {
			d = uint64(i)<<30 | 1<<10 | 1<<6 | 0b01
		}

		reg.Write(pt+8*i, uint32(d))
		reg.Write(pt+8*i+4, uint32(d>>32))
	}

	reg.Write(SMMU_BASE+smmuCBAR0+4*cb, cbarS1Bypass2)
	reg.Write(SMMU_BASE+smmuCBA2R0+4*cb, cba2rVA64)

	reg.Write(b+cbTCR2, cbTCR2Value)
	reg.Write(b+cbMAIR0, cbMAIR0Value)
	reg.Write64(uint64(b+cbTTBR0), uint64(pt))
	reg.Write(b+cbTCR, cbTCRValue)

	reg.Write(b+cbTLBIASID, 0)
	reg.Write(b+cbTLBSYNC, 0)

	if !pollTime(b+cbTLBSTATUS, 1, 0, 10*time.Millisecond) {
		return fmt.Errorf("context bank %d TLB sync timeout", cb)
	}

	reg.Write(b+cbFSR, reg.Read(b+cbFSR))
	reg.Write(b+cbSCTLR, cbSCTLRValue)

	return nil
}

// SMMUMapMDSSIdentity routes the display stream (ID 0x420, mask 0x2)
// through context bank 8 over the DDR identity table at pt (§8), so
// that DPU fetch and DSI command DMA addresses equal physical ones.
// EnableDisplayPower is not required: the SMMU is clocked by the boot
// chain.
func SMMUMapMDSSIdentity(pt uint32) error {
	smr := reg.Read(SMMU_BASE + smmuSMR0 + 4*mdssSMR)
	s2cr := reg.Read(SMMU_BASE + smmuS2CR0 + 4*mdssSMR)

	// refuse a group somebody else programmed; ours is reprogrammed
	if smr&smrValid != 0 && s2cr&s2crTypeCBMask != s2crTypeTrans|mdssCB {
		return fmt.Errorf("stream match group %d in use (SMR %#x S2CR %#x)", mdssSMR, smr, s2cr)
	}

	reg.Write(SMMU_BASE+smmuSMR0+4*mdssSMR, 0)

	if err := identityBank(mdssCB, pt); err != nil {
		return err
	}

	reg.Write(SMMU_BASE+smmuS2CR0+4*mdssSMR, s2crTypeTrans|mdssCB)
	reg.Write(SMMU_BASE+smmuSMR0+4*mdssSMR, smrValid|mdssStreamMask<<16|mdssStreamID)

	return nil
}

// SMMUFaults returns the global fault status and the display context
// bank's fault status, with the FSR format field (0x400, not a fault)
// masked off: both zero means no fault.
func SMMUFaults() (gfsr, cbfsr uint32) {
	gfsr = reg.Read(SMMU_BASE + smmuGFSR)
	cbfsr = reg.Read(contextBankBase(mdssCB)+cbFSR) &^ cbFSRFormat

	return
}

// SMMUFaultDetail returns the display context bank's raw fault status,
// faulting address and syndrome.
func SMMUFaultDetail() (fsr uint32, far uint64, fsynr0 uint32) {
	cb := contextBankBase(mdssCB)

	fsr = reg.Read(cb + cbFSR)
	far = reg.Read64(uint64(cb + cbFAR))
	fsynr0 = reg.Read(cb + cbFSYNR0)

	return
}
