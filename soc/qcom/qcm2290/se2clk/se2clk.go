// Qualcomm QCM2290 QUPv3 SE2 clock root: rates, register encodings and
// baud planning
// https://github.com/usbarmory/tamago
//
// Copyright (c) the qcm2290 package authors
//
// Use of this source code is governed by the license
// that can be found in the LICENSE file.

// Package se2clk is the arithmetic half of driving the SE2 clock root
// directly (bmx docs/mcu-link-spec.md §2.1, §4.3): the root's rate table,
// the CFG/M/N/D encodings and the choice of root rate and engine divider
// for a baud rate. It touches no hardware, so it is tested on the host;
// the qcm2290 package does the register access.
package se2clk

import (
	"fmt"
)

// Sources of the SE2 root (CFG bits 10:8) and their rates (§2.1).
var SourceHz = map[uint32]uint64{
	0: 19200000,  // crystal
	1: 600000000, // GPLL0 main ("early")
	2: 300000000, // GPLL0 AUX2 (GPLL0 / 2)
	4: 384000000, // GPLL6 main (GPLL6 / 2)
}

// CFG fields (§2.1).
const (
	CFGPreDivMask = 0x1f
	CFGSrcShift   = 8
	CFGSrcMask    = 0x7 << CFGSrcShift
	CFGModeShift  = 12
	CFGModeMask   = 0x3 << CFGModeShift
	CFGModeDual   = 2 // dual-edge M/N
	CFGHWCtl      = 1 << 20

	cfgFields = CFGPreDivMask | CFGSrcMask | CFGModeMask
)

// Root is one root configuration: source, divider h (stored as 2h, so
// half steps are integers) and an optional m/n.
type Root struct {
	Src uint32
	H2  uint32 // 2 x h: 2 = divide by 1, 3 = 1.5, ... 32 = 16
	M   uint32 // 0 = no M/N
	N   uint32
}

// Table is the set of root rates bmx uses (Linux's frequency table for
// this root, §2.1); the engine is not run above 128 MHz.
var Table = []Root{
	{Src: 2, H2: 2, M: 384, N: 15625}, // 7.3728 MHz
	{Src: 2, H2: 2, M: 768, N: 15625}, // 14.7456
	{Src: 0, H2: 2},                   // 19.2 (crystal)
	{Src: 2, H2: 2, M: 1536, N: 15625},
	{Src: 2, H2: 2, M: 8, N: 75},  // 32
	{Src: 2, H2: 2, M: 4, N: 25},  // 48
	{Src: 2, H2: 2, M: 16, N: 75}, // 64
	{Src: 2, H2: 8},               // 75
	{Src: 2, H2: 2, M: 4, N: 15},  // 80
	{Src: 2, H2: 2, M: 8, N: 25},  // 96
	{Src: 2, H2: 6},               // 100
	{Src: 2, H2: 2, M: 128, N: 375},
	{Src: 2, H2: 2, M: 28, N: 75},      // 112
	{Src: 2, H2: 2, M: 6144, N: 15625}, // 117.9648
	{Src: 2, H2: 5},                    // 120
	{Src: 4, H2: 6},                    // 128
}

// Crystal is the root's reset configuration (CFG bits 0: crystal, bypass).
var Crystal = Root{Src: 0, H2: 2}

// UsesMN reports whether the configuration needs the M/N counter.
func (r Root) UsesMN() bool { return r.N != 0 && r.M != r.N }

// Hz is the root rate: source / h x m / n.
func (r Root) Hz() uint64 {
	src, ok := SourceHz[r.Src]
	if !ok || r.H2 == 0 {
		return 0
	}

	if !r.UsesMN() {
		return src * 2 / uint64(r.H2)
	}

	return src * 2 * uint64(r.M) / (uint64(r.H2) * uint64(r.N))
}

// String names the configuration.
func (r Root) String() string {
	h := fmt.Sprintf("%d", r.H2/2)
	if r.H2%2 != 0 {
		h += ".5"
	}

	s := fmt.Sprintf("source %d (%d Hz) / %s", r.Src, SourceHz[r.Src], h)

	if r.UsesMN() {
		s += fmt.Sprintf(" x %d/%d", r.M, r.N)
	}

	return s + fmt.Sprintf(" = %d Hz", r.Hz())
}

// PreDiv is the CFG pre-divider field (2h - 1). The crystal at divide-by-1
// uses 0 (bypass), its reset value.
func (r Root) PreDiv() uint32 {
	if r.Src == 0 && r.H2 == 2 {
		return 0
	}

	return r.H2 - 1
}

// CFG returns the CFG value for r, a read-modify-write of found: bits 4:0,
// 10:8 and 13:12 change, everything else (bit 20 hardware control) is
// kept.
func (r Root) CFG(found uint32) uint32 {
	v := found &^ cfgFields
	v |= r.PreDiv() & CFGPreDivMask
	v |= r.Src << CFGSrcShift & CFGSrcMask

	if r.UsesMN() {
		v |= CFGModeDual << CFGModeShift
	}

	return v
}

// MND returns the M, N and D register values (§2.1: N = ~(n - m), D = ~d
// with d = n clamped to [m, 2(n - m)]); used is false when the
// configuration does not use M/N (the registers are then not written).
func (r Root) MND() (m, n, d uint32, used bool) {
	if r.N == 0 {
		return 0, 0, 0, false
	}

	dv := r.N

	if dv < r.M {
		dv = r.M
	}

	if hi := 2 * (r.N - r.M); dv > hi {
		dv = hi
	}

	return r.M & 0xffff, ^(r.N - r.M) & 0xffff, ^dv & 0xffff, r.UsesMN()
}

// Decode reads a configuration back from CFG and the M/N registers.
func Decode(cfg, mreg, nreg uint32) Root {
	r := Root{Src: cfg & CFGSrcMask >> CFGSrcShift}

	if pd := cfg & CFGPreDivMask; pd == 0 {
		r.H2 = 2
	} else {
		r.H2 = pd + 1
	}

	if cfg&CFGModeMask != 0 {
		r.M = mreg & 0xffff
		r.N = (^nreg&0xffff + r.M) & 0xffff
	}

	return r
}

// Plan is how a baud rate is produced: a root rate and an engine divider.
type Plan struct {
	Baud         int
	Root         Root
	RootHz       uint64
	Divider      uint32
	Oversampling uint32
	ActualBaud   float64
	ErrorPPM     int
}

// MaxErrorPPM is the largest divider error PlanBaud accepts.
const MaxErrorPPM = 10000

// PlanBaud picks the root rate from roots and the engine divider with the
// least error for baud (§4.3: divider = ceil(rate / (baud x ovs)), at most
// 4095); ties go to the lowest root rate, so 1 200 000 baud stays on the
// crystal. Rates outside 300-8 000 000 and errors above MaxErrorPPM are
// refused.
func PlanBaud(baud int, ovs uint32, roots []Root) (Plan, error) {
	if baud < 300 || baud > 8000000 {
		return Plan{}, fmt.Errorf("baud %d outside 300..8000000", baud)
	}

	if ovs == 0 {
		return Plan{}, fmt.Errorf("oversampling 0")
	}

	req := uint64(baud) * uint64(ovs)

	var (
		best    Plan
		bestErr = int64(1) << 62
		found   bool
	)

	for _, r := range roots {
		hz := r.Hz()
		if hz == 0 {
			continue
		}

		div := (hz + req - 1) / req

		if div < 1 {
			div = 1
		}

		if div > 4095 {
			continue
		}

		actual := float64(hz) / float64(div*uint64(ovs))
		ppm := int64((actual - float64(baud)) / float64(baud) * 1e6)
		abs := ppm

		if abs < 0 {
			abs = -abs
		}

		if abs < bestErr || abs == bestErr && hz < best.RootHz {
			bestErr, found = abs, true
			best = Plan{
				Baud: baud, Root: r, RootHz: hz, Divider: uint32(div),
				Oversampling: ovs, ActualBaud: actual, ErrorPPM: int(ppm),
			}
		}
	}

	if !found {
		return best, fmt.Errorf("no root rate reaches %d baud", baud)
	}

	if bestErr > MaxErrorPPM {
		return best, fmt.Errorf("%d baud: best is %v / %d / %d = %.0f baud, %d ppm off (limit %d)",
			baud, best.Root, best.Divider, ovs, best.ActualBaud, best.ErrorPPM, MaxErrorPPM)
	}

	return best, nil
}
