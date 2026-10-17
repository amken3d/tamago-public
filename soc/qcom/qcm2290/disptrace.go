// Qualcomm QCM2290 display register write trace (diagnostics)
// https://github.com/usbarmory/tamago
//
// Copyright (c) the qcm2290 package authors
//
// Use of this source code is governed by the license
// that can be found in the LICENSE file.

package qcm2290

import (
	"sync/atomic"

	"github.com/usbarmory/tamago/internal/reg"
)

// The display trace records, in order and with a timestamp, every
// register write the display stack makes: MMIO writes of the clock,
// power, SMMU, DSI PHY, DSI controller and DPU code (including the GCC
// display branches and the SMMU table words), and every ANX7625 bridge
// register access (writes, page-switch dummy writes and reads). It is
// off by default and costs one atomic load per write while off.
//
// Recording neither allocates nor prints: entries go to a fixed array
// and recording stops when it is full (the first DisplayTraceSize
// accesses are kept, later ones only counted), so that a bring-up's
// start is never overwritten. MMIO reads and polls are not recorded.
//
// The shared clock and power helpers are also used by the camera code:
// its writes are recorded too while the trace is on.

// DisplayTraceSize is the number of trace entries kept.
const DisplayTraceSize = 8192

// TraceKind is the kind of a display trace entry.
type TraceKind uint8

// Display trace entry kinds
const (
	TraceMMIO     TraceKind = iota // MMIO write
	TraceI2CWrite                  // bridge register write
	TraceI2CPage                   // bridge page-switch dummy write (value 0)
	TraceI2CRead                   // bridge register read
)

// TraceEntry is one recorded access.
type TraceEntry struct {
	// Time is the system counter time in nanoseconds
	Time int64
	// Val is the value written (or read)
	Val uint64
	// Addr is the MMIO physical address, or the bridge page's 7-bit
	// I2C address
	Addr uint32
	Kind TraceKind
	// Reg is the bridge register (I2C kinds only)
	Reg uint8
}

var dispTrace struct {
	on   atomic.Bool
	n    atomic.Uint32
	lost atomic.Uint32
	buf  [DisplayTraceSize]TraceEntry
}

// TraceDisplay starts or stops recording; entries already recorded are
// kept.
func TraceDisplay(on bool) {
	dispTrace.on.Store(on)
}

// DisplayTracing reports whether the trace is recording.
func DisplayTracing() bool {
	return dispTrace.on.Load()
}

// ClearDisplayTrace discards the recorded entries.
func ClearDisplayTrace() {
	dispTrace.n.Store(0)
	dispTrace.lost.Store(0)
}

// DisplayTrace returns the recorded entries, in order (a view of the
// trace buffer, not a copy), and the number of accesses not recorded
// because the buffer was full.
func DisplayTrace() (entries []TraceEntry, lost uint32) {
	n := min(dispTrace.n.Load(), DisplayTraceSize)
	return dispTrace.buf[:n], dispTrace.lost.Load()
}

func traceRecord(k TraceKind, addr uint32, r uint8, v uint64) {
	if !dispTrace.on.Load() {
		return
	}

	if dispTrace.n.Load() >= DisplayTraceSize {
		dispTrace.lost.Add(1)
		return
	}

	i := dispTrace.n.Add(1) - 1

	if i >= DisplayTraceSize {
		dispTrace.lost.Add(1)
		return
	}

	dispTrace.buf[i] = TraceEntry{
		Time: nanotime(),
		Addr: addr,
		Val:  v,
		Kind: k,
		Reg:  r,
	}
}

// traceWrite is reg.Write, recorded in the display trace (before the
// write, so that a write which hangs the bus is the last entry).
func traceWrite(addr uint32, val uint32) {
	traceRecord(TraceMMIO, addr, 0, uint64(val))
	reg.Write(addr, val)
}

// traceWrite64 is reg.Write64, recorded in the display trace.
func traceWrite64(addr uint64, val uint64) {
	traceRecord(TraceMMIO, uint32(addr), 0, val)
	reg.Write64(addr, val)
}

// traceI2C records a bridge register access.
func traceI2C(k TraceKind, dev, r, v uint8) {
	traceRecord(k, uint32(dev), r, uint64(v))
}
