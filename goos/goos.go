// Custom GOOS support
// https://github.com/usbarmory/tamago
//
// Copyright (c) The TamaGo Authors. All Rights Reserved.
//
// Use of this source code is governed by the license
// that can be found in the LICENSE file.

// Package goos describes required, as well as optional, runtime
// functions/variables for custom GOOS implementations as supported by the
// GOOSPKG variable for [runtime/goos] overlay.
//
// These hooks act as a "Rosetta Stone" for integration of a freestanding Go
// runtime within an arbitrary environment, whether bare metal or OS supported.
//
// For bare metal examples see the following packages: [usbarmory], [uefi],
// [microvm].
//
// For OS supported examples see the following tamago packages: [linux],
// [applet].
//
// This package is a stub and is only used for documentation purposes,
// applications need to define the described functions/variables or import them
// from external packages (such as the ones provided by [tamago]) relevant to
// the target environment.
//
// [tamago]: https://github.com/usbarmory/tamago
// [usbarmory]: https://github.com/usbarmory/tamago/tree/master/board/usbarmory
// [uefi]: https://github.com/usbarmory/go-boot/tree/main/uefi
// [microvm]: https://github.com/usbarmory/tamago/tree/master/board/firecracker/microvm
// [linux]: https://github.com/usbarmory/tamago/tree/master/user/linux
// [applet]: https://github.com/usbarmory/GoTEE/tree/master/applet
package goos

import "unsafe"

// Required variables.
var (
	// RamStart defines the start address of the physical or virtual memory
	// available to the runtime for allocation (including the code segment
	// which must be mapped within).
	RamStart uint

	// RamSize defines the total size of the physical or virtual memory
	// available to the runtime for allocation (including the code segment
	// which must be mapped within).
	RamSize uint

	// RamStackOffset, defines the negative offset from the end of the
	// available memory for stack allocation.
	RamStackOffset uint
)

// CPUInit handles immediate startup CPU initialization as it represents the
// first instruction set executed.
func CPUinit()

// Hwinit0 takes care of the lower level initialization triggered before
// runtime setup (pre World start).
//
// It must be defined using Go's Assembler to retain Go's commitment to
// backward compatibility, otherwise extreme care must be taken as the lack of
// World start does not allow memory allocation.
func Hwinit0()

// InitRNG initializes random number generation.
func InitRNG()

// GetRandomData generates len(b) random bytes and writes them into b.
func GetRandomData(b []byte)

// Nanotime returns the system time in nanoseconds.
//
// Before [Hwinit1] it must be defined using Go's Assembler to retain Go's
// commitment to backward compatibility, otherwise extreme care must be taken
// as the lack of World start does not allow memory allocation.
func Nanotime() int64

// Printk handles character printing to standard output.
//
// Before [Hwinit1] it must be defined using Go's Assembler to retain Go's
// commitment to backward compatibility, otherwise extreme care must be taken
// as the lack of World start does not allow memory allocation.
func Printk(c byte)

// Hwinit1 takes care of the lower level initialization triggered early in
// runtime setup (post World start).
func Hwinit1()

// NumCPU is the number of hardware processors the runtime should schedule
// across; it sets the initial GOMAXPROCS. It defaults to 1 (uniprocessor)
// and must be set before osinit (e.g. in [Hwinit0]) by SMP-capable
// platforms, which must also provide [Task] (and typically [ProcID] and
// [Wake]) to start and coordinate the additional processors.
var NumCPU int32 = 1

// Optional variables/functions.
var (
	// Bloc is an optional variable which can be set to redefine the heap
	// memory start address, this is typically only required on OS
	// supported environments.
	Bloc uintptr

	// Exit is an optional function which can be set to override default
	// runtime termination.
	Exit func(code int32)

	// Idle is an optional function which can be set to implement CPU idle
	// time management.
	Idle func(until int64)

	// ProcID is an optional function which can be set to provide the
	// processor identifier for tracing purposes.
	ProcID func() uint64

	// Task is an optional function which can be set to provide an
	// implementation for HW/OS threading (e.g. [runtime.newosproc]).
	Task func(sp, mp, gp, fn unsafe.Pointer)

	// Wake is an optional function which can be set to provide a wake up
	// call for halted processors as reported by [ProcID], it must be
	// defined as required to handle [Idle] implementations which halt a
	// processor.
	Wake func(procid uint64)

	// PreemptM is an optional function which can be set to deliver an
	// asynchronous preemption request (an inter-processor interrupt) to
	// the processor identified by procid, as reported by [ProcID]. The
	// receiving processor's interrupt handler is expected to notice the
	// pending request (see [runtime.preemptM]) and bring the interrupted
	// goroutine to a safe point. Leaving it unset keeps preemption
	// cooperative-only, the single-processor behaviour.
	//
	// The implementation MUST be nosplit-safe: the runtime calls it from
	// the interrupt handler's time-slice fan-out, on the exception stack.
	PreemptM func(procid uint64)

	// AsyncPreempt arms the trap-frame preemption tier: with it set, the
	// interrupt handler redirects an interrupted goroutine through
	// runtime.asyncPreempt at async-safe points, reaching even loops that
	// make no calls. Leave unset until the platform's [PreemptM] delivery
	// is proven; the cooperative poison tier works without it.
	AsyncPreempt bool

	// SchedTick, when set, points at a counter an ordinary goroutine
	// advances continuously. It arms the runtime's interrupt-context
	// deadman (see runtime.tamagoDeadmanCheck): interrupts keep firing
	// when the scheduler wedges, so a long stretch of interrupts with no
	// counter advance triggers a lock-free scheduler-state dump. A
	// bring-up diagnostic; leave nil in normal operation.
	SchedTick *uint32

	// RawTicks, when set, returns a free-running microsecond-class
	// counter read with zero locks -- the deadman's clock, so its
	// starvation window is measured in time rather than interrupt
	// counts (which race ahead by orders of magnitude in an interrupt
	// storm). Same contract as RawPutc: MUST be nosplit-safe.
	RawTicks func() uint32

	// RawPutc, when set, writes one byte to the console bypassing every
	// lock and buffer -- the deadman's output path, usable when any lock
	// may be held by a dead core. It MUST be nosplit-safe: a frameless
	// MMIO poll-and-write, no allocation, no locks.
	RawPutc func(c byte)

	// IRQAck, when set, is called by the interrupt handler on the
	// relay-model core before signalling the service goroutine: the
	// platform acknowledges the sources it can handle entirely in place
	// (a periodic tick, a wake doorbell) and returns true if NOTHING is
	// left pending, in which case the handler returns with the interrupt
	// mask untouched and the service goroutine is not involved. This
	// keeps the tick -- and the time-slice fan-out riding it -- alive
	// through a stopped world, when the service goroutine cannot run.
	// Returning false takes the classic path: relay, and return masked
	// until serviced. MUST be nosplit-safe.
	IRQAck func() bool

	// DeadmanHook, when set, is called at the end of the runtime deadman's
	// dump so the platform can append its own evidence (exception
	// breadcrumbs, interrupt counters, trapped console buffers). Same
	// contract as RawPutc: nosplit-safe, no locks, no allocation.
	DeadmanHook func()

	// Write, when set, replaces per-byte [Printk] for console output:
	// the runtime delivers each write(2) payload whole. A multiprocessor
	// platform can then serialize the console at write granularity --
	// whole fmt.Printf payloads stay contiguous under a lock no matter
	// which processor carries the writing goroutine -- where byte
	// granularity would interleave concurrent writers mid-line. Output
	// printed before the platform assigns the variable still arrives
	// through [Printk] one byte at a time.
	//
	// The runtime calls it from nosplit context, on arbitrary
	// processors, and during panic: it must not allocate, must not
	// depend on the scheduler, and must tolerate concurrent callers.
	// p aliases runtime stack memory and MUST NOT be retained after
	// the call returns.
	Write func(p []byte)
)
