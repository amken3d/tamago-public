// Qualcomm QCM2290 Camera Control Interface (CCI) I2C master (polled)
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
	"sync"
	"time"

	"github.com/usbarmory/tamago/internal/reg"
)

// The CCI is a two-master I2C controller of the camera subsystem: on
// the UNO Q with the Media Carrier, master 0 reaches the carrier's
// DISPLAY connector (panel controller, touch), its I/O expander and
// CAMERA0, master 1 the second camera connector.
//
// Written from bmx's docs/display-spec.md §3; see
// docs/display-provenance.md.

// CCI_BASE is the CCI register base (§1).
const CCI_BASE = 0x05c1b000

// GCC clocks and power of the CCI (§3.1)
const (
	gccCameraAHBCBCR  = 0x17008
	gccCameraXOCBCR   = 0x17028
	gccCAMSSTopGDSC   = 0x58004
	gccCAMSSTopAHBCmd = 0x58010
	gccCAMSSTopAHB    = 0x58028
	gccCCICmdRCGR     = 0x56000
	gccCCICBCR        = 0x56018

	cciCfgXO = 0x001 // crystal, divide by 1

	// TLMM function of the CCI pins
	cciPinFunction = 1
)

// cciPins are the SDA/SCL pins of master 0 and master 1 (§3.1).
var cciPins = [2][2]int{{22, 23}, {29, 30}}

// CCI registers (§3.2)
const (
	cciHWVersion     = 0x000
	cciResetCmd      = 0x004
	cciQueueStart    = 0x008
	cciHaltReq       = 0x034
	cciIRQGlobalClr  = 0xc00
	cciIRQMask0      = 0xc04
	cciIRQClear0     = 0xc08
	cciIRQStatus0    = 0xc0c
	cciMasterStride  = 0x100
	cciSCLCtl        = 0x100
	cciSDACtl0       = 0x104
	cciSDACtl1       = 0x108
	cciSDACtl2       = 0x10c
	cciMiscCtl       = 0x110
	cciReadData      = 0x118
	cciReadBufLevel  = 0x11c
	cciQueueBase     = 0x300
	cciQueueMStride  = 0x200
	cciQueueQStride  = 0x100
	cciExecWordCnt   = 0x0
	cciCurWordCnt    = 0x4
	cciLoadData      = 0x10
	cciResetAll      = 0x0f73f3f7
	cciIRQResetDone  = 1 << 24
	cciIRQHaltAck0   = 1 << 25
	cciIRQErrMask0   = 0x18000ee6
	cciIRQErrMask1   = 0x60ee6000
	cciIRQReadDone0  = 1 << 0
	cciIRQQ0Report0  = 1 << 4
	cciIRQQ1Report0  = 1 << 8
	cciMaster1Shift  = 12
	cciReadQueue     = 1
	cciWriteQueue    = 0
	cciCmdSetParam   = 1
	cciCmdReport     = 8 | 1<<8
	cciCmdWrite      = 9
	cciCmdRead       = 10
	cciMaxWrite      = 11
	cciTransferLimit = 50 * time.Millisecond
)

// cciMasterReset is the RESET_CMD mask of each master alone.
var cciMasterReset = [2]uint32{0x3f1, 0x3f001}

// CCIMaxRead is the largest read one transfer returns (§3.3).
const CCIMaxRead = 12

// Standard-mode timing counters (§3.2), the same for both masters.
var cciTiming = [5]uint32{
	201<<16 | 174, // SCL_CTL
	204<<16 | 231, // SDA_CTL_0
	22<<16 | 162,  // SDA_CTL_1
	227,           // SDA_CTL_2
	0<<8 | 6<<4 | 3,
}

// EnableCCIPower clocks and powers the CCI and muxes its pins (§3.1):
// camera AHB and XO branches, the CAMSS top GDSC, the CAMSS top AHB and
// the CCI core clock (19.2 MHz crystal). It must precede any CCI or
// CAMSS register access.
//
// The camera AHB branch reads "off" (0x8000000b) while running, like the
// GCC display branches of §5.1 (observed under bmx 2026-10-16): the two
// camera branches are enabled without waiting on their status.
func EnableCCIPower() error {
	clkOnAdvisory(GCC_BASE, gccCameraAHBCBCR, cbcrEnable)
	clkOnAdvisory(GCC_BASE, gccCameraXOCBCR, cbcrEnable)

	if err := gdscOn(GCC_BASE, gccCAMSSTopGDSC); err != nil {
		return fmt.Errorf("camss top: %v", err)
	}

	if err := rcgUpdate(GCC_BASE, gccCAMSSTopAHBCmd, cciCfgXO); err != nil {
		return fmt.Errorf("camss top ahb: %v", err)
	}

	if err := dispClkOn(GCC_BASE, gccCAMSSTopAHB); err != nil {
		return fmt.Errorf("camss top ahb: %v", err)
	}

	if err := rcgUpdate(GCC_BASE, gccCCICmdRCGR, cciCfgXO); err != nil {
		return fmt.Errorf("cci: %v", err)
	}

	if err := dispClkOn(GCC_BASE, gccCCICBCR); err != nil {
		return fmt.Errorf("cci: %v", err)
	}

	for _, m := range cciPins {
		for _, pin := range m {
			g, err := NewGPIO(pin)

			if err != nil {
				return err
			}

			g.SelectFunction(cciPinFunction)
			g.SetPull(PULL_NONE)
		}
	}

	return nil
}

// CCI represents the Camera Control Interface. Every transfer holds an
// internal lock, so concurrent callers see whole transfers.
type CCI struct {
	Base uint32

	mu sync.Mutex
}

// Version returns the hardware version (0x10020009 on this part).
func (c *CCI) Version() uint32 {
	return reg.Read(c.Base + cciHWVersion)
}

// Status returns IRQ_STATUS_0, which latches events whether or not the
// interrupt is routed.
func (c *CCI) Status() uint32 {
	return reg.Read(c.Base + cciIRQStatus0)
}

func cciDoneMask(m, q int) uint32 {
	if q == cciReadQueue {
		return cciIRQReadDone0 << (cciMaster1Shift * m)
	}

	return cciIRQQ0Report0 << (cciMaster1Shift * m)
}

func cciErrMask(m int) uint32 {
	if m == 0 {
		return cciIRQErrMask0
	}

	return cciIRQErrMask1
}

func (c *CCI) clear(v uint32) {
	reg.Write(c.Base+cciIRQClear0, v)
	reg.Write(c.Base+cciIRQGlobalClr, 1)
}

// waitIRQ polls IRQ_STATUS_0 for any of mask, clears what was seen and
// returns it.
func (c *CCI) waitIRQ(mask uint32, timeout time.Duration) (uint32, bool) {
	start := time.Now()

	for {
		if v := reg.Read(c.Base + cciIRQStatus0); v&mask != 0 {
			c.clear(v)
			return v, true
		}

		if time.Since(start) > timeout {
			return reg.Read(c.Base + cciIRQStatus0), false
		}
	}
}

func (c *CCI) setTiming(m int) {
	base := c.Base + uint32(m)*cciMasterStride

	for i, v := range cciTiming {
		reg.Write(base+cciSCLCtl+uint32(i)*4, v)
	}
}

// Init unmasks the completion, error, reset and halt events, resets the
// controller and programs both masters' timing (§3.3).
func (c *CCI) Init() error {
	c.mu.Lock()
	defer c.mu.Unlock()

	mask := uint32(cciIRQResetDone | cciIRQHaltAck0 | cciIRQHaltAck0<<1)

	for m := 0; m < 2; m++ {
		mask |= cciDoneMask(m, cciReadQueue) | cciDoneMask(m, cciWriteQueue) |
			cciIRQQ1Report0<<(cciMaster1Shift*m) | cciErrMask(m)
	}

	reg.Write(c.Base+cciIRQMask0, mask)
	c.clear(0xffffffff)

	reg.Write(c.Base+cciResetCmd, cciResetAll)

	if _, ok := c.waitIRQ(cciIRQResetDone, cciTransferLimit); !ok {
		return fmt.Errorf("CCI reset timeout (status %#x)", c.Status())
	}

	c.setTiming(0)
	c.setTiming(1)

	return nil
}

// recover halts and resets one master after a NACK or error (§3.3).
func (c *CCI) recover(m int) {
	reg.Write(c.Base+cciHaltReq, 1<<m)
	c.waitIRQ(cciIRQHaltAck0<<m, cciTransferLimit)

	reg.Write(c.Base+cciResetCmd, cciMasterReset[m])
	c.waitIRQ(cciIRQResetDone, cciTransferLimit)

	c.setTiming(m)
}

// run loads command words into queue q of master m, executes them and
// waits for completion.
func (c *CCI) run(m, q int, words []uint32) error {
	qb := c.Base + cciQueueBase + uint32(m)*cciQueueMStride + uint32(q)*cciQueueQStride

	for _, w := range words {
		reg.Write(qb+cciLoadData, w)
	}

	reg.Write(qb+cciExecWordCnt, reg.Read(qb+cciCurWordCnt))
	reg.Write(c.Base+cciQueueStart, 1<<(2*m+q))

	errs := cciErrMask(m)
	v, ok := c.waitIRQ(cciDoneMask(m, q)|errs, cciTransferLimit)

	switch {
	case !ok:
		c.recover(m)
		return fmt.Errorf("timeout (status %#x)", v)
	case v&errs&^cciDoneMask(m, q) != 0:
		c.recover(m)

		// NACK bits: master 0 queue 0/1 bits 27/28, master 1 29/30
		if v&(3<<(27+2*m)) != 0 {
			return errors.New("NACK")
		}

		return fmt.Errorf("bus error (status %#x)", v)
	}

	return nil
}

func checkMaster(m int) error {
	if m != 0 && m != 1 {
		return errors.New("CCI master must be 0 or 1")
	}

	return nil
}

// Write transmits 1-11 bytes to a 7-bit address on master m, through
// queue 0: SET_PARAM, WRITE with the data, REPORT.
func (c *CCI) Write(m int, addr uint8, data []byte) error {
	if err := checkMaster(m); err != nil {
		return err
	}

	n := len(data)

	if n < 1 || n > cciMaxWrite {
		return fmt.Errorf("CCI write length must be 1-%d", cciMaxWrite)
	}

	// the WRITE command byte and the data, four bytes per word, least
	// significant byte first
	b := append([]byte{byte(cciCmdWrite | n<<4)}, data...)
	words := []uint32{cciCmdSetParam | uint32(addr&0x7f)<<4}

	for i := 0; i < len(b); i += 4 {
		var w uint32

		for j := 0; j < 4 && i+j < len(b); j++ {
			w |= uint32(b[i+j]) << (8 * j)
		}

		words = append(words, w)
	}

	words = append(words, cciCmdReport)

	c.mu.Lock()
	defer c.mu.Unlock()

	return c.run(m, cciWriteQueue, words)
}

// Read receives 1-12 bytes from a 7-bit address on master m, through
// queue 1: SET_PARAM, READ. On a freshly initialised master bare reads
// NACK until one write has completed on it (§3.3).
func (c *CCI) Read(m int, addr uint8, buf []byte) error {
	if err := checkMaster(m); err != nil {
		return err
	}

	n := len(buf)

	if n < 1 || n > CCIMaxRead {
		return fmt.Errorf("CCI read length must be 1-%d", CCIMaxRead)
	}

	c.mu.Lock()
	defer c.mu.Unlock()

	words := []uint32{
		cciCmdSetParam | uint32(addr&0x7f)<<4,
		cciCmdRead | uint32(n)<<4,
	}

	if err := c.run(m, cciReadQueue, words); err != nil {
		return err
	}

	mb := c.Base + uint32(m)*cciMasterStride
	want := n/4 + 1

	if lvl := int(reg.Read(mb + cciReadBufLevel)); lvl < want {
		return fmt.Errorf("read buffer holds %d words, want %d", lvl, want)
	}

	// the first byte is not data; the n data bytes follow, least
	// significant byte first
	i := -1

	for w := 0; w < want; w++ {
		v := reg.Read(mb + cciReadData)

		for j := 0; j < 4; j, i = j+1, i+1 {
			if i >= 0 && i < n {
				buf[i] = byte(v >> (8 * j))
			}
		}
	}

	// residue would shift the next read's data
	for lvl := reg.Read(mb + cciReadBufLevel); lvl > 0 && lvl < 64; lvl-- {
		reg.Read(mb + cciReadData)
	}

	return nil
}

// ReadReg writes a one-byte register address, then reads len(buf)
// bytes: the queue has no combined repeated-start transfer, so this is
// two transfers with a stop between them.
func (c *CCI) ReadReg(m int, addr, regAddr uint8, buf []byte) error {
	if err := c.Write(m, addr, []byte{regAddr}); err != nil {
		return err
	}

	return c.Read(m, addr, buf)
}
