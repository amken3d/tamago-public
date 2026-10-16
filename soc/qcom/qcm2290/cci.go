// Qualcomm Camera Control Interface (CCI) I2C master driver (polled)
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

	"github.com/usbarmory/tamago/internal/reg"
)

// CCI registers (drivers/i2c/busses/i2c-qcom-cci.c, cci_v2_data)
const (
	CCI_BASE = 0x05c1b000

	CCI_HW_VERSION     = 0x000
	CCI_RESET_CMD      = 0x004
	CCI_RESET_CMD_MASK = 0x0f73f3f7
	CCI_QUEUE_START    = 0x008
	CCI_HALT_REQ       = 0x034

	CCI_IRQ_GLOBAL_CLEAR_CMD = 0xc00
	CCI_IRQ_MASK_0           = 0xc04
	CCI_IRQ_CLEAR_0          = 0xc08
	CCI_IRQ_STATUS_0         = 0xc0c

	// CCI_IRQ_STATUS_0 / CCI_IRQ_MASK_0, master 0 (master 1 is the
	// same layout shifted by 12, see cciMasterIRQ)
	CCI_M0_RD_DONE   = 1 << 0
	CCI_M0_Q0_REPORT = 1 << 4
	CCI_M0_Q1_REPORT = 1 << 8
	CCI_RST_DONE_ACK = 1 << 24
	CCI_M0_HALT_ACK  = 1 << 25
	CCI_M1_HALT_ACK  = 1 << 26
	CCI_M0_Q0_NACK   = 1 << 27
	CCI_M0_Q1_NACK   = 1 << 28
	CCI_M1_Q0_NACK   = 1 << 29
	CCI_M1_Q1_NACK   = 1 << 30
	CCI_M0_ERROR     = 0x18000ee6
	CCI_M1_ERROR     = 0x60ee6000

	// queue command words
	CCI_I2C_SET_PARAM     = 1
	CCI_I2C_REPORT        = 8
	CCI_I2C_WRITE         = 9
	CCI_I2C_READ          = 10
	CCI_I2C_REPORT_IRQ_EN = 1 << 8

	// transfer limits (cci_v2_data quirks)
	CCIMaxWrite = 11
	CCIMaxRead  = 12

	cciQueueWrite = 0
	cciQueueRead  = 1

	// cciPollSpins bounds status polling, sized like i2cPollSpins: a
	// 12-byte read at the ~50 kHz XO-derived SCL is ~2.5 ms
	cciPollSpins = 400000
)

// per-master register offsets
func cciSCLCtl(m int) uint32       { return uint32(0x100 + 0x100*m) }
func cciSDACtl0(m int) uint32      { return uint32(0x104 + 0x100*m) }
func cciSDACtl1(m int) uint32      { return uint32(0x108 + 0x100*m) }
func cciSDACtl2(m int) uint32      { return uint32(0x10c + 0x100*m) }
func cciMiscCtl(m int) uint32      { return uint32(0x110 + 0x100*m) }
func cciReadData(m int) uint32     { return uint32(0x118 + 0x100*m) }
func cciReadBufLevel(m int) uint32 { return uint32(0x11c + 0x100*m) }

// per-master, per-queue register offsets
func cciExecWordCnt(m, q int) uint32 { return uint32(0x300 + 0x200*m + 0x100*q) }
func cciCurWordCnt(m, q int) uint32  { return uint32(0x304 + 0x200*m + 0x100*q) }
func cciLoadData(m, q int) uint32    { return uint32(0x310 + 0x200*m + 0x100*q) }

// GCC camera subsystem clocks and power (drivers/clk/qcom/gcc-qcm2290.c)
const (
	gccCameraAHBCBCR = 0x17008 // critical (always-on) in Linux
	gccCameraXOCBCR  = 0x17028 // critical (always-on) in Linux

	gccCAMSSTopGDSCR      = 0x58004
	gccCAMSSTopAHBCmdRCGR = 0x58010
	gccCAMSSTopAHBCBCR    = 0x58028

	gccCAMSSCCICmdRCGR = 0x56000
	gccCAMSSCCI0CBCR   = 0x56018
)

// CCI master pins: SDA, SCL (TLMM function 1 = cci_i2c)
var cciPins = [2][2]int{
	{22, 23},
	{29, 30},
}

const cciPinFunc = 1

// cciStandard holds the 100 kHz standard-mode timing counters of
// cci_v2_data. They are specified against a 37.5 MHz CCI clock; this
// driver clocks the CCI from the 19.2 MHz crystal instead (no GPLL0
// post-divider dependency), which scales SCL to ~51 kHz with every
// setup and hold time proportionally longer -- slower, never out of
// spec.
var cciStandard = struct {
	thigh, tlow, tsuSto, tsuSta, thdDat, thdSta, tbuf uint32
	sclStretch, trdhld, tsp                           uint32
}{201, 174, 204, 231, 22, 162, 227, 0, 6, 3}

// EnableCCIPower brings up the camera subsystem top power domain and
// the clocks the CCI needs, and muxes the I2C pins of both masters.
// It must precede any CCI register access: an access to the unclocked
// block hangs the bus.
func EnableCCIPower() error {
	// the camera AHB/XO branches Linux keeps permanently on
	reg.Set(GCC_BASE+gccCameraAHBCBCR, branchEnable)
	reg.Set(GCC_BASE+gccCameraXOCBCR, branchEnable)

	// CAMSS top GDSC: clear the software collapse bit, wait for power
	reg.Clear(GCC_BASE+gccCAMSSTopGDSCR, gdscSWCollapse)

	for i := 0; ; i++ {
		if reg.Get(GCC_BASE+gccCAMSSTopGDSCR, gdscPowerOn) {
			break
		}

		if i >= clkPollSpins {
			return errors.New("CAMSS top GDSC stuck off")
		}
	}

	// CAMSS top AHB (register interface) and CCI core, both XO/1
	if err := rcgUpdate(GCC_BASE, gccCAMSSTopAHBCmdRCGR, cfgXODiv1); err != nil {
		return errors.New("camss top ahb rcg: " + err.Error())
	}

	if err := dispClkOn(GCC_BASE, gccCAMSSTopAHBCBCR); err != nil {
		return errors.New("camss top ahb: " + err.Error())
	}

	if err := rcgUpdate(GCC_BASE, gccCAMSSCCICmdRCGR, cfgXODiv1); err != nil {
		return errors.New("cci rcg: " + err.Error())
	}

	if err := dispClkOn(GCC_BASE, gccCAMSSCCI0CBCR); err != nil {
		return errors.New("cci clk: " + err.Error())
	}

	for _, pins := range cciPins {
		for _, n := range pins {
			gpio, err := NewGPIO(n)

			if err != nil {
				return err
			}

			gpio.SetPull(PULL_NONE)
			gpio.SelectFunction(cciPinFunc)
		}
	}

	return nil
}

// CCI represents the Camera Control Interface, a two-master I2C
// controller driven through per-master command queues.
type CCI struct {
	Base uint32
}

// Version returns the CCI hardware version register, a safe first
// access to confirm the block is clocked and powered.
func (c *CCI) Version() uint32 {
	return reg.Read(c.Base + CCI_HW_VERSION)
}

// Status returns the raw interrupt status, for diagnostics.
func (c *CCI) Status() uint32 {
	return reg.Read(c.Base + CCI_IRQ_STATUS_0)
}

func (c *CCI) clear(val uint32) {
	reg.Write(c.Base+CCI_IRQ_CLEAR_0, val)
	reg.Write(c.Base+CCI_IRQ_GLOBAL_CLEAR_CMD, 1)
}

// poll waits for any of the mask bits in the interrupt status (which
// latches regardless of interrupt delivery) and clears what it saw.
func (c *CCI) poll(mask uint32) (status uint32, ok bool) {
	for i := 0; i < cciPollSpins; i++ {
		status = reg.Read(c.Base + CCI_IRQ_STATUS_0)

		if status&mask != 0 {
			c.clear(status)
			return status, true
		}
	}

	return status, false
}

// Init resets the controller and programs standard-mode timing on both
// masters. EnableCCIPower must have been called.
func (c *CCI) Init() error {
	// unmask what the polls below wait on, as Linux cci_init does
	m0, _, e0 := cciMasterIRQ(0)
	m1, _, e1 := cciMasterIRQ(1)
	reg.Write(c.Base+CCI_IRQ_MASK_0, m0|m1|e0|e1|CCI_RST_DONE_ACK|CCI_M0_HALT_ACK|CCI_M1_HALT_ACK)
	c.clear(0xffffffff)

	reg.Write(c.Base+CCI_RESET_CMD, CCI_RESET_CMD_MASK)

	if _, ok := c.poll(CCI_RST_DONE_ACK); !ok {
		return errors.New("CCI reset timeout")
	}

	for m := 0; m < 2; m++ {
		c.timing(m)
	}

	return nil
}

func (c *CCI) timing(m int) {
	t := cciStandard

	reg.Write(c.Base+cciSCLCtl(m), t.thigh<<16|t.tlow)
	reg.Write(c.Base+cciSDACtl0(m), t.tsuSto<<16|t.tsuSta)
	reg.Write(c.Base+cciSDACtl1(m), t.thdDat<<16|t.thdSta)
	reg.Write(c.Base+cciSDACtl2(m), t.tbuf)
	reg.Write(c.Base+cciMiscCtl(m), t.sclStretch<<8|t.trdhld<<4|t.tsp)
}

// cciMasterIRQ returns the completion and error status bits of a
// master: master 1's RD_DONE/REPORT bits sit 12 above master 0's.
func cciMasterIRQ(m int) (done, nack, errs uint32) {
	done = uint32(CCI_M0_RD_DONE|CCI_M0_Q0_REPORT|CCI_M0_Q1_REPORT) << (12 * m)

	if m == 0 {
		return done, CCI_M0_Q0_NACK | CCI_M0_Q1_NACK, CCI_M0_ERROR
	}

	return done, CCI_M1_Q0_NACK | CCI_M1_Q1_NACK, CCI_M1_ERROR
}

// recover halts and resets a master after an error, as the Linux ISR
// does (HALT_REQ -> HALT_ACK -> per-master RESET_CMD), then restores
// its timing.
func (c *CCI) recover(m int) {
	haltAck := uint32(CCI_M0_HALT_ACK)
	resetMask := uint32(0x000003f1)

	if m == 1 {
		haltAck = CCI_M1_HALT_ACK
		resetMask = 0x0003f001
	}

	reg.Write(c.Base+CCI_HALT_REQ, 1<<m)
	c.poll(haltAck)

	reg.Write(c.Base+CCI_RESET_CMD, resetMask)
	c.poll(CCI_RST_DONE_ACK)

	c.timing(m)
}

// run executes the words loaded into a queue and waits for the report.
func (c *CCI) run(m, q int) error {
	reg.Write(c.Base+cciExecWordCnt(m, q), reg.Read(c.Base+cciCurWordCnt(m, q)))
	reg.Write(c.Base+CCI_QUEUE_START, 1<<(m*2+q))

	done, nack, errs := cciMasterIRQ(m)
	status, ok := c.poll(done | errs)

	switch {
	case status&nack != 0:
		c.recover(m)
		return errors.New("NACK")
	case status&errs != 0:
		c.recover(m)
		return fmt.Errorf("bus error (status %#x)", status)
	case !ok:
		c.recover(m)
		return errors.New("timeout")
	}

	return nil
}

func (c *CCI) load(m, q int, val uint32) {
	reg.Write(c.Base+cciLoadData(m, q), val)
}

// Write transmits buf (1-11 bytes) to a 7-bit slave address on master
// m, ending with a stop.
func (c *CCI) Write(m int, addr uint8, buf []byte) error {
	n := len(buf)

	if n == 0 || n > CCIMaxWrite {
		return fmt.Errorf("write length must be 1-%d bytes", CCIMaxWrite)
	}

	q := cciQueueWrite

	c.load(m, q, CCI_I2C_SET_PARAM|uint32(addr&0x7f)<<4)

	// the WRITE command byte and its payload are packed four bytes per
	// queue word, little-endian
	load := make([]byte, (n+1+3)&^3)
	load[0] = byte(CCI_I2C_WRITE | n<<4)
	copy(load[1:], buf)

	for i := 0; i < len(load); i += 4 {
		c.load(m, q, uint32(load[i])|uint32(load[i+1])<<8|uint32(load[i+2])<<16|uint32(load[i+3])<<24)
	}

	c.load(m, q, CCI_I2C_REPORT|CCI_I2C_REPORT_IRQ_EN)

	return c.run(m, q)
}

// Read receives len(buf) (1-12) bytes from a 7-bit slave address on
// master m, ending with a stop.
func (c *CCI) Read(m int, addr uint8, buf []byte) error {
	n := len(buf)

	if n == 0 || n > CCIMaxRead {
		return fmt.Errorf("read length must be 1-%d bytes", CCIMaxRead)
	}

	q := cciQueueRead

	c.load(m, q, CCI_I2C_SET_PARAM|uint32(addr&0x7f)<<4)
	c.load(m, q, CCI_I2C_READ|uint32(n)<<4)

	if err := c.run(m, q); err != nil {
		return err
	}

	words := reg.Read(c.Base + cciReadBufLevel(m))

	if want := uint32(n/4 + 1); words != want {
		return fmt.Errorf("read buffer holds %d words, want %d", words, want)
	}

	// the buffer leads with one byte that is not data (the LS byte of
	// the first word), then the bytes read, LS byte first
	idx := -1

	for ; words > 0; words-- {
		val := reg.Read(c.Base + cciReadData(m))

		for i := 0; i < 4; i++ {
			if idx >= 0 && idx < n {
				buf[idx] = byte(val >> (8 * i))
			}

			idx++
		}
	}

	return nil
}

// ReadReg performs a register read: a one-byte register-address write,
// a stop, then an n-byte read. The CCI queue has no repeated-start
// write-then-read, as in Linux, which issues the same two transfers.
func (c *CCI) ReadReg(m int, addr, regAddr uint8, buf []byte) error {
	if err := c.Write(m, addr, []byte{regAddr}); err != nil {
		return err
	}

	return c.Read(m, addr, buf)
}
