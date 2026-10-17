// Qualcomm QCM2290 TLMM GPIO driver
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

// TLMM (Top Level Mode Multiplexer) registers
// (drivers/pinctrl/qcom/pinctrl-qcm2290.c)
const (
	TLMM_BASE = 0x00500000

	// per-GPIO register group stride
	tlmmStride = 0x1000

	// GPIO_CFG
	GPIO_CFG    = 0x00
	CFG_OE      = 9
	CFG_DRV     = 6 // (n+1)*2 mA
	CFG_FUNC    = 2 // 0 = GPIO
	CFG_PULL    = 0
	PULL_NONE   = 0b00
	PULL_DOWN   = 0b01
	PULL_KEEPER = 0b10
	PULL_UP     = 0b11

	// GPIO_IN_OUT
	GPIO_IN_OUT = 0x04
	GPIO_IN     = 0
	GPIO_OUT    = 1

	// TLMM pin count (gpio-ranges in the platform device tree)
	tlmmPins = 127
)

// GPIO instance
type GPIO struct {
	num  int
	base uint32
}

// NewGPIO returns a TLMM GPIO instance for the argument pin.
func NewGPIO(num int) (*GPIO, error) {
	if num < 0 || num >= tlmmPins {
		return nil, errors.New("invalid GPIO number")
	}

	return &GPIO{
		num:  num,
		base: TLMM_BASE + uint32(num)*tlmmStride,
	}, nil
}

// SelectFunction selects the pin function (0 = GPIO, alternates per the
// platform pin assignment tables).
func (gpio *GPIO) SelectFunction(fn uint32) {
	reg.SetN(gpio.base+GPIO_CFG, CFG_FUNC, 0xf, fn)
}

// Function returns the selected pin function.
func (gpio *GPIO) Function() uint32 {
	return reg.GetN(gpio.base+GPIO_CFG, CFG_FUNC, 0xf)
}

// SetPull configures the pin bias (PULL_NONE, PULL_DOWN, PULL_KEEPER,
// PULL_UP).
func (gpio *GPIO) SetPull(pull uint32) {
	reg.SetN(gpio.base+GPIO_CFG, CFG_PULL, 0b11, pull)
}

// SetDrive sets the pin drive strength in mA (2 to 16, even values; the
// field holds mA/2 - 1).
func (gpio *GPIO) SetDrive(mA int) {
	if mA < 2 {
		mA = 2
	}

	if mA > 16 {
		mA = 16
	}

	reg.SetN(gpio.base+GPIO_CFG, CFG_DRV, 0b111, uint32(mA/2-1))
}

// Out configures the pin as a GPIO output.
func (gpio *GPIO) Out() {
	gpio.SelectFunction(0)
	reg.Set(gpio.base+GPIO_CFG, CFG_OE)
}

// In configures the pin as a GPIO input.
func (gpio *GPIO) In() {
	gpio.SelectFunction(0)
	reg.Clear(gpio.base+GPIO_CFG, CFG_OE)
}

// High sets the output level high.
func (gpio *GPIO) High() {
	reg.Set(gpio.base+GPIO_IN_OUT, GPIO_OUT)
}

// Low sets the output level low.
func (gpio *GPIO) Low() {
	reg.Clear(gpio.base+GPIO_IN_OUT, GPIO_OUT)
}

// Value returns the pin input level.
func (gpio *GPIO) Value() bool {
	return reg.Get(gpio.base+GPIO_IN_OUT, GPIO_IN)
}
