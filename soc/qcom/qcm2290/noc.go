// Qualcomm QCM2290 interconnect QoS (system NoC and BIMC master ports)
// https://github.com/usbarmory/tamago
//
// Copyright (c) the qcm2290 package authors
//
// Use of this source code is governed by the license
// that can be found in the LICENSE file.

package qcm2290

import (
	"fmt"

	"github.com/usbarmory/tamago/internal/reg"
)

// Bus arbitration for DMA masters: the system NoC's per-master default
// priority, and the DDR controller's (BIMC) per-master QoS blocks.
//
// Written from bmx's docs/usb-spec.md §7 (see docs/usb-provenance.md).
// Only the SNOC ports §7.1 lists are implemented in the hardware: an
// access to any other port's register hangs the bus until the watchdog
// resets the SoC, so the helpers below refuse every port not in that
// table rather than trusting their callers.

const (
	// SNOC master QoS control register of port n: SNOC_QOS_BASE + n x
	// SNOC_QOS_STRIDE (§7.1)
	SNOC_QOS_BASE   = 0x01895008
	SNOC_QOS_STRIDE = 0x1000

	// SNOC QoS control fields
	SNOCPrioShift = 4
	SNOCPrioMask  = 0x7 << SNOCPrioShift // default (fixed) priority, 0-7
	SNOCUrgFwd    = 1 << 3               // urgency forwarding enable

	// BIMC master QoS block of port n: BIMC_QOS_BASE + n x
	// BIMC_QOS_STRIDE (§7.2)
	BIMC_QOS_BASE   = 0x04488300
	BIMC_QOS_STRIDE = 0x4000

	bimcBKEEn   = 0x00
	bimcHealth0 = 0x40
)

// SNOC master ports (§7.1).
const (
	SNOCPortQUP0      = 0
	SNOCPortQPIC      = 1
	SNOCPortQDSSBAM   = 2
	SNOCPortIPA       = 3
	SNOCPortCamNRT    = 4
	SNOCPortMDP0      = 5
	SNOCPortTIC       = 8
	SNOCPortVideoP0   = 9
	SNOCPortCamRT     = 10
	SNOCPortQDSSETR   = 12
	SNOCPortVideoProc = 13
	SNOCPortSDCC1     = 17
	SNOCPortPIMEM     = 20
	SNOCPortCrypto    = 22
	SNOCPortSDCC2     = 23
	SNOCPortUSB3      = 24
)

// SNOCPorts names every implemented SNOC master port, in port order.
var SNOCPorts = []struct {
	Port int
	Name string
}{
	{SNOCPortQUP0, "QUP 0"},
	{SNOCPortQPIC, "QPIC"},
	{SNOCPortQDSSBAM, "QDSS BAM"},
	{SNOCPortIPA, "IPA"},
	{SNOCPortCamNRT, "camera NoC (non-real-time)"},
	{SNOCPortMDP0, "MDP0 (display DPU)"},
	{SNOCPortTIC, "TIC"},
	{SNOCPortVideoP0, "video P0"},
	{SNOCPortCamRT, "camera NoC (real-time)"},
	{SNOCPortQDSSETR, "QDSS ETR"},
	{SNOCPortVideoProc, "video processor"},
	{SNOCPortSDCC1, "SDCC 1"},
	{SNOCPortPIMEM, "PIMEM"},
	{SNOCPortCrypto, "crypto"},
	{SNOCPortSDCC2, "SDCC 2"},
	{SNOCPortUSB3, "USB 3.0 controller"},
}

// QoS values applied by the drivers (§7.1 recommendations / Linux values).
const (
	SNOCQoSUSB     = 0x70 // priority 7: no isochronous misses with the panel live
	SNOCQoSDisplay = 0x38 // priority 3, urgency forwarding
	SNOCQoSCamNRT  = 0x30 // priority 3
	SNOCQoSCamRT   = 0x38 // priority 3, urgency forwarding
)

func snocImplemented(port int) bool {
	for _, p := range SNOCPorts {
		if p.Port == port {
			return true
		}
	}

	return false
}

func snocAddr(port int) (uint32, error) {
	if !snocImplemented(port) {
		return 0, fmt.Errorf("SNOC port %d is not implemented (an access would hang the bus)", port)
	}

	return SNOC_QOS_BASE + uint32(port)*SNOC_QOS_STRIDE, nil
}

// SNOCQoS reads an implemented SNOC master port's QoS control register.
func SNOCQoS(port int) (uint32, error) {
	addr, err := snocAddr(port)

	if err != nil {
		return 0, err
	}

	return reg.Read(addr), nil
}

// SetSNOCQoS sets an implemented SNOC master port's default priority
// (bits 6:4) and urgency forwarding (bit 3) from val, keeping the
// register's other bits: the priority field first, then the urgency bit
// (the order of §7.1).
func SetSNOCQoS(port int, val uint32) error {
	addr, err := snocAddr(port)

	if err != nil {
		return err
	}

	v := reg.Read(addr)
	v = v&^SNOCPrioMask | val&SNOCPrioMask
	traceWrite(addr, v)

	v = v&^SNOCUrgFwd | val&SNOCUrgFwd
	traceWrite(addr, v)

	return nil
}

// BIMC master ports whose QoS blocks §7.2 describes.
var BIMCPorts = []struct {
	Port int
	Name string
}{
	{0, "apps CPU"},
	{1, "GPU"},
	{2, "SNOC to BIMC (real-time)"},
	{3, "SNOC to BIMC (non-real-time)"},
	{4, "SMMU table walker (TCU)"},
	{6, "SNOC to BIMC"},
}

// BIMCQoSBlock is one BIMC master port's QoS block.
type BIMCQoSBlock struct {
	BKEEn  uint32    // bit 0: enable (clear = bypass)
	Health [4]uint32 // bits 1:0 priority level, 9:8 request priority, 31 limit commands (not HEALTH_3)
}

func bimcAddr(port int) (uint32, error) {
	for _, p := range BIMCPorts {
		if p.Port == port {
			return BIMC_QOS_BASE + uint32(port)*BIMC_QOS_STRIDE, nil
		}
	}

	return 0, fmt.Errorf("BIMC port %d is not one §7.2 describes", port)
}

// BIMCQoS reads a BIMC master port's QoS block.
func BIMCQoS(port int) (b BIMCQoSBlock, err error) {
	base, err := bimcAddr(port)

	if err != nil {
		return
	}

	b.BKEEn = reg.Read(base + bimcBKEEn)

	for i := range b.Health {
		b.Health[i] = reg.Read(base + bimcHealth0 + uint32(4*i))
	}

	return
}

// SetBIMCQoS writes a BIMC master port's HEALTH registers, then its
// BKE_EN. Not applied by any driver (§7.2: optional).
func SetBIMCQoS(port int, b BIMCQoSBlock) error {
	base, err := bimcAddr(port)

	if err != nil {
		return err
	}

	for i, h := range b.Health {
		traceWrite(base+bimcHealth0+uint32(4*i), h)
	}

	traceWrite(base+bimcBKEEn, b.BKEEn)

	return nil
}
