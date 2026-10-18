// Copyright (c) the qcm2290 package authors
//
// Use of this source code is governed by the license
// that can be found in the LICENSE file.

package se2clk

import "testing"

// §2.1's table, in MHz, in table order.
var tableMHz = []float64{7.3728, 14.7456, 19.2, 29.4912, 32, 48, 64, 75, 80, 96, 100, 102.4, 112, 117.9648, 120, 128}

func TestTableRates(t *testing.T) {
	if len(Table) != len(tableMHz) {
		t.Fatalf("%d entries", len(Table))
	}

	for i, r := range Table {
		if want := uint64(tableMHz[i]*1e6 + 0.5); r.Hz() != want {
			t.Errorf("entry %d: %v, want %d", i, r, want)
		}
	}
}

// Every exact rate of §4.3 at oversampling 16, with the root it names.
func TestExactRates(t *testing.T) {
	for _, c := range []struct {
		baud   int
		rootHz uint64
		div    uint32
	}{
		{1200000, 19200000, 1},
		{2000000, 32000000, 1},
		{3000000, 48000000, 1},
		{4000000, 64000000, 1},
		{5000000, 80000000, 1},
		{6000000, 96000000, 1},
		{7500000, 120000000, 1},
		{8000000, 128000000, 1},
		{115200, 7372800, 4},
		{1000000, 32000000, 2},
	} {
		p, err := PlanBaud(c.baud, 16, Table)
		if err != nil || p.ErrorPPM != 0 || p.RootHz != c.rootHz || p.Divider != c.div {
			t.Errorf("%d baud: %+v (%v), want %d Hz / %d", c.baud, p, err, c.rootHz, c.div)
		}
	}

	if p, _ := PlanBaud(1200000, 16, Table); p.Root != Crystal {
		t.Errorf("1.2 Mbaud not on the crystal: %v", p.Root)
	}

	if _, err := PlanBaud(9000000, 16, Table); err == nil {
		t.Errorf("9 Mbaud accepted")
	}
}

// The encodings measured on the hardware (§2.1).
func TestMeasuredEncodings(t *testing.T) {
	r96 := Root{Src: 2, H2: 2, M: 8, N: 25}

	if cfg := r96.CFG(0x00100000); cfg != 0x00102201 {
		t.Errorf("96 MHz CFG %#x", cfg)
	}

	if m, n, d, used := r96.MND(); !used || m != 8 || n != 0xffee || d != 0xffe6 {
		t.Errorf("96 MHz MND %#x %#x %#x %v", m, n, d, used)
	}

	if cfg := (Root{Src: 4, H2: 8}).CFG(0x00100000); cfg != 0x00100407 {
		t.Errorf("GPLL6 / 4 CFG %#x", cfg)
	}

	if cfg := Crystal.CFG(0x00102201); cfg != 0x00100000 {
		t.Errorf("crystal CFG %#x", cfg)
	}

	if got := Decode(0x00102201, 8, 0xffee); got != r96 {
		t.Errorf("decode %+v", got)
	}

	if got := Decode(0x00100000, 0, 0); got != Crystal {
		t.Errorf("decode crystal %+v", got)
	}
}
