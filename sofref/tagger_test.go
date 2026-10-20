package sofref

import (
	"errors"
	"testing"
	"time"
)

const (
	hz  = 80_000_000
	t0  = 1_791_151_735 * second
	sec = uint64(hz)
)

func newTagger(t *testing.T, cfg Config) *Tagger {
	t.Helper()
	if cfg.TickHz == 0 {
		cfg.TickHz = hz
	}
	tg, err := New(cfg)
	if err != nil {
		t.Fatal(err)
	}
	return tg
}

func pps(t *testing.T, tg *Tagger, tick uint64, utc int64, want error) []Tag {
	t.Helper()
	tags, err := tg.PPS(tick, utc)
	if !errors.Is(err, want) {
		t.Fatalf("PPS(%d, %d): error %v, want %v", tick, utc, err, want)
	}
	return tags
}

func TestInterpolatesBetweenPulses(t *testing.T) {
	tg := newTagger(t, Config{UncNs: 20})
	base := uint64(1_000)
	// Counter 10 ppm fast: 1 s is 80_000_800 ticks.
	span := sec + 800
	if tags := pps(t, tg, base, t0, nil); len(tags) != 0 {
		t.Fatalf("first pulse gave %d tags", len(tags))
	}
	tg.SOF(base+span/4, 5000, 0)
	tg.SOF(base+span/2, 5500, 3)
	tg.SOF(base+span+10, 6000, 0) // after the next pulse
	tags := pps(t, tg, base+span, t0+second, nil)
	want := []Tag{
		{Frame: 5000, TGnssNs: t0 + 250_000_000, UncNs: 20},
		{Frame: 5500, Microframe: 3, TGnssNs: t0 + 500_000_000, UncNs: 20},
	}
	if len(tags) != len(want) {
		t.Fatalf("got %d tags, want %d", len(tags), len(want))
	}
	for i := range want {
		tags[i].Seen = time.Time{}
		if tags[i] != want[i] {
			t.Errorf("tag %d: %+v, want %+v", i, tags[i], want[i])
		}
	}
	tags = pps(t, tg, base+2*span, t0+2*second, nil)
	if len(tags) != 1 || tags[0].Frame != 6000 || tags[0].TGnssNs != t0+second+125 {
		t.Fatalf("SOF after a pulse goes into the next interval: %+v", tags)
	}
}

func TestRoundsToNearestNs(t *testing.T) {
	tg := newTagger(t, Config{TickHz: 3})
	pps(t, tg, 0, t0, nil)
	tg.SOF(1, 1, 0)
	tg.SOF(2, 2, 0)
	tags := pps(t, tg, 3, t0+second, nil)
	if tags[0].TGnssNs != t0+333_333_333 || tags[1].TGnssNs != t0+666_666_667 {
		t.Fatalf("got %d, %d", tags[0].TGnssNs-t0, tags[1].TGnssNs-t0)
	}
}

func TestRejectedPulseBreaksTheChain(t *testing.T) {
	tg := newTagger(t, Config{})
	pps(t, tg, 0, t0, nil)
	tg.SOF(sec/2, 1, 0)
	// A missed pulse: 2 s gap.
	pps(t, tg, 2*sec, t0+2*second, ErrInterval)
	tg.SOF(2*sec+sec/2, 2, 0)
	if tags := pps(t, tg, 3*sec, t0+3*second, nil); len(tags) != 0 {
		t.Fatalf("interval after a rejected pulse gave tags: %+v", tags)
	}
	tg.SOF(3*sec+sec/2, 3, 0)
	if tags := pps(t, tg, 4*sec, t0+4*second, nil); len(tags) != 1 || tags[0].Frame != 3 {
		t.Fatalf("chain did not resume: %+v", tags)
	}
	pps(t, tg, 5*sec+sec/2000, t0+5*second, nil) // 500 ppm fast
	pps(t, tg, 6*sec, t0+6*second, nil)
	pps(t, tg, 7*sec-sec/500, t0+7*second, ErrInterval) // 2000 ppm slow
	pps(t, tg, 8*sec-sec/500, t0+9*second, ErrLabel)
	pps(t, tg, 9*sec-sec/500, t0+10*second, nil)
	pps(t, tg, 10*sec, 12, ErrTimeRange)
	if s := tg.Stats(); s.Pulses != 10 || s.Rejected != 4 || s.Tags != 1 || s.Untagged != 2 {
		t.Fatalf("stats %+v", s)
	}
}

func TestCounterReset(t *testing.T) {
	tg := newTagger(t, Config{})
	pps(t, tg, 10*sec, t0, nil)
	tg.SOF(10*sec+5, 1, 0)
	// The device restarted: its counter begins again near 0.
	tg.SOF(100, 2, 0)
	pps(t, tg, sec/3, t0+second, ErrCounterReset)
	tg.SOF(sec/2, 3, 0)
	pps(t, tg, sec/3+sec, t0+2*second, nil)
	tg.SOF(sec/3+sec+5, 4, 0)
	if tags := pps(t, tg, sec/3+2*sec, t0+3*second, nil); len(tags) != 1 || tags[0].Frame != 4 {
		t.Fatalf("after a reset: %+v", tags)
	}
	if s := tg.Stats(); s.Late != 1 || s.Untagged != 2 {
		t.Fatalf("stats %+v", s)
	}
}

func TestCounterResetBeforeFirstPulse(t *testing.T) {
	tg := newTagger(t, Config{MaxPending: 4})
	tg.SOF(10*sec, 1, 0)
	tg.SOF(10*sec+5, 2, 0)
	// The device restarted before any pulse.
	tg.SOF(100, 3, 0)
	tg.SOF(sec/2, 4, 0)
	pps(t, tg, sec, t0, nil)
	tg.SOF(sec+sec/2, 5, 0)
	tags := pps(t, tg, 2*sec, t0+second, nil)
	if len(tags) != 1 || tags[0].Frame != 5 || tags[0].TGnssNs != t0+second/2 {
		t.Fatalf("tags %+v", tags)
	}
	if s := tg.Stats(); s.Untagged != 4 || s.Tags != 1 || s.Overflow != 0 {
		t.Fatalf("stats %+v", s)
	}
}

func TestPendingIsBounded(t *testing.T) {
	tg := newTagger(t, Config{MaxPending: 4})
	pps(t, tg, 0, t0, nil)
	for i := uint64(1); i <= 6; i++ {
		tg.SOF(i*1000, i, 0)
	}
	tags := pps(t, tg, sec, t0+second, nil)
	if len(tags) != 4 || tags[0].Frame != 3 || tg.Stats().Overflow != 2 {
		t.Fatalf("tags %+v, stats %+v", tags, tg.Stats())
	}
}

func TestSeenAndValidation(t *testing.T) {
	now := time.Unix(100, 0)
	tg := newTagger(t, Config{Now: func() time.Time { return now }})
	pps(t, tg, 0, t0, nil)
	if err := tg.SOF(1, 1, 8); !errors.Is(err, ErrSOF) {
		t.Fatalf("microframe 8: %v", err)
	}
	if err := tg.SOF(1, 1<<63, 0); !errors.Is(err, ErrSOF) {
		t.Fatalf("frame 2^63: %v", err)
	}
	tg.SOF(1, 1, 0)
	if tags := pps(t, tg, sec, t0+second, nil); !tags[0].Seen.Equal(now) {
		t.Fatalf("Seen %v", tags[0].Seen)
	}
	if _, err := New(Config{}); err == nil {
		t.Fatal("New without TickHz")
	}
	if _, err := New(Config{TickHz: 1, UncNs: -1}); err == nil {
		t.Fatal("New with UncNs -1")
	}
}

func TestReset(t *testing.T) {
	tg := newTagger(t, Config{})
	pps(t, tg, 0, t0, nil)
	tg.SOF(5, 1, 0)
	tg.Reset()
	if tags := pps(t, tg, sec, t0+second, nil); len(tags) != 0 {
		t.Fatalf("tags after Reset: %+v", tags)
	}
}
