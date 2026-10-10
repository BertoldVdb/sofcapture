// Package sofref turns a device's PPS and USB SOF captures into SOF tags for
// sofd's MQTT GNSS reference (form A of sofd/MQTT-GUIDE.md).
//
// The device timestamps the GNSS PPS edge and some USB start-of-frame packets
// on one free-running counter. The Tagger gives each SOF the GNSS time
// interpolated between the two pulses around it, so tags come out at the
// pulse after their SOF. Package mqtt publishes them.
package sofref

import (
	"errors"
	"fmt"
	"math/bits"
	"time"
)

const (
	second = int64(time.Second)
	// Allowed pulse interval error: crystals may be 200 ppm off, a missed
	// pulse or a glitch is far more.
	intervalTol = 1000e-6
	// Years 2000 and 2100: sofd's limits.
	minUnixNs = 946_684_800 * second
	maxUnixNs = 4_102_444_800 * second
	// Highest frame number sofd accepts.
	maxFrame = 1<<63 - 1
)

var (
	// ErrInterval: the pulse did not come 1 s (of TickHz) after the previous one.
	ErrInterval = errors.New("sofref: pulse interval is not 1 s")
	// ErrLabel: the pulse's UTC is not 1 s after the previous pulse's.
	ErrLabel = errors.New("sofref: pulse time is not 1 s after the previous")
	// ErrCounterReset: the counter went backwards (device restart).
	ErrCounterReset = errors.New("sofref: counter went backwards")
	// ErrTimeRange: the pulse's UTC is not a Unix time between 2000 and 2100.
	ErrTimeRange = errors.New("sofref: pulse time out of range")
	// ErrSOF: a SOF with an invalid frame or microframe.
	ErrSOF = errors.New("sofref: invalid SOF")
)

// Config for a Tagger.
type Config struct {
	// TickHz is the nominal counter frequency. Only used to check the pulse
	// interval (±1000 ppm); interpolation uses the measured one.
	TickHz uint64
	// UncNs is the 1σ uncertainty sent with each tag (default 100).
	UncNs int64
	// MaxPending bounds the SOFs waiting for their next pulse (default 1024).
	MaxPending int
	// Now is the clock for Tag.Seen (default time.Now).
	Now func() time.Time
}

// Tag is one SOF with its GNSS time, as sofd expects it on
// controllers/<id>/ref/<client>.
type Tag struct {
	Frame      uint64 `json:"frame"`
	Microframe uint8  `json:"microframe,omitempty"`
	TGnssNs    int64  `json:"t_gnss_ns"`
	UncNs      int64  `json:"unc_ns"`
	// Seen is when the SOF was handed to the Tagger; the feeder drops tags
	// that got too old to be matched by sofd.
	Seen time.Time `json:"-"`
}

// Stats counts what the Tagger did since it was made.
type Stats struct {
	Pulses   uint64 // PPS calls
	Rejected uint64 // pulses that returned an error
	Tags     uint64 // tags returned
	Late     uint64 // SOFs reported after the pulse that follows them
	Untagged uint64 // SOFs whose pulses were not both accepted
	Overflow uint64 // SOFs dropped because MaxPending was reached
}

type sof struct {
	tick, frame uint64
	microframe  uint8
	seen        time.Time
}

type pulse struct {
	tick  uint64
	utcNs int64
}

// Tagger is not safe for concurrent use.
type Tagger struct {
	cfg     Config
	pending []sof
	prev    *pulse
	// prev was accepted, so it can start an interval.
	prevOK bool
	stats  Stats
}

// New returns a Tagger; cfg.TickHz is required.
func New(cfg Config) (*Tagger, error) {
	if cfg.TickHz == 0 {
		return nil, errors.New("sofref: TickHz is required")
	}
	if cfg.UncNs == 0 {
		cfg.UncNs = 100
	}
	if cfg.UncNs < 1 || cfg.UncNs > second {
		return nil, fmt.Errorf("sofref: UncNs %d is not within 1 ns..1 s", cfg.UncNs)
	}
	if cfg.MaxPending <= 0 {
		cfg.MaxPending = 1024
	}
	if cfg.Now == nil {
		cfg.Now = time.Now
	}
	return &Tagger{cfg: cfg}, nil
}

// SOF reports that the SOF of USB frame `frame` was captured at `tick`.
// frame must contain the 11-bit bus frame number in its low bits; an
// unwrapped count is better. microframe is 0-7 on high speed, else 0.
func (t *Tagger) SOF(tick, frame uint64, microframe uint8) error {
	if frame > maxFrame || microframe > 7 {
		return fmt.Errorf("%w: frame %d, microframe %d", ErrSOF, frame, microframe)
	}
	if t.prev != nil && tick <= t.prev.tick {
		t.stats.Late++
		return nil
	}
	if len(t.pending) == t.cfg.MaxPending {
		t.pending = t.pending[1:]
		t.stats.Overflow++
	}
	t.pending = append(t.pending, sof{tick, frame, microframe, t.cfg.Now()})
	return nil
}

// PPS reports a PPS edge captured at `tick` and its UTC in Unix ns. The caller
// labels the pulse with its second and applies any receiver correction
// (u-blox qErr), and reports pulses only while the receiver has a fix.
//
// It returns the tags of the SOFs since the previous pulse when both pulses
// were accepted. The first pulse, and the first after a rejected one, gives no
// tags. Call it promptly: sofd matches a tag only within ~1.9 s of its frame.
func (t *Tagger) PPS(tick uint64, utcNs int64) ([]Tag, error) {
	t.stats.Pulses++
	err := t.check(tick, utcNs)
	if errors.Is(err, ErrCounterReset) {
		t.stats.Untagged += uint64(len(t.pending))
		t.pending = nil
	}
	var tags []Tag
	n := 0
	for _, s := range t.pending {
		if s.tick > tick {
			break
		}
		n++
		if err == nil && t.prevOK {
			tags = append(tags, t.tag(s, tick, utcNs))
		}
	}
	t.stats.Untagged += uint64(n - len(tags))
	t.stats.Tags += uint64(len(tags))
	t.pending = append(t.pending[:0], t.pending[n:]...)
	t.prev, t.prevOK = &pulse{tick, utcNs}, err == nil
	if err != nil {
		t.stats.Rejected++
	}
	return tags, err
}

func (t *Tagger) check(tick uint64, utcNs int64) error {
	if utcNs < minUnixNs || utcNs >= maxUnixNs {
		return fmt.Errorf("%w: %d", ErrTimeRange, utcNs)
	}
	p := t.prev
	if p == nil {
		return nil
	}
	if tick <= p.tick {
		return ErrCounterReset
	}
	hz := float64(t.cfg.TickHz)
	if d := float64(tick - p.tick); d < hz*(1-intervalTol) || d > hz*(1+intervalTol) {
		return fmt.Errorf("%w: %.0f ticks", ErrInterval, d)
	}
	if utcNs-p.utcNs != second {
		return fmt.Errorf("%w: %d ns", ErrLabel, utcNs-p.utcNs)
	}
	return nil
}

// tag interpolates between t.prev and the pulse at (tick, utcNs).
func (t *Tagger) tag(s sof, tick uint64, utcNs int64) Tag {
	span, d := tick-t.prev.tick, s.tick-t.prev.tick
	// d * 1e9 / span, rounded; 128-bit so any TickHz fits.
	hi, lo := bits.Mul64(d, uint64(utcNs-t.prev.utcNs))
	lo, c := bits.Add64(lo, span/2, 0)
	q, _ := bits.Div64(hi+c, lo, span)
	return Tag{
		Frame:      s.frame,
		Microframe: s.microframe,
		TGnssNs:    t.prev.utcNs + int64(q),
		UncNs:      t.cfg.UncNs,
		Seen:       s.seen,
	}
}

// Reset forgets pulses and pending SOFs, for a device restart or a lost fix.
func (t *Tagger) Reset() {
	t.stats.Untagged += uint64(len(t.pending))
	t.pending, t.prev, t.prevOK = nil, nil, false
}

// Stats returns the counters.
func (t *Tagger) Stats() Stats {
	return t.stats
}
