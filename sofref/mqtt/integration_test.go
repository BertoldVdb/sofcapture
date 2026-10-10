package mqtt

import (
	"context"
	"encoding/json"
	"net/url"
	"os"
	"testing"
	"time"

	"github.com/BertoldVdb/sofcapture/sofref"
	"github.com/eclipse/paho.golang/autopaho"
	"github.com/eclipse/paho.golang/paho"
)

// Runs against a live sofd with a simulated controller:
//
//	sofd --simulate run --config mqtt/testdata/sofd.hujson
//	SOFREF_TEST_BROKER=mqtt://localhost:1883 go test ./mqtt -run Live -v
//
// A stand-in device labels sofd's own frames with CLOCK_REALTIME + 5 µs as
// "GNSS", so sofd should report realtime_minus_gnss_ns near -5000. One batch
// is then sent 2.3 s late; sofd must reject it without moving the fit.
func TestLiveAgainstSofd(t *testing.T) {
	broker := os.Getenv("SOFREF_TEST_BROKER")
	if broker == "" {
		t.Skip("SOFREF_TEST_BROKER not set")
	}
	const prefix, offset = "sofd/sofreftest", 5000
	ctx, cancel := context.WithTimeout(context.Background(), 40*time.Second)
	defer cancel()
	opts := Options{Broker: broker, Prefix: prefix, Client: "itest"}

	ctrl, err := Lookup(ctx, opts, Query{Port: "sim0-1"})
	if err != nil {
		t.Fatal(err)
	}
	if ctrl != "sim0" {
		t.Fatalf("lookup gave %q", ctrl)
	}

	models := make(chan model, 100)
	watch(t, ctx, broker, prefix+"/controllers/sim0/model", models)

	events := make(chan Event, 10)
	opts.Controller = ctrl
	// The stand-in device takes its frames from the model.
	opts.ModelHz = 50
	opts.OnEvent = func(e Event) { events <- e }
	f, err := Connect(ctx, opts)
	if err != nil {
		t.Fatal(err)
	}
	defer f.Close(context.Background())

	tg, err := sofref.New(sofref.Config{TickHz: 1e9, UncNs: 300})
	if err != nil {
		t.Fatal(err)
	}
	nextPulse := func() *time.Timer {
		now := time.Now()
		return time.NewTimer(now.Truncate(time.Second).Add(time.Second + 20*time.Millisecond).Sub(now))
	}
	pulse := nextPulse()
	// 0: locking; 1: hold back the next pulse's tags; 2: holding or sent late; 3: recovered.
	phase := 0
	inconsistent := false
	var late <-chan time.Time
	var held []sofref.Tag
	accepted := false
	for {
		select {
		case <-ctx.Done():
			t.Fatalf("timed out in phase %d (stats %+v, stale %d)", phase, tg.Stats(), f.Stale())
		case e := <-events:
			t.Logf("event %s", e.Raw)
			switch {
			case e.Event == "ref_accepted":
				accepted = true
				if phase == 2 && inconsistent {
					phase = 3
				}
			// Sampling just started: tags may predate it.
			case e.Event == "ref_rejected" && e.Reason == "epoch" && phase == 0:
			// The late batch, matched to the wrong wrap.
			case e.Event == "ref_rejected" && e.Reason == "inconsistent" && phase == 2:
				inconsistent = true
			default:
				t.Fatalf("unexpected event %s", e.Raw)
			}
		case <-pulse.C:
			edge := time.Now().Truncate(time.Second).UnixNano()
			tags, err := tg.PPS(uint64(edge), edge+offset)
			if err != nil {
				t.Logf("pulse: %v", err)
			}
			if phase == 1 && len(tags) > 0 {
				// Zero Seen so the feeder does not drop them as stale.
				for i := range tags {
					tags[i].Seen = time.Time{}
				}
				held, tags, late = tags, nil, time.After(2300*time.Millisecond)
				phase = 2
			}
			if err := f.Publish(ctx, tags); err != nil {
				t.Fatal(err)
			}
			pulse = nextPulse()
		case <-late:
			t.Logf("publishing %d tags 2.3 s late", len(held))
			if err := f.Publish(ctx, held); err != nil {
				t.Fatal(err)
			}
			late = nil
		case m := <-models:
			if m.FrameStart.BusFrame == nil || m.FrameStart.TRealtimeNs == nil {
				continue
			}
			if err := tg.SOF(uint64(*m.FrameStart.TRealtimeNs), uint64(*m.FrameStart.BusFrame), 0); err != nil {
				t.Fatal(err)
			}
			if m.Gnss == nil || m.Gnss.Board != "mqtt:itest" || m.ModelGnss == nil || m.ModelGnss.N < 20 {
				continue
			}
			d := *m.Gnss.RealtimeMinusGnssNs
			if d < -offset-1000 || d > -offset+1000 {
				t.Fatalf("phase %d: realtime - gnss = %d, want about %d", phase, d, -offset)
			}
			if phase == 0 && accepted {
				t.Logf("locked: realtime - gnss = %d ns, model_gnss %d points, resid %.0f ns", d, m.ModelGnss.N, m.ModelGnss.ResidRmsNs)
				phase = 1
			}
			if phase == 3 {
				t.Logf("after the late batch: realtime - gnss = %d ns, resid %.0f ns", d, m.ModelGnss.ResidRmsNs)
				return
			}
		}
	}
}

type model struct {
	FrameStart struct {
		BusFrame    *uint32 `json:"bus_frame"`
		TRealtimeNs *int64  `json:"t_realtime_ns"`
	} `json:"frame_start"`
	ModelGnss *struct {
		N          int     `json:"n"`
		ResidRmsNs float64 `json:"resid_rms_ns"`
	} `json:"model_gnss"`
	Gnss *struct {
		Board               string `json:"board"`
		RealtimeMinusGnssNs *int64 `json:"realtime_minus_gnss_ns"`
	} `json:"gnss"`
}

func watch(t *testing.T, ctx context.Context, broker, topic string, out chan<- model) {
	u, _ := url.Parse(broker)
	cfg := autopaho.ClientConfig{
		ServerUrls: []*url.URL{u},
		KeepAlive:  20,
		OnConnectionUp: func(cm *autopaho.ConnectionManager, _ *paho.Connack) {
			go cm.Subscribe(ctx, &paho.Subscribe{Subscriptions: []paho.SubscribeOptions{{Topic: topic}}})
		},
		ClientConfig: paho.ClientConfig{
			ClientID: "sofref-itest-watch",
			OnPublishReceived: []func(paho.PublishReceived) (bool, error){func(pr paho.PublishReceived) (bool, error) {
				var m model
				if json.Unmarshal(pr.Packet.Payload, &m) == nil {
					select {
					case out <- m:
					default:
					}
				}
				return true, nil
			}},
		},
	}
	cm, err := autopaho.NewConnection(ctx, cfg)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { cm.Disconnect(context.Background()) })
}
