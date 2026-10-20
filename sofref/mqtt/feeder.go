// Package mqtt publishes sofref tags to sofd as an external GNSS reference.
//
// The Feeder asks sofd to sample the controller (a retained want request,
// cleared by the will when the feeder disappears), publishes each pulse's
// tags as one message and reports sofd's ref_* events for this client.
// The client name must be listed in sofd's gnss.external.
package mqtt

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"net/url"
	"os"
	"sync"
	"sync/atomic"
	"time"

	"github.com/BertoldVdb/sofcapture/sofref"
	"github.com/eclipse/paho.golang/autopaho"
	"github.com/eclipse/paho.golang/paho"
)

// sofd's limit per message.
const maxBatch = 256

// Options for Connect and Lookup.
type Options struct {
	// Broker URL, e.g. "mqtt://localhost:1883".
	Broker string
	// Prefix of sofd's topics (default "sofd/<this host's name>").
	Prefix string
	// Client is the name in sofd's gnss.external.
	Client string
	// ClientID for the broker (default "sofref-<Client>@<Prefix>/<Controller>").
	ClientID string
	// Controller id, e.g. "0000:00:14.0"; see Lookup.
	Controller string
	// RateHz asked from sofd; 0 uses sofd's default.
	RateHz float64
	// ModelHz asks sofd to publish the controller's model this often; 0 leaves
	// sofd's configured rate (1 Hz by default). The feeder does not need it.
	ModelHz  float64
	Username string
	Password string
	// MaxAge drops tags whose SOF was reported longer ago (default 1.5 s):
	// sofd may match those to the wrong frame.
	MaxAge time.Duration
	// OnEvent receives sofd's events for this client (optional, must not block).
	OnEvent func(Event)
	// OnError receives connection errors (optional).
	OnError func(error)
}

// Event is one of sofd's controller events for this client: ref_accepted,
// ref_rejected, ref_busy, ref_lost or chrony_busy.
type Event struct {
	Event       string `json:"event"`
	Controller  string `json:"controller"`
	Client      string `json:"client"`
	Reason      string `json:"reason,omitempty"`
	Message     string `json:"message,omitempty"`
	Owner       string `json:"owner,omitempty"`
	FrameOffset *int64 `json:"frame_offset,omitempty"`
	// The whole payload.
	Raw json.RawMessage `json:"-"`
}

// Feeder publishes tags for one controller.
type Feeder struct {
	opts  Options
	cm    *autopaho.ConnectionManager
	ref   string
	want  string
	stale atomic.Uint64
	// Held by announce, so Close waits for it.
	mu     sync.Mutex
	closed bool
}

func (o *Options) defaults() error {
	if o.Broker == "" {
		return errors.New("sofref/mqtt: Broker is required")
	}
	if o.Prefix == "" {
		host, err := os.Hostname()
		if err != nil {
			return fmt.Errorf("sofref/mqtt: no Prefix and no host name: %w", err)
		}
		o.Prefix = "sofd/" + host
	}
	if o.MaxAge == 0 {
		o.MaxAge = 1500 * time.Millisecond
	}
	return nil
}

func (o *Options) config(clientID string) (autopaho.ClientConfig, error) {
	u, err := url.Parse(o.Broker)
	if err != nil {
		return autopaho.ClientConfig{}, fmt.Errorf("sofref/mqtt: broker: %w", err)
	}
	cfg := autopaho.ClientConfig{
		ServerUrls:                    []*url.URL{u},
		KeepAlive:                     20,
		CleanStartOnInitialConnection: true,
		ReconnectBackoff:              autopaho.NewConstantBackoff(2 * time.Second),
		OnConnectError:                o.OnError,
		ClientConfig:                  paho.ClientConfig{ClientID: clientID, OnClientError: o.OnError},
	}
	if o.Username != "" {
		cfg.ConnectUsername, cfg.ConnectPassword = o.Username, []byte(o.Password)
	}
	return cfg, nil
}

// Connect connects to the broker and asks sofd to sample the controller. It
// waits for the first connection; after that it reconnects by itself.
func Connect(ctx context.Context, opts Options) (*Feeder, error) {
	if err := opts.defaults(); err != nil {
		return nil, err
	}
	if opts.Client == "" || opts.Controller == "" {
		return nil, errors.New("sofref/mqtt: Client and Controller are required")
	}
	if opts.ClientID == "" {
		opts.ClientID = "sofref-" + opts.Client + "@" + opts.Prefix + "/" + opts.Controller
	}
	ctrl := opts.Prefix + "/controllers/" + opts.Controller
	f := &Feeder{opts: opts, ref: ctrl + "/ref/" + opts.Client, want: ctrl + "/want/" + opts.Client}
	cfg, err := opts.config(opts.ClientID)
	if err != nil {
		return nil, err
	}
	cfg.SetWillMessage(f.want, nil, 1, true)
	cfg.OnConnectionUp = func(cm *autopaho.ConnectionManager, _ *paho.Connack) {
		// Must not block.
		go f.announce(cm, ctrl+"/event")
	}
	cfg.OnPublishReceived = []func(paho.PublishReceived) (bool, error){f.onPublish}
	f.cm, err = autopaho.NewConnection(context.Background(), cfg)
	if err != nil {
		return nil, err
	}
	if err := f.cm.AwaitConnection(ctx); err != nil {
		f.cm.Disconnect(context.Background())
		return nil, err
	}
	return f, nil
}

// announce runs on every (re)connection: clean start drops subscriptions.
func (f *Feeder) announce(cm *autopaho.ConnectionManager, events string) {
	f.mu.Lock()
	defer f.mu.Unlock()
	if f.closed {
		return
	}
	ctx, cancel := context.WithTimeout(context.Background(), 10*time.Second)
	defer cancel()
	if _, err := cm.Subscribe(ctx, &paho.Subscribe{Subscriptions: []paho.SubscribeOptions{{Topic: events, QoS: 1}}}); err != nil {
		f.report(fmt.Errorf("sofref/mqtt: subscribe: %w", err))
	}
	req := map[string]float64{}
	if f.opts.RateHz > 0 {
		req["rate_hz"] = f.opts.RateHz
	}
	if f.opts.ModelHz > 0 {
		req["model_hz"] = f.opts.ModelHz
	}
	want, _ := json.Marshal(req)
	if _, err := cm.Publish(ctx, &paho.Publish{Topic: f.want, QoS: 1, Retain: true, Payload: want}); err != nil {
		f.report(fmt.Errorf("sofref/mqtt: want: %w", err))
	}
}

func (f *Feeder) onPublish(pr paho.PublishReceived) (bool, error) {
	var ev Event
	if json.Unmarshal(pr.Packet.Payload, &ev) != nil || ev.Client != f.opts.Client {
		return false, nil
	}
	ev.Raw = append(json.RawMessage(nil), pr.Packet.Payload...)
	if f.opts.OnEvent != nil {
		f.opts.OnEvent(ev)
	}
	return true, nil
}

func (f *Feeder) report(err error) {
	if f.opts.OnError != nil {
		f.opts.OnError(err)
	}
}

// Publish sends the tags of one pulse, dropping those older than MaxAge.
// Without a connection it returns an error and the tags are lost: they
// would be too old by the time it is back.
func (f *Feeder) Publish(ctx context.Context, tags []sofref.Tag) error {
	fresh := make([]sofref.Tag, 0, len(tags))
	for _, t := range tags {
		if !t.Seen.IsZero() && time.Since(t.Seen) > f.opts.MaxAge {
			f.stale.Add(1)
			continue
		}
		fresh = append(fresh, t)
	}
	for len(fresh) > 0 {
		n := min(len(fresh), maxBatch)
		payload, err := json.Marshal(struct {
			Tags []sofref.Tag `json:"tags"`
		}{fresh[:n]})
		if err != nil {
			return err
		}
		if _, err := f.cm.Publish(ctx, &paho.Publish{Topic: f.ref, Payload: payload}); err != nil {
			return fmt.Errorf("sofref/mqtt: publish: %w", err)
		}
		fresh = fresh[n:]
	}
	return nil
}

// Stale counts tags Publish dropped for being older than MaxAge.
func (f *Feeder) Stale() uint64 {
	return f.stale.Load()
}

// Close withdraws the want request and disconnects.
func (f *Feeder) Close(ctx context.Context) error {
	f.mu.Lock()
	f.closed = true
	f.mu.Unlock()
	_, err := f.cm.Publish(ctx, &paho.Publish{Topic: f.want, QoS: 1, Retain: true})
	return errors.Join(err, f.cm.Disconnect(ctx))
}
