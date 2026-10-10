package mqtt

import (
	"context"
	"crypto/rand"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"

	"github.com/eclipse/paho.golang/autopaho"
	"github.com/eclipse/paho.golang/paho"
)

// Query names a USB device: Device ("vvvv:pppp") with an optional Serial,
// a sysfs Port ("1-1.2"), or Busnum and Devnum.
type Query struct {
	Device string `json:"device,omitempty"`
	Serial string `json:"serial,omitempty"`
	Port   string `json:"port,omitempty"`
	Busnum uint32 `json:"busnum,omitempty"`
	Devnum uint32 `json:"devnum,omitempty"`
}

// Lookup asks sofd which controller a device is on, over its own short
// connection. It uses Broker, Prefix, Username and Password from opts.
func Lookup(ctx context.Context, opts Options, q Query) (string, error) {
	if err := opts.defaults(); err != nil {
		return "", err
	}
	id := make([]byte, 8)
	rand.Read(id)
	nonce := hex.EncodeToString(id)
	reply := "sofref/" + nonce + "/reply"
	cfg, err := opts.config("sofref-lookup-" + nonce)
	if err != nil {
		return "", err
	}
	got := make(chan []byte, 1)
	cfg.OnPublishReceived = []func(paho.PublishReceived) (bool, error){func(pr paho.PublishReceived) (bool, error) {
		if pr.Packet.Topic == reply {
			select {
			case got <- pr.Packet.Payload:
			default:
			}
		}
		return true, nil
	}}
	cm, err := autopaho.NewConnection(ctx, cfg)
	if err != nil {
		return "", err
	}
	defer cm.Disconnect(context.Background())
	if err := cm.AwaitConnection(ctx); err != nil {
		return "", err
	}
	if _, err := cm.Subscribe(ctx, &paho.Subscribe{Subscriptions: []paho.SubscribeOptions{{Topic: reply, QoS: 1}}}); err != nil {
		return "", err
	}
	body, _ := json.Marshal(q)
	req := &paho.Publish{
		Topic:      opts.Prefix + "/cmd/lookup",
		QoS:        1,
		Payload:    body,
		Properties: &paho.PublishProperties{ResponseTopic: reply, CorrelationData: id},
	}
	if _, err := cm.Publish(ctx, req); err != nil {
		return "", err
	}
	select {
	case <-ctx.Done():
		return "", fmt.Errorf("sofref/mqtt: no reply from sofd: %w", ctx.Err())
	case b := <-got:
		var r struct {
			OK         bool   `json:"ok"`
			Error      string `json:"error"`
			Controller string `json:"controller"`
			Supported  bool   `json:"supported"`
		}
		if err := json.Unmarshal(b, &r); err != nil {
			return "", fmt.Errorf("sofref/mqtt: reply: %w", err)
		}
		if !r.OK {
			return "", fmt.Errorf("sofref/mqtt: lookup: %s", r.Error)
		}
		if !r.Supported {
			return r.Controller, errors.New("sofref/mqtt: sofd cannot sample controller " + r.Controller)
		}
		return r.Controller, nil
	}
}
