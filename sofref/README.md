# sofref

This Go module is for programs that want to give [sofd](../sofd) a GNSS
reference of their own. That means a USB device that timestamps the GNSS PPS
edge and some USB start-of-frame (SOF) packets on one counter. sofd then
treats the device like a sofgnss board. The controller's samples gain
`model_gnss`, and sofd can feed chrony from it.

The module does the work between the device and sofd's MQTT interface, which
is form A in [MQTT-GUIDE.md](../sofd/MQTT-GUIDE.md#part-2-feeding-a-gnss-reference-from-your-program).
`sofref.Tagger` gives each captured SOF its GNSS time by interpolating between
the two pulses around it. It has no I/O and no dependencies.
`sofref/mqtt.Feeder` asks sofd to sample the controller, publishes the tags of
each pulse as one message, drops tags that got too old and passes on sofd's
events for the client. `mqtt.Lookup` finds the controller of a USB device.

```sh
go get github.com/BertoldVdb/sofcapture/sofref
```

## Usage

```go
ctrl, err := mqtt.Lookup(ctx, mqtt.Options{Broker: "mqtt://localhost:1883"}, mqtt.Query{Device: "1234:5678", Serial: "ABC123"})

f, err := mqtt.Connect(ctx, mqtt.Options{
	Broker:     "mqtt://localhost:1883",
	Client:     "myapp",                      // listed in sofd's gnss.external
	Controller: ctrl,
	OnEvent:    func(e mqtt.Event) { log.Printf("sofd: %s", e.Raw) },
})
defer f.Close(context.Background())

tg, err := sofref.New(sofref.Config{TickHz: 80_000_000, UncNs: 20})

// For each SOF your device timestamps, in capture order:
tg.SOF(tick, frame, 0)

// For each PPS edge, as soon as you know its UTC second:
tags, err := tg.PPS(tick, utcNs)        // err: the pulse was rejected (see below)
f.Publish(ctx, tags)
```

sofd only accepts tags from clients in its configuration:

```jsonc
"gnss": { "external": { "myapp": { "chrony": "/run/chrony.myapp.sock" } } }
```

## What the caller must do

- Label the pulses. `PPS(tick, utcNs)` takes the UTC of the edge itself, so
  you have to work out which second each pulse belongs to. u-blox TIM-TP
  announces the next pulse and NMEA ZDA reports the previous one. Apply the
  receiver's correction (qErr on u-blox), and only report pulses while the
  receiver has a fix, because sofd doesn't check external references for GNSS
  lock.
- Report promptly. sofd matches a tag's frame number by the time the tag
  arrives, and that only works within about 1.9 s of the frame. A tag comes
  out at the pulse after its SOF, so it can already be 1 s old. Call `PPS` and
  `Publish` straight after the edge. The Feeder drops tags whose SOF was
  reported more than `MaxAge` (1.5 s) ago and counts them in `Stale()`. If a
  late tag still reaches sofd, sofd rejects it (`no_match` or
  `inconsistent`) without affecting the fit.
- Give real frame numbers. The low 11 bits of `frame` must be the USB bus
  frame number. An unwrapped count is better than a bare 11-bit number,
  because sofd then locks the offset and catches slips. On high-speed devices
  `microframe` is 0–7.
- Tag at a sensible rate. You need at least one SOF per second. More than
  sofd's sampling rate (50 Hz by default) adds nothing.

## Tagger

A pulse is accepted when it comes 1 s after the previous one within ±1000 ppm
of `TickHz`, and its UTC is exactly 1 s after the previous pulse's. A SOF is
tagged only when the pulses on both sides of it were accepted. So the first
pulse gives no tags, and neither does the first one after a rejected pulse.

| Error from `PPS` | Meaning |
|---|---|
| `ErrInterval` | not 1 s after the previous pulse, so a pulse was missed or this one is a glitch |
| `ErrLabel` | UTC not 1 s after the previous pulse's |
| `ErrCounterReset` | the counter went backwards, usually a device restart. Pending SOFs are dropped |
| `ErrTimeRange` | UTC not between 2000 and 2100 |

Errors wrap these values, so test them with `errors.Is`. If you can, filter
PPS glitches on the device. An extra edge is rejected, but it also breaks the
chain for one more pulse.

`Reset()` forgets everything, for example when the receiver loses its fix.
`Stats()` counts pulses, rejected pulses, tags, and SOFs that were late (reported
after the following pulse), untagged or dropped on overflow (`MaxPending`,
default 1024). A Tagger is not safe for concurrent use.

## Feeder

- Every time it connects, it subscribes to `controllers/<id>/event` and
  publishes a retained `controllers/<id>/want/<client>` request with `RateHz`
  (or sofd's default rate). The feeder itself doesn't need the model. Only set
  `ModelHz` if your program also reads `controllers/<id>/model` and wants it
  more often than sofd's default of once a second. The feeder's will clears
  the request, so it goes away with the feeder. `Close` clears it too.
- `Publish` sends one `{"tags":[…]}` message per call at QoS 0, split in
  pieces above 256 tags. Without a connection it returns an error and the tags
  are lost. They would be too old by the time the connection is back anyway.
- `OnEvent` receives sofd's events for this client: `ref_accepted`,
  `ref_rejected` (with `Reason`), `ref_busy` (`Owner`), `ref_lost` and
  `chrony_busy`. MQTT-GUIDE.md describes each one.
- `Prefix` defaults to `sofd/<this host's name>` and `ClientID` to
  `sofref-<Client>@<Prefix>/<Controller>`.

## Tests

```sh
go test ./...
```

`mqtt/integration_test.go` runs against a live sofd with a simulated
controller and the local broker:

```sh
sofd --simulate run --config mqtt/testdata/sofd.hujson &
SOFREF_TEST_BROKER=mqtt://localhost:1883 go test ./mqtt -run Live -v
```

A stand-in device labels sofd's own frames with CLOCK_REALTIME + 5 µs as
"GNSS". The test checks that sofd reports `realtime_minus_gnss_ns` near
−5000 ns, then sends one batch 2.3 s late and checks that sofd rejects it
(`inconsistent`) without moving the fit. On the development PC it locked at
−5013 ns with an 18 ns fit residual, and read −4974 ns after the late batch. With the
inconsistency check disabled in sofd, the same late batch moved the result to
+636 ms.
