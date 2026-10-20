# Timed USB frames over MQTT: a guide for client programs

This guide is for programs that talk to sofd over MQTT. It has two parts.

1. Consuming the host and GNSS time of USB start-of-frame (SOF) packets. This
   covers what to subscribe to and how to turn a USB frame number into a
   timestamp.
2. Feeding sofd a GNSS reference from your own program. If your program knows
   the GNSS time of some SOFs, for example because it has its own USB device
   with a GNSS receiver, it can publish them and sofd uses them like a sofgnss
   board.

The [README](README.md) describes every topic and field in detail. This guide
shows how to use them.

The examples use `sofd/pc1` as the topic prefix (the default is
`sofd/<hostname>`) and `0000:00:14.0` as the controller id, which is its PCI
address.

## Background

A USB host controller starts a frame every 1 ms at full speed, or a microframe
every 125 µs at high speed. It announces each one with a SOF packet that
carries the 11-bit frame number. Every device on that controller sees the same
SOFs, also behind hubs, so the frame number is a clock shared between the host
and its devices. It wraps every 2048 frames (2.048 s).

sofd samples the controller's frame counter and fits it against three clocks:

| Model | Clock | Meaning |
|---|---|---|
| `model_raw` | `CLOCK_MONOTONIC_RAW` | host-local, never steered |
| `model_realtime` | `CLOCK_REALTIME` | the host's wall clock, as chrony or NTP steer it |
| `model_gnss` | GNSS UTC (Unix ns) | from a GNSS reference on that controller, such as a sofgnss board |

Each model gives the start time of any counter value `c` in the same epoch:

```
t = t_ref_ns + (c - counter_ref) * period_ns
```

The counter is in units of `unit_ns`. That is microframes (125 000 ns) on
high-speed controllers and frames (1 000 000 ns) on full-speed ones. The 1 ms
frame `f` starts at counter `f × 1 000 000 / unit_ns`.

## Part 1: consuming timed SOFs

### 1. Find the controller

Your device sits on one controller, and `cmd/lookup` tells you which. You can
look it up by `device` (`"vvvv:pppp"`) plus `serial`, by sysfs `port`
(`"1-2"`), or by `busnum` and `devnum`:

```sh
mosquitto_rr -V 5 -t sofd/pc1/cmd/lookup -e me/reply -m '{"device":"1234:5678","serial":"ABC123"}'
# {"ok":true,"controller":"0000:00:14.0","supported":true,"buses":[1,2],"device":{…}}
```

`cmd/list` returns every controller with its devices, its current `rate_hz`
and the boards attached to it. The retained `controllers/+/info` topics hold
the same information per controller.

#### Commands without MQTT 5 (CI pipelines, MQTT 3.1.1 clients)

`mosquitto_rr` above relies on the MQTT 5 response topic. MQTT 3.1.1 doesn't
have one, so every `cmd/*` also accepts the reply topic in its body:

| Field | Meaning |
|---|---|
| `reply_to` | the topic for the reply. It is a full topic name, not relative to the prefix and not inside it, and it can't contain `+` or `#`. |
| `correlation` | optional, any JSON value. It is copied unchanged into the reply. |

sofd removes both fields before the command sees the request, so you can add
them to any command. The reply is published once, at QoS 1 and not retained.
Subscribe to the reply topic before you send the command, or you will miss it.
Don't retain the command: sofd ignores retained commands.
A `reply_to` in the body wins over an MQTT 5 response topic. Errors come back
the same way, with `"ok":false`, an `"error"` and your `correlation`. A command
without any reply topic still runs, but its answer only goes to the log.

```sh
mosquitto_sub -V mqttv311 -C 1 -W 10 -t ci/job42/lookup &     # subscribe first
sleep 0.5
mosquitto_pub -V mqttv311 -t sofd/pc1/cmd/lookup \
  -m '{"device":"1234:5678","serial":"ABC123","reply_to":"ci/job42/lookup","correlation":{"job":42}}'
wait
# {"buses":[1,2],"controller":"0000:00:14.0","correlation":{"job":42},"device":{…},"ok":true,"supported":true}
```

`-C 1` exits after the reply and `-W 10` gives up after 10 s, which also
covers the case where sofd isn't running. When pipelines run in parallel, use
a reply topic that is unique per job as above, or match on `correlation`.

### 2. Make sure it is sampled

A controller with a sofgnss board on it is already sampled, at
`sampler.default_rate_hz` (50 Hz by default). For any other controller, ask
for it with a retained request on your own `want` topic. Set your MQTT will to
an empty retained payload on the same topic, so the request goes away when
your client does:

```sh
mosquitto_pub -r -t 'sofd/pc1/controllers/0000:00:14.0/want/myapp' -m '{"rate_hz":20}'   # or '{}' = default rate
mosquitto_pub -r -t 'sofd/pc1/controllers/0000:00:14.0/want/myapp' -n                    # withdraw
```

Some clients can't set a will, CI pipelines for example. They can give the
request a lifetime with `ttl_s` and renew it. Publish it not retained, roughly
every `ttl_s`/3. sofd checks once a second and drops the request when no
renewal has arrived within `ttl_s`. If the client crashes, sofd stops sampling
within `ttl_s` and the broker holds no stale request.

```sh
while true; do   # e.g. in the background for the duration of the job
  mosquitto_pub -V mqttv311 -t 'sofd/pc1/controllers/0000:00:14.0/want/ci-job42' -m '{"rate_hz":20,"ttl_s":10}'
  sleep 3
done
```

- Each message replaces the request, so the rate and the lifetime start over.
  A message without `ttl_s` makes the request permanent again until it is
  withdrawn. An empty payload withdraws it at once.
- `ttl_s` must be a number in (0, 86400]. sofd ignores a request with any
  other value and logs it, so a typo can't turn into a request that never
  ends.
- Don't retain a TTL request. After a sofd restart the broker delivers the
  retained copy again, sofd counts it as fresh, and sampling continues for
  another `ttl_s`.
- The lifetime is in the message body on purpose. MQTT 5's message expiry only
  expires the copy the broker stores, not sofd's request, and MQTT 3.1.1
  clients can't set it anyway.
- `controllers/<id>/info` lists each request under `wants` with its fields and
  `expires_ms` (Unix ms, or null without a TTL). You can use it to check that
  your renewals arrive.

The controller is sampled at the highest rate anyone asks for. Each sample
can cost up to 125 µs of CPU spin, and more samples improve the fit only a
little, so 10 to 50 Hz is plenty.

A request can also change what sofd publishes:

| Field | Default | Meaning |
|---|---|---|
| `rate_hz` | `sampler.default_rate_hz` (50) | sampling rate |
| `model_hz` | `sampler.model_hz` (1) | how often `model` is published. The highest request wins, up to the sampling rate. |
| `samples` | `false` | with `true`, every sample is published on `sample` while this request stands |
| `ttl_s` | none | lifetime, see above |

```sh
mosquitto_pub -r -t 'sofd/pc1/controllers/0000:00:14.0/want/myapp' -m '{"model_hz":5,"samples":true}'
```

The sampling rate decides how good the fits and the chrony feed are. The model
rate only decides how often you receive a copy, and 1 Hz is enough for
converting frames as in step 4.

### 3. Subscribe

| Topic | Use it for |
|---|---|
| `controllers/<id>/model` | The normal choice. It is retained and holds the latest fits, so you get it as soon as you subscribe. sofd publishes it `model_hz` times a second (once by default), and straight away when the epoch changes, a fit appears or disappears, or the GNSS reference changes. An empty payload means sampling stopped. |
| `controllers/<id>/sample` | Every sample with all its fields, not retained. It is only published while some client asks with `"samples":true`. Useful for logging or your own fitting. |
| `controllers/<id>/event` | `realtime_step` (CLOCK_REALTIME was stepped), `error` and `recovered`. |
| `gnss/<serial>/status` | The board's state. `ubx` means it is running normally. |

A `model` message looks like this:

```jsonc
{"epoch":1791151735041, "unit_ns":125000, "counter":1234567,
 "model_raw":{"counter_ref":…, "t_ref_ns":…, "period_ns":125000.0029, "ppm":-0.02, "resid_rms_ns":180, "n":64},
 "model_realtime":{…},
 "model_gnss":{…},                                   // null without a valid GNSS reference
 "frame_start":{"bus_frame":1234, "counter":1234560,
                "t_raw_ns":…, "t_realtime_ns":…, "t_gnss_ns":…},
 "gnss":{"board":"E072A1F9D198", "source":"tim_tp", "frame_offset":-14336,
         "realtime_minus_gnss_ns":-412}}
```

`frame_start` is a ready-made answer. The 1 ms frame `bus_frame` started at
`t_realtime_ns` in host time and at `t_gnss_ns` in GNSS time. For any other
frame you use the models.

### 4. Time a frame your device saw

A device usually only knows the 11-bit frame number, which wraps every
2.048 s. To place it, note your clock (CLOCK_REALTIME) when the device reports
the frame. Then predict from `model_realtime` which frame was running at that
moment and pick the frame with the right number nearest to it. This works with
a model that is several seconds old, as long as your clock is within a few
hundred ms of the sofd host's:

```python
#!/usr/bin/env python3
"""Host and GNSS time of USB frames, from sofd's MQTT model topic."""
import json
import time
import paho.mqtt.client as mqtt

PREFIX = "sofd/pc1"
CONTROLLER = "0000:00:14.0"
CLIENT = "myapp"
model = None


def on_message(client, userdata, msg):
    global model
    model = json.loads(msg.payload) if msg.payload else None   # empty: sampling stopped


def frame_counter(m, f11, when_ns):
    """sofd's counter (in m["epoch"]) of the 1 ms frame numbered f11 (0..2047)
    nearest to the host time when_ns (CLOCK_REALTIME, Unix ns): within
    ±1.024 s of it. None without a realtime fit."""
    fit, fs = m["model_realtime"], m["frame_start"]
    if fit is None:
        return None
    per_frame = 1_000_000 // m["unit_ns"]   # 8 counts per frame on HS, 1 on FS
    # Frame running at when_ns, counted from frame_start.
    near = round((fit["counter_ref"] - fs["counter"] + (when_ns - fit["t_ref_ns"]) / fit["period_ns"]) / per_frame)
    d = (f11 - fs["bus_frame"] - near) % 2048
    if d >= 1024:
        d -= 2048
    return fs["counter"] + (near + d) * per_frame


def frame_time(m, f11, when_ns, clock="model_gnss"):
    """Start of frame f11, reported at host time when_ns, in ns of `clock`
    (model_gnss, model_realtime or model_raw). None without that fit."""
    fit = m.get(clock)
    c = frame_counter(m, f11, when_ns)
    if fit is None or c is None:
        return None
    # Integer base plus a small float term: t_ref_ns does not fit a double exactly.
    return fit["t_ref_ns"] + round((c - fit["counter_ref"]) * fit["period_ns"])


client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, protocol=mqtt.MQTTv5)
want = f"{PREFIX}/controllers/{CONTROLLER}/want/{CLIENT}"
client.will_set(want, b"", qos=1, retain=True)
client.on_message = on_message
client.connect("localhost")
client.publish(want, "{}", qos=1, retain=True)
client.subscribe(f"{PREFIX}/controllers/{CONTROLLER}/model")
client.loop_start()

# ... when the device reports frame 1234:
#   when = time.time_ns()
#   t_gnss = frame_time(model, 1234, when)
#   t_host = frame_time(model, 1234, when, "model_realtime")
```

For microframe `u` (0 to 7) of a high-speed frame, add `u × 125 000 ns` to the
result, or add `u` to the counter before applying the model.

If your device already keeps an unwrapped frame count, as sofgnss does, you
only need to match its low 11 bits once as above. Remember the offset
`host frame − device frame` and from then on you can convert without the ±1 s
limit. Keep checking the low 11 bits on every frame, and match again when the
`epoch` changes.

### 5. Checks a client should make

- A counter belongs to one epoch. A new epoch means a new count, which happens
  when sampling restarts, the controller resets, the host suspends or the
  speed changes. Throw away offsets and stored counters from the old epoch.
  Epochs stay unique across sofd restarts.
- The 11-bit match only works if `when_ns` is within about ±1 s of the frame.
  Take it when the device reports the frame, not later when you get round to
  converting it. The age of the model itself doesn't matter for the match,
  because extrapolating the fit over 1 s adds about 100 ns at most, and less
  with a larger `sampler.window`. A model older than a few seconds means sofd
  has stopped publishing, so check `status`.
- `model_gnss` and `frame_start.t_gnss_ns` are null unless the GNSS reference
  is valid. That needs at least 5 points, with the newest tag matched and
  received within the last 5 s. They go null within 5 s of the reference
  stopping, so a non-null `model_gnss` is current. `gnss.source` is `tim_tp`,
  which is accurate, or `zda`, which is labelled from NMEA and also fine once
  running. `gnss/<serial>/status` should be `ubx`.
- The retained `model` is emptied when sampling stops, on a controller error
  and when sofd stops. `status` is `online` while sofd runs.
- `bus_frame` is null on a controller whose count doesn't give the frame
  number on the wire, which is EHCI through the generic path. The 11-bit match
  can't work there, so use form B or sofd's counter directly.
- `t_ref_ns`, `t_realtime_ns` and `t_gnss_ns` are about 1.8 × 10¹⁸ ns, and a
  double only holds them to about 256 ns. Keep them as 64-bit integers and use
  floating point only for the small difference. Python's `json` already does
  this. In C use `int64_t`, and in JavaScript parse them as `BigInt`.
- The time is the start of the frame at the host controller. Your device sees
  the SOF a little later because of cable, hub and PHY delays. That is tens to
  hundreds of ns, and more behind a full-speed transaction translator.
  Calibrate the offset once if it matters to you.
- On the development laptop the host noise per sample is 165 to 440 ns rms.
  The fits are much better than a single sample, and the board's own SOF
  capture adds about 45 ns rms. The `resid_rms_ns` of each model shows the
  current scatter.

## Part 2: feeding a GNSS reference from your program

A sofgnss board gives the controller it is plugged into its `model_gnss`. A
program that knows the GNSS time of USB frames some other way can provide it
instead, for example with its own USB device that has a timing receiver and
captures SOFs. sofd then treats it like a board. The controller's samples gain
`model_gnss`, `frame_start.t_gnss_ns` and `gnss`, other clients consume them as
in Part 1, and sofd can feed chrony from it.

If your device captures PPS and SOF on one counter, the Go module
[sofref](../sofref) does the interpolation, the publishing and the checks
below for you.

### Enabling a reference (sofd configuration)

A GNSS reference steers what every client, and possibly the system clock,
believes the time is. That is why sofd only accepts tags from clients named in
its configuration file, and only the configuration can send them to chrony:

```jsonc
"gnss": {
  "external": {
    "myapp": {                                  // the <client> in the topic
      "chrony": "/run/chrony.myapp.sock",       // optional, as for boards
      "sof_delay_ns": 0                         // subtracted from every tag
    }
  }
}
```

A `*` entry accepts any client not listed by name, with its settings. Use it
only on a broker where every client is trusted. It is a good idea to restrict
publishing to `controllers/+/ref/+` with broker ACLs as well.

### Publishing tags

1. Find the controller your device is on with `cmd/lookup`, as in Part 1.
2. Ask for sampling with a retained `controllers/<id>/want/<client>` and a will
   that clears it. Tags on their own don't start sampling.
3. Publish tags on `sofd/pc1/controllers/<id>/ref/<client>`, not retained
   (sofd ignores retained tags), at QoS 0 or 1, in one of the two forms below.

Neither form needs NTP or a correct host clock, so an external reference can
bring up chrony on a machine without network time.

#### Form A: the device's frame number (simple, prompt tags)

```jsonc
{"frame": 9213374, "t_gnss_ns": 1791151735000000000, "unc_ns": 20}
```

sofd matches the frame number by the time the tag arrives on its own monotonic
clock, the same way it does for sofgnss boards. The frame has already
happened, so it must be the latest host frame with those low 11 bits before
the tag arrived. This is only unambiguous if each tag arrives within about
1.9 s of its frame, since the number wraps after 2.048 s. Publish tags as they
come, or in small batches whose oldest tag is still that young.

A tag that arrives 1.9 to 2.0 s after its frame is rejected with `no_match`. A
later one may be matched one wrap off, but then it disagrees with the fit by
2.048 s and is rejected as `inconsistent`. An occasional delay therefore costs
you some tags but doesn't corrupt the fit. Tags that are always more than 2 s
late do cause trouble, and those should use form B.

A bare 11-bit number (below 2048) is matched afresh every time. If `frame` is
an unwrapped count, sofd locks the offset between your count and its own on
the first match and checks every later tag against it. After three mismatches
in a row it matches again. This catches a slip of your count by a multiple of
2048. In both cases the low 11 bits must be the bus frame number, because a
count with another origin gets matched to the wrong frame.

#### Form B: sofd's counter (any latency, batches)

```jsonc
{"epoch": 1791152014974, "counter": 73706148, "t_gnss_ns": 1791151735000000000, "unc_ns": 20}
```

Here the client converts its device's frame to sofd's counter itself, using
`frame_counter()` from [Part 1, step 4](#4-time-a-frame-your-device-saw) with
the time the device reported the frame. The ±1 s rule applies at that point.
It then sends the counter together with the model's `epoch`. sofd has nothing
left to resolve, so tags may be any age and can be batched freely. A tag whose
`epoch` isn't the controller's current one is dropped with `ref_rejected` and
reason `epoch`. It means the counter restarted and the client has to convert
again with the new model. The client must stay subscribed to `model` while it
sends.

#### Fields

Both forms accept a batch: `{"tags": [ {…}, {…} ]}`.

| Field | Meaning |
|---|---|
| `frame` | Form A. Your device's frame number. It must contain at least the 11-bit bus frame, and an unwrapped count is better. |
| `epoch`, `counter` | Form B. sofd's epoch and counter of the SOF, in microframes on high speed. |
| `microframe` | Form A, optional. The microframe the time refers to, 0 to 7 on high speed (default 0). It must be 0 on full speed. In form B it is part of `counter`. |
| `t_gnss_ns` | The UTC of that SOF at your device, as an integer in Unix nanoseconds between 2000 and 2100. No leap smearing and not GPS time. |
| `unc_ns` | Optional. The 1σ uncertainty in ns, from 1 ns to 1 s (default 100). Tags are weighted by 1/unc². |

Some guidelines:

- Send at least one tag per second, and at most about the sampling rate. The
  fit holds the tags of the last `sampler.gnss_window_s` (6 s). GNSS time is
  only used and sent to chrony with at least 5 tags and the newest received
  under 5 s ago.
- There is one reference per controller, and a sofgnss board on the
  controller whose receiver works takes precedence. Otherwise the first allowed client to send
  valid tags owns the controller until it has sent nothing, or only rejected
  tags, for 10 s. Other clients get `ref_busy`.
- Only send tags you trust. sofd doesn't check GNSS lock for external
  references, so hold tags back while your receiver has no fix.

### What you get back

Events arrive on `controllers/<id>/event`, each with `controller` and
`client`. sofd sends them when the state changes, not for every tag.

| Event | When |
|---|---|
| `ref_accepted` | Tags are used again. `frame_offset` is set for an unwrapped form A count and null otherwise. |
| `ref_rejected` | Comes with a `reason`, listed below. |
| `ref_busy` | Another reference holds the controller. `owner` is a board serial or `mqtt:<client>`. |
| `ref_lost` | The controller is free again. `reason` is `silent` (no tags for 10 s) or `rejected` (only rejected tags for 10 s). |
| `chrony_busy` | Another reference already feeds the configured chrony socket (`owner`). |

The reasons for `ref_rejected` are:

| Reason | Meaning |
|---|---|
| `no_match` | A form A frame fits no host frame of the last ~1.9 s. |
| `epoch` | In form B the epoch is old. In form A the frame may predate the controller's current epoch, because it arrived less than ~1.9 s after the epoch began. |
| `microframe` | Non-zero on a full-speed controller. |
| `no_bus_frame` | The controller gives no bus frame number, so use form B. |
| `inconsistent` | The tag is more than 0.5 ms off `model_gnss`. It was matched to the wrong wrap or your time jumped. After 5 such tags spread over 3 s the fit restarts from your tags. |
| `not_allowed` | The client isn't in `gnss.external` and there is no `*` entry. |
| `invalid` | The tag is malformed, and `message` says why. |

The controller's samples and model name the reference in `gnss`:

```jsonc
"gnss":{"board":"mqtt:myapp", "source":"external", "frame_offset":…, "realtime_minus_gnss_ns":…}
```

In form B `frame_offset` is null.

### Example feeder

```python
#!/usr/bin/env python3
"""Feed sofd GNSS-timed SOF tags from your own device."""
import json
import paho.mqtt.client as mqtt

PREFIX, CONTROLLER, CLIENT = "sofd/pc1", "0000:00:14.0", "myapp"
want = f"{PREFIX}/controllers/{CONTROLLER}/want/{CLIENT}"
ref = f"{PREFIX}/controllers/{CONTROLLER}/ref/{CLIENT}"

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, protocol=mqtt.MQTTv5)
client.will_set(want, b"", qos=1, retain=True)
client.connect("localhost")
client.loop_start()
client.publish(want, "{}", qos=1, retain=True)


def on_tag(frame, t_gnss_ns, unc_ns):
    """Form A: call as soon as your device has timed a SOF against GNSS."""
    client.publish(ref, json.dumps({"frame": frame, "t_gnss_ns": t_gnss_ns, "unc_ns": unc_ns}))
```

For form B, also subscribe to `controllers/<id>/model`. Convert each frame
with `frame_counter(model, f11, time.time_ns())` from Part 1 when the device
reports it, and send `{"epoch": model["epoch"], "counter": …, "t_gnss_ns": …}`
whenever it suits you.

### Tested

On the development PC a stand-in client took frames from sofd's own `model`
topic and labelled them as "GNSS" with the host time plus 5 µs. sofd's
`realtime_minus_gnss_ns` came back at −5023 ns with bare 11-bit frames in
1.5 s batches, −4999 ns with an unwrapped count and −5015 ns with form B in
4 s batches. The chrony socket received +5 µs offsets. A client that published
while the sofgnss board was attached got a single `ref_busy`.
