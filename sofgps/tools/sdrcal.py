#!/usr/bin/python3
"""Reference analysis for the sofgps SDR calibration bursts.

Run with the system Python (needs numpy, scipy, pyserial), not the ESP-IDF one.

  log       Capture the device's $PVDB sentences with host timestamps while
            calibration bursts run:  sdrcal.py log --mode NOISE -o cal.log
  tone      SDR frequency error from a square-wave recording (SQ2/SQ20/SQ40):
            sdrcal.py tone rec.cs16 --fs 2.048e6 --fc 162e6 --mode SQ2 [--log cal.log]
  noise     Burst timing from a NOISE recording, paired with the CAL sentences:
            sdrcal.py noise rec.cs16 --fs 2.048e6 --fc 160e6 --log cal.log --iq-start <unix>
            Gives the SDR sample-clock error (= LO error: the MSi SDR shares
            one crystal), sample index <-> device tick, residuals, and the
            measured PPS->burst offset.
  selftest  Synthesise recordings with known errors and check both analyses.

IQ formats: cs16 (default), cs8, cu8, cf32; interleaved I/Q.
Frequencies and rates are nominal; errors are reported in ppm.

Model: the SDR's true sample rate is fs*(1+es) and its true LO fc*(1+es).
The device's counter runs at tick_hz*(1+ed); ed comes from the PPS sentences
in the log when present (GPS time), else 0 (device time).
"""
import argparse
import json
import math
import os
import re
import sys
import time

import numpy as np
from scipy.signal import czt

TICK_HZ = 80_000_000
CHIP_RATE = 40_000_000
CAL_BITS = 1 << 18
BURST_S = CAL_BITS / CHIP_RATE  # 6.5536 ms
SQUARE_F0 = {"SQ2": 2e6, "SQ20": 20e6, "SQ40": 40e6}


# ---------------------------------------------------------------- sequences --

def noise_bits():
    """The NOISE burst: degree-18 m-sequence plus one zero pad bit."""
    out = np.zeros(CAL_BITS, np.uint8)
    s = 0x3FFFF
    for i in range(CAL_BITS - 1):
        out[i] = s & 1
        fb = (s ^ (s >> 7)) & 1
        s = (s >> 1) | (fb << 17)
    return out


# ------------------------------------------------------------------ IQ file --

def open_iq(path, fmt):
    if fmt == "cs16":
        raw = np.memmap(path, np.int16, "r")
        scale, offset = 1 / 32768, 0.0
    elif fmt == "cs8":
        raw = np.memmap(path, np.int8, "r")
        scale, offset = 1 / 128, 0.0
    elif fmt == "cu8":
        raw = np.memmap(path, np.uint8, "r")
        scale, offset = 1 / 128, -127.5
    elif fmt == "cf32":
        raw = np.memmap(path, np.float32, "r")
        scale, offset = 1.0, 0.0
    else:
        sys.exit(f"unknown format {fmt}")
    n = len(raw) // 2

    def read(start, count):
        start = max(0, start)
        count = max(0, min(count, n - start))
        seg = raw[2 * start:2 * (start + count)].astype(np.float64)
        seg = (seg + offset) * scale
        return seg[0::2] + 1j * seg[1::2]

    return n, read


def find_bursts(n, read, fs):
    """Rough burst start samples. Power is averaged over a burst length so weak
    bursts still show up."""
    block = max(16, int(fs * 0.5e-3))
    chunk = block * 4096
    powers = []
    for start in range(0, n, chunk):
        x = read(start, chunk)
        m = len(x) // block
        if m == 0:
            break
        powers.append(np.mean(np.abs(x[:m * block].reshape(m, block)) ** 2, axis=1))
    p = np.concatenate(powers)
    width = max(1, int(round(BURST_S * fs / block)))
    smooth = np.convolve(p, np.ones(width) / width, mode="full")[:len(p)]  # trailing average
    med = np.median(smooth)
    sigma = 1.4826 * np.median(np.abs(smooth - med)) + 1e-30
    on = smooth > med + 8 * sigma
    dyn = 10 * np.log10(np.max(smooth) / med) if med > 0 else 0.0
    starts, last = [], -10**12
    min_gap = int(0.5 * fs / block)  # bursts are ~1 s apart
    blen = int(round(BURST_S * fs))
    for i in np.flatnonzero(on[1:] & ~on[:-1]) + 1:
        if i - last >= min_gap:
            last = i
            # Refine: find the burst-long window with the most energy.
            s0 = max(0, int((i - 2 * width) * block))
            x = read(s0, 4 * blen)
            fine = max(1, blen // 256)
            m = len(x) // fine
            if m <= blen // fine:
                continue
            e = np.sum(np.abs(x[:m * fine].reshape(m, fine)) ** 2, axis=1)
            w = blen // fine
            sums = np.convolve(e, np.ones(w), mode="valid")
            starts.append(s0 + int(np.argmax(sums)) * fine)
    return starts, dyn


# ------------------------------------------------------------- log parsing --

SENT = re.compile(r"\$PVDB,SOFPPS,([A-Z0-9]+),([^*]*)\*")


def read_log(path):
    """Parse 'unix_time $PVDB,...' lines into (pps, cal) lists."""
    pps, cal = [], []
    with open(path) as f:
        for line in f:
            parts = line.split(None, 1)
            if len(parts) != 2:
                continue
            m = SENT.search(parts[1])
            if not m:
                continue
            host = float(parts[0])
            kind, fields = m.group(1), m.group(2).split(",")
            if kind == "PPS":
                pps.append({"host": host, "tick": int(fields[0])})
            elif kind == "CAL" and len(fields) >= 4:
                cal.append({"host": host, "tick": int(fields[0]), "mode": fields[1],
                            "synced": fields[2] == "1", "n": int(fields[3])})
    return pps, cal


def device_clock(pps):
    """(ed, tick_hz_true) from consecutive PPS ticks; (0, TICK_HZ) without PPS."""
    ticks = np.array([p["tick"] for p in pps], np.int64)
    if len(ticks) < 3:
        return 0.0, float(TICK_HZ)
    d = np.diff(ticks).astype(np.float64)
    d = d[np.abs(d - TICK_HZ) < TICK_HZ * 1e-3]  # skip missing PPS
    if len(d) < 2:
        return 0.0, float(TICK_HZ)
    hz = float(np.median(d))
    return hz / TICK_HZ - 1, hz


# --------------------------------------------------------------------- tone --

def cmd_tone(a):
    n, read = open_iq(a.iq, a.format)
    f0 = SQUARE_F0[a.mode]
    ed = 0.0
    if a.log:
        ed, _ = device_clock(read_log(a.log)[0])
    starts, dyn = find_bursts(n, read, a.fs)
    if not starts:
        sys.exit(f"no bursts found (dynamic range {dyn:.1f} dB)")
    # Odd harmonics inside the SDR band, away from DC (LO leakage).
    lo, hi = a.fc - 0.45 * a.fs, a.fc + 0.45 * a.fs
    ks = [k for k in range(1, int(hi / f0) + 2, 2) if lo < k * f0 < hi and abs(k * f0 - a.fc) > 0.01 * a.fs]
    if not ks:
        sys.exit("no odd harmonic of the square wave falls inside the SDR band")
    blen = int(BURST_S * a.fs * 0.9)
    nfft = 1 << (int(math.ceil(math.log2(blen))) + 4)
    win = np.hanning(blen)
    freqs = np.fft.fftfreq(nfft, 1 / a.fs)
    est = []
    for s in starts:
        x = read(s + int(BURST_S * a.fs * 0.05), blen)
        if len(x) < blen:
            continue
        spec = np.abs(np.fft.fft(x * win, nfft))
        for k in ks:
            f_rf = k * f0 * (1 + ed)
            expect = f_rf - a.fc  # baseband with es = 0
            span = abs(f_rf) * a.ppm_range * 1e-6 + 4 * a.fs / blen
            idx = np.flatnonzero(np.abs(freqs - expect) <= span)
            if len(idx) < 3:
                continue
            i = idx[np.argmax(spec[idx])]
            y0, y1, y2 = np.log(spec[(i - 1) % nfft] + 1e-30), np.log(spec[i] + 1e-30), np.log(spec[(i + 1) % nfft] + 1e-30)
            frac = 0.5 * (y0 - y2) / (y0 - 2 * y1 + y2) if (y0 - 2 * y1 + y2) != 0 else 0.0
            f_meas = freqs[i] + frac * a.fs / nfft
            # f_meas = f_rf/(1+es) - fc  ->  es
            es = f_rf / (f_meas + a.fc) - 1
            snr = 20 * np.log10(spec[i] / (np.median(spec[idx]) + 1e-30))
            est.append((es, snr, k))
    if not est:
        sys.exit("no harmonic line found")
    arr = np.array([e[0] for e in est])
    w = np.array([10 ** (e[1] / 20) for e in est])
    es = float(np.average(arr, weights=w))
    res = {
        "bursts": len(starts),
        "harmonics": sorted({e[2] for e in est}),
        "device_ppm": ed * 1e6,
        "sdr_ppm": es * 1e6,
        "sdr_ppm_std": float(np.std(arr) * 1e6),
        "lo_offset_hz_at_fc": es * a.fc,
    }
    report(a, res, [
        f"{res['bursts']} bursts, harmonics {res['harmonics']} (x {f0/1e6:g} MHz)",
        f"device clock {res['device_ppm']:+.4f} ppm ({'from PPS' if a.log and ed else 'assumed 0'})",
        f"SDR clock    {res['sdr_ppm']:+.4f} ppm  (std over lines/bursts {res['sdr_ppm_std']:.4f} ppm)",
        f"LO offset at {a.fc/1e6:g} MHz: {res['lo_offset_hz_at_fc']:+.1f} Hz",
    ])
    return res


# -------------------------------------------------------------------- noise --

_bits_cache = None


def reference(fs, fc, es, ed, nfft):
    """Expected baseband NOISE burst for SDR clock error es and device clock
    error ed. Built in the frequency domain so the band limit is exact."""
    global _bits_cache
    if _bits_cache is None:
        _bits_cache = noise_bits().astype(np.float64)
    rc = CHIP_RATE * (1 + ed)
    # SDR bin m (-N/2..N/2-1) <-> true RF frequency F.
    df = (1 + es) * fs / nfft
    f_start = (1 + es) * (fc - fs / 2)
    w = np.exp(-2j * np.pi * df / rc)
    a = np.exp(2j * np.pi * f_start / rc)
    seq = czt(_bits_cache, nfft, w, a)          # sum_i b_i exp(-j2pi F i / rc)
    F = f_start + df * np.arange(nfft)
    shape = np.sinc(F / rc) * np.exp(-1j * np.pi * F / rc)  # one chip, starting at 0
    spec = np.fft.ifftshift(seq * shape)
    return np.fft.ifft(spec)


def correlate_burst(read, s0, ref_spec, nfft, fs, margin, coarse=False):
    """Burst start (fractional sample), peak/median quality and carrier phase."""
    x = read(s0 - margin, nfft)
    if len(x) < nfft:
        x = np.concatenate([x, np.zeros(nfft - len(x))])
    C = np.fft.fft(x) * np.conj(ref_spec)
    c = np.fft.ifft(C)
    valid = c[:2 * margin]
    if coarse:  # acquisition: only the peak quality matters
        return None, float(np.max(np.abs(valid)) / (np.median(np.abs(valid)) + 1e-30)), 0.0
    # Near a spectral null, sidelobes can beat a peak that falls between
    # samples, so find the peak on an 8x upsampled correlation first.
    up = 8
    Cz = np.zeros(nfft * up, np.complex128)
    Cz[:nfft // 2] = C[:nfft // 2]
    Cz[-(nfft // 2):] = C[nfft // 2:]
    cu = np.abs(np.fft.ifft(Cz)[:2 * margin * up])
    p = int(np.argmax(cu)) / up
    k = np.fft.fftfreq(nfft) * nfft

    def at(taus):
        return np.exp(2j * np.pi * np.outer(taus, k) / nfft) @ C / nfft

    center, half = p, 1.0 / up
    for _ in range(3):
        taus = center + np.linspace(-half, half, 41)
        v = at(taus)
        i = int(np.argmax(np.abs(v)))
        center, half = taus[i], half / 20
    peak = at(np.array([center]))[0]
    floor = np.median(np.abs(valid))
    return s0 - margin + center, abs(peak) / (floor + 1e-30), float(np.angle(peak))


def cmd_noise(a):
    n, read = open_iq(a.iq, a.format)
    pps, cal = read_log(a.log)
    cal = [c for c in cal if c["mode"] == "NOISE"]
    if not cal:
        sys.exit("no NOISE CAL sentences in the log")
    ed, tick_hz = device_clock(pps)
    starts, dyn = find_bursts(n, read, a.fs)
    if not starts:
        sys.exit(f"no bursts found (dynamic range {dyn:.1f} dB)")
    margin = int(10e-3 * a.fs)  # coarse starts are good to a few ms
    blen = int(math.ceil(BURST_S * a.fs))
    nfft = 1 << int(math.ceil(math.log2(blen + 4 * margin)))

    es = a.ppm * 1e-6 if a.ppm is not None else None
    if es is None:
        # Search the SDR ppm on the first burst, in steps under ~1/(2T).
        step = 0.4 / (BURST_S * a.fc)
        best = (0, 0.0)
        for e in np.arange(-a.ppm_range * 1e-6, a.ppm_range * 1e-6 + step, step):
            ref = np.fft.fft(reference(a.fs, a.fc, e, ed, nfft))
            _, q, _ = correlate_burst(read, starts[0], ref, nfft, a.fs, margin, coarse=True)
            if q > best[0]:
                best = (q, e)
        es = best[1]
        print(f"acquisition: SDR {es*1e6:+.3f} ppm (peak/median {best[0]:.0f})", file=sys.stderr)

    for it in range(4):
        ref = np.fft.fft(reference(a.fs, a.fc, es, ed, nfft))
        found = []
        for s in starts:
            pos, q, ph = correlate_burst(read, s, ref, nfft, a.fs, margin)
            if q > a.min_quality:
                found.append((pos, q, ph))
        if len(found) < 3:
            sys.exit(f"only {len(found)} bursts correlated; check --fc/--fs/--ppm")
        pairs = pair_bursts(a, found, cal, tick_hz)
        t = np.array([(c["tick"] - pairs[0][1]["tick"]) / tick_hz for _, c in pairs])
        pos = np.array([f[0] for f, _ in pairs])
        b, a0 = np.polyfit(t, pos, 1)
        es_new = b / a.fs - 1
        if abs(es_new - es) < 1e-9:
            es = es_new
            break
        es = es_new
    resid_s = (pos - (a0 + b * t)) / b
    offsets = pps_to_cal(pps, cal, tick_hz)
    res = {
        "bursts": len(pairs),
        "device_ppm": ed * 1e6,
        "sdr_ppm": es * 1e6,
        "lo_offset_hz_at_fc": es * a.fc,
        "resid_rms_ns": float(np.sqrt(np.mean(resid_s ** 2)) * 1e9),
        "resid_max_ns": float(np.max(np.abs(resid_s)) * 1e9),
        "anchor": {"tick": pairs[0][1]["tick"], "n": pairs[0][1]["n"], "sample": float(a0)},
        "samples_per_second": float(b),
        "pps_to_cal_us": offsets,
    }
    report(a, res, [
        f"{res['bursts']} bursts paired (CAL n {pairs[0][1]['n']}..{pairs[-1][1]['n']})",
        f"device clock {res['device_ppm']:+.4f} ppm ({'from PPS' if len(pps) > 2 else 'assumed 0'})",
        f"SDR clock    {res['sdr_ppm']:+.5f} ppm -> LO offset at {a.fc/1e6:g} MHz {res['lo_offset_hz_at_fc']:+.2f} Hz",
        f"timing residual rms {res['resid_rms_ns']:.2f} ns, max {res['resid_max_ns']:.2f} ns",
        f"mapping: sample {a0:.3f} = tick {pairs[0][1]['tick']}, {b:.4f} samples per second",
        ("PPS->burst offset: mean {mean:.3f} us, std {std:.3f} us, n {n}".format(**offsets)
         if offsets else "PPS->burst offset: no PPS in log"),
    ])
    return res


def pair_bursts(a, found, cal, tick_hz):
    """Match bursts to CAL sentences. The first match needs --first-n or --iq-start."""
    cal = sorted(cal, key=lambda c: c["n"])
    if a.first_n is not None:
        first = next((c for c in cal if c["n"] == a.first_n), None)
        if first is None:
            sys.exit(f"CAL n={a.first_n} not in log")
    elif a.iq_start is not None:
        t_host = a.iq_start + found[0][0] / a.fs
        # The CAL sentence arrives a few ms after its burst.
        cand = [c for c in cal if -0.05 < c["host"] - t_host < 0.45]
        if len(cand) != 1:
            sys.exit(f"cannot pair the first burst by host time ({len(cand)} candidates); use --first-n")
        first = cand[0]
    else:
        sys.exit("pairing needs --iq-start (unix time of the first sample) or --first-n")
    pairs = []
    for f in found:
        dt = (f[0] - found[0][0]) / a.fs  # seconds, good to ppm
        want = first["tick"] + dt * tick_hz
        c = min(cal, key=lambda c: abs(c["tick"] - want))
        if abs(c["tick"] - want) < 0.01 * tick_hz:
            pairs.append((f, c))
    return pairs


def pps_to_cal(pps, cal, tick_hz):
    ticks = sorted(p["tick"] for p in pps)
    if not ticks:
        return None
    offs = []
    for c in cal:
        if not c["synced"]:
            continue
        i = np.searchsorted(ticks, c["tick"]) - 1
        if i >= 0 and c["tick"] - ticks[i] < tick_hz * 0.1:
            offs.append((c["tick"] - ticks[i]) / tick_hz * 1e6)
    if not offs:
        return None
    return {"mean": float(np.mean(offs)), "std": float(np.std(offs)), "n": len(offs)}


def report(a, res, lines):
    if getattr(a, "json", False):
        print(json.dumps(res, indent=2))
    else:
        print("\n".join(lines))


# ---------------------------------------------------------------------- log --

def cmd_log(a):
    import serial

    def sentence(body):
        s = 0
        for ch in body:
            s ^= ord(ch)
        return f"${body}*{s:02X}\r\n".encode()

    p = serial.Serial(a.port, 115200, timeout=0.2)
    p.write(sentence(f"PVDB,SOFPPS,CAL,{a.mode}"))
    last_ping = time.time()
    end = time.time() + a.seconds if a.seconds else float("inf")
    count = 0
    with open(a.output, "w") as out:
        try:
            while time.time() < end:
                line = p.readline()
                now = time.time()
                if now - last_ping > 20:
                    p.write(sentence("PVDB,SOFPPS,PING"))
                    last_ping = now
                if not line.startswith(b"$PVDB"):
                    continue
                text = line.decode(errors="replace").strip()
                out.write(f"{now:.6f} {text}\n")
                if ",CAL," in text:
                    count += 1
                    print(f"\r{count} bursts", end="", file=sys.stderr, flush=True)
        except KeyboardInterrupt:
            pass
        finally:
            p.write(sentence("PVDB,SOFPPS,CAL,OFF"))
            p.close()
    print(f"\nwrote {a.output}", file=sys.stderr)


# ----------------------------------------------------------------- selftest --

def synth(path_iq, path_log, mode, fs, fc, es, ed, seconds, snr_db, rng):
    """Synthetic recording and log with known clock errors."""
    tick_hz = TICK_HZ * (1 + ed)
    n = int(seconds * fs)
    x = np.zeros(n, np.complex128)
    lines = []
    iq_start = 1_700_000_000.0
    pps0 = int(0.3 * tick_hz)
    nfft_ref = 1 << int(math.ceil(math.log2(BURST_S * fs * 1.3)))
    base_ref = None
    if mode == "NOISE":
        base_ref = np.fft.fft(reference(fs, fc, es, ed, nfft_ref))
    freqs = np.fft.fftfreq(nfft_ref)
    for i in range(int(seconds) - 1):
        pps_tick = pps0 + round(i * tick_hz)
        cal_tick = pps_tick + 256 + int(rng.integers(0, 40))
        t_true = cal_tick / tick_hz
        pos = t_true * fs * (1 + es)
        lines.append(f"{iq_start + pps_tick / tick_hz + 0.002:.6f} $PVDB,SOFPPS,PPS,{pps_tick}*00")
        lines.append(f"{iq_start + t_true + 0.009:.6f} $PVDB,SOFPPS,CAL,{cal_tick},{mode},1,{i}*00")
        k0 = int(math.floor(pos))
        frac = pos - k0
        if mode == "NOISE":
            carrier = np.exp(-2j * np.pi * fc * (1 + es) * t_true)
            w = np.fft.ifft(base_ref * np.exp(-2j * np.pi * freqs * frac)) * carrier
        else:
            f0 = SQUARE_F0[mode]
            m = np.arange(nfft_ref)
            w = np.zeros(nfft_ref, np.complex128)
            kmax = int((fc + fs) / f0) + 2
            for k in range(1, kmax, 2):
                f_rf = k * f0 * (1 + ed)
                fb = f_rf / (1 + es) - fc
                if abs(fb) < fs / 2:
                    # Random phase; only the frequency matters.
                    w += (2 / (np.pi * k)) * np.exp(2j * np.pi * (fb / fs * (m - frac) + rng.random()))
            w[int(BURST_S * fs):] = 0
        end = min(n, k0 + nfft_ref)
        x[k0:end] += w[:end - k0]
    sig = np.sqrt(np.mean(np.abs(x[x != 0]) ** 2))
    noise = sig / (10 ** (snr_db / 20)) / np.sqrt(2)
    x += noise * (rng.standard_normal(n) + 1j * rng.standard_normal(n))
    peak = np.max(np.abs(np.concatenate([x.real, x.imag])))
    q = np.empty(2 * n, np.int16)
    q[0::2] = np.round(x.real / peak * 30000)
    q[1::2] = np.round(x.imag / peak * 30000)
    q.tofile(path_iq)
    with open(path_log, "w") as f:
        f.write("\n".join(lines) + "\n")
    return iq_start


def cmd_selftest(a):
    rng = np.random.default_rng(1)
    d = a.dir
    os.makedirs(d, exist_ok=True)
    es, ed = 23.4e-6, -3.1e-6
    ok = True

    # Tone: SQ2 around 162 MHz.
    fs, fc = 2.048e6, 162.3e6
    iq, log = os.path.join(d, "sq2.cs16"), os.path.join(d, "sq2.log")
    synth(iq, log, "SQ2", fs, fc, es, ed, 6, 10, rng)
    ns = argparse.Namespace(iq=iq, format="cs16", fs=fs, fc=fc, mode="SQ2", log=log, ppm_range=100, json=False)
    print("== tone, SQ2 at 162.3 MHz")
    r = cmd_tone(ns)
    err = r["sdr_ppm"] - es * 1e6
    print(f"   error {err:+.4f} ppm")
    ok &= abs(err) < 0.02

    # Noise: exactly on the 160 MHz null, unknown SDR ppm (acquisition).
    fs, fc = 2.048e6, 160.0e6
    iq, log = os.path.join(d, "noise.cs16"), os.path.join(d, "noise.log")
    iq_start = synth(iq, log, "NOISE", fs, fc, es, ed, 8, -5, rng)
    ns = argparse.Namespace(iq=iq, format="cs16", fs=fs, fc=fc, log=log, ppm=None, ppm_range=40,
                            iq_start=iq_start, first_n=None, min_quality=8, json=False)
    print("== noise, 160.0 MHz (on the null), SNR -5 dB per sample")
    r = cmd_noise(ns)
    err = r["sdr_ppm"] - es * 1e6
    print(f"   error {err:+.5f} ppm")
    ok &= abs(err) < 0.005 and r["resid_rms_ns"] < 5 and abs(r["device_ppm"] - ed * 1e6) < 0.01
    print("selftest", "PASSED" if ok else "FAILED")
    return 0 if ok else 1


# --------------------------------------------------------------------- main --

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("log", help="record $PVDB sentences with host timestamps")
    p.add_argument("--port", default="/dev/ttyACM0")
    p.add_argument("--mode", default="NOISE", choices=["NOISE", "SQ2", "SQ20", "SQ40"])
    p.add_argument("--seconds", type=float, default=0, help="0: until Ctrl-C")
    p.add_argument("-o", "--output", default="cal.log")

    def common(p):
        p.add_argument("iq")
        p.add_argument("--format", default="cs16", choices=["cs16", "cs8", "cu8", "cf32"])
        p.add_argument("--fs", type=float, required=True, help="nominal sample rate, Hz")
        p.add_argument("--fc", type=float, required=True, help="nominal centre frequency, Hz")
        p.add_argument("--ppm-range", type=float, default=100, help="SDR error search range, ppm")
        p.add_argument("--json", action="store_true")

    p = sub.add_parser("tone", help="SDR frequency error from a square-wave recording")
    common(p)
    p.add_argument("--mode", required=True, choices=list(SQUARE_F0))
    p.add_argument("--log", help="log with PPS sentences, for the device clock error")

    p = sub.add_parser("noise", help="burst timing from a NOISE recording")
    common(p)
    p.add_argument("--log", required=True)
    p.add_argument("--ppm", type=float, help="SDR error if known (e.g. from 'tone'); else acquired")
    p.add_argument("--iq-start", type=float, help="unix time of the first IQ sample (coarse)")
    p.add_argument("--first-n", type=int, help="CAL n of the first burst in the recording")
    p.add_argument("--min-quality", type=float, default=8, help="peak/median correlation threshold")

    p = sub.add_parser("selftest", help="synthesise recordings and check the analyses")
    p.add_argument("--dir", default="/tmp/sdrcal-selftest")

    a = ap.parse_args()
    if a.cmd == "log":
        cmd_log(a)
    elif a.cmd == "tone":
        cmd_tone(a)
    elif a.cmd == "noise":
        cmd_noise(a)
    else:
        sys.exit(cmd_selftest(a))


if __name__ == "__main__":
    main()
