#!/usr/bin/env python3
"""Capture I/Q through UHD (SoapyUHD over the plutosky module) and verify it.

capture  records a stream to <prefix>.sc16 (interleaved int16 I/Q) with the
         per-packet UHD metadata in <prefix>.json: every recv's tick, sample
         count and error code, plus the wall-clock time of the first and last
         recv, so the sample rate can be checked against the host clock.
check    reads <prefix>.json and reports timestamp continuity, gaps, the rate
         derived from ticks versus wall time, and a timed start's accuracy.
analyze  reads <prefix>.sc16 and reports the spectrum occupancy, the DC offset
         and I/Q balance, and searches for an LTE PSS. A found PSS repeats
         every 5 ms of the cell's clock, so the spacing of its correlation
         peaks measures the sample rate against a base station, and a clean
         peak needs correct I/Q ordering and sign.
"""

import argparse
import json
import sys
import time

import numpy as np

PSS_ROOTS = (25, 29, 34)
PSS_LENGTH = 63
LTE_SUBCARRIER_HZ = 15_000.0
PSS_PERIOD_S = 0.005
# The PSS is 1.08 MHz wide, so 1.92 MS/s is the smallest LTE rate that holds it.
PSS_RATE_HZ = 1_920_000.0
PSS_FFT = 128
CS16_FULL_SCALE = 32767.0


def zadoff_chu(root):
    n = np.arange(PSS_LENGTH)
    seq = np.exp(-1j * np.pi * root * n * (n + 1) / PSS_LENGTH)
    seq[31] = 0.0  # the DC subcarrier is empty
    return seq


def pss_time_domain(root):
    """One OFDM symbol of the PSS at 1.92 MS/s, without cyclic prefix."""
    grid = np.zeros(PSS_FFT, dtype=np.complex128)
    seq = zadoff_chu(root)
    # Subcarriers -31..-1 and 1..31 around DC, DC itself left empty.
    grid[1:32] = seq[32:63]
    grid[PSS_FFT - 31:] = seq[0:31]
    return np.fft.ifft(grid) * np.sqrt(PSS_FFT)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="mode", required=True)

    cap = sub.add_parser("capture")
    cap.add_argument("--args", required=True, help="UHD device address")
    cap.add_argument("--rate", type=float, default=30.72e6)
    cap.add_argument("--freq", type=float, default=751e6)
    cap.add_argument("--gain", type=float, default=40.0)
    cap.add_argument("--seconds", type=float, default=2.0)
    cap.add_argument("--wire", default="cs12", help="plutosky wire format stream argument")
    cap.add_argument("--start-in", type=float, default=None,
                     help="issue a timed start this many seconds ahead")
    cap.add_argument("--keep", type=float, default=None,
                     help="seconds of samples to keep from the start, default all")
    cap.add_argument("prefix")

    chk = sub.add_parser("check")
    chk.add_argument("prefix")

    ana = sub.add_parser("analyze")
    ana.add_argument("--rate", type=float, required=True)
    ana.add_argument("--pss-offsets", default="",
                     help="comma-separated carrier offsets in Hz from the capture center to search")
    ana.add_argument("--pss-raster", type=float, default=0.0,
                     help="search offsets on this raster across the whole capture bandwidth")
    ana.add_argument("--max-samples", type=int, default=0)
    ana.add_argument("prefix")

    fm = sub.add_parser("fm")
    fm.add_argument("--rate", type=float, required=True)
    fm.add_argument("--fm-offset", type=float, default=0.0,
                    help="carrier offset in Hz from the capture center")
    fm.add_argument("--max-samples", type=int, default=0)
    fm.add_argument("prefix")
    return parser.parse_args()


def capture(options):
    import uhd

    usrp = uhd.usrp.MultiUSRP(options.args)
    usrp.set_rx_rate(options.rate)
    usrp.set_rx_freq(uhd.types.TuneRequest(options.freq), 0)
    usrp.set_rx_gain(options.gain, 0)
    rate = usrp.get_rx_rate()
    freq = usrp.get_rx_freq(0)
    gain = usrp.get_rx_gain(0)
    print(f"rate {rate:.0f} freq {freq:.0f} gain {gain:.1f}", flush=True)

    stream_args = uhd.usrp.StreamArgs("sc16", "sc16")
    stream_args.channels = [0]
    stream_args.args = uhd.types.DeviceAddr(f"wire={options.wire}")
    rx = usrp.get_rx_stream(stream_args)
    spb = rx.get_max_num_samps()
    per_recv = spb * 64
    buffer = np.zeros((1, per_recv), dtype=np.uint32)
    md = uhd.types.RXMetadata()

    total = int(options.seconds * rate)
    keep = total if options.keep is None else min(total, int(options.keep * rate))
    samples = np.zeros(keep, dtype=np.uint32)
    command = uhd.types.StreamCMD(uhd.types.StreamMode.num_done)
    command.num_samps = total
    requested_tick = None
    if options.start_in is None:
        command.stream_now = True
    else:
        at = usrp.get_time_now().get_real_secs() + options.start_in
        command.stream_now = False
        command.time_spec = uhd.types.TimeSpec(at)
        requested_tick = round(at * rate)
    rx.issue_stream_cmd(command)

    recvs = []
    got = 0
    wall_first = None
    wall_last = None
    while got < total:
        n = rx.recv(buffer, md, 3.0)
        now = time.monotonic()
        code = int(md.error_code)
        tick = md.time_spec.to_ticks(rate) if md.has_time_spec else None
        recvs.append([tick, int(n), code])
        if code != 0 and n == 0:
            if code == int(uhd.types.RXMetadataErrorCode.timeout):
                print("timeout", file=sys.stderr)
                break
            continue
        if n == 0:
            continue
        if wall_first is None:
            wall_first = now
        wall_last = now
        if got < keep:
            take = min(n, keep - got)
            samples[got:got + take] = buffer[0, :take]
        got += n
        if md.end_of_burst:
            break
    del rx

    meta = {
        "rate": rate, "freq": freq, "gain": gain, "wire": options.wire,
        "requested": total, "got": got, "kept": min(got, keep),
        "requested_tick": requested_tick,
        "wall_first": wall_first, "wall_last": wall_last,
        "recvs": recvs,
    }
    with open(options.prefix + ".json", "w") as f:
        json.dump(meta, f)
    samples[:min(got, keep)].tofile(options.prefix + ".sc16")
    print(f"got {got} of {total} samples, kept {min(got, keep)}, {len(recvs)} recvs", flush=True)
    return check(options)


def check(options):
    with open(options.prefix + ".json") as f:
        meta = json.load(f)
    rate = meta["rate"]
    recvs = [r for r in meta["recvs"] if r[1] > 0 and r[0] is not None]
    gaps = 0
    missing = 0
    overlaps = 0
    expected = None
    for tick, n, code in recvs:
        if expected is not None and tick != expected:
            if tick > expected:
                gaps += 1
                missing += tick - expected
            else:
                overlaps += 1
        expected = tick + n
    codes = {}
    for _, _, code in meta["recvs"]:
        codes[code] = codes.get(code, 0) + 1
    if not recvs:
        print("no samples received")
        print("CONTINUITY FAIL")
        return 1
    first_tick = recvs[0][0]
    last_tick = recvs[-1][0] + recvs[-1][1]
    span_ticks = last_tick - first_tick
    wall = None
    if meta["wall_first"] is not None and meta["wall_last"] is not None and len(recvs) > 1:
        # The last recv's samples ended a little before it returned, so the
        # comparison runs between the first recv's start and the last recv's
        # start, which the ticks address exactly.
        wall = meta["wall_last"] - meta["wall_first"]
        ticks_between = recvs[-1][0] - first_tick
    print(f"recvs {len(meta['recvs'])} error codes {codes}")
    print(f"first tick {first_tick} ({first_tick / rate:.6f} s), span {span_ticks} ticks = {span_ticks / rate:.6f} s")
    print(f"timestamp gaps {gaps} (missing {missing} samples), overlaps {overlaps}")
    if meta["requested_tick"] is not None:
        err = first_tick - meta["requested_tick"]
        print(f"timed start: requested tick {meta['requested_tick']}, first {first_tick}, error {err} ticks ({err / rate * 1e6:.1f} us)")
    if wall:
        derived = ticks_between / wall
        print(f"rate from ticks vs host clock: {derived:.1f} Hz over {wall:.3f} s "
              f"({(derived / rate - 1) * 1e6:+.0f} ppm, includes host scheduling jitter)")
    ok = gaps == 0 and overlaps == 0 and meta["got"] >= meta["requested"]
    print("CONTINUITY", "OK" if ok else "FAIL")
    return 0 if ok else 1


def load_samples(prefix, max_samples):
    raw = np.fromfile(prefix + ".sc16", dtype=np.int16)
    if max_samples:
        raw = raw[:2 * max_samples]
    return (raw[0::2].astype(np.float32) + 1j * raw[1::2].astype(np.float32)) / CS16_FULL_SCALE


def spectrum(iq, rate, bins=2048):
    segments = len(iq) // bins
    x = iq[:segments * bins].reshape(segments, bins) * np.hanning(bins)
    psd = np.mean(np.abs(np.fft.fftshift(np.fft.fft(x, axis=1), axes=1)) ** 2, axis=0)
    freqs = np.fft.fftshift(np.fft.fftfreq(bins, 1.0 / rate))
    return freqs, 10 * np.log10(psd + 1e-20)


def shift_and_decimate(iq, rate, offset_hz):
    """Mix the carrier at offset_hz to DC and resample to the PSS rate."""
    n = np.arange(len(iq))
    mixed = iq * np.exp(-2j * np.pi * offset_hz / rate * n)
    factor = rate / PSS_RATE_HZ
    if abs(factor - round(factor)) < 1e-9:
        factor = int(round(factor))
        # A short FIR keeps the 1.08 MHz PSS and rejects the rest of the carrier.
        taps = 8 * factor + 1
        window = np.sinc(np.arange(taps) - taps // 2) * np.hamming(taps)
        window = np.sinc((np.arange(taps) - taps // 2) / factor) * np.hamming(taps) / factor
        filtered = np.convolve(mixed, window, mode="same")
        return filtered[::factor]
    raise SystemExit(f"rate {rate} is not an integer multiple of {PSS_RATE_HZ}")


def pss_search(iq, rate, offsets):
    """Correlate every PSS root at each carrier offset and report the best."""
    references = {root: pss_time_domain(root) for root in PSS_ROOTS}
    period = int(round(PSS_PERIOD_S * PSS_RATE_HZ))
    results = []
    for offset in offsets:
        x = shift_and_decimate(iq, rate, offset)
        power = np.mean(np.abs(x) ** 2) + 1e-12
        for root, ref in references.items():
            corr = np.abs(np.correlate(x, ref, mode="valid")) ** 2 / (power * PSS_FFT)
            peak = int(np.argmax(corr))
            ratio = corr[peak] / (np.median(corr) + 1e-12)
            results.append((ratio, offset, root, peak, corr))
    results.sort(key=lambda r: -r[0])
    return results, period


def analyze_pss(best, period, rate):
    ratio, offset, root, peak, corr = best
    print(f"best PSS: root {root} (N_ID_2 {PSS_ROOTS.index(root)}) at offset {offset / 1e6:+.3f} MHz, "
          f"peak/median {ratio:.0f}")
    if ratio < 30:
        print("PSS not convincing, skipping periodicity")
        return
    # Peaks every 5 ms: take the strongest sample in each expected window.
    half = period // 2
    positions = []
    start = peak % period
    pos = start
    while pos + half < len(corr):
        lo = max(0, pos - 20)
        hi = min(len(corr), pos + 21)
        local = lo + int(np.argmax(corr[lo:hi]))
        if corr[local] > 0.25 * corr[peak]:
            positions.append(local)
        pos += period
    if len(positions) < 3:
        print(f"only {len(positions)} PSS peaks, need 3 for a period estimate")
        return
    positions = np.array(positions, dtype=np.float64)
    fit = np.polyfit(np.arange(len(positions)), positions, 1)
    measured = fit[0]
    ppm = (measured / period - 1) * 1e6
    resid = positions - np.polyval(fit, np.arange(len(positions)))
    print(f"PSS peaks {len(positions)} over {len(corr) / PSS_RATE_HZ:.3f} s, "
          f"spacing {measured:.3f} samples at {PSS_RATE_HZ:.0f} S/s (nominal {period})")
    print(f"sample rate versus the cell: {ppm:+.1f} ppm, peak jitter {np.max(np.abs(resid)):.2f} samples")
    lost = int(round((measured - period) * len(positions)))
    print("no sample loss inside the capture" if abs(measured - period) < 0.5 else
          f"period error implies about {lost} samples gained or lost across the capture")
    print("PSS PERIOD", "OK" if abs(ppm) < 50 and np.max(np.abs(resid)) < 2 else "FAIL")


def analyze(options):
    iq = load_samples(options.prefix, options.max_samples)
    rate = options.rate
    print(f"{len(iq)} samples ({len(iq) / rate:.3f} s) at {rate:.0f} S/s")
    dc = np.mean(iq)
    rms = np.sqrt(np.mean(np.abs(iq) ** 2))
    print(f"rms {rms:.4f} of full scale, dc {dc.real:+.4f}{dc.imag:+.4f}j, peak {np.max(np.abs(iq)):.3f}")
    i_rms = np.sqrt(np.mean(iq.real ** 2))
    q_rms = np.sqrt(np.mean(iq.imag ** 2))
    corr_iq = np.mean(iq.real * iq.imag) / (i_rms * q_rms + 1e-12)
    print(f"I/Q rms ratio {i_rms / (q_rms + 1e-12):.4f}, I/Q correlation {corr_iq:+.4f}")

    freqs, psd = spectrum(iq, rate)
    floor = np.percentile(psd, 20)
    above = psd > floor + 10
    print(f"spectrum: noise floor {floor:.1f} dB, {np.count_nonzero(above) / len(psd) * 100:.1f}% of bins 10 dB above it")
    # Occupied blocks, merged over neighbouring bins.
    blocks = []
    start = None
    for i, on in enumerate(above):
        if on and start is None:
            start = i
        if not on and start is not None:
            blocks.append((start, i))
            start = None
    if start is not None:
        blocks.append((start, len(above)))
    for lo, hi in blocks:
        width = freqs[hi - 1] - freqs[lo]
        if width < 200e3:
            continue
        center = (freqs[hi - 1] + freqs[lo]) / 2
        level = np.max(psd[lo:hi]) - floor
        print(f"  block {freqs[lo] / 1e6:+8.3f} .. {freqs[hi - 1] / 1e6:+8.3f} MHz "
              f"(width {width / 1e6:.2f} MHz, center {center / 1e6:+.3f} MHz, {level:.0f} dB)")

    offsets = []
    if options.pss_offsets:
        offsets = [float(v) for v in options.pss_offsets.split(",") if v]
    if options.pss_raster > 0:
        span = rate / 2 - PSS_RATE_HZ / 2
        offsets = list(np.arange(-span, span, options.pss_raster))
    if offsets:
        results, period = pss_search(iq, rate, offsets)
        for ratio, offset, root, peak, _ in results[:5]:
            print(f"  offset {offset / 1e6:+.3f} MHz root {root}: peak/median {ratio:.0f}")
        analyze_pss(results[0], period, rate)
    return 0


def wbfm_demod(iq, rate):
    """Quadrature FM discriminator: instantaneous frequency of the carrier."""
    prod = iq[1:] * np.conj(iq[:-1])
    return np.angle(prod) * (rate / (2 * np.pi))


def analyze_fm(options):
    iq = load_samples(options.prefix, options.max_samples)
    rate = options.rate
    if options.fm_offset:
        n = np.arange(len(iq))
        iq = iq * np.exp(-2j * np.pi * options.fm_offset / rate * n)
    rms = np.sqrt(np.mean(np.abs(iq) ** 2))
    print(f"{len(iq)} samples ({len(iq) / rate:.3f} s) at {rate:.0f} S/s, rms {rms:.4f} of full scale")
    disc = wbfm_demod(iq, rate)
    # The multiplex signal is about 100 kHz wide, so decimate the discriminator
    # output to an MPX rate that still holds the 19 kHz stereo pilot and the
    # 38 kHz stereo subcarrier.
    mpx_rate = 250_000.0
    factor = int(round(rate / mpx_rate))
    mpx_rate = rate / factor
    taps = 4 * factor + 1
    window = np.sinc((np.arange(taps) - taps // 2) / factor) * np.hamming(taps) / factor
    mpx = np.convolve(disc, window, mode="same")[::factor]
    mpx = mpx - np.mean(mpx)
    bins = 1 << int(np.floor(np.log2(min(len(mpx), 1 << 16))))
    segs = len(mpx) // bins
    x = mpx[:segs * bins].reshape(segs, bins) * np.hanning(bins)
    spec = np.mean(np.abs(np.fft.rfft(x, axis=1)) ** 2, axis=0)
    freqs = np.fft.rfftfreq(bins, 1.0 / mpx_rate)
    db = 10 * np.log10(spec + 1e-20)
    floor = np.median(db)

    def tone(target, span=2000.0):
        lo = np.searchsorted(freqs, target - span)
        hi = np.searchsorted(freqs, target + span)
        peak = lo + int(np.argmax(db[lo:hi]))
        return freqs[peak], db[peak] - floor

    audio_hi = np.searchsorted(freqs, 15_000.0)
    audio_power = 10 * np.log10(np.mean(spec[1:audio_hi]) + 1e-20) - floor
    pilot_f, pilot_snr = tone(19_000.0)
    stereo_f, stereo_snr = tone(38_000.0, span=4000.0)
    print(f"MPX at {mpx_rate:.0f} S/s: audio band (0-15 kHz) {audio_power:.0f} dB above floor")
    print(f"19 kHz stereo pilot: strongest tone at {pilot_f:.1f} Hz, {pilot_snr:.0f} dB above floor")
    print(f"38 kHz stereo band: strongest tone at {stereo_f:.1f} Hz, {stereo_snr:.0f} dB above floor")
    if pilot_snr > 20:
        ppm = (pilot_f / 19_000.0 - 1) * 1e6
        print(f"PILOT FOUND: the 19 kHz pilot places the sample rate at {ppm:+.0f} ppm "
              f"(includes the station's own reference error)")
        print("FM DEMOD OK, this is a stereo FM broadcast")
    elif audio_power > 6:
        print("FM audio present but no clear stereo pilot (mono station or weak signal)")
        print("FM DEMOD OK")
    else:
        print("no FM audio: not tuned to a station, or the signal is too weak")
        print("FM DEMOD FAIL")
    return 0


def main():
    options = parse_args()
    if options.mode == "capture":
        return capture(options)
    if options.mode == "check":
        return check(options)
    if options.mode == "fm":
        return analyze_fm(options)
    return analyze(options)


if __name__ == "__main__":
    sys.exit(main())
