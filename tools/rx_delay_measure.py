#!/usr/bin/env python3
"""Measure the TX-to-RX timestamp delay of a SoapySDR device and the
rx_time_offset_ns that makes it match a reference device.

The tool transmits a known burst at a timed instant and records receive across
it with hardware timestamps. The delay is the receive timestamp at which the
burst arrives minus the time it was sent. SoapyUHD passes those timestamps to
UHD clients unchanged, so this is the delay osmo-trx sees. Route TX into RX
through a cable and at least 30 dB of attenuation, or rely on antenna coupling
at a low TX gain.

osmo-trx shifts receive by a fixed per-device delay. Running the board as
hardware=B200 at 4 samples per symbol applies the B200's 61.846 us, so the
offset to configure is the measured delay minus that.

    rx_delay_measure.py --args driver=plutosky,addr=192.168.200.10

Run it with the rx_time_offset_ns you intend to replace, or none. The
recommendation accounts for whichever is in --args.
"""

import argparse
import json
import sys

import numpy as np

# osmo-trx's B2XX_TIMING_4_4SPS for UHD 3.9 and later.
B200_4SPS_DELAY_NS = 61846
GSM_RATE_HZ = 1625e3 / 6
GSM_SPS = 4
NS_PER_SECOND = 1e9
RX_TIME_OFFSET_ARG = "rx_time_offset_ns"

BURST_SAMPLES = 4096
BURST_AMPLITUDE = 0.5
BURST_SEED = 0x51
# A moving average this long keeps the burst off the band edges.
BURST_SMOOTH_SAMPLES = 2
# How far ahead of the board's current time each burst is scheduled. Covers the
# write and the daemon's transmit lead with room to spare.
TX_LEAD_S = 0.25
RX_LEAD_S = 0.01
RX_SPAN_S = 0.02
TIMEOUT_US = 1_000_000
READ_CHUNK_SAMPLES = 8192
# Correlation peak over the median correlation magnitude. Below this the peak is
# more likely noise than the burst.
MIN_PEAK_RATIO = 8.0
# A run further than this many sample periods from the median is an outlier.
OUTLIER_SAMPLES = 2.0


def make_burst(samples=BURST_SAMPLES, seed=BURST_SEED):
    """A band-limited pseudorandom QPSK burst with a sharp autocorrelation."""
    rng = np.random.default_rng(seed)
    symbols = (rng.integers(0, 2, samples) * 2 - 1) + 1j * (rng.integers(0, 2, samples) * 2 - 1)
    kernel = np.ones(BURST_SMOOTH_SAMPLES) / BURST_SMOOTH_SAMPLES
    burst = np.convolve(symbols, kernel, mode="same")
    return (BURST_AMPLITUDE * burst / np.max(np.abs(burst))).astype(np.complex64)


def locate_burst(rx, burst):
    """Sample index of the burst's first sample in rx, with sub-sample precision,
    and the ratio of the correlation peak to the median magnitude."""
    n = len(rx) + len(burst)
    size = 1 << (n - 1).bit_length()
    corr = np.fft.ifft(np.fft.fft(rx, size) * np.conj(np.fft.fft(burst, size)))
    mag = np.abs(corr[: len(rx) - len(burst) + 1])
    peak = int(np.argmax(mag))
    fraction = 0.0
    if 0 < peak < len(mag) - 1:
        left, center, right = mag[peak - 1], mag[peak], mag[peak + 1]
        denominator = left - 2 * center + right
        if denominator != 0:
            fraction = 0.5 * (left - right) / denominator
    median = float(np.median(mag))
    ratio = float(mag[peak] / median) if median > 0 else float("inf")
    return peak + fraction, ratio


def delay_ns(window_start_tick, index, tx_tick, rate_hz):
    """Receive timestamp of the burst minus its transmit time.

    Works in integer ticks until the final difference, because the board's
    timestamps can be large enough that seconds as a float lose whole samples.
    """
    return ((window_start_tick - tx_tick) + index) / rate_hz * NS_PER_SECOND


def split_outliers(delays, tolerance_ns):
    """Delays within tolerance of their median, and the rest."""
    median = float(np.median(delays))
    kept = [d for d in delays if abs(d - median) <= tolerance_ns]
    rejected = [d for d in delays if abs(d - median) > tolerance_ns]
    return kept, rejected


def current_offset_ns(device_args):
    """The rx_time_offset_ns already in a device-argument string, or 0."""
    for item in device_args.split(","):
        key, _, value = item.partition("=")
        if key.strip() == RX_TIME_OFFSET_ARG:
            return int(value)
    return 0


def recommended_offset_ns(measured_ns, reference_ns, current_ns):
    """rx_time_offset_ns that moves the measured delay onto the reference.

    The measurement already includes the current offset, which subtracts from
    receive timestamps, so the device's own delay is measured plus current.
    """
    return int(round(measured_ns + current_ns - reference_ns))


def transmit_was_late(soapy, device, tx_stream):
    """Drains the transmit status queue and reports whether any packet was late."""
    late = False
    while True:
        status = device.readStreamStatus(tx_stream, timeoutUs=0)
        if status.ret == soapy.SOAPY_SDR_TIMEOUT:
            return late
        if status.ret == soapy.SOAPY_SDR_TIME_ERROR:
            late = True


class Window:
    """A span of receive ticks filled from continuously streamed chunks.

    Receive runs continuously and the chunks before the window are discarded,
    which keeps the board from waiting on a timed start and the host from
    falling behind between runs.
    """

    def __init__(self, start_tick, length):
        self.start_tick = start_tick
        self.samples = np.zeros(length, dtype=np.complex64)
        self.filled = np.zeros(length, dtype=bool)

    @property
    def end_tick(self):
        return self.start_tick + len(self.samples)

    def add(self, chunk_tick, chunk):
        """Copies the part of chunk inside the window. True once a chunk reaches its end."""
        lo = max(chunk_tick, self.start_tick)
        hi = min(chunk_tick + len(chunk), self.end_tick)
        if hi > lo:
            self.samples[lo - self.start_tick:hi - self.start_tick] = chunk[lo - chunk_tick:hi - chunk_tick]
            self.filled[lo - self.start_tick:hi - self.start_tick] = True
        return chunk_tick + len(chunk) >= self.end_tick

    @property
    def complete(self):
        return bool(self.filled.all())


def measure_once(soapy, device, rx_stream, tx_stream, burst, rate_hz):
    """Delay in ns, correlation peak ratio, and a reason to discard the run or None."""
    transmit_was_late(soapy, device, tx_stream)
    now_tick = soapy.timeNsToTicks(device.getHardwareTime(), rate_hz)
    tx_tick = now_tick + int(round(TX_LEAD_S * rate_hz))
    window = Window(tx_tick - int(round(RX_LEAD_S * rate_hz)), int(round(RX_SPAN_S * rate_hz)))

    sent = device.writeStream(tx_stream, [burst], len(burst),
                              soapy.SOAPY_SDR_HAS_TIME | soapy.SOAPY_SDR_END_BURST,
                              soapy.ticksToTimeNs(tx_tick, rate_hz), TIMEOUT_US)
    if sent.ret != len(burst):
        raise RuntimeError(f"transmit wrote {sent.ret} of {len(burst)} samples")

    chunk = np.zeros(READ_CHUNK_SAMPLES, dtype=np.complex64)
    while True:
        result = device.readStream(rx_stream, [chunk], len(chunk), timeoutUs=TIMEOUT_US)
        if result.ret == soapy.SOAPY_SDR_OVERFLOW:
            continue
        if result.ret <= 0:
            raise RuntimeError(f"receive failed: {soapy.errToStr(result.ret)}")
        if not result.flags & soapy.SOAPY_SDR_HAS_TIME:
            raise RuntimeError("receive returned no timestamp")
        chunk_tick = soapy.timeNsToTicks(result.timeNs, rate_hz)
        if window.add(chunk_tick, chunk[:result.ret]):
            break

    index, ratio = locate_burst(window.samples, burst)
    if transmit_was_late(soapy, device, tx_stream):
        reason = "the burst reached the board late"
    elif not window.complete:
        reason = "receive dropped samples inside the window"
    else:
        reason = None
    return delay_ns(window.start_tick, index, tx_tick, rate_hz), ratio, reason


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--args", required=True, help="SoapySDR device arguments")
    parser.add_argument("--rate", type=float, default=GSM_RATE_HZ * GSM_SPS,
                        help="sample rate in Hz (default: GSM at 4 samples per symbol)")
    parser.add_argument("--freq", type=float, default=915e6,
                        help="TX and RX frequency in Hz (default 915e6)")
    parser.add_argument("--tx-gain", type=float, default=40.0)
    parser.add_argument("--rx-gain", type=float, default=30.0)
    parser.add_argument("--repeats", type=int, default=10)
    parser.add_argument("--reference-ns", type=float, default=B200_4SPS_DELAY_NS,
                        help="delay the client already applies (default: osmo-trx's B200 at 4 sps)")
    parser.add_argument("--json", action="store_true", help="print one JSON result line")
    options = parser.parse_args(argv)

    import SoapySDR as soapy

    rx_dir, tx_dir = soapy.SOAPY_SDR_RX, soapy.SOAPY_SDR_TX
    device = soapy.Device(options.args)
    device.setSampleRate(rx_dir, 0, options.rate)
    device.setSampleRate(tx_dir, 0, options.rate)
    rate_hz = device.getSampleRate(rx_dir, 0)
    if abs(device.getSampleRate(tx_dir, 0) - rate_hz) > 1e-3:
        sys.exit(f"TX rate {device.getSampleRate(tx_dir, 0)} differs from RX rate {rate_hz}")
    device.setFrequency(rx_dir, 0, options.freq)
    device.setFrequency(tx_dir, 0, options.freq)
    device.setGain(rx_dir, 0, options.rx_gain)
    device.setGain(tx_dir, 0, options.tx_gain)

    print(f"hardware time       {device.getHardwareTime()} ns", file=sys.stderr)
    rx_stream = device.setupStream(rx_dir, soapy.SOAPY_SDR_CF32, [0])
    tx_stream = device.setupStream(tx_dir, soapy.SOAPY_SDR_CF32, [0])
    device.activateStream(tx_stream)
    device.activateStream(rx_stream)
    burst = make_burst()

    delays = []
    weakest = float("inf")
    try:
        for i in range(options.repeats):
            delay, ratio, reason = measure_once(soapy, device, rx_stream, tx_stream, burst,
                                                rate_hz)
            if reason:
                print(f"run {i}: {reason}, skipped", file=sys.stderr)
                continue
            weakest = min(weakest, ratio)
            if ratio < MIN_PEAK_RATIO:
                print(f"run {i}: correlation peak only {ratio:.1f}x the median, skipped",
                      file=sys.stderr)
                continue
            delays.append(delay)
    finally:
        device.deactivateStream(tx_stream)
        device.deactivateStream(rx_stream)
        device.closeStream(rx_stream)
        device.closeStream(tx_stream)

    if not delays:
        sys.exit("no run found the burst: raise --tx-gain or --rx-gain, or check the loopback")

    delays, outliers = split_outliers(delays, OUTLIER_SAMPLES * NS_PER_SECOND / rate_hz)
    for outlier in outliers:
        print(f"outlier {outlier:.1f} ns dropped", file=sys.stderr)
    measured = float(np.median(delays))
    current = current_offset_ns(options.args)
    result = {
        "rate_hz": rate_hz,
        "runs": len(delays),
        "outliers": len(outliers),
        "measured_delay_ns": round(measured, 1),
        "spread_ns": round(float(np.max(delays) - np.min(delays)), 1),
        "sample_period_ns": round(NS_PER_SECOND / rate_hz, 1),
        "weakest_peak_ratio": round(weakest, 1),
        "current_rx_time_offset_ns": current,
        "device_delay_ns": round(measured + current, 1),
        "reference_ns": options.reference_ns,
        "rx_time_offset_ns": recommended_offset_ns(measured, options.reference_ns, current),
    }
    if options.json:
        print(json.dumps(result))
        return
    print(f"rate                {rate_hz:.1f} Hz ({result['sample_period_ns']} ns per sample)")
    print(f"runs                {len(delays)}, spread {result['spread_ns']} ns, "
          f"{len(outliers)} outlier(s) dropped")
    print(f"measured delay      {result['measured_delay_ns']} ns with rx_time_offset_ns={current}")
    print(f"device delay        {result['device_delay_ns']} ns")
    print(f"reference           {options.reference_ns} ns")
    print(f"rx_time_offset_ns={result['rx_time_offset_ns']}")


if __name__ == "__main__":
    main()
