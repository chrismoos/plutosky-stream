import numpy as np
import pytest

from rx_delay_measure import (B200_4SPS_DELAY_NS, current_offset_ns, delay_ns, locate_burst,
                              make_burst, recommended_offset_ns, split_outliers, Window)

RATE_HZ = 1625e3 / 6 * 4
NOISE_SEED = 7


def received(burst, at, total=8192, gain=0.01, noise=0.001):
    """burst delayed by a possibly fractional number of samples, scaled, in noise."""
    padded = np.zeros(total, dtype=np.complex128)
    padded[: len(burst)] = burst
    freqs = np.fft.fftfreq(total)
    shifted = np.fft.ifft(np.fft.fft(padded) * np.exp(-2j * np.pi * freqs * at))
    rng = np.random.default_rng(NOISE_SEED)
    hiss = noise * (rng.standard_normal(total) + 1j * rng.standard_normal(total))
    return (gain * shifted + hiss).astype(np.complex64)


@pytest.mark.parametrize("at", [0.0, 25.0, 1000.0, 1000.25, 1000.5, 2500.8])
def test_locates_whole_and_fractional_delays(at):
    burst = make_burst()
    index, ratio = locate_burst(received(burst, at), burst)
    assert index == pytest.approx(at, abs=0.1)
    assert ratio > 50


def test_noise_alone_gives_a_weak_peak():
    burst = make_burst()
    index, ratio = locate_burst(received(burst, 1000.0, gain=0.0), burst)
    assert ratio < 8


def test_burst_is_repeatable_and_scaled():
    assert np.array_equal(make_burst(), make_burst())
    assert np.max(np.abs(make_burst())) == pytest.approx(0.5, rel=1e-6)


def test_delay_is_receive_time_of_burst_minus_transmit_time():
    window_start = 1000
    tx_tick = window_start + 10
    assert delay_ns(window_start, 10.0, tx_tick, RATE_HZ) == pytest.approx(0.0)
    assert delay_ns(window_start, 110.5, tx_tick, RATE_HZ) == pytest.approx(100.5 / RATE_HZ * 1e9)


def test_delay_keeps_sub_sample_precision_at_large_timestamps():
    huge = 2 ** 60
    delay = delay_ns(huge - 10833, 10833 + 90.25, huge, RATE_HZ)
    assert delay == pytest.approx(90.25 / RATE_HZ * 1e9, abs=0.01)


def test_reads_current_offset_from_device_args():
    assert current_offset_ns("type=soapy,driver=plutosky,hardware=B200") == 0
    assert current_offset_ns("type=soapy,rx_time_offset_ns=30000,hardware=B200") == 30000
    assert current_offset_ns("rx_time_offset_ns=-1200") == -1200


def test_recommendation_is_device_delay_minus_reference():
    assert recommended_offset_ns(91846, B200_4SPS_DELAY_NS, 0) == 30000
    # Measured with 30 us already applied: the device delay is unchanged.
    assert recommended_offset_ns(61846, B200_4SPS_DELAY_NS, 30000) == 30000
    assert recommended_offset_ns(50000, B200_4SPS_DELAY_NS, 0) == -11846


def test_outliers_far_from_the_median_are_dropped():
    delays = [83900.0, 84100.0, 83923.0, 237823000.0, 84500.0, -5000000.0]
    kept, rejected = split_outliers(delays, 1846.0)
    assert sorted(kept) == [83900.0, 83923.0, 84100.0, 84500.0]
    assert sorted(rejected) == [-5000000.0, 237823000.0]


def test_consistent_runs_keep_everything():
    kept, rejected = split_outliers([100.0, 200.0, 300.0], 1846.0)
    assert kept == [100.0, 200.0, 300.0]
    assert rejected == []


def chunk(tick, length):
    return np.arange(tick, tick + length).astype(np.complex64)


def test_window_discards_chunks_before_it_and_fills_from_overlaps():
    window = Window(1000, 300)
    assert not window.add(0, chunk(0, 500))
    assert not window.add(500, chunk(500, 600))
    assert window.add(1100, chunk(1100, 300))
    assert window.complete
    assert np.array_equal(window.samples.real, np.arange(1000, 1300))


def test_window_ends_when_a_chunk_reaches_its_end():
    window = Window(1000, 300)
    assert not window.add(900, chunk(900, 200))
    assert window.add(1100, chunk(1100, 200))
    assert window.complete


def test_window_reports_a_gap_inside_it():
    window = Window(1000, 300)
    window.add(950, chunk(950, 100))
    assert window.add(1150, chunk(1150, 200))
    assert not window.complete
