"""The module through UHD and the SoapyUHD bridge, against the fake daemon.

Skipped when the UHD Python API or the SoapyUHD bridge is not installed. The
scenarios themselves live in uhd_checks.py.
"""

import glob
import json
import os
import shutil
import subprocess
import sys

import pytest

# The UHD bindings can fail to load with a plain ImportError when libuhd is
# missing, which importorskip only catches from pytest 8.2 on.
try:
    import uhd
except ImportError:
    pytest.skip("the UHD Python API is not installed", allow_module_level=True)
np = pytest.importorskip("numpy")

HERE = os.path.dirname(os.path.abspath(__file__))
CHECKS = os.path.join(HERE, "uhd_checks.py")
CHECK_TIMEOUT_S = 120
RATE_HZ = 5e6
# The interface rate osmo-trx sets on a B200.
MASTER_CLOCK_HZ = 26e6
TONE_HZ = 100000.0
TONE_SCALE = 0.5 * 2047
SAMPLE_FULL_SCALE = 2048.0
CS16_MSB_SCALE = 16
FILE_SAMPLES = 1000000
PHASE_PROBE_SAMPLES = 64
RX_SAMPLES = 200000
TIMED_START_S = 0.3
BURST_SAMPLES = 123457
TWO_CHANNEL_BURST_SAMPLES = 50000
MAX_NONBLOCKING_STATUS_POLL_S = 0.25


def bridge_modules():
    directories = [os.path.join(uhd.get_lib_path(), "uhd", "modules")]
    directories += [p for p in os.environ.get("UHD_MODULE_PATH", "").split(":") if p]
    found = []
    for directory in directories:
        found += glob.glob(os.path.join(directory, "*soapy*"))
    return found


if not bridge_modules():
    pytest.skip("the SoapyUHD bridge is not installed in UHD's modules directory",
                allow_module_level=True)


def uhd_example(name):
    candidates = [shutil.which(name)]
    for prefix in (uhd.get_pkg_path(), "/usr", "/usr/local"):
        for sub in ("libexec", "lib"):
            candidates.append(os.path.join(prefix, sub, "uhd", "examples", name))
    for path in candidates:
        if path and os.access(path, os.X_OK):
            return path
    return None


@pytest.fixture(scope="module")
def uhd_args(daemon):
    return "type=soapy," + daemon.device_args


def run_check(soapy_env, daemon, uhd_args, mode, *extra):
    command = [sys.executable, CHECKS, mode, "--args", uhd_args, "--addr", daemon.addr,
               "--port", str(daemon.port)] + list(extra)
    done = subprocess.run(command, env=soapy_env, capture_output=True, text=True,
                          timeout=CHECK_TIMEOUT_S)
    for line in done.stdout.splitlines():
        if line.startswith("RESULT "):
            assert done.returncode == 0, done.stdout + done.stderr
            return json.loads(line[len("RESULT "):])
    pytest.fail("%s gave no result (exit %d):\n%s%s"
                % (mode, done.returncode, done.stdout, done.stderr))


def tone(ticks):
    phase = 2.0 * np.pi * TONE_HZ / RATE_HZ * ticks
    return np.round(np.cos(phase) * TONE_SCALE) + 1j * np.round(np.sin(phase) * TONE_SCALE)


def test_uhd_finds_the_device(soapy_env, daemon, uhd_args):
    finder = uhd_example("uhd_find_devices") or shutil.which("uhd_find_devices")
    if finder is None:
        pytest.skip("uhd_find_devices is not installed")
    done = subprocess.run([finder, "--args", uhd_args], env=soapy_env, capture_output=True,
                          text=True, timeout=CHECK_TIMEOUT_S)
    assert done.returncode == 0, done.stdout + done.stderr
    assert "driver: plutosky" in done.stdout
    assert "type: soapy" in done.stdout


def test_uhd_usrp_probe_lists_the_frontends(soapy_env, daemon, uhd_args):
    prober = shutil.which("uhd_usrp_probe")
    if prober is None:
        pytest.skip("uhd_usrp_probe is not installed")
    done = subprocess.run([prober, "--args", uhd_args], env=soapy_env, capture_output=True,
                          text=True, timeout=CHECK_TIMEOUT_S)
    assert done.returncode == 0, done.stdout + done.stderr
    out = done.stdout
    assert "Clock sources: internal" in out
    assert "Time sources:  internal" in out
    assert out.count("Freq range: 70.000 to 6000.000 MHz") == 4
    assert out.count("Gain range PGA: -3.0 to 73.0 step 1.0 dB") == 2
    assert out.count("Gain range PGA: 0.0 to 89.8 step 0.2 dB") == 2
    assert out.count("Sensors: rssi, lo_locked") == 2


def test_uhd_properties(soapy_env, daemon, uhd_args):
    result = run_check(soapy_env, daemon, uhd_args, "probe")
    assert result["mboard"] == "FAKE0001"
    assert result["clock_sources"] == ["internal"]
    assert result["time_sources"] == ["internal"]
    assert result["rx_channels"] == 2 and result["tx_channels"] == 2
    assert result["master_clock_rate"] == result["rx_rate"] == RATE_HZ
    assert result["master_clock_rate_set"] == [MASTER_CLOCK_HZ, RATE_HZ]
    assert result["mboard_sensors"] == ["temperature_c", "rx_lo_locked", "tx_lo_locked"]
    for direction in ("rx", "tx"):
        assert result[direction]["gain_names"] == ["PGA"]
        assert result[direction]["freq_range"] == [70e6, 6e9]
        assert result[direction]["lo_locked"] is True
    assert result["rx"]["gain_range"] == [-3.0, 73.0, 1.0]
    assert result["tx"]["gain_range"] == [0.0, 89.75, 0.25]
    assert result["rx"]["antennas"] == ["A_BALANCED"]
    assert result["tx"]["antennas"] == ["A"]
    assert result["rx"]["sensors"] == ["rssi", "lo_locked"]
    assert result["tx"]["sensors"] == ["lo_locked"]
    assert result["rx_freq"] == 1e9
    assert result["tx_gain"] == 79.75
    # clear_command_time writes a command time of zero, which must not reset
    # the timebase.
    assert result["time_after_command_time"] < 1.0


def test_uhd_zero_timeout_tx_status_poll_returns_immediately(soapy_env, daemon, uhd_args):
    result = run_check(soapy_env, daemon, uhd_args, "tx_poll")
    assert result["received"] == [False] * 8
    assert result["elapsed_s"] < MAX_NONBLOCKING_STATUS_POLL_S


@pytest.mark.parametrize("fmt", ["sc16", "fc32"])
def test_uhd_receive_matches_the_tone_with_contiguous_timestamps(soapy_env, daemon, uhd_args,
                                                                 fmt):
    result = run_check(soapy_env, daemon, uhd_args, "rx", "--format", fmt,
                       "--nsamps", str(RX_SAMPLES))
    assert result["errors"] == []
    assert result["got"] >= RX_SAMPLES
    assert result["mismatches"] == 0
    assert result["tick_errors"] == 0


def test_uhd_timed_start_lands_on_the_requested_tick(soapy_env, daemon, uhd_args):
    result = run_check(soapy_env, daemon, uhd_args, "rx", "--format", "sc16",
                       "--start-in", str(TIMED_START_S), "--nsamps", str(RX_SAMPLES))
    assert result["errors"] == []
    assert result["first_tick"] == result["requested_tick"]
    assert result["mismatches"] == 0
    assert result["tick_errors"] == 0


def test_uhd_finite_burst_ends_with_end_of_burst(soapy_env, daemon, uhd_args):
    result = run_check(soapy_env, daemon, uhd_args, "rx", "--format", "fc32",
                       "--stream-mode", "num_done", "--start-in", str(TIMED_START_S),
                       "--nsamps", str(BURST_SAMPLES))
    assert result["errors"] == []
    assert result["got"] == BURST_SAMPLES
    assert result["end_of_burst"] is True
    assert result["after_burst"] == 0
    assert result["first_tick"] == result["requested_tick"]
    assert result["mismatches"] == 0


@pytest.mark.parametrize("fmt", ["sc16", "fc32"])
def test_uhd_receive_both_channels(soapy_env, daemon, uhd_args, fmt):
    result = run_check(soapy_env, daemon, uhd_args, "rx", "--format", fmt, "--chans", "0,1",
                       "--stream-mode", "num_done", "--nsamps",
                       str(TWO_CHANNEL_BURST_SAMPLES))
    assert result["errors"] == []
    assert result["got"] == TWO_CHANNEL_BURST_SAMPLES
    assert result["mismatches"] == 0
    assert result["tick_errors"] == 0


@pytest.mark.parametrize("fmt,dtype,scale", [
    ("short", "int16", 1.0 / CS16_MSB_SCALE),
    ("float", "float32", SAMPLE_FULL_SCALE),
])
def test_rx_samples_to_file(soapy_env, daemon, uhd_args, tmp_path, fmt, dtype, scale):
    tool = uhd_example("rx_samples_to_file")
    if tool is None:
        pytest.skip("the UHD examples are not installed")
    path = str(tmp_path / ("rx_%s.dat" % fmt))
    done = subprocess.run(
        [tool, "--args", uhd_args, "--rate", str(RATE_HZ), "--freq", "1e9", "--nsamps",
         str(FILE_SAMPLES), "--type", fmt, "--file", path],
        env=soapy_env, capture_output=True, text=True, timeout=CHECK_TIMEOUT_S)
    assert done.returncode == 0, done.stdout + done.stderr

    raw = np.fromfile(path, dtype=dtype).astype(np.float64).reshape(-1, 2) * scale
    samples = raw[:, 0] + 1j * raw[:, 1]
    assert len(samples) == FILE_SAMPLES
    # The file has no timestamps, so find where the tone starts, then every
    # sample after it must follow with no gap.
    period = int(RATE_HZ / TONE_HZ)
    start = None
    for offset in range(period):
        if np.allclose(samples[:PHASE_PROBE_SAMPLES],
                       tone(np.arange(offset, offset + PHASE_PROBE_SAMPLES)), atol=1e-3):
            start = offset
            break
    assert start is not None, "received samples do not follow the fake daemon's tone"
    expected = tone(np.arange(start, start + FILE_SAMPLES))
    assert np.count_nonzero(np.abs(samples - expected) > 1e-3) == 0


@pytest.mark.parametrize("fmt", ["fc32", "sc16"])
def test_uhd_timed_burst_and_late_burst(soapy_env, daemon, uhd_args, fmt):
    result = run_check(soapy_env, daemon, uhd_args, "tx", "--format", fmt)
    report = result["report"]
    assert result["sent"] == [12000, 8000, 5000]
    assert report["first_ticks"] == result["timed_tick"]
    assert report["end_of_burst_ticks"] == [result["timed_end"], result["late_end"]]
    assert report["bursts"] == 2
    assert report["count_gaps"] == 0 and report["tick_gaps"] == 0
    assert report["decode_errors"] == 0 and report["bad_stream_id"] == 0
    assert report["late"] == result["late_packets"]
    assert report["sample_checksum"] == result["checksum"]
    assert not result["tx_rf_enabled"]
    assert ["burst_ack", result["timed_end"]] in result["events"]
    assert any(event == "time_error" for event, _ in result["events"])
