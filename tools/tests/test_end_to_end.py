"""End to end tests: the plutosky module against tools/fake_daemon.py.

Each test runs plutosky_probe against the module. Nothing here touches a board.
"""

import json
import os
import re
import subprocess
import sys
import time

import pytest

import control
import vita

RESULT_LINE = re.compile(r"^RESULT (?P<fields>.*)$", re.MULTILINE)
PROBE_TIMEOUT_S = 60.0
RECEIVE_SECONDS = 0.6
TRANSMIT_BURST_ELEMS = 2000
TRANSMIT_BURSTS = 3
SCHEDULE_AHEAD_S = 0.05
SCHEDULE_BEHIND_S = -0.5
STATUS_SETTLE_S = 0.3
# Runs of a transmit check, when the fake daemon's socket keeps dropping datagrams.
TRANSMIT_ATTEMPTS = 3


def run_probe(probe_bin, env, *args, expect_success=True):
    completed = subprocess.run([probe_bin] + list(args), env=env, capture_output=True,
                               text=True, timeout=PROBE_TIMEOUT_S)
    if expect_success and completed.returncode != 0:
        fail_probe(args, completed)
    match = RESULT_LINE.search(completed.stdout)
    assert match is not None, "no RESULT line\n" + completed.stdout + completed.stderr
    fields = {}
    for token in match.group("fields").split():
        key, _, value = token.partition("=")
        fields[key] = value
    fields["_stdout"] = completed.stdout
    fields["_stderr"] = completed.stderr
    fields["_completed"] = completed
    return fields


def fail_probe(args, completed):
    pytest.fail("%s failed (%d)\nstdout:\n%s\nstderr:\n%s"
                % (" ".join(args), completed.returncode, completed.stdout, completed.stderr))


def run_transmit(probe_bin, env, daemon, *args, expect_success=True):
    """Runs the probe and returns its result and the fake daemon's report of
    what it received.

    The fake daemon is Python, and on a loaded runner its socket buffer can
    fill and the kernel drop datagrams the module did send. The kernel counts
    those drops, so a run that has any says nothing about the module and is
    repeated.
    """
    for _ in range(TRANSMIT_ATTEMPTS):
        got = run_probe(probe_bin, env, *args, expect_success=False)
        with control.ControlClient(daemon.addr, daemon.port, client="pytest") as client:
            report = client.request("tx_report")
        if not report.get("socket_drops"):
            if expect_success and got["_completed"].returncode != 0:
                fail_probe(args, got["_completed"])
            return got, report
    pytest.fail("the fake daemon's socket dropped %d datagrams in each of %d runs"
                % (report["socket_drops"], TRANSMIT_ATTEMPTS))


def test_discovery_finds_the_daemon(probe_bin, soapy_env, daemon):
    found = run_probe(probe_bin, soapy_env, "--find", "--args=" + daemon.discovery_args)
    assert int(found["devices"]) == 1


def test_probe_reports_the_device(probe_bin, soapy_env, daemon):
    found = run_probe(probe_bin, soapy_env, "--probe", "--args=" + daemon.device_args)
    assert found["driver"] == "plutosky"
    assert found["hardware"] == "FAKE0001"
    assert float(found["rate"]) > 0

    text = found["_stdout"]
    assert "stream arg duplex_tx_rate_limit_sps =  [" in text
    assert "stream arg tx_only_rate_limit_sps = 36000000.000000" in text
    # The probe output is the module's whole configuration surface, so a
    # missing control call shows up here rather than in a later test.
    for expected in ("RX channels: 2", "TX channels: 2",
                     "antennas: A_BALANCED", "antenna: A_BALANCED",
                     "antennas: A\n", "antenna: A\n",
                     "native format: CS12", "stream arg wire", "stream arg mtu",
                     "Sensor temperature_c", "sensor rssi", "Hardware time:"):
        assert expected in text, "probe output is missing %r\n%s" % (expected, text)


def test_duplex_tx_rate_limit_device_argument(probe_bin, soapy_env, daemon):
    found = run_probe(probe_bin, soapy_env, "--probe",
                      "--args=" + daemon.device_args + ",duplex_tx_rate_limit_sps=8000000")
    assert "stream arg duplex_tx_rate_limit_sps = 8000000.000000" in found["_stdout"]


def test_tx_only_rate_limit_device_argument(probe_bin, soapy_env, daemon):
    found = run_probe(probe_bin, soapy_env, "--probe",
                      "--args=" + daemon.device_args + ",tx_only_rate_limit_sps=39000000")
    assert "stream arg tx_only_rate_limit_sps = 39000000.000000" in found["_stdout"]


@pytest.mark.parametrize("name", ["tx_pace_headroom", "tx_send_rate_multiplier"])
def test_removed_tx_multiplier_is_rejected(probe_bin, soapy_env, daemon, name):
    completed = subprocess.run(
        [probe_bin, "--probe", "--args=" + daemon.device_args + "," + name + "=1.5"],
        env=soapy_env, capture_output=True, text=True, timeout=PROBE_TIMEOUT_S)
    assert completed.returncode != 0
    assert "TX send-rate multipliers are removed" in completed.stderr


@pytest.mark.parametrize("wire", list(vita.FORMATS))
@pytest.mark.parametrize("mtu", [1500, 9000])
def test_receive_every_wire_format_and_mtu(probe_bin, soapy_env, daemon, wire, mtu):
    got = run_probe(probe_bin, soapy_env, "--rate", "--args=" + daemon.device_args,
                    "--wire=" + wire, "--mtu=%d" % mtu, "--secs=%g" % RECEIVE_SECONDS)
    expected_mtu = vita.samples_per_packet(wire, mtu)
    assert int(got["mtu"]) == expected_mtu
    assert int(got["elems"]) > 0
    assert int(got["reads"]) > 0
    # A whole packet fits one read, so nothing should fragment and no packet
    # should be lost or land out of sequence.
    assert int(got["fragments"]) == 0
    assert int(got["overflows"]) == 0
    assert int(got["time_jumps"]) == 0
    assert int(got["elems"]) % expected_mtu == 0


@pytest.mark.parametrize("wire,host_format", [
    ("cs12", "CF32"), ("cs12", "CS16"), ("cs12", "CS12"),
    ("cs8", "CF32"), ("cs8", "CS16"), ("cs8", "CS8"),
    ("bfp8", "CF32"), ("bfp8", "CS16"),
])
def test_receive_host_formats(probe_bin, soapy_env, daemon, wire, host_format):
    got = run_probe(probe_bin, soapy_env, "--rate", "--args=" + daemon.device_args,
                    "--wire=" + wire, "--format=" + host_format,
                    "--secs=%g" % RECEIVE_SECONDS)
    assert int(got["elems"]) > 0
    assert int(got["overflows"]) == 0
    assert int(got["time_jumps"]) == 0


def test_a_short_read_reports_more_fragments(probe_bin, soapy_env, daemon):
    chunk = 100
    got = run_probe(probe_bin, soapy_env, "--rate", "--args=" + daemon.device_args,
                    "--wire=cs12", "--format=CS16", "--read=%d" % chunk,
                    "--secs=%g" % RECEIVE_SECONDS)
    packet = vita.samples_per_packet("cs12", 1500)
    assert int(got["read"]) == chunk
    assert int(got["elems"]) > 0
    assert int(got["fragments"]) > 0
    # Only the read that finishes a packet clears the flag, so a packet takes
    # several reads and the samples still arrive in an unbroken sequence.
    assert int(got["time_jumps"]) == 0
    assert int(got["reads"]) > int(got["elems"]) / packet
    assert int(got["fragments"]) >= int(got["elems"]) // packet


def test_receive_two_channels_halves_the_packet(probe_bin, soapy_env, daemon):
    got = run_probe(probe_bin, soapy_env, "--rate", "--args=" + daemon.device_args,
                    "--wire=cs12", "--channels=0,1", "--format=CS16",
                    "--secs=%g" % RECEIVE_SECONDS)
    # A packet holds interleaved samples, so each channel gets half of them.
    assert int(got["mtu"]) == vita.samples_per_packet("cs12", 1500) // 2
    assert int(got["elems"]) > 0
    assert int(got["time_jumps"]) == 0


def test_timed_transmit_is_verified_by_the_daemon(probe_bin, soapy_env, daemon):
    got, report = run_transmit(probe_bin, soapy_env, daemon, "--tx",
                               "--args=" + daemon.device_args, "--wire=cs12", "--format=CF32",
                               "--burst=%d" % TRANSMIT_BURST_ELEMS,
                               "--bursts=%d" % TRANSMIT_BURSTS,
                               "--at=%g" % SCHEDULE_AHEAD_S)
    assert int(got["written"]) == TRANSMIT_BURST_ELEMS * TRANSMIT_BURSTS
    assert int(got["errors"]) == 0
    assert int(got["late"]) == 0
    assert int(got["underrun"]) == 0
    assert "duplex_packets=0" in got["_stderr"]
    assert "tx_only_rate_limit_sps=36000000" in got["_stderr"]

    assert report["dir"] == "tx"
    assert report["samples"] == TRANSMIT_BURST_ELEMS * TRANSMIT_BURSTS
    assert report["bursts"] == TRANSMIT_BURSTS
    assert len(report["end_of_burst_ticks"]) == TRANSMIT_BURSTS
    assert report["count_gaps"] == 0
    assert report["tick_gaps"] == 0
    assert report["decode_errors"] == 0
    assert report["bad_stream_id"] == 0
    assert report["bad_packet_class"] == 0
    assert report["late"] == 0
    # The daemon learns the host address from one empty datagram before the
    # samples start.
    assert report["hello_packets"] >= 1

    packets_per_burst = -(-TRANSMIT_BURST_ELEMS // vita.samples_per_packet("cs12", 1500))
    assert report["packets"] == packets_per_burst * TRANSMIT_BURSTS



def test_duplex_probe_runs_both_streams_and_stops_tx(probe_bin, soapy_env, daemon):
    # The fake daemon is Python on a shared CI runner. Transmit is paced close to
    # the sample rate so its socket buffer does not overflow, and an underrun,
    # which the fake counts whenever its reader falls behind the wall clock, is
    # not a failure. The probe fails on any underrun, so the result is checked
    # field by field instead. The burst fits the fake's TX window and starts
    # well after the whole burst can be sent, so a slow runner does not make
    # the probe send late.
    got, report = run_transmit(probe_bin, soapy_env, daemon, "--duplex",
                               "--args=" + daemon.device_args, "--wire=cs12", "--rate=1000000",
                               "--secs=0.1", "--at=0.3", "--duplex_tx_rate_limit_sps=1200000",
                               expect_success=False)
    assert int(got["rx_elems"]) > 0
    assert int(got["tx_elems"]) == int(got["target_elems"])
    assert int(got["rx_overflows"]) == 0
    assert int(got["rx_time_jumps"]) == 0
    assert int(got["rx_errors"]) == 0
    assert int(got["tx_errors"]) == 0
    assert int(got["tx_timeouts"]) == 0
    assert int(got["tx_late"]) == 0
    assert int(got["tx_gaps"]) == 0
    assert re.search(r"duplex_packets=[1-9]", got["_stderr"])

    with control.ControlClient(daemon.addr, daemon.port, client="pytest") as client:
        sensors = client.request("get_sensors")
    assert report["count_gaps"] == 0
    assert report["samples"] == int(got["tx_elems"])
    assert report["decode_errors"] == 0
    assert not sensors["tx_rf_enabled"]


def test_duplex_with_a_transmit_rate_twice_the_receive_rate(probe_bin, soapy_env, daemon):
    # The timebase counts at the transmit rate here, so a receive sample spans
    # two counts. Receive timestamps must still advance one sample at a time
    # and every transmit packet must follow the one before it. As in the
    # equal-rate test, the burst fits the fake's TX window and starts well
    # after the whole burst can be sent.
    got, report = run_transmit(probe_bin, soapy_env, daemon, "--duplex",
                               "--args=" + daemon.device_args, "--wire=cs12", "--rate=1000000",
                               "--tx_rate=2000000", "--secs=0.05", "--at=0.3",
                               "--duplex_tx_rate_limit_sps=2400000", expect_success=False)
    assert float(got["rate"]) == 1000000
    assert float(got["tx_rate"]) == 2000000
    assert int(got["rx_elems"]) > 0
    assert int(got["tx_elems"]) == int(got["target_elems"]) == 100000
    assert int(got["rx_time_jumps"]) == 0
    assert int(got["rx_errors"]) == 0
    assert int(got["tx_errors"]) == 0
    assert int(got["tx_late"]) == 0
    assert int(got["tx_gaps"]) == 0

    with control.ControlClient(daemon.addr, daemon.port, client="pytest") as client:
        rates = client.request("get_sample_rate")
    assert report["count_gaps"] == 0
    assert report["tick_gaps"] == 0
    assert report["late"] == 0
    assert report["samples"] == int(got["tx_elems"])
    assert (rates["rx_hz"], rates["tx_hz"], rates["tick_rate_hz"]) == (1000000, 2000000, 2000000)
    assert (rates["rx_ticks_per_sample"], rates["tx_ticks_per_sample"]) == (2, 1)


def test_duplex_with_a_transmit_rate_below_the_receive_rate(probe_bin, soapy_env, daemon):
    got, report = run_transmit(probe_bin, soapy_env, daemon, "--duplex",
                               "--args=" + daemon.device_args, "--wire=cs12", "--rate=1500000",
                               "--tx_rate=1000000", "--secs=0.1", "--at=0.3",
                               "--duplex_tx_rate_limit_sps=1200000", expect_success=False)
    assert int(got["tx_elems"]) == int(got["target_elems"]) == 100000
    assert int(got["rx_time_jumps"]) == 0
    assert int(got["tx_late"]) == 0
    assert int(got["tx_gaps"]) == 0

    with control.ControlClient(daemon.addr, daemon.port, client="pytest") as client:
        rates = client.request("get_sample_rate")
    assert report["tick_gaps"] == 0
    assert report["late"] == 0
    assert report["samples"] == int(got["tx_elems"])
    # Neither rate is a multiple of the other, so the timebase runs faster than both.
    assert rates["tick_rate_hz"] == 3000000
    assert (rates["rx_ticks_per_sample"], rates["tx_ticks_per_sample"]) == (2, 3)


def test_rates_with_no_common_radio_rate_refuse_the_replaced_direction(daemon):
    with control.ControlClient(daemon.addr, daemon.port, client="pytest") as client:
        client.request("set_sample_rate", dir="rx", hz=10000000)
        rates = client.request("set_sample_rate", dir="tx", hz=4000000)
        assert (rates["hz"], rates["rx_hz"], rates["tx_hz"]) == (4000000, 4000000, 4000000)
        with pytest.raises(control.ControlError, match="10000000"):
            client.request("open_stream", dir="rx", chans=[0], format="cs12", mtu=1500)
        client.request("set_sample_rate", dir="rx", hz=8000000)
        opened = client.request("open_stream", dir="rx", chans=[0], format="cs12", mtu=1500)
        assert opened["stream_id"]


def test_transmit_in_the_past_is_reported_as_a_time_error(probe_bin, soapy_env, daemon):
    got = run_probe(probe_bin, soapy_env, "--tx", "--args=" + daemon.device_args,
                    "--wire=cs12", "--burst=%d" % TRANSMIT_BURST_ELEMS, "--bursts=1",
                    "--at=%g" % SCHEDULE_BEHIND_S)
    assert int(got["written"]) == TRANSMIT_BURST_ELEMS
    assert int(got["late"]) >= 1


def test_block_floating_point_transmit_pads_to_whole_blocks(probe_bin, soapy_env, daemon):
    elems = 1000
    _, report = run_transmit(probe_bin, soapy_env, daemon, "--tx",
                             "--args=" + daemon.device_args, "--wire=bfp8", "--format=CS16",
                             "--burst=%d" % elems, "--bursts=1", "--at=%g" % SCHEDULE_AHEAD_S)

    assert report["bursts"] == 1
    assert report["tick_gaps"] == 0
    assert report["decode_errors"] == 0
    # The last packet rounds up to a whole block, so the daemon sees a few
    # more samples than the caller wrote.
    rounded = -(-elems // vita.BFP8_BLOCK_SAMPLES) * vita.BFP8_BLOCK_SAMPLES
    assert report["samples"] == rounded
    assert report["samples"] % vita.BFP8_BLOCK_SAMPLES == 0


def test_hardware_time_round_trips(probe_bin, soapy_env, daemon):
    with control.ControlClient(daemon.addr, daemon.port, client="pytest") as client:
        rate = float(client.request("get_sample_rate")["hz"])
        before = client.request("get_hardware_time")["ticks"]
        time.sleep(0.2)
        after = client.request("get_hardware_time")["ticks"]
    assert after > before
    assert 0.1 * rate < after - before < 0.5 * rate
