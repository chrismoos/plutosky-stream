#!/usr/bin/env python3
"""Drives the plutosky module through UHD and the SoapyUHD bridge.

Each mode opens the device with the UHD Python API, runs one scenario against
tools/fake_daemon.py and prints a `RESULT <json>` line. test_uhd.py runs it
in a subprocess, so a crash inside UHD fails one test instead of the session.
"""

import argparse
import json
import math
import os
import sys
import time
import traceback

import numpy as np
import uhd

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import control  # noqa: E402

RATE_HZ = 5e6
# The interface rate osmo-trx sets on a B200.
MASTER_CLOCK_HZ = 26e6
FREQUENCY_HZ = 1e9
TX_GAIN_DB = 79.75
# The fake daemon's default tone, in the 12-bit sample scale.
TONE_HZ = 100000.0
TONE_SCALE = 0.5 * 2047
SAMPLE_FULL_SCALE = 2048.0
CS16_MSB_SCALE = 16
FIRST_RECV_TIMEOUT_S = 3.0
RECV_TIMEOUT_S = 0.5
AFTER_BURST_TIMEOUT_S = 0.3
TX_LEAD_S = 0.3
TX_LATE_S = 0.1
TX_START_TIME_S = 10.0
ASYNC_WAIT_S = 2.0
ASYNC_POLL_S = 0.1
SEND_TIMEOUT_S = 1.0
COMMAND_TIME_S = 100.0
CHECKSUM_MULTIPLIER = 31
CHECKSUM_MASK = 0xFFFFFFFF
SAMPLE_MASK = 0xFFFF
SAMPLES_PER_PACKET = 481
STATUS_POLLS = 8


def tone(ticks):
    phase = 2.0 * math.pi * TONE_HZ / RATE_HZ * ticks
    return np.round(np.cos(phase) * TONE_SCALE) + 1j * np.round(np.sin(phase) * TONE_SCALE)


def open_usrp(args):
    usrp = uhd.usrp.MultiUSRP(args)
    usrp.set_rx_rate(RATE_HZ)
    usrp.set_tx_rate(RATE_HZ)
    return usrp


def set_master_clock(usrp):
    usrp.set_master_clock_rate(MASTER_CLOCK_HZ)
    return [usrp.get_master_clock_rate(), usrp.get_rx_rate()]


def probe(args):
    usrp = open_usrp(args)
    result = {
        "mboard": usrp.get_mboard_name(),
        "master_clock_rate": usrp.get_master_clock_rate(),
        "master_clock_rate_set": set_master_clock(usrp),
        "clock_sources": list(usrp.get_clock_sources(0)),
        "time_sources": list(usrp.get_time_sources(0)),
        "mboard_sensors": list(usrp.get_mboard_sensor_names()),
        "rx_channels": usrp.get_rx_num_channels(),
        "tx_channels": usrp.get_tx_num_channels(),
    }
    for name, direction in (("rx", "rx"), ("tx", "tx")):
        gain_range = getattr(usrp, "get_%s_gain_range" % direction)(0)
        freq_range = getattr(usrp, "get_%s_freq_range" % direction)(0)
        result[name] = {
            "gain_names": list(getattr(usrp, "get_%s_gain_names" % direction)(0)),
            "gain_range": [gain_range.start(), gain_range.stop(), gain_range.step()],
            "freq_range": [freq_range.start(), freq_range.stop()],
            "antennas": list(getattr(usrp, "get_%s_antennas" % direction)(0)),
            "sensors": list(getattr(usrp, "get_%s_sensor_names" % direction)(0)),
            "lo_locked": getattr(usrp, "get_%s_sensor" % direction)("lo_locked", 0).to_bool(),
        }
    result["rx_rate"] = usrp.get_rx_rate()

    usrp.set_rx_freq(uhd.types.TuneRequest(FREQUENCY_HZ), 0)
    result["rx_freq"] = usrp.get_rx_freq(0)
    usrp.set_tx_gain(TX_GAIN_DB, 0)
    result["tx_gain"] = usrp.get_tx_gain(0)

    usrp.set_time_now(uhd.types.TimeSpec(0.0))
    usrp.set_command_time(uhd.types.TimeSpec(COMMAND_TIME_S))
    usrp.clear_command_time()
    result["time_after_command_time"] = usrp.get_time_now().get_real_secs()
    return result


def receive(args, fmt, chans, mode_name, start_in, nsamps):
    usrp = open_usrp(args)
    usrp.set_rx_freq(uhd.types.TuneRequest(FREQUENCY_HZ), 0)
    stream_args = uhd.usrp.StreamArgs(fmt, "sc16")
    stream_args.channels = chans
    rx = usrp.get_rx_stream(stream_args)
    spb = rx.get_max_num_samps()
    # One element per complex sample, as pyuhd sizes a buffer by its last axis.
    buffer = np.zeros((len(chans), spb), dtype=np.complex64 if fmt == "fc32" else np.uint32)
    md = uhd.types.RXMetadata()

    mode = getattr(uhd.types.StreamMode, mode_name)
    command = uhd.types.StreamCMD(mode)
    command.num_samps = nsamps
    requested_tick = None
    if start_in is None:
        command.stream_now = True
    else:
        at = usrp.get_time_now().get_real_secs() + start_in
        command.stream_now = False
        command.time_spec = uhd.types.TimeSpec(at)
        requested_tick = round(at * RATE_HZ)
    rx.issue_stream_cmd(command)

    got = 0
    first_tick = None
    next_tick = None
    mismatches = 0
    tick_errors = 0
    end_of_burst = False
    errors = []
    while got < nsamps:
        n = rx.recv(buffer, md, FIRST_RECV_TIMEOUT_S if got == 0 else RECV_TIMEOUT_S)
        if md.error_code != uhd.types.RXMetadataErrorCode.none:
            errors.append(str(md.error_code))
            break
        tick = md.time_spec.to_ticks(RATE_HZ)
        if first_tick is None:
            first_tick = tick
        elif tick != next_tick:
            tick_errors += 1
        next_tick = tick + n
        expected = tone(np.arange(tick, tick + n))
        for c in range(len(chans)):
            if fmt == "fc32":
                samples = buffer[c, :n].astype(np.complex128) * SAMPLE_FULL_SCALE
            else:
                iq = buffer[c, :n].view(np.int16).astype(np.float64) / CS16_MSB_SCALE
                samples = iq[0::2] + 1j * iq[1::2]
            # The fake daemon inverts the second channel.
            want = expected if chans[c] == 0 else -expected
            mismatches += int(np.count_nonzero(np.abs(samples - want) > 1e-3))
        got += n
        if md.end_of_burst:
            end_of_burst = True
            break

    after_burst = 0
    if mode == uhd.types.StreamMode.start_cont:
        rx.issue_stream_cmd(uhd.types.StreamCMD(uhd.types.StreamMode.stop_cont))
    else:
        after_burst = rx.recv(buffer, md, AFTER_BURST_TIMEOUT_S)
    # SoapyUHD streamers keep a bare pointer to the device, so a streamer must
    # go before the device it came from.
    del rx
    return {
        "got": got,
        "first_tick": first_tick,
        "requested_tick": requested_tick,
        "tick_errors": tick_errors,
        "mismatches": mismatches,
        "end_of_burst": end_of_burst,
        "after_burst": after_burst,
        "errors": errors,
    }


def checksum(values):
    """The fake daemon's transmit checksum over 12-bit I and Q."""
    total = 0
    for i, q in values:
        total = (total * CHECKSUM_MULTIPLIER + (int(i) & SAMPLE_MASK)) & CHECKSUM_MASK
        total = (total * CHECKSUM_MULTIPLIER + (int(q) & SAMPLE_MASK)) & CHECKSUM_MASK
    return total


def to_host(fmt, values):
    if fmt == "fc32":
        return ((values[:, 0] + 1j * values[:, 1]) / SAMPLE_FULL_SCALE).astype(
            np.complex64).reshape(1, -1)
    iq = (values * CS16_MSB_SCALE).astype(np.int16).reshape(-1)
    return iq.view(np.uint32).reshape(1, -1)


def transmit(args, fmt, addr, port):
    usrp = open_usrp(args)
    usrp.set_tx_freq(uhd.types.TuneRequest(FREQUENCY_HZ), 0)
    usrp.set_time_now(uhd.types.TimeSpec(TX_START_TIME_S))
    stream_args = uhd.usrp.StreamArgs(fmt, "sc16")
    stream_args.channels = [0]
    tx = usrp.get_tx_stream(stream_args)
    md = uhd.types.TXMetadata()

    rng = np.random.default_rng(1)
    timed = rng.integers(-2048, 2048, size=(20000, 2))
    late = rng.integers(-2048, 2048, size=(5000, 2))
    split = 12000

    timed_at = usrp.get_time_now().get_real_secs() + TX_LEAD_S
    md.start_of_burst = True
    md.end_of_burst = False
    md.has_time_spec = True
    md.time_spec = uhd.types.TimeSpec(timed_at)
    sent = [tx.send(to_host(fmt, timed[:split]), md, SEND_TIMEOUT_S)]
    md.start_of_burst = False
    md.has_time_spec = False
    md.end_of_burst = True
    sent.append(tx.send(to_host(fmt, timed[split:]), md, SEND_TIMEOUT_S))

    late_at = usrp.get_time_now().get_real_secs() - TX_LATE_S
    md.start_of_burst = True
    md.end_of_burst = True
    md.has_time_spec = True
    md.time_spec = uhd.types.TimeSpec(late_at)
    sent.append(tx.send(to_host(fmt, late), md, SEND_TIMEOUT_S))

    events = []
    async_md = uhd.types.TXAsyncMetadata()
    deadline = time.monotonic() + ASYNC_WAIT_S
    while time.monotonic() < deadline:
        if tx.recv_async_msg(async_md, ASYNC_POLL_S):
            tick = async_md.time_spec.to_ticks(RATE_HZ) if async_md.has_time_spec else None
            events.append([str(async_md.event_code).split(".")[-1], tick])

    with control.ControlClient(addr, port) as client:
        report = client.request("tx_report")
        tx_rf_enabled = client.request("get_sensors")["tx_rf_enabled"]
    del tx

    timed_tick = round(timed_at * RATE_HZ)
    late_tick = round(late_at * RATE_HZ)
    return {
        "sent": sent,
        "events": events,
        "report": report,
        "tx_rf_enabled": tx_rf_enabled,
        "timed_tick": timed_tick,
        "timed_end": timed_tick + len(timed),
        "late_end": late_tick + len(late),
        "late_packets": -(-len(late) // SAMPLES_PER_PACKET),
        "checksum": checksum(np.vstack([timed, late])),
    }


def poll_tx_status(args):
    usrp = open_usrp(args)
    tx = usrp.get_tx_stream(uhd.usrp.StreamArgs("fc32", "sc16"))
    metadata = uhd.types.TXAsyncMetadata()
    started = time.monotonic()
    received = [tx.recv_async_msg(metadata, 0.0) for _ in range(STATUS_POLLS)]
    elapsed = time.monotonic() - started
    del tx
    return {"elapsed_s": elapsed, "received": received}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("mode", choices=["probe", "rx", "tx", "tx_poll"])
    parser.add_argument("--args", required=True, help="UHD device address")
    parser.add_argument("--format", default="fc32", choices=["fc32", "sc16"])
    parser.add_argument("--chans", default="0")
    parser.add_argument("--stream-mode", default="start_cont", choices=["start_cont", "num_done"])
    parser.add_argument("--start-in", type=float, default=None)
    parser.add_argument("--nsamps", type=int, default=100000)
    parser.add_argument("--addr", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=control.DEFAULT_CONTROL_PORT)
    options = parser.parse_args()

    try:
        if options.mode == "probe":
            result = probe(options.args)
        elif options.mode == "rx":
            chans = [int(c) for c in options.chans.split(",")]
            result = receive(options.args, options.format, chans, options.stream_mode,
                             options.start_in, options.nsamps)
        elif options.mode == "tx_poll":
            result = poll_tx_status(options.args)
        else:
            result = transmit(options.args, options.format, options.addr, options.port)
    except Exception:
        traceback.print_exc()
        sys.stdout.flush()
        # The traceback still holds the streamers, and a streamer freed after
        # its device crashes the teardown.
        os._exit(2)
    print("RESULT " + json.dumps(result), flush=True)


if __name__ == "__main__":
    main()
