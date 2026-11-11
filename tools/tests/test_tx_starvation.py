"""A transmit client that stops writing: the module limits its status logging,
stops the stream, and starts it again on the next write.

Runs the module through the SoapySDR Python bindings in a subprocess, against
tools/fake_daemon.py. Skipped when the bindings are not installed.
"""

import re
import subprocess
import sys

import pytest

pytest.importorskip("SoapySDR")

RUN_TIMEOUT_S = 60.0
CLIENT = r'''
import sys, time
import numpy as np
import SoapySDR as soapy

device = soapy.Device(sys.argv[1])
device.setSampleRate(soapy.SOAPY_SDR_TX, 0, 1083333)
tx = device.setupStream(soapy.SOAPY_SDR_TX, soapy.SOAPY_SDR_CF32, [0])
device.activateStream(tx)
chunk = np.zeros(1250, np.complex64)

def write_for(seconds):
    ok = 0
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        ok += device.writeStream(tx, [chunk], len(chunk), 0, 0, 100000).ret == len(chunk)
    return ok

write_for(0.5)
print("STARVING", flush=True)
time.sleep(3.0)
print("RESUMED_WRITES", write_for(0.5), flush=True)
device.deactivateStream(tx)
device.closeStream(tx)
'''


def run_client(env, daemon):
    # One stream, so the module's log lines stay in order with the markers.
    return subprocess.run([sys.executable, "-u", "-c", CLIENT, daemon.device_args], env=env,
                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                          timeout=RUN_TIMEOUT_S)


def test_a_starved_stream_is_stopped_quietly_and_restarts_on_write(soapy_env, daemon):
    starts_before = daemon.log().count("start_stream")
    completed = run_client(soapy_env, daemon)
    output = completed.stdout
    assert completed.returncode == 0, output

    starving = output[output.index("STARVING"):output.index("RESUMED_WRITES")]
    assert starving.count("no write for") == 1, output
    # Status reports arrive many times a second. At most one line a second.
    assert len(re.findall(r"TX timing status", starving)) <= 4, output

    resumed = int(re.search(r"RESUMED_WRITES (\d+)", output).group(1))
    assert resumed > 0, output
    assert daemon.log().count("start_stream") - starts_before >= 2, daemon.log()
