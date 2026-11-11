"""Fixtures for the end-to-end tests: a built module and a running fake daemon."""

import os
import re
import shutil
import signal
import subprocess
import sys
import time

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS_DIR = os.path.dirname(HERE)
ROOT_DIR = os.path.dirname(TOOLS_DIR)
SOAPY_DIR = os.path.join(ROOT_DIR, "soapy")
DEFAULT_BUILD_DIR = os.path.join(SOAPY_DIR, "build")
DAEMON = os.path.join(TOOLS_DIR, "fake_daemon.py")

DAEMON_START_TIMEOUT_S = 15.0
DAEMON_STOP_TIMEOUT_S = 5.0
LISTENING_LINE = re.compile(
    r"listening on (?P<addr>\S+):(?P<port>\d+) discovery=(?P<discovery>\d+)")

sys.path.insert(0, TOOLS_DIR)


@pytest.fixture(scope="session")
def build_dir():
    """The module build tree, configured and built if it is not there yet."""
    path = os.environ.get("PLUTOSKY_BUILD_DIR", DEFAULT_BUILD_DIR)
    probe = os.path.join(path, "plutosky_probe")
    module = os.path.join(path, "libplutosky.so")
    if not (os.path.exists(probe) and os.path.exists(module)):
        if shutil.which("cmake") is None:
            pytest.fail("cmake is needed to build the module, or set PLUTOSKY_BUILD_DIR")
        subprocess.run(["cmake", "-S", SOAPY_DIR, "-B", path], check=True,
                       capture_output=True, text=True)
        subprocess.run(["cmake", "--build", path, "-j"], check=True,
                       capture_output=True, text=True)
    assert os.path.exists(probe), probe + " was not built"
    assert os.path.exists(module), module + " was not built"
    return path


@pytest.fixture(scope="session")
def probe_bin(build_dir):
    return os.path.join(build_dir, "plutosky_probe")


@pytest.fixture(scope="session")
def soapy_env(build_dir):
    env = dict(os.environ)
    env["SOAPY_SDR_PLUGIN_PATH"] = build_dir
    # SoapySDR searches its install root before the plugin path, so an
    # installed copy of the module would shadow the build under test.
    env["SOAPY_SDR_ROOT"] = build_dir
    return env


class Daemon:
    def __init__(self, process, addr, port, discovery_port, log_path):
        self.process = process
        self.addr = addr
        self.port = port
        self.discovery_port = discovery_port
        self.log_path = log_path

    @property
    def device_args(self):
        return "driver=plutosky,addr=%s,port=%d" % (self.addr, self.port)

    @property
    def discovery_args(self):
        return "driver=plutosky,discovery_port=%d" % self.discovery_port

    def log(self):
        with open(self.log_path, "r") as handle:
            return handle.read()


@pytest.fixture(scope="session")
def daemon(tmp_path_factory):
    """A fake daemon on ephemeral ports, so the tests never fight for 30000."""
    log_path = str(tmp_path_factory.mktemp("daemon") / "fake_daemon.log")
    log = open(log_path, "w")
    process = subprocess.Popen(
        [sys.executable, DAEMON, "--control-port", "0", "--discovery-port", "0", "--verbose"],
        cwd=ROOT_DIR, stdout=log, stderr=subprocess.STDOUT)

    deadline = time.monotonic() + DAEMON_START_TIMEOUT_S
    match = None
    while time.monotonic() < deadline:
        if process.poll() is not None:
            log.close()
            with open(log_path) as handle:
                pytest.fail("fake_daemon exited early:\n" + handle.read())
        with open(log_path) as handle:
            match = LISTENING_LINE.search(handle.read())
        if match:
            break
        time.sleep(0.05)
    if match is None:
        process.kill()
        log.close()
        pytest.fail("fake_daemon did not report a listening port")

    running = Daemon(process, match.group("addr"), int(match.group("port")),
                     int(match.group("discovery")), log_path)
    yield running

    process.send_signal(signal.SIGTERM)
    try:
        process.wait(timeout=DAEMON_STOP_TIMEOUT_S)
    except subprocess.TimeoutExpired:
        process.kill()
    log.close()
