"""Check physical TX attenuation against the fake daemon, without RF hardware."""

import argparse
import pathlib
import socket
import subprocess
import sys
import time

import SoapySDR

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "tools"))
from control import ControlClient, ControlError


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--daemon", required=True)
    parser.add_argument("--csr-json", required=True)
    args = parser.parse_args()
    with socket.socket() as reservation:
        reservation.bind(("127.0.0.1", 0))
        port = reservation.getsockname()[1]
    daemon = subprocess.Popen([
        args.daemon, "--fake", "--csr-json", args.csr_json,
        "--control-port", str(port), "--discovery-port", "0",
    ], stdout=subprocess.DEVNULL)
    try:
        for _ in range(100):
            try:
                observer = ControlClient("127.0.0.1", port=port)
                break
            except (OSError, ControlError):
                if daemon.poll() is not None:
                    raise RuntimeError("fake daemon exited")
                time.sleep(0.02)
        else:
            raise RuntimeError("fake daemon did not start")
        with observer:
            def physical_gain():
                return observer.request("get_gain", dir="tx", chan=0)["db"]

            device = SoapySDR.Device(f"driver=plutosky,addr=127.0.0.1,port={port}")
            gain_range = device.getGainRange(SoapySDR.SOAPY_SDR_TX, 0)
            assert gain_range.minimum() == 0.0
            assert gain_range.maximum() == 89.75
            try:
                device.setGain(SoapySDR.SOAPY_SDR_TX, 0, -16.0)
            except RuntimeError:
                pass
            else:
                raise AssertionError("legacy negative TX gain was accepted")
            for direction in (SoapySDR.SOAPY_SDR_RX, SoapySDR.SOAPY_SDR_TX):
                device.setFrequency(direction, 0, 915e6)
                assert device.getFrequency(direction, 0) == 915e6
                device.setBandwidth(direction, 0, 5e6)
                assert device.getBandwidth(direction, 0) > 0
            device.setGainMode(SoapySDR.SOAPY_SDR_RX, 0, False)
            device.setGain(SoapySDR.SOAPY_SDR_RX, 0, 30)
            assert device.getGain(SoapySDR.SOAPY_SDR_RX, 0) == 30
            device.setGain(SoapySDR.SOAPY_SDR_TX, 0, 73.75)
            assert device.getGain(SoapySDR.SOAPY_SDR_TX, 0) == 73.75
            assert physical_gain() == 0.0
            stream = device.setupStream(SoapySDR.SOAPY_SDR_TX, SoapySDR.SOAPY_SDR_CF32, [0])
            assert physical_gain() == 0.0
            device.activateStream(stream)
            assert physical_gain() == 73.75
            device.deactivateStream(stream)
            assert physical_gain() == 0.0
            device.activateStream(stream)
            assert physical_gain() == 73.75
            device.closeStream(stream)
            assert physical_gain() == 0.0
            del device
            print("TX configuration, empty activation, restart and close mute checks passed")
    finally:
        daemon.terminate()
        daemon.wait(timeout=5)


if __name__ == "__main__":
    main()
