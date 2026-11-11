"""Client for the plutosky-stream gRPC control service.

The service is defined in proto/plutosky/v1/control.proto. Message classes are
built at run time from tools/control.desc, which tools/gen_proto.sh compiles
from that file, so no generated Python code is needed. Requests and replies are
plain dicts: enums are their lowercase names without the type prefix ("rx",
"cs12", "fast_attack"). Shared by the packet tools, the fake daemon and the
integration tests.
"""

import json
import os
import queue
import re
import socket
import threading

import grpc
from google.protobuf import descriptor_pb2, descriptor_pool, message_factory

DEFAULT_CONTROL_PORT = 30000
DISCOVERY_PORT = 30001
DISCOVERY_PROBE = b"plutosky-stream?"
DEFAULT_TIMEOUT_S = 5.0
# A sample rate change reruns the LVDS calibration, which takes a few seconds.
SLOW_CALL_TIMEOUT_S = 15.0
SESSION_METADATA_KEY = "plutosky-session"
SERVICE = "plutosky.v1.Control"
PACKAGE = "plutosky.v1"
DESCRIPTOR_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "control.desc")

# op: (RPC, request message, reply message, needs a session)
RPCS = {
    "get_info": ("GetInfo", "GetInfoRequest", "BoardInfo", False),
    "set_frequency": ("SetFrequency", "SetFrequencyRequest", "FrequencyReply", True),
    "get_frequency": ("GetFrequency", "ChannelRef", "FrequencyReply", False),
    "set_sample_rate": ("SetSampleRate", "SetSampleRateRequest", "SampleRateReply", True),
    "get_sample_rate": ("GetSampleRate", "Empty", "SampleRateReply", False),
    "set_bandwidth": ("SetBandwidth", "SetBandwidthRequest", "BandwidthReply", True),
    "get_bandwidth": ("GetBandwidth", "ChannelRef", "BandwidthReply", False),
    "set_gain": ("SetGain", "SetGainRequest", "GainReply", True),
    "get_gain": ("GetGain", "ChannelRef", "GainReply", False),
    "set_gain_mode": ("SetGainMode", "SetGainModeRequest", "GainModeReply", True),
    "list_antennas": ("ListAntennas", "ListAntennasRequest", "AntennaList", False),
    "set_antenna": ("SetAntenna", "SetAntennaRequest", "AntennaReply", True),
    "set_loopback": ("SetLoopback", "SetLoopbackRequest", "LoopbackReply", True),
    "get_hardware_time": ("GetHardwareTime", "Empty", "HardwareTime", False),
    "set_hardware_time": ("SetHardwareTime", "HardwareTime", "HardwareTime", True),
    "get_sensors": ("GetSensors", "Empty", "Sensors", False),
    "open_stream": ("OpenStream", "OpenStreamRequest", "OpenStreamReply", True),
    "start_stream": ("StartStream", "StartStreamRequest", "Empty", True),
    "stop_stream": ("StopStream", "StreamRef", "Empty", True),
    "close_stream": ("CloseStream", "StreamRef", "Empty", True),
}
SLOW_OPS = {"set_sample_rate", "start_stream"}
# tools/fake_daemon.py only: what its transmit checker saw, as JSON.
FAKE_TX_REPORT_METHOD = "/plutosky.fake.v1.Fake/TxReport"


class ControlError(Exception):
    def __init__(self, message, code=None):
        super().__init__(message)
        self.code = code


def _load_messages():
    files = descriptor_pb2.FileDescriptorSet()
    with open(DESCRIPTOR_PATH, "rb") as handle:
        files.ParseFromString(handle.read())
    pool = descriptor_pool.DescriptorPool()
    for proto in files.file:
        pool.Add(proto)
    classes = {}
    for proto in files.file:
        for message in proto.message_type:
            descriptor = pool.FindMessageTypeByName("%s.%s" % (PACKAGE, message.name))
            if hasattr(message_factory, "GetMessageClass"):
                classes[message.name] = message_factory.GetMessageClass(descriptor)
            else:
                classes[message.name] = message_factory.MessageFactory(pool).GetPrototype(
                    descriptor)
    return classes


MESSAGES = _load_messages()


def _enum_prefix(enum_descriptor):
    return re.sub(r"(?<!^)(?=[A-Z])", "_", enum_descriptor.name).upper() + "_"


def to_dict(message):
    """A message as a dict, with every field present and enums as names."""
    out = {}
    for field in message.DESCRIPTOR.fields:
        value = getattr(message, field.name)
        if field.message_type is not None:
            if field.label == field.LABEL_REPEATED:
                out[field.name] = [to_dict(item) for item in value]
            elif message.HasField(field.name):
                out[field.name] = to_dict(value)
            continue
        if field.enum_type is not None:
            prefix = _enum_prefix(field.enum_type)

            def name(number, enum=field.enum_type, prefix=prefix):
                value = enum.values_by_number.get(number)
                label = value.name if value is not None else str(number)
                return label[len(prefix):].lower() if label.startswith(prefix) else label
            out[field.name] = [name(v) for v in value] if field.label == field.LABEL_REPEATED \
                else name(value)
            continue
        if field.label == field.LABEL_REPEATED:
            out[field.name] = list(value)
        elif field.has_presence and not message.HasField(field.name):
            out[field.name] = None
        else:
            out[field.name] = value
    return out


def from_dict(name, values):
    """Builds message `name` from a dict, ignoring unknown keys and None."""
    message = MESSAGES[name]()
    fields = message.DESCRIPTOR.fields_by_name
    for key, value in (values or {}).items():
        field = fields.get(key)
        if field is None or value is None:
            continue
        if field.enum_type is not None:
            prefix = _enum_prefix(field.enum_type)

            def number(label, enum=field.enum_type, prefix=prefix):
                if isinstance(label, int):
                    return label
                found = enum.values_by_name.get(prefix + str(label).upper())
                if found is None:
                    raise ControlError("`%s` is not a %s" % (label, enum.name),
                                       grpc.StatusCode.INVALID_ARGUMENT)
                return found.number
            if field.label == field.LABEL_REPEATED:
                getattr(message, key).extend(number(v) for v in value)
            else:
                setattr(message, key, number(value))
        elif field.message_type is not None:
            if field.label == field.LABEL_REPEATED:
                for item in value:
                    getattr(message, key).add().CopyFrom(
                        from_dict(field.message_type.name, item))
            else:
                getattr(message, key).CopyFrom(from_dict(field.message_type.name, value))
        elif field.label == field.LABEL_REPEATED:
            getattr(message, key).extend(value)
        elif field.type in (field.TYPE_DOUBLE, field.TYPE_FLOAT):
            setattr(message, key, float(value))
        elif field.type in (field.TYPE_STRING,):
            setattr(message, key, str(value))
        elif field.type == field.TYPE_BOOL:
            setattr(message, key, bool(value))
        else:
            setattr(message, key, int(value))
    return message


def method(rpc):
    return "/%s/%s" % (SERVICE, rpc)


def discover(timeout=0.4, port=DISCOVERY_PORT, targets=("127.0.0.1", "255.255.255.255")):
    """Returns (address, reply) for every daemon that answers the probe."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.settimeout(timeout)
    for target in targets:
        try:
            sock.sendto(DISCOVERY_PROBE, (target, port))
        except OSError:
            continue

    found = []
    seen = set()
    while True:
        try:
            datagram, peer = sock.recvfrom(4096)
        except (socket.timeout, OSError):
            break
        try:
            reply = to_dict(MESSAGES["DiscoveryReply"].FromString(datagram))
        except Exception:
            continue
        if reply["serial"] in seen:
            continue
        seen.add(reply["serial"])
        found.append((peer[0], reply))
    sock.close()
    return found


class ControlClient:
    """One channel to the daemon. The first request that changes the radio or
    a stream opens a session, which holds radio control until close()."""

    def __init__(self, addr="127.0.0.1", port=DEFAULT_CONTROL_PORT, timeout=DEFAULT_TIMEOUT_S,
                 client="tools"):
        self.addr = addr
        self.port = port
        self.timeout = timeout
        self.client = client
        self.channel = grpc.insecure_channel("%s:%d" % (addr, port))
        self.lock = threading.Lock()
        self.session_id = None
        self.session_call = None
        self.commands = None
        self.events = queue.Queue()
        self.info = self.request("get_info", client=client)

    def close(self):
        if self.commands is not None:
            self.commands.put(None)
        if self.session_call is not None:
            self.session_call.cancel()
        self.channel.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def _open_session(self):
        self.commands = queue.Queue()
        self.commands.put(from_dict("SessionCommand", {"start": {"client": self.client}}))

        def outgoing(commands=self.commands):
            while True:
                command = commands.get()
                if command is None:
                    return
                yield command

        call = self.channel.stream_stream(
            method("Session"),
            request_serializer=lambda message: message.SerializeToString(),
            response_deserializer=MESSAGES["SessionEvent"].FromString,
        )(outgoing())
        try:
            first = next(call)
        except grpc.RpcError as error:
            raise ControlError("Session: %s" % error.details(), error.code()) from None
        self.session_id = first.opened.session_id
        self.session_call = call
        threading.Thread(target=self._read_session, args=(call,), daemon=True).start()

    def _read_session(self, call):
        try:
            for event in call:
                if event.WhichOneof("event") != "status":
                    continue
                for stream in event.status.streams:
                    status = to_dict(stream)
                    status["event"] = "status"
                    status["ticks"] = event.status.ticks
                    self.events.put(status)
        except grpc.RpcError:
            pass

    def set_status_interval(self, interval_ms):
        with self.lock:
            if self.session_id is None:
                self._open_session()
        self.commands.put(from_dict("SessionCommand",
                                    {"set_status_interval": {"interval_ms": interval_ms}}))

    def request(self, op, **params):
        if op == "tx_report":
            return self._fake_tx_report(params)
        entry = RPCS.get(op)
        if entry is None:
            raise ControlError("no operation named `%s`" % op, grpc.StatusCode.UNIMPLEMENTED)
        rpc, request_name, reply_name, needs_session = entry
        metadata = ()
        with self.lock:
            if needs_session:
                if self.session_id is None:
                    self._open_session()
                metadata = ((SESSION_METADATA_KEY, self.session_id),)
        call = self.channel.unary_unary(
            method(rpc),
            request_serializer=lambda message: message.SerializeToString(),
            response_deserializer=MESSAGES[reply_name].FromString,
        )
        timeout = max(self.timeout, SLOW_CALL_TIMEOUT_S) if op in SLOW_OPS else self.timeout
        try:
            reply = call(from_dict(request_name, params), timeout=timeout, metadata=metadata)
        except grpc.RpcError as error:
            raise ControlError("%s: %s" % (op, error.details()), error.code()) from None
        return to_dict(reply)

    def _fake_tx_report(self, params):
        call = self.channel.unary_unary(
            FAKE_TX_REPORT_METHOD,
            request_serializer=lambda value: json.dumps(value).encode("utf-8"),
            response_deserializer=lambda data: json.loads(data),
        )
        try:
            return call(params, timeout=self.timeout)
        except grpc.RpcError as error:
            raise ControlError("tx_report: %s" % error.details(), error.code()) from None

    def drain_events(self, timeout=0.0):
        """Collects the status events queued so far, waiting up to timeout for
        one when none is queued."""
        out = []
        try:
            out.append(self.events.get(timeout=timeout) if timeout > 0
                       else self.events.get_nowait())
        except queue.Empty:
            return out
        while True:
            try:
                out.append(self.events.get_nowait())
            except queue.Empty:
                return out
