"""The Python codecs against the vectors the C++ tests also check.

Both halves of tools/vectors/wire_formats.json are pinned this way, so a change
to either implementation has to be deliberate.
"""

import json
import os

import pytest

import vita

VECTORS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       "vectors", "wire_formats.json")


@pytest.fixture(scope="module")
def vectors():
    with open(VECTORS) as handle:
        return json.load(handle)


def test_samples_per_packet(vectors):
    assert len(vectors["samples_per_packet"]) == 6
    for case in vectors["samples_per_packet"]:
        fmt, mtu = case["format"], case["mtu"]
        count = vita.samples_per_packet(fmt, mtu)
        assert count == case["samples"]
        assert vita.payload_bytes_for(fmt, count) == case["payload_bytes"]
        assert vita.packet_words(fmt, count) == case["packet_words"]
        # A full packet has to fit the MTU even when it ends a burst and grows
        # the trailer.
        on_wire = (case["packet_words"] * vita.WORD_BYTES + vita.IPV4_HEADER_BYTES
                   + vita.UDP_HEADER_BYTES)
        assert on_wire + vita.VITA_TRAILER_BYTES <= mtu


def test_codec_vectors(vectors):
    for case in vectors["codec"]:
        samples = [tuple(s) for s in case["samples"]]
        payload = vita.pack(case["format"], samples)
        assert payload.hex() == case["payload_hex"], case["name"]
        decoded = vita.unpack(case["format"], payload, len(samples))
        assert [list(s) for s in decoded] == case["decoded"], case["name"]


def test_block_floating_point_vectors(vectors):
    for case in vectors["bfp8_blocks"]:
        samples = [tuple(s) for s in case["samples"]]
        peak = max(max(abs(i), abs(q)) for i, q in samples)
        assert peak == case["peak_magnitude"], case["name"]
        assert vita.bfp8_exponent(peak) == case["exponent"], case["name"]

        payload = vita.pack_bfp8(samples)
        assert payload.hex() == case["payload_hex"], case["name"]
        assert payload[vita.BFP8_BLOCK_BYTES - 1] == case["exponent"]
        mantissas = [vita._sign_extend(b, vita.MANTISSA_BITS) for b in payload[:32]]
        assert mantissas == case["mantissas"], case["name"]
        assert [list(s) for s in vita.unpack_bfp8(payload, len(samples))] == case["decoded"]


def test_packet_vectors(vectors):
    for case in vectors["packets"]:
        samples = [tuple(s) for s in case["samples"]]
        packet = vita.build_packet(case["stream_id"],
                                   vita.PACKET_CLASS_BY_FORMAT[case["format"]],
                                   case["packet_count"], case["ticks"],
                                   vita.pack(case["format"], samples),
                                   end_of_burst=case["end_of_burst"])
        assert packet.hex() == case["packet_hex"], case["name"]

        parsed = vita.parse_packet(packet)
        assert parsed["stream_id"] == case["stream_id"]
        assert parsed["count"] == case["packet_count"] % vita.PACKET_COUNT_MODULUS
        assert parsed["ticks"] == case["ticks"]
        assert parsed["format"] == case["format"]
        assert parsed["end_of_burst"] == case["end_of_burst"]
        assert parsed["tsf"] == vita.TSF_SAMPLE_COUNT
        assert parsed["tsi"] == vita.TSI_NONE
        assert parsed["information_class"] == vita.INFORMATION_CLASS


def test_block_floating_point_rounds_halves_away_from_zero():
    assert vita.bfp8_round(3, 1) == 2
    assert vita.bfp8_round(-3, 1) == -2
    assert vita.bfp8_round(7, 2) == 2
    assert vita.bfp8_round(-7, 2) == -2
    assert vita.bfp8_exponent(127) == 0
    assert vita.bfp8_exponent(128) == 1
    assert vita.bfp8_exponent(2047) == 4
    assert vita.bfp8_exponent(2048) == 5


def test_block_floating_point_decodes_a_partial_range():
    samples = [(n * 7 - 100, 50 - n * 3) for n in range(32)]
    payload = vita.pack_bfp8(samples)
    whole = vita.unpack_bfp8(payload, len(samples))
    assert vita.unpack_bfp8(payload, 5, first=20) == whole[20:25]
