"""Unit tests for fragmented TCP reads and binary blob framing."""

import unittest

from app.protocol import TYPE_REQUEST, encode_message, recv_message


class ProtocolTest(unittest.TestCase):
    def test_round_trip_with_multiple_blobs(self):
        encoded = encode_message(
            TYPE_REQUEST,
            {"operation": "inference", "sequence": 7},
            {"front": b"jpeg", "points": b"\x00\x01\x02"},
        )
        message_type, metadata, blobs = recv_message(FragmentedReader(encoded, 3))
        self.assertEqual(message_type, TYPE_REQUEST)
        self.assertEqual(metadata["sequence"], 7)
        self.assertEqual(blobs["front"], b"jpeg")
        self.assertEqual(blobs["points"], b"\x00\x01\x02")


class FragmentedReader:
    def __init__(self, value, fragment_size):
        self.value = value
        self.fragment_size = fragment_size
        self.offset = 0

    def recv(self, requested):
        size = min(requested, self.fragment_size)
        chunk = self.value[self.offset:self.offset + size]
        self.offset += len(chunk)
        return chunk


if __name__ == "__main__":
    unittest.main()
