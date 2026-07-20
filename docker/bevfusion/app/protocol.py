"""Versioned binary framing shared by the BEVFusion TCP client and server."""

import json
import struct


MAGIC = b"AJBF"
VERSION = 1
HEADER = struct.Struct("!4sBBHII")
MAX_METADATA_BYTES = 1024 * 1024
MAX_PAYLOAD_BYTES = 128 * 1024 * 1024

TYPE_REQUEST = 1
TYPE_RESPONSE = 2
TYPE_PING = 3
TYPE_PONG = 4
TYPE_ERROR = 5


class ProtocolError(RuntimeError):
    """Raised when a peer sends malformed or unsupported data."""


def encode_message(message_type, metadata=None, blobs=None):
    metadata = dict(metadata or {})
    blobs = blobs or {}
    descriptors = []
    payload_parts = []
    offset = 0
    for name, value in blobs.items():
        if not isinstance(value, (bytes, bytearray, memoryview)):
            raise TypeError("blob {!r} must be bytes-like".format(name))
        value = bytes(value)
        descriptors.append({"name": str(name), "offset": offset, "size": len(value)})
        payload_parts.append(value)
        offset += len(value)
    metadata["blobs"] = descriptors
    metadata_bytes = json.dumps(
        metadata, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    payload = b"".join(payload_parts)
    _validate_sizes(len(metadata_bytes), len(payload))
    return HEADER.pack(
        MAGIC, VERSION, int(message_type), 0, len(metadata_bytes), len(payload)
    ) + metadata_bytes + payload


def recv_message(sock):
    header = recv_exact(sock, HEADER.size)
    magic, version, message_type, _flags, metadata_size, payload_size = HEADER.unpack(
        header
    )
    if magic != MAGIC:
        raise ProtocolError("invalid protocol magic")
    if version != VERSION:
        raise ProtocolError(
            "unsupported protocol version {} (expected {})".format(version, VERSION)
        )
    _validate_sizes(metadata_size, payload_size)
    metadata_raw = recv_exact(sock, metadata_size)
    payload = recv_exact(sock, payload_size)
    try:
        metadata = json.loads(metadata_raw.decode("utf-8"))
    except (UnicodeDecodeError, ValueError) as error:
        raise ProtocolError("invalid JSON metadata: {}".format(error))
    blobs = decode_blobs(metadata, payload)
    return message_type, metadata, blobs


def send_message(sock, message_type, metadata=None, blobs=None):
    sock.sendall(encode_message(message_type, metadata, blobs))


def recv_exact(sock, size):
    chunks = []
    remaining = size
    while remaining:
        chunk = sock.recv(remaining)
        if not chunk:
            raise EOFError("peer closed with {} bytes remaining".format(remaining))
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def decode_blobs(metadata, payload):
    blobs = {}
    occupied = []
    for item in metadata.get("blobs", []):
        try:
            name = str(item["name"])
            offset = int(item["offset"])
            size = int(item["size"])
        except (KeyError, TypeError, ValueError) as error:
            raise ProtocolError("invalid blob descriptor: {}".format(error))
        end = offset + size
        if offset < 0 or size < 0 or end > len(payload):
            raise ProtocolError("blob {!r} exceeds payload bounds".format(name))
        if name in blobs:
            raise ProtocolError("duplicate blob name {!r}".format(name))
        for previous_start, previous_end in occupied:
            if offset < previous_end and end > previous_start:
                raise ProtocolError("overlapping blob descriptors")
        occupied.append((offset, end))
        blobs[name] = payload[offset:end]
    return blobs


def _validate_sizes(metadata_size, payload_size):
    if metadata_size < 0 or metadata_size > MAX_METADATA_BYTES:
        raise ProtocolError("metadata size {} is out of range".format(metadata_size))
    if payload_size < 0 or payload_size > MAX_PAYLOAD_BYTES:
        raise ProtocolError("payload size {} is out of range".format(payload_size))

