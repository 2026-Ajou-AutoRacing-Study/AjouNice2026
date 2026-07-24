"""Versioned binary framing compatible with docker/bevfusion/app/protocol.py."""

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
    pass


def encode_message(message_type, metadata=None, blobs=None):
    metadata = dict(metadata or {})
    blobs = blobs or {}
    descriptors = []
    payload_parts = []
    offset = 0
    for name, value in blobs.items():
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


def send_message(sock, message_type, metadata=None, blobs=None):
    sock.sendall(encode_message(message_type, metadata, blobs))


def recv_message(sock):
    header = recv_exact(sock, HEADER.size)
    magic, version, message_type, _flags, metadata_size, payload_size = HEADER.unpack(
        header
    )
    if magic != MAGIC or version != VERSION:
        raise ProtocolError("invalid magic or unsupported protocol version")
    _validate_sizes(metadata_size, payload_size)
    try:
        metadata = json.loads(recv_exact(sock, metadata_size).decode("utf-8"))
    except (UnicodeDecodeError, ValueError) as error:
        raise ProtocolError("invalid JSON metadata: {}".format(error))
    payload = recv_exact(sock, payload_size)
    blobs = {}
    occupied = []
    for item in metadata.get("blobs", []):
        name = str(item["name"])
        offset = int(item["offset"])
        size = int(item["size"])
        end = offset + size
        if offset < 0 or size < 0 or end > len(payload) or name in blobs:
            raise ProtocolError("invalid blob descriptor")
        for previous_start, previous_end in occupied:
            if offset < previous_end and end > previous_start:
                raise ProtocolError("overlapping blob descriptors")
        occupied.append((offset, end))
        blobs[name] = payload[offset:end]
    return message_type, metadata, blobs


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


def _validate_sizes(metadata_size, payload_size):
    if metadata_size < 0 or metadata_size > MAX_METADATA_BYTES:
        raise ProtocolError("metadata size is out of range")
    if payload_size < 0 or payload_size > MAX_PAYLOAD_BYTES:
        raise ProtocolError("payload size is out of range")

