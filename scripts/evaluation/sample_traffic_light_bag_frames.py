#!/usr/bin/env python3
"""Sample original front-camera JPEGs from a read-only ROS1 bag, offline.

No image decoder, inference runtime, or ROS import is needed by the reusable
``sample_messages`` function. JPEG bytes are copied, never re-encoded. Header
and bag-record timestamps are recorded separately and are never rewritten.
The sampling interval uses bag-record time, not the possibly-zero header time.

Example (the output directory must not exist)::

    /usr/bin/python3 scripts/evaluation/sample_traffic_light_bag_frames.py \
        --bag /path/to/input.bag --output /tmp/traffic_frames_new \
        --interval-sec 2 --max-frames 150

Weather/time of day are UNKNOWN unless explicitly supplied with an annotation
source. JPEG marker checks are structural checks, not a full pixel decode.
An interrupted/failed export may leave images, but never a success manifest.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import re


DEFAULT_TOPIC = "/sensor/camera/front/compressed"
UNKNOWN = "UNKNOWN"
_SOF_MARKERS = {0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7,
                0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF}


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def stamp_fields(stamp):
    """Keep integer ROS timestamp fields; do not round through float seconds."""
    secs, nsecs = stamp.secs, stamp.nsecs
    if (not isinstance(secs, int) or isinstance(secs, bool)
            or not isinstance(nsecs, int) or isinstance(nsecs, bool)
            or secs < 0 or not 0 <= nsecs < 1_000_000_000):
        raise ValueError("Invalid ROS timestamp seconds/nanoseconds")
    return {"secs": secs, "nsecs": nsecs}


def jpeg_dimensions(payload):
    """Read SOF dimensions without decoding pixels; reject malformed markers."""
    if (len(payload) < 4 or not payload.startswith(b"\xff\xd8\xff")
            or not payload.endswith(b"\xff\xd9")):
        raise ValueError("JPEG payload is empty, truncated, or lacks SOI/EOI magic")
    offset = 2
    dimensions = None
    while offset < len(payload) - 2:
        if payload[offset] != 0xFF:
            raise ValueError("Malformed JPEG segment marker")
        while offset < len(payload) and payload[offset] == 0xFF:
            offset += 1
        if offset >= len(payload):
            raise ValueError("Truncated JPEG marker")
        marker = payload[offset]
        offset += 1
        if marker in {0x00, 0xD8, 0xD9} or 0xD0 <= marker <= 0xD7:
            raise ValueError("Unexpected standalone JPEG marker before scan")
        if marker == 0x01:  # TEM has no segment length.
            continue
        if offset + 2 > len(payload):
            raise ValueError("Truncated JPEG segment length")
        length = int.from_bytes(payload[offset:offset + 2], "big")
        end = offset + length
        if length < 2 or end > len(payload) - 2:
            raise ValueError("Invalid JPEG segment length")
        if marker in _SOF_MARKERS:
            if length < 8:
                raise ValueError("Truncated JPEG SOF")
            height = int.from_bytes(payload[offset + 3:offset + 5], "big")
            width = int.from_bytes(payload[offset + 5:offset + 7], "big")
            components = payload[offset + 7]
            if not width or not height or not components or length < 8 + 3 * components:
                raise ValueError("Invalid JPEG dimensions/components")
            dimensions = (width, height)
        if marker == 0xDA:  # SOS: the rest is compressed scan data, not segments.
            if dimensions is None or length < 6:
                raise ValueError("JPEG scan has no valid SOF/dimensions")
            components = payload[offset + 2]
            if not components or length != 6 + 2 * components or end >= len(payload) - 2:
                raise ValueError("Invalid or empty JPEG scan")
            return dimensions
        offset = end
    raise ValueError("JPEG has no scan header")


def compressed_jpeg(message):
    """Validate the supported CompressedImage format and return original bytes."""
    if getattr(message, "_type", "sensor_msgs/CompressedImage") != "sensor_msgs/CompressedImage":
        raise ValueError("Expected sensor_msgs/CompressedImage")
    original_format = message.format
    if not isinstance(original_format, str):
        raise ValueError("CompressedImage.format must be a string")
    normalized = original_format.strip().lower()
    if normalized not in {"jpeg", "jpg"} and not re.fullmatch(
            r"[a-z0-9_]+;\s*(?:jpeg|jpg)\s+compressed(?:\s+[a-z0-9_]+)?", normalized):
        raise ValueError("Unsupported JPEG CompressedImage.format: {!r}".format(original_format))
    if not isinstance(message.data, (bytes, bytearray, memoryview, list, tuple)):
        raise ValueError("CompressedImage.data must be a byte sequence")
    try:
        payload = bytes(message.data)
    except (TypeError, ValueError) as error:
        raise ValueError("Invalid CompressedImage byte sequence") from error
    width, height = jpeg_dimensions(payload)
    return payload, width, height


def annotation_fields(weather=UNKNOWN, time_of_day=UNKNOWN, annotation_source=None):
    values = (weather, time_of_day)
    if any(not isinstance(value, str) or not value.strip() for value in values):
        raise ValueError("Weather/time-of-day annotations must be nonempty strings")
    weather, time_of_day = (value.strip() for value in values)
    source = annotation_source.strip() if annotation_source else None
    if (weather != UNKNOWN or time_of_day != UNKNOWN) and not source:
        raise ValueError("Explicit weather/time-of-day needs --annotation-source verification evidence")
    return {"weather": weather, "time_of_day": time_of_day,
            "source": source, "status": "user_supplied" if source else "unknown"}


def sample_messages(messages, output_dir, source_metadata, *, topic=DEFAULT_TOPIC,
                    interval_sec=0.0, max_frames=None, weather=UNKNOWN,
                    time_of_day=UNKNOWN, annotation_source=None):
    """Write JPEGs to a fresh directory and return a JSON-serializable manifest.

    ``messages`` yields (topic, CompressedImage-like object, bag record stamp).
    No ROS dependency is required. The caller writes manifest.json only after
    any final source-integrity checks. On failure, partial images are retained
    for audit, without a manifest claiming success.
    """
    if not isinstance(topic, str) or not topic.startswith("/"):
        raise ValueError("An explicit absolute ROS image topic is required")
    if not math.isfinite(interval_sec) or interval_sec < 0:
        raise ValueError("interval_sec must be finite and nonnegative")
    interval_ns = round(interval_sec * 1_000_000_000)
    if interval_sec > 0 and interval_ns == 0:
        raise ValueError("Positive intervals must be at least one nanosecond")
    if max_frames is not None and (not isinstance(max_frames, int)
                                  or isinstance(max_frames, bool) or max_frames <= 0):
        raise ValueError("max_frames must be a positive integer or None")
    annotations = annotation_fields(weather, time_of_day, annotation_source)
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=False)
    (output / "images").mkdir()
    source = dict(source_metadata)
    source["topics"] = [topic]
    manifest = {
        "schema_version": 1,
        "source": source,
        "sampler": {"code_path": str(Path(__file__).resolve()),
                    "code_sha256": sha256_file(__file__), "interval_sec": interval_sec,
                    "max_frames": max_frames, "interval_clock": "bag_record_stamp",
                    "image_bytes": "original_CompressedImage.data_no_reencoding",
                    "jpeg_validation": "marker_structure_and_SOF_dimensions_no_pixel_decode"},
        "annotations": annotations, "frames": [],
    }
    previous_ns = None
    next_sample_ns = None
    matching_messages_seen = 0
    for message_topic, message, record_stamp in messages:
        if message_topic != topic:
            continue
        matching_messages_seen += 1
        record = stamp_fields(record_stamp)
        record_ns = record["secs"] * 1_000_000_000 + record["nsecs"]
        if previous_ns is not None and record_ns < previous_ns:
            raise ValueError("Bag record timestamps must be nondecreasing")
        previous_ns = record_ns
        payload, width, height = compressed_jpeg(message)
        header_stamp = stamp_fields(message.header.stamp)
        if next_sample_ns is not None and record_ns < next_sample_ns:
            continue
        relative = "images/frame_{:06d}.jpg".format(len(manifest["frames"]))
        with (output / relative).open("xb") as stream:
            stream.write(payload)
        manifest["frames"].append({
            "image_path": relative, "sha256": hashlib.sha256(payload).hexdigest(),
            "topic": message_topic, "message_format": message.format,
            "header_stamp": header_stamp, "record_stamp": record,
            "width": width, "height": height,
        })
        next_sample_ns = record_ns + interval_ns
        if max_frames is not None and len(manifest["frames"]) >= max_frames:
            break
    if not manifest["frames"]:
        raise ValueError("No matching JPEG frames found for {!r}".format(topic))
    manifest["matching_messages_seen"] = matching_messages_seen
    manifest["frame_count"] = len(manifest["frames"])
    return manifest


def sample_bag(bag_path, output_dir, *, hash_bag=False, **sample_options):
    """ROS wrapper: open only in read mode and publish no ROS messages."""
    bag_path = Path(bag_path).resolve(strict=True)
    if not bag_path.is_file():
        raise ValueError("Input bag must be a regular file")
    if Path(output_dir).exists() or Path(output_dir).is_symlink():
        raise FileExistsError("Output directory must not exist: {}".format(output_dir))
    initial = bag_path.stat()
    source = {"bag_path": str(bag_path), "size_bytes": initial.st_size,
              "mtime_ns": initial.st_mtime_ns, "sha256": sha256_file(bag_path) if hash_bag else None}
    import rosbag  # Deliberately lazy: pure tests and --help work without ROS.
    topic = sample_options.get("topic", DEFAULT_TOPIC)
    with rosbag.Bag(str(bag_path), "r") as bag:
        manifest = sample_messages(bag.read_messages(topics=[topic]), output_dir,
                                   source, **sample_options)
    final = bag_path.stat()
    if (initial.st_size, initial.st_mtime_ns) != (final.st_size, final.st_mtime_ns):
        raise ValueError("Source bag changed during export; no success manifest written")
    manifest_path = Path(output_dir) / "manifest.json"
    with manifest_path.open("x", encoding="utf-8") as stream:
        json.dump(manifest, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="Fresh output directory; must not exist")
    parser.add_argument("--topic", default=DEFAULT_TOPIC)
    parser.add_argument("--interval-sec", type=float, default=0.0)
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument("--weather", default=UNKNOWN)
    parser.add_argument("--time-of-day", default=UNKNOWN)
    parser.add_argument("--annotation-source", help="Evidence for explicit user-verified annotations")
    parser.add_argument("--hash-bag", action="store_true", help="Opt in to reading the entire bag for SHA256")
    args = vars(parser.parse_args(argv))
    bag_path, output = args.pop("bag"), args.pop("output")
    manifest = sample_bag(bag_path, output, **args)
    print(json.dumps({"manifest": str((output / "manifest.json").resolve()),
                      "frame_count": manifest["frame_count"]}))


if __name__ == "__main__":
    main()
