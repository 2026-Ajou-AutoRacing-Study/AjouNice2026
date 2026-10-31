#!/usr/bin/env python3
"""Pure sampler tests; ROS, OpenCV, Pillow, YOLO, and GPUs are not required."""

import hashlib
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

from sample_traffic_light_bag_frames import (
    DEFAULT_TOPIC, annotation_fields, compressed_jpeg, jpeg_dimensions,
    sample_bag, sample_messages, sha256_file,
)


def jpeg_fixture(width=1280, height=720, scan=b"\x12\x34\x56"):
    """Structural JPEG fixture; the sampler intentionally never decodes pixels."""
    sof = (b"\xff\xc0\x00\x0b\x08" + height.to_bytes(2, "big")
           + width.to_bytes(2, "big") + b"\x01\x01\x11\x00")
    sos = b"\xff\xda\x00\x08\x01\x01\x00\x00\x3f\x00"
    return b"\xff\xd8" + sof + sos + scan + b"\xff\xd9"


def stamp(secs, nsecs=0):
    return SimpleNamespace(secs=secs, nsecs=nsecs)


def message(payload=None, image_format="bgr8; jpeg compressed bgr8", header=None):
    return SimpleNamespace(_type="sensor_msgs/CompressedImage", format=image_format,
                           data=jpeg_fixture() if payload is None else payload,
                           header=SimpleNamespace(stamp=stamp(0) if header is None else header))


def row(secs, nsecs=0, **kwargs):
    return DEFAULT_TOPIC, message(**kwargs), stamp(secs, nsecs)


class TrafficLightBagSamplerTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.output = self.root / "new_export"
        self.source = {"bag_path": "/original/input.bag", "size_bytes": 1234,
                       "mtime_ns": 123456789, "sha256": None}

    def sample(self, messages, **kwargs):
        return sample_messages(messages, self.output, self.source, **kwargs)

    def test_original_bytes_hashes_format_and_integer_timestamps_are_preserved(self):
        original = jpeg_fixture(scan=b"\x7f\x00\xff\x00\x66")
        manifest = self.sample([row(900, 987654321, payload=original,
                                    header=stamp(123, 123456789))])
        frame = manifest["frames"][0]
        self.assertEqual((self.output / frame["image_path"]).read_bytes(), original)
        self.assertEqual(frame["sha256"], hashlib.sha256(original).hexdigest())
        self.assertEqual(frame["header_stamp"], {"secs": 123, "nsecs": 123456789})
        self.assertEqual(frame["record_stamp"], {"secs": 900, "nsecs": 987654321})
        self.assertEqual(frame["message_format"], "bgr8; jpeg compressed bgr8")
        self.assertEqual((frame["width"], frame["height"]), (1280, 720))
        self.assertFalse(Path(frame["image_path"]).is_absolute())
        self.assertEqual(manifest["source"]["topics"], [DEFAULT_TOPIC])
        self.assertEqual(manifest["source"]["bag_path"], "/original/input.bag")
        self.assertIsNone(manifest["source"]["sha256"])
        self.assertEqual(manifest["sampler"]["code_sha256"],
                         sha256_file(Path(__file__).with_name("sample_traffic_light_bag_frames.py")))
        json.dumps(manifest, allow_nan=False)

    def test_zero_header_is_not_replaced_by_record_time(self):
        manifest = self.sample([row(1234, header=stamp(0))])
        self.assertEqual(manifest["frames"][0]["header_stamp"], {"secs": 0, "nsecs": 0})

    def test_no_weather_or_time_of_day_is_inferred_from_source_path(self):
        self.source["bag_path"] = "/Sunny13_night/input.bag"
        manifest = self.sample([row(1)])
        self.assertEqual(manifest["annotations"],
                         {"weather": "UNKNOWN", "time_of_day": "UNKNOWN",
                          "source": None, "status": "unknown"})

    def test_explicit_annotations_require_evidence_and_are_not_normalized(self):
        for kwargs in ({"weather": "Sunny13"}, {"time_of_day": "night"}):
            with self.assertRaises(ValueError):
                annotation_fields(**kwargs)
        result = annotation_fields("Sunny13", "UNKNOWN", "user verified simulator settings")
        self.assertEqual(result["weather"], "Sunny13")
        self.assertEqual(result["time_of_day"], "UNKNOWN")
        self.assertEqual(result["status"], "user_supplied")

    def test_existing_output_is_refused_without_touching_it(self):
        self.output.mkdir()
        sentinel = self.output / "sentinel"
        sentinel.write_bytes(b"keep")
        with self.assertRaises(FileExistsError):
            self.sample([row(1)])
        self.assertEqual(sentinel.read_bytes(), b"keep")
        self.assertEqual(list(self.output.iterdir()), [sentinel])

    def test_sampling_uses_record_time_and_exact_interval_boundary(self):
        rows = [row(10), row(11, 999999999), row(12), row(13), row(14)]
        manifest = self.sample(rows, interval_sec=2)
        self.assertEqual([frame["record_stamp"]["secs"] for frame in manifest["frames"]],
                         [10, 12, 14])
        self.assertEqual(manifest["sampler"]["interval_clock"], "bag_record_stamp")

    def test_limit_does_not_consume_an_extra_message(self):
        def messages():
            yield row(1)
            yield row(2)
            raise AssertionError("Iterator consumed past max_frames")
        manifest = self.sample(messages(), max_frames=2)
        self.assertEqual(manifest["frame_count"], 2)
        self.assertEqual(manifest["matching_messages_seen"], 2)

    def test_non_target_topics_are_not_decoded(self):
        manifest = self.sample([("/other/topic", object(), object()), row(1)])
        self.assertEqual(manifest["frame_count"], 1)

    def test_invalid_configuration_does_not_create_output(self):
        cases = [{"interval_sec": -1}, {"interval_sec": float("nan")},
                 {"interval_sec": float("inf")}, {"max_frames": 0},
                 {"max_frames": -1}, {"max_frames": True}, {"max_frames": 1.5},
                 {"weather": "Sunny13"}, {"topic": "relative"}]
        for kwargs in cases:
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                self.sample([row(1)], **kwargs)
            self.assertFalse(self.output.exists())

    def test_bad_matching_frame_is_not_silently_skipped_by_interval(self):
        with self.assertRaises(ValueError):
            self.sample([row(1), row(2, payload=b"broken")], interval_sec=10)
        self.assertFalse((self.output / "manifest.json").exists())

    def test_backward_record_stamp_is_rejected(self):
        with self.assertRaises(ValueError):
            self.sample([row(2), row(1)])

    def test_no_matching_frames_is_an_explicit_error(self):
        with self.assertRaises(ValueError):
            self.sample([])
        self.assertFalse((self.output / "manifest.json").exists())

    def test_supported_original_formats_and_byte_sequences(self):
        for image_format in ("jpeg", "jpg", "RGB8; JPEG compressed BGR8", "mono8; jpeg compressed"):
            with self.subTest(image_format=image_format):
                payload, width, height = compressed_jpeg(message(
                    payload=list(jpeg_fixture(64, 32)), image_format=image_format))
                self.assertEqual(payload, jpeg_fixture(64, 32))
                self.assertEqual((width, height), (64, 32))

    def test_unsupported_formats_and_message_types_are_rejected(self):
        for image_format in ("", "png", "bgr8; png compressed bgr8", "16UC1; compressedDepth png"):
            with self.subTest(image_format=image_format), self.assertRaises(ValueError):
                compressed_jpeg(message(image_format=image_format))
        wrong_type = message()
        wrong_type._type = "sensor_msgs/Image"
        with self.assertRaises(ValueError):
            compressed_jpeg(wrong_type)
        for data in (7, "jpeg", [-1], [256]):
            with self.subTest(data=data), self.assertRaises(ValueError):
                compressed_jpeg(message(payload=data))

    def test_empty_wrong_magic_truncated_segments_and_no_scan_are_rejected(self):
        cases = (b"", b"\x89PNG\r\n", b"\xff\xd8\xff\xd9", jpeg_fixture()[:-2],
                 b"\xff\xd8\xff\xe0\xff\xff\x00\xff\xd9",
                 jpeg_fixture().replace(b"\xff\xda", b"\xff\xe1"),
                 jpeg_fixture(width=0), jpeg_fixture(height=0))
        for payload in cases:
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                jpeg_dimensions(payload)

    def test_bag_wrapper_uses_read_mode_and_writes_success_manifest(self):
        bag_path = self.root / "input.bag"
        bag_path.write_bytes(b"fake bag; ROS reader is injected")
        original = bag_path.read_bytes()
        fake_bag = mock.MagicMock()
        fake_bag.__enter__.return_value = fake_bag
        fake_bag.read_messages.return_value = iter([row(1)])
        fake_rosbag = SimpleNamespace(Bag=mock.Mock(return_value=fake_bag))
        with mock.patch.dict(sys.modules, {"rosbag": fake_rosbag}):
            manifest = sample_bag(bag_path, self.output)
        fake_rosbag.Bag.assert_called_once_with(str(bag_path.resolve()), "r")
        fake_bag.read_messages.assert_called_once_with(topics=[DEFAULT_TOPIC])
        self.assertEqual(bag_path.read_bytes(), original)
        self.assertEqual(json.loads((self.output / "manifest.json").read_text()), manifest)
        self.assertEqual(manifest["source"]["size_bytes"], len(original))
        self.assertEqual(manifest["source"]["mtime_ns"], bag_path.stat().st_mtime_ns)
        self.assertIsNone(manifest["source"]["sha256"])

    def test_whole_bag_hash_is_explicitly_optional(self):
        bag_path = self.root / "input.bag"
        bag_path.write_bytes(b"fixture bag")
        fake_bag = mock.MagicMock()
        fake_bag.__enter__.return_value = fake_bag
        fake_bag.read_messages.return_value = iter([row(1)])
        with mock.patch.dict(sys.modules, {"rosbag": SimpleNamespace(Bag=lambda *args: fake_bag)}):
            manifest = sample_bag(bag_path, self.output, hash_bag=True)
        self.assertEqual(manifest["source"]["sha256"], hashlib.sha256(b"fixture bag").hexdigest())

    def test_source_change_during_read_has_no_success_manifest(self):
        bag_path = self.root / "input.bag"
        bag_path.write_bytes(b"fixture bag")

        def changing_source():
            yield row(1)
            bag_path.write_bytes(b"changed fixture bag")

        fake_bag = mock.MagicMock()
        fake_bag.__enter__.return_value = fake_bag
        fake_bag.read_messages.return_value = changing_source()
        with mock.patch.dict(sys.modules, {"rosbag": SimpleNamespace(Bag=lambda *args: fake_bag)}):
            with self.assertRaisesRegex(ValueError, "Source bag changed"):
                sample_bag(bag_path, self.output)
        self.assertFalse((self.output / "manifest.json").exists())

    def test_pure_sampler_works_when_ros_import_is_unavailable(self):
        with mock.patch.dict(sys.modules, {"rosbag": None}):
            manifest = self.sample([row(1)])
        self.assertEqual(manifest["frame_count"], 1)


if __name__ == "__main__":
    unittest.main()
