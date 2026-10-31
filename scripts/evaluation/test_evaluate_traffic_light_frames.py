#!/usr/bin/env python3
"""Offline runner regression tests using fake predictions, never YOLO/GPUs."""

import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

import cv2
import numpy as np

from evaluate_traffic_light_frames import (evaluate_manifest, model_names, observation_label,
                                           offline_environment, prediction_boxes, read_manifest)


def prediction(boxes=()):
    return [SimpleNamespace(boxes=SimpleNamespace(
        xyxy=np.asarray([box[:4] for box in boxes]),
        conf=np.asarray([box[4] for box in boxes]),
        cls=np.asarray([box[5] if len(box) > 5 else 0 for box in boxes])))]


class TrafficLightFrameRunnerTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.samples = self.root / "samples"
        (self.samples / "images").mkdir(parents=True)
        self.output = self.root / "results"
        self.weights = self.root / "fixture.pt"
        self.weights.write_bytes(b"fake weights; injected predictor, never loaded")
        self.manifest = self.samples / "manifest.json"
        image = np.zeros((72, 128, 3), np.uint8)
        cv2.circle(image, (64, 20), 4, (0, 0, 255), -1)
        ok, encoded = cv2.imencode(".jpg", image)
        self.assertTrue(ok)
        payload = encoded.tobytes()
        frames = []
        for index in range(2):
            relative = "images/frame_{:06d}.jpg".format(index)
            (self.samples / relative).write_bytes(payload)
            frames.append({"image_path": relative, "sha256": hashlib.sha256(payload).hexdigest(),
                           "width": 128, "height": 72,
                           "header_stamp": {"secs": 0, "nsecs": index},
                           "record_stamp": {"secs": 123, "nsecs": index}})
        self.data = {"schema_version": 1, "frame_count": 2, "frames": frames,
                     "source": {"bag_path": "/original/input.bag"},
                     "annotations": {"weather": "UNKNOWN", "time_of_day": "UNKNOWN"}}
        self.save_manifest()
        self.model = SimpleNamespace(names={0: "traffic_light"},
                                     predict=mock.Mock(return_value=prediction([[57, 12, 71, 28, .9]])))

    def save_manifest(self):
        self.manifest.write_text(json.dumps(self.data))

    def run_fixture(self, **kwargs):
        return evaluate_manifest(self.manifest, self.output, weights=self.weights,
                                 model=self.model, **kwargs)

    def test_end_to_end_predictions_only_preserves_originals_and_records_geometry(self):
        originals = {path: path.read_bytes() for path in self.samples.rglob("*.jpg")}
        with mock.patch.dict(sys.modules, {"ultralytics": None}):
            summary = self.run_fixture()
        self.assertEqual(summary["evaluation_mode"], "predictions_only")
        self.assertFalse(summary["accuracy_computed"])
        self.assertNotIn("accuracy", summary)
        self.assertEqual(summary["frames_with_human_gt"], 0)
        self.assertEqual(summary["frame_count"], 2)
        self.assertEqual(summary["bbox_count"], 2)
        rows = [json.loads(line) for line in (self.output / "results.jsonl").read_text().splitlines()]
        self.assertIsNone(rows[0]["human_ground_truth"])
        self.assertEqual(rows[0]["diagnosis"]["selected_index"], 0)
        self.assertEqual(rows[0]["diagnosis"]["baseline_observation"], [["red", "circle", None]])
        self.assertEqual(rows[0]["source_frame"]["header_stamp"], {"secs": 0, "nsecs": 0})
        self.assertEqual(rows[0]["review_geometry"]["selected_crop_source_xyxy"], [57, 12, 71, 28])
        self.assertEqual(rows[1]["review_geometry"]["tile_xyxy_contact_sheet"], [480, 0, 960, 240])
        sheet = cv2.imread(str(self.output / "review_contact_sheet.jpg"))
        self.assertEqual(list(sheet.shape[1::-1]), summary["contact_sheet"]["dimensions_wh"])
        for path, payload in originals.items():
            self.assertEqual(path.read_bytes(), payload)
        call = self.model.predict.call_args.kwargs
        self.assertEqual((call["conf"], call["imgsz"], call["device"]), (.25, 640, "cpu"))
        self.assertFalse(call["save"])

    def test_preflight_hash_mismatch_fails_before_model_or_output(self):
        (self.samples / self.data["frames"][0]["image_path"]).write_bytes(b"tampered")
        with self.assertRaisesRegex(ValueError, "SHA256 mismatch"):
            self.run_fixture()
        self.model.predict.assert_not_called()
        self.assertFalse(self.output.exists())

    def test_missing_jpeg_magic_is_rejected_even_with_updated_hash(self):
        frame = self.data["frames"][0]
        (self.samples / frame["image_path"]).write_bytes(b"not a jpeg")
        frame["sha256"] = hashlib.sha256(b"not a jpeg").hexdigest()
        self.save_manifest()
        with self.assertRaisesRegex(ValueError, "not a complete JPEG"):
            self.run_fixture()

    def test_existing_output_is_never_overwritten(self):
        self.output.mkdir()
        (self.output / "sentinel").write_bytes(b"keep")
        with self.assertRaises(FileExistsError):
            self.run_fixture()
        self.assertEqual((self.output / "sentinel").read_bytes(), b"keep")

    def test_no_detection_is_not_called_off_or_ground_truth(self):
        self.model.predict.return_value = prediction()
        summary = self.run_fixture(max_frames=1)
        row = json.loads((self.output / "results.jsonl").read_text())
        self.assertEqual(row["observation_label"], "no_lamp_observation")
        self.assertEqual(row["diagnosis"]["crop_status"], "no_selected_box")
        self.assertIsNone(row["human_ground_truth"])
        self.assertEqual(summary["bbox_count"], 0)

    def test_all_boxes_and_actual_picker_not_highest_confidence_are_recorded(self):
        self.model.predict.return_value = prediction([
            [2, 5, 16, 25, .99], [57, 12, 71, 28, .3]])
        summary = self.run_fixture(max_frames=1)
        row = json.loads((self.output / "results.jsonl").read_text())
        self.assertEqual(summary["bbox_count"], 2)
        self.assertEqual(len(row["diagnosis"]["boxes"]), 2)
        self.assertEqual(row["diagnosis"]["selected_index"], 1)
        self.assertEqual(row["diagnosis"]["selection_mode"], "baseline_picker")

    def test_decoded_dimension_mismatch_has_no_success_summary(self):
        self.data["frames"][0]["width"] = 999
        self.save_manifest()
        with self.assertRaisesRegex(ValueError, "Decoded dimensions"):
            self.run_fixture()
        self.model.predict.assert_not_called()
        self.assertFalse((self.output / "summary.json").exists())

    def test_human_annotation_stays_separate_and_is_not_scored(self):
        gt_path = self.root / "human.json"
        human = {"phase": "red_plus_left_arrow", "reviewer": "manual"}
        gt_path.write_text(json.dumps({self.data["frames"][0]["image_path"]: human}))
        summary = self.run_fixture(human_gt_json=gt_path)
        row = json.loads((self.output / "results.jsonl").read_text().splitlines()[0])
        self.assertEqual(row["human_ground_truth"], human)
        self.assertEqual(row["diagnosis"]["baseline_observation"], [["red", "circle", None]])
        self.assertFalse(summary["accuracy_computed"])
        self.assertEqual(summary["frames_with_human_gt"], 1)

    def test_max_frames_and_contact_sheet_limits(self):
        summary = self.run_fixture(max_frames=1, contact_sheet_max_frames=1, contact_sheet_columns=1)
        self.assertEqual(summary["frame_count"], 1)
        self.assertEqual(summary["manifest_frame_count"], 2)
        self.assertEqual(summary["contact_sheet"]["dimensions_wh"], [480, 240])
        self.model.predict.assert_called_once()

    def test_non_housing_model_and_boxes_are_rejected(self):
        with self.assertRaises(ValueError):
            model_names(SimpleNamespace(names={0: "person", 1: "car"}))
        with self.assertRaises(ValueError):
            prediction_boxes(prediction([[1, 1, 2, 2, .9, 3]]))
        with self.assertRaises(ValueError):
            prediction_boxes(prediction([[1, 1, 2, 2, float("nan")]]))

    def test_unsafe_duplicate_and_escaped_paths_are_rejected(self):
        for invalid in ("../outside.jpg", "/tmp/outside.jpg", self.data["frames"][0]["image_path"]):
            self.data["frames"][1]["image_path"] = invalid
            self.save_manifest()
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                read_manifest(self.manifest)

    def test_bad_parameters_do_not_create_output(self):
        for kwargs in ({"conf": float("nan")}, {"conf": 2}, {"imgsz": 0},
                       {"max_frames": 0}, {"contact_sheet_columns": 0}, {"contact_sheet_max_frames": 0}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                self.run_fixture(**kwargs)
            self.assertFalse(self.output.exists())

    def test_offline_environment_is_temporary_and_restored_on_error(self):
        with mock.patch.dict(os.environ, {"YOLO_OFFLINE": "previous", "YOLO_CONFIG_DIR": "/old"}):
            with self.assertRaises(RuntimeError):
                with offline_environment():
                    directory = Path(os.environ["YOLO_CONFIG_DIR"])
                    self.assertTrue(directory.is_dir())
                    self.assertEqual(os.environ["YOLO_OFFLINE"], "true")
                    raise RuntimeError("fixture")
            self.assertEqual(os.environ["YOLO_OFFLINE"], "previous")
            self.assertEqual(os.environ["YOLO_CONFIG_DIR"], "/old")
            self.assertFalse(directory.exists())

    def test_arrow_labels_are_predictions_not_single_color_softmax(self):
        self.assertEqual(observation_label([["red", "circle", None], ["green", "arrow", True]]),
                         "red_circle+green_arrow_left")


if __name__ == "__main__":
    unittest.main()
