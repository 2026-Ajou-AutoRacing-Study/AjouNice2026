"""Pure tests for research dataset gates and metrics; no Torch or training."""

import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch


SPEC = importlib.util.spec_from_file_location("crop_pilot", Path(__file__).with_name("train_traffic_light_crop_probe.py"))
pilot = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(pilot)


class CropPilotTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.path = self.root / "dataset.json"
        self.rows = []
        for index, (split, group, bits) in enumerate([
                ("train", "pose1", [1, 0, 0, 0]), ("train", "pose2", [0, 1, 1, 1]),
                ("val", "pose3", [1, 0, 0, 0]), ("test", "pose4", [0, 1, 1, 1])]):
            image = self.root / (str(index) + ".jpg")
            image.write_bytes(("fixture-not-image-" + str(index)).encode())
            self.rows.append({"image_path": image.name, "image_sha256": hashlib.sha256(image.read_bytes()).hexdigest(),
                              "crop_xyxy": [2, 3, 20, 10], "labels": dict(zip(pilot.CLASSES, bits)),
                              "split": split, "group_id": group, "environment": "Sunny13",
                              "native_gt_verified": True})

    def load(self, rows=None, **options):
        self.path.write_text(json.dumps({"records": self.rows if rows is None else rows}))
        return pilot.load_dataset(self.path, size_reader=lambda path: (100, 50), **options)

    def test_all_gates_pass_without_torch_or_image_dependencies(self):
        before = set(sys.modules)
        records, meta = self.load()
        self.assertEqual(len(records), 4)
        self.assertEqual(meta["pos_weight_from_train_only"], dict.fromkeys(pilot.CLASSES, 1.0))
        self.assertEqual(records[0]["crop_integer_xyxy"], [2, 3, 20, 10])
        self.assertFalse(any(name == "torch" or name.startswith("torch.") for name in set(sys.modules) - before))

    def test_group_cannot_leak_across_splits(self):
        self.rows[3]["group_id"] = self.rows[0]["group_id"]
        with self.assertRaisesRegex(ValueError, "group.*crosses"):
            self.load()

    def test_same_jpeg_sha_cannot_leak_even_with_renamed_path_and_group(self):
        (self.root / "3.jpg").write_bytes((self.root / "0.jpg").read_bytes())
        self.rows[3]["image_sha256"] = self.rows[0]["image_sha256"]
        with self.assertRaisesRegex(ValueError, "identical image bytes.*crosses"):
            self.load()

    def test_missing_split_single_train_group_and_single_bit_population_rejected(self):
        for change, pattern in [
                (lambda rows: rows[3].update(split="val"), "all be nonempty"),
                (lambda rows: rows[1].update(group_id="pose1"), "two distinct"),
                (lambda rows: rows[1]["labels"].update(left_arrow=0), "positive and negative.*left_arrow")]:
            rows = copy.deepcopy(self.rows)
            change(rows)
            with self.subTest(pattern=pattern), self.assertRaisesRegex(ValueError, pattern):
                self.load(rows)

    def test_native_gt_and_all_four_known_bits_required(self):
        for replacement in (None, False, 1, "true"):
            rows = copy.deepcopy(self.rows)
            rows[0]["native_gt_verified"] = replacement
            with self.assertRaises(ValueError):
                self.load(rows)
        for labels in ({"red": 1}, dict.fromkeys(pilot.CLASSES, None),
                       {**self.rows[0]["labels"], "red": 1.0},
                       {**self.rows[0]["labels"], "unknown": 0}):
            with self.assertRaises(ValueError):
                pilot.validate_labels(labels)

    def test_bad_sha_and_unsafe_paths_rejected(self):
        self.rows[0]["image_sha256"] = "f" * 64
        with self.assertRaisesRegex(ValueError, "SHA256 mismatch"):
            self.load()
        for value in (str(self.root / "0.jpg"), "../0.jpg", ""):
            self.rows[0]["image_path"] = value
            with self.assertRaises(ValueError):
                self.load()

    def test_symlink_outside_manifest_root_rejected(self):
        with tempfile.TemporaryDirectory() as other:
            outside = Path(other) / "outside.jpg"
            outside.write_bytes(b"outside")
            (self.root / "escaped.jpg").symlink_to(outside)
            self.rows[0]["image_path"] = "escaped.jpg"
            with self.assertRaisesRegex(ValueError, "escapes"):
                self.load()

    def test_crop_nonfinite_empty_and_outside_source_rejected(self):
        for box in ([0, 0, float("nan"), 10], [0, 0, float("inf"), 10], [-1, 0, 10, 10],
                    [0, 0, 101, 10], [10, 0, 10, 10], [0, 0, 10, 51],
                    [True, 0, 10, 10], [.1, 0, .9, 10], [0, 0, 10]):
            with self.subTest(box=box), self.assertRaises(ValueError):
                pilot.validate_crop(box, 100, 50)
        self.assertEqual(pilot.validate_crop([.9, .9, 10.9, 10.9], 100, 50), (0, 0, 10, 10))

    def test_letterbox_preserves_ratio_and_fits_width224_height128(self):
        self.assertEqual(pilot.letterbox_geometry(100, 50), ((224, 112), (0, 8)))
        self.assertEqual(pilot.letterbox_geometry(10, 100), ((13, 128), (105, 0)))
        self.assertEqual(pilot.letterbox_geometry(224, 128), ((224, 128), (0, 0)))
        with self.assertRaises(ValueError):
            pilot.letterbox_geometry(0, 10)

    def test_metrics_per_class_and_exact_set_are_distinct(self):
        result = pilot.metric_summary([[1, 0, 1, 0], [1, 1, 0, 1]], [[1, 0, 1, 0], [0, 1, 1, 0]])
        self.assertEqual(result["exact_set_accuracy"], .5)
        self.assertEqual(result["per_class"]["red"]["precision"], 1)
        self.assertEqual(result["per_class"]["red"]["recall"], .5)
        self.assertAlmostEqual(result["per_class"]["green"]["f1"], 2 / 3)
        self.assertEqual(result["per_class"]["left_arrow"]["f1"], 0)
        self.assertIsNone(pilot.metric_summary([], [])["exact_set_accuracy"])
        with self.assertRaises(ValueError):
            pilot.metric_summary([[1, 0, 0, 0]], [])

    def test_metrics_subset_by_environment_and_group(self):
        records, _ = self.load()
        result = pilot.evaluation_metrics(records, [row["bits"] for row in records])
        self.assertEqual(result["test"]["overall"]["exact_set_accuracy"], 1)
        self.assertEqual(result["train"]["by_environment"]["Sunny13"]["count"], 2)
        self.assertEqual(result["test"]["by_group_id"]["pose4"]["count"], 1)

    def test_hsv_circle_and_left_arrow_have_independent_bits(self):
        self.assertEqual(pilot.hsv_bits([("red", "circle", None), ("green", "arrow", True)]), [1, 0, 0, 1])
        self.assertEqual(pilot.hsv_bits([("green", "circle", None), ("green", "arrow", True)]), [0, 0, 1, 1])

    def manual_background(self):
        # The same original JPEG may legitimately supply another ROI, but its
        # existing pose/split must remain identical to the signal-head record.
        background = copy.deepcopy(self.rows[0])
        background.update(native_gt_verified=False, annotation_kind="manual_background_no_lamps",
                          manual_qc_accepted=True, manual_no_lamp_qc_accepted=True,
                          labels=dict.fromkeys(pilot.CLASSES, 0), crop_xyxy=[30, 20, 50, 40])
        return background

    def test_manual_background_requires_explicit_opt_in_and_keeps_false_native_flag(self):
        background = self.manual_background()
        rows = self.rows + [background]
        with self.assertRaisesRegex(ValueError, "native_gt_verified:true"):
            self.load(rows)
        prepared, metadata = self.load(rows, allow_manual_background=True)
        self.assertIs(prepared[-1]["native_gt_verified"], False)
        self.assertEqual(prepared[-1]["annotation_kind"], "manual_background_no_lamps")
        self.assertEqual(prepared[-1]["bits"], [0, 0, 0, 0])
        self.assertEqual(metadata["annotation_counts"], {"native_verified": 4, "manual_background_no_lamps": 1})
        self.assertEqual(metadata["unique_image_sha256_count"], 4)

    def test_background_positive_labels_missing_qc_unknown_and_fabricated_native_rejected(self):
        mutations = [lambda row: row["labels"].update(green=1),
                     lambda row: row.pop("manual_no_lamp_qc_accepted"),
                     lambda row: row.update(manual_qc_accepted=False),
                     lambda row: row.update(manual_no_lamp_qc_accepted=1),
                     lambda row: row.update(annotation_kind="unknown_unreadable_signal"),
                     lambda row: row.update(native_gt_verified=None),
                     lambda row: row.update(native_gt_verified=True)]
        for mutation in mutations:
            background = self.manual_background()
            mutation(background)
            with self.subTest(background=background), self.assertRaises(ValueError):
                self.load(self.rows + [background], allow_manual_background=True)

    def test_generic_unverified_native_record_is_still_blocked_with_background_opt_in(self):
        rows = copy.deepcopy(self.rows)
        rows[0]["native_gt_verified"] = False
        with self.assertRaises(ValueError):
            self.load(rows, allow_manual_background=True)

    def test_background_same_image_cannot_cross_split_even_with_renamed_pose(self):
        background = self.manual_background()
        background.update(split="test", group_id="renamed_pose")
        with self.assertRaisesRegex(ValueError, "identical image bytes.*crosses"):
            self.load(self.rows + [background], allow_manual_background=True)

    def test_background_same_pose_cannot_cross_split_even_with_different_image(self):
        background = self.manual_background()
        background.update(split="test", image_path=self.rows[3]["image_path"],
                          image_sha256=self.rows[3]["image_sha256"])
        with self.assertRaisesRegex(ValueError, "group.*crosses"):
            self.load(self.rows + [background], allow_manual_background=True)


if __name__ == "__main__":
    unittest.main()
