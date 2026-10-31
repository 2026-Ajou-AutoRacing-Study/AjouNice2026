"""순수 큐레이션·계약 테스트. 실제 자료 큐레이션·Torch·학습 실행 없음."""

import copy
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).parent))
SPEC = importlib.util.spec_from_file_location("curator", Path(__file__).with_name("curate_traffic_light_cnn_dataset.py"))
curator = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(curator)
from train_traffic_light_crop_probe import load_dataset
from train_traffic_light_cnn import quality_gate


class CuratorTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.dataset, self.qc = self.root / "raw.json", self.root / "qc.json"
        self.rows, self.qc_rows = [], []
        for i, (split, group, bits) in enumerate([
                ("train", "p1", [1, 0, 0, 0]), ("train", "p2", [0, 1, 1, 1]),
                ("val", "p3", [1, 0, 0, 0]), ("test", "p4", [0, 1, 1, 1])]):
            image = self.root / (str(i) + ".jpg")
            image.write_bytes(("이미지 대역 " + str(i)).encode())
            row = {"image_path": image.name, "image_sha256": curator.sha256(image),
                   "crop_xyxy": [1, 2, 20, 10], "labels": dict(zip(curator.CLASSES, bits)),
                   "native_gt_verified": True, "split": split, "group_id": group,
                   "environment": "Sunny13", "provenance": {"native_observed_state": 1}}
            self.rows.append(row)
            self.qc_rows.append({"image_sha256": row["image_sha256"],
                                 "corrected_crop_xyxy": [1, 2, 20, 10],
                                 "manual_qc_accepted": True, "visual_labels_known": True,
                                 "visual_labels": row["labels"].copy()})

    def prepare(self):
        self.dataset.write_text(json.dumps({"records": self.rows}), encoding="utf-8")
        self.qc.write_text(json.dumps({"records": self.qc_rows}), encoding="utf-8")

    def run_curate(self, qc_paths=None, background=None):
        self.prepare()
        return curator.curate(self.dataset, qc_paths or [self.qc], background,
                              size_reader=lambda path: (100, 60))

    def test_positive_path_matches_native_visual_and_preserves_provenance(self):
        doc, summary = self.run_curate()
        self.assertEqual(summary["accepted_records"], 4)
        self.assertEqual(summary["rejected_records"], 0)
        self.assertEqual(doc["records"][0]["provenance"], self.rows[0]["provenance"])
        self.assertTrue(doc["records"][0]["manual_qc_accepted"])
        ref = doc["records"][0]["curation_provenance"]["qc_references"][0]
        self.assertEqual(ref["sha256"], curator.sha256(self.qc))
        self.assertTrue(summary["training_population_gate_passed"])

    def test_corrected_roi_is_applied_not_approval_copied_to_old_crop(self):
        self.qc_rows[0]["corrected_crop_xyxy"] = [0, 0, 30, 20]
        doc, _ = self.run_curate()
        row = doc["records"][0]
        self.assertEqual(row["crop_xyxy"], [0, 0, 30, 20])
        self.assertTrue(row["curation_provenance"]["explicit_corrected_roi_applied"])
        self.assertEqual(self.rows[0]["crop_xyxy"], [1, 2, 20, 10])

    def test_reject_unknown_false_visual_mismatch_missing_qc(self):
        cases = [("visual_labels_known", False, "visual_labels_unknown"),
                 ("manual_qc_accepted", False, "manual_qc_not_accepted"),
                 ("visual_labels", dict.fromkeys(curator.CLASSES, 0), "native_or_background_visual_label_mismatch")]
        for field, value, reason in cases:
            prior = copy.deepcopy(self.qc_rows[0])
            self.qc_rows[0][field] = value
            _, summary = self.run_curate()
            self.assertEqual(summary["rejected"][0]["reason"], reason)
            self.qc_rows[0] = prior
        self.qc_rows.pop(0)
        _, summary = self.run_curate()
        self.assertEqual(summary["rejected"][0]["reason"], "missing_exact_sha_roi_qc")

    def test_conflicting_explicit_qc_files_fail_without_mtime_selection(self):
        self.prepare()
        conflicting = copy.deepcopy(self.qc_rows)
        conflicting[0]["manual_qc_accepted"] = False
        second = self.root / "second.json"
        second.write_text(json.dumps({"records": conflicting}))
        with self.assertRaisesRegex(ValueError, "판정 충돌"):
            curator.load_qc([self.qc, second])

    def test_by_sha_only_schema_supported(self):
        self.prepare()
        self.qc.write_text(json.dumps({"by_image_sha256": {
            row["image_sha256"]: {key: value for key, value in row.items() if key != "image_sha256"}
            for row in self.qc_rows}}))
        doc, _ = curator.curate(self.dataset, [self.qc], size_reader=lambda path: (100, 60))
        self.assertEqual(len(doc["records"]), 4)

    def test_qc_out_of_bounds_and_image_changed_are_rejected(self):
        self.qc_rows[0]["corrected_crop_xyxy"] = [0, 0, 101, 20]
        _, summary = self.run_curate()
        self.assertIn("outside source image bounds", summary["rejected"][0]["reason"])
        self.qc_rows[0]["corrected_crop_xyxy"] = [1, 2, 20, 10]
        (self.root / "0.jpg").write_bytes("변경".encode())
        _, summary = self.run_curate()
        self.assertEqual(summary["rejected"][0]["reason"], "image_sha256_mismatch")

    def background(self):
        row = copy.deepcopy(self.rows[0])
        row.update(crop_xyxy=[40, 40, 60, 55], labels=dict.fromkeys(curator.CLASSES, 0),
                   annotation_kind=curator.BACKGROUND, native_gt_verified=False)
        return row

    def test_background_requires_exact_sha_roi_and_explicit_no_lamp_qc(self):
        background = self.background()
        path = self.root / "background.json"
        path.write_text(json.dumps({"records": [background]}))
        qc = {"image_sha256": background["image_sha256"], "crop_xyxy": background["crop_xyxy"],
              "annotation_kind": curator.BACKGROUND, "manual_qc_accepted": True,
              "manual_no_lamp_qc_accepted": True, "visual_labels_known": True,
              "visual_labels": background["labels"]}
        self.qc_rows.append(qc)
        doc, summary = self.run_curate(background=path)
        row = doc["records"][-1]
        self.assertFalse(row["native_gt_verified"])
        self.assertTrue(row["manual_no_lamp_qc_accepted"])
        self.assertEqual(summary["accepted_counts"][curator.BACKGROUND]["overall"]["records"], 1)
        # 실제 공용 loader 및 full-CNN 수동 QC gate와 호환하는지 순수 검사.
        curated = self.root / "curated.json"
        curated.write_text(json.dumps(doc))
        records, metadata = load_dataset(curated, size_reader=lambda path: (100, 60), allow_manual_background=True)
        self.assertEqual(metadata["annotation_counts"][curator.BACKGROUND], 1)
        self.assertEqual(quality_gate(records)["manual_background_record_count"], 1)
        self.qc_rows[-1]["manual_no_lamp_qc_accepted"] = False
        _, summary = self.run_curate(background=path)
        self.assertEqual(summary["rejected"][-1]["reason"], "manual_no_lamp_qc_not_accepted")

    def test_background_cannot_reassign_source_frame_split_or_group(self):
        background = self.background()
        background["split"] = "test"
        path = self.root / "background.json"
        path.write_text(json.dumps({"records": [background]}))
        _, summary = self.run_curate(background=path)
        self.assertEqual(summary["rejected"][-1]["reason"], "background_source_frame_split_group_mismatch")

    def test_no_implicit_roi_approval_for_background(self):
        background = self.background()
        path = self.root / "background.json"
        path.write_text(json.dumps({"records": [background]}))
        _, summary = self.run_curate(background=path)
        self.assertEqual(summary["rejected"][-1]["reason"], "missing_exact_sha_roi_qc")


if __name__ == "__main__":
    unittest.main()
