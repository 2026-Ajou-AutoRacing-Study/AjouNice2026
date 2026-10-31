#!/usr/bin/env python3
"""Synthetic sheet/index integrity only; never visual-label certification."""
import hashlib
import io
import json
from contextlib import redirect_stdout
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

from PIL import Image

import prepare_traffic_light_housing_review as housing
import prepare_traffic_light_cnn_crop_qc as cnn_review


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class PrepareReviewMetadataTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name)
        self.image = self.directory/"synthetic.jpg"
        Image.new("RGB", (96, 64), (40, 80, 120)).save(self.image)
        self.source = self.directory/"input.json"
        self.templates = self.directory/"templates.json"
        self.templates.write_text(json.dumps({"groups": {"fixture": {
            "C1256W000016": [10, 10, 50, 22], "C1256W000018": [55, 12, 85, 22]}}}))
        self.document = {"records": [{"image_path": self.image.name, "image_sha256": digest(self.image),
            "group_id": "fixture", "environment": "synthetic", "split": "test",
            "boxes": [{"native_id": "C1256W000016", "xyxy": [10, 10, 50, 22], "proposal_only": True}],
            "labels": {"red": 1, "yellow": 0, "green": 0, "left_arrow": 0}}]}
        self.source.write_text(json.dumps(self.document))

    def run_housing(self, output, templates=True):
        argv = [housing.__file__, "--dataset", str(self.source), "--output", str(output)]
        if templates:
            argv += ["--templates", str(self.templates)]
        with patch.object(sys, "argv", argv), redirect_stdout(io.StringIO()): housing.main()
        return json.loads((output/"index.json").read_text())

    def run_cnn(self, output):
        with patch.object(sys, "argv", [cnn_review.__file__, "--dataset", str(self.source),
             "--output", str(output)]), redirect_stdout(io.StringIO()): cnn_review.main()
        return json.loads((output/"index.json").read_text())

    def proposal_row(self):
        row = dict(self.document["records"][0], crop_integer_xyxy=[10, 10, 50, 22])
        with Image.open(self.image) as original:
            crop = original.convert("RGB").crop(row["crop_integer_xyxy"])
        row["provenance"] = {"yolo_crop_proposal": {"native_id_joined": "C1256W000016",
            "crop_pixel_sha256_rgb": hashlib.sha256(crop.tobytes()).hexdigest()}}
        return row

    def test_template_sheets_include_each_frame_head_and_do_not_approve_sources(self):
        self.document["records"].append(dict(self.document["records"][0]))
        self.source.write_text(json.dumps(self.document))
        before = self.source.read_bytes(), self.image.read_bytes()
        index = self.run_housing(self.directory/"housing")
        self.assertEqual(len(index["entries"]), 4)
        self.assertEqual([entry["record_index"] for entry in index["entries"]], [0, 0, 1, 1])
        self.assertTrue(index["review_does_not_approve_labels"])
        self.assertEqual(index["source_sha256"], digest(self.source))
        self.assertEqual((self.source.read_bytes(), self.image.read_bytes()), before)
        self.assertNotIn("manual_direct_visual_qc_accepted", self.source.read_text())

    def test_reference_mode_is_representative_per_environment_pose_not_all_frames(self):
        second = dict(self.document["records"][0])
        self.document["records"] += [second, dict(second, environment="another_synthetic")]
        self.source.write_text(json.dumps(self.document))
        index = self.run_housing(self.directory/"reference", templates=False)
        self.assertEqual([entry["record_index"] for entry in index["entries"]], [0, 2])
        self.assertTrue(index["review_does_not_approve_labels"])

    def test_existing_review_directory_never_overwritten(self):
        output = self.directory/"housing"; output.mkdir()
        marker = output/"user.txt"; marker.write_text("keep")
        with self.assertRaises(FileExistsError): self.run_housing(output)
        self.assertEqual(marker.read_text(), "keep")

    def test_CNN_crop_index_preserves_original_row_index_pixel_hash_and_QC_flags(self):
        proposal = self.proposal_row()
        self.document["records"].append(proposal)
        self.source.write_text(json.dumps(self.document))
        before = self.source.read_bytes(), self.image.read_bytes()
        index = self.run_cnn(self.directory/"cnn")
        self.assertEqual(len(index["proposal_rows"]), 1)
        self.assertEqual(index["proposal_rows"][0]["record_index"], 1)
        self.assertEqual(index["proposal_rows"][0]["pixel_sha256_rgb"],
                         proposal["provenance"]["yolo_crop_proposal"]["crop_pixel_sha256_rgb"])
        self.assertTrue(index["no_QC_flags_changed"])
        self.assertEqual((self.source.read_bytes(), self.image.read_bytes()), before)

    def test_CNN_changed_proposal_pixels_fail_closed(self):
        proposal = self.proposal_row()
        proposal["provenance"]["yolo_crop_proposal"]["crop_pixel_sha256_rgb"] = "0"*64
        self.document["records"] = [proposal]
        self.source.write_text(json.dumps(self.document))
        with self.assertRaisesRegex(ValueError, "Proposal pixels changed"):
            self.run_cnn(self.directory/"bad_cnn")

    def test_housing_preparer_must_detect_changed_original_JPEG(self):
        Image.new("RGB", (96, 64), "red").save(self.image)
        with self.assertRaises(ValueError): self.run_housing(self.directory/"changed")


if __name__ == "__main__":
    unittest.main()
