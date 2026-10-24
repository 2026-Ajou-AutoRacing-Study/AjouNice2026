#!/usr/bin/env python3
"""Synthetic metadata regression, NOT visual-label/identity/lamp-GT certification."""
import copy
import io
import json
from contextlib import redirect_stdout
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

from PIL import Image

import curate_traffic_light_identity_qc as curator
import prepare_traffic_light_housing_review as prepare


class IdentityFixture:
    def __init__(self, directory, frames=1):
        self.directory = Path(directory)
        self.source = self.directory/"manifest.json"
        self.templates = self.directory/"templates.json"
        self.review = self.directory/"review"
        self.index = self.review/"index.json"
        self.confirmation = self.directory/"confirmation.json"
        self.output = self.directory/"identity_qc.json"
        rows = []
        for index in range(frames):
            image = self.directory/("frame_%02d.jpg" % index)
            Image.new("RGB", (96, 64), (index*20, 30, 80)).save(image)
            rows.append({"image_path": image.name, "image_sha256": curator.digest(image),
                         "group_id": "fixture_C", "group_regulatory_id": 201290,
                         "environment": "synthetic", "split": "test",
                         "labels": {"red": 1, "yellow": 0, "green": 0, "left_arrow": 0},
                         "native_before": [{"requested_id": "C1256W000016"}]})
        self.document = {"records": rows}
        self.source.write_text(json.dumps(self.document))
        self.templates.write_text(json.dumps({"groups": {"fixture_C": {"C1256W000016": [10, 10, 50, 22]}}}))
        with patch.object(sys, "argv", [prepare.__file__, "--dataset", str(self.source),
             "--templates", str(self.templates), "--output", str(self.review)]), redirect_stdout(io.StringIO()):
            prepare.main()
        self.approve()

    def approve(self):
        review = json.loads(self.index.read_text())
        self.confirmation.write_text(json.dumps({"review_index_sha256": curator.digest(self.index),
            "all_listed_crops_directly_reviewed": True, "accepted": True,
            "reviewed_crop_count": len(review["entries"]),
            "reviewed_sheets": {name: curator.digest(self.review/name)
                                for name in {entry["sheet"] for entry in review["entries"]}}}))

    def change_index(self, change):
        value = json.loads(self.index.read_text())
        change(value)
        self.index.write_text(json.dumps(value))

    def curate(self):
        with patch.object(sys, "argv", [curator.__file__, "--dataset", str(self.source),
             "--review-index", str(self.index), "--confirmation", str(self.confirmation),
             "--output", str(self.output)]), redirect_stdout(io.StringIO()):
            curator.main()
        return json.loads(self.output.read_text())


class CurateIdentityMetadataTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.fixture = IdentityFixture(self.temporary.name)

    def test_approved_metadata_preserves_sources_and_remains_identity_only(self):
        tracked = [self.fixture.source, self.fixture.index, self.fixture.confirmation,
                   self.fixture.directory/"frame_00.jpg", self.fixture.review/"crops_000.png"]
        before = {path: path.read_bytes() for path in tracked}
        result = self.fixture.curate()
        self.assertFalse(result["pixel_lamp_labels_certified"])
        row = result["records"][0]
        self.assertIs(row["annotation_complete"], False)
        self.assertIs(row["identity_annotations_reviewed"], True)
        self.assertEqual(row["annotation_scope"], "named_housing_identity_evaluation_only_not_detector_training_GT")
        self.assertNotIn("labels", row)
        self.assertNotIn("native_before", row)
        self.assertFalse(row["boxes"][0]["independent_human_GT"])
        self.assertTrue(row["boxes"][0]["manual_direct_visual_qc_accepted"])
        self.assertEqual({path: path.read_bytes() for path in tracked}, before)

    def test_missing_or_false_explicit_approval_rejected(self):
        approval = json.loads(self.fixture.confirmation.read_text())
        for field in ("accepted", "all_listed_crops_directly_reviewed"):
            for value in (False, None, "true"):
                changed = dict(approval, **{field: value})
                self.fixture.confirmation.write_text(json.dumps(changed))
                with self.assertRaisesRegex(ValueError, "Explicit completed"):
                    self.fixture.curate()
                self.assertFalse(self.fixture.output.exists())

    def test_changed_review_index_after_confirmation_rejected(self):
        self.fixture.change_index(lambda value: value["entries"][0].update(tile=9))
        with self.assertRaisesRegex(ValueError, "Review index changed"):
            self.fixture.curate()

    def test_changed_sheet_after_confirmation_rejected(self):
        Image.new("RGB", (1280, 840), "red").save(self.fixture.review/"crops_000.png")
        with self.assertRaisesRegex(ValueError, "Review sheet changed"):
            self.fixture.curate()

    def test_changed_manifest_and_changed_original_JPEG_rejected(self):
        self.fixture.source.write_text(self.fixture.source.read_text()+"\n")
        with self.assertRaisesRegex(ValueError, "Source manifest changed"):
            self.fixture.curate()
        self.fixture.source.write_text(json.dumps(self.fixture.document))
        Image.new("RGB", (96, 64), "red").save(self.fixture.directory/"frame_00.jpg")
        with self.assertRaisesRegex(ValueError, "JPEG changed"):
            self.fixture.curate()

    def test_wrong_crop_count_and_duplicate_frame_head_rejected(self):
        confirmation = json.loads(self.fixture.confirmation.read_text())
        confirmation["reviewed_crop_count"] += 1
        self.fixture.confirmation.write_text(json.dumps(confirmation))
        with self.assertRaisesRegex(ValueError, "count mismatch"):
            self.fixture.curate()
        self.fixture.change_index(lambda value: value["entries"].append(copy.deepcopy(value["entries"][0])))
        self.fixture.approve()
        with self.assertRaisesRegex(ValueError, "Repeated frame/head"):
            self.fixture.curate()

    def test_no_existing_output_overwrite(self):
        self.fixture.output.write_text("user_owned_existing_output")
        with self.assertRaisesRegex(ValueError, "Fresh output"):
            self.fixture.curate()
        self.assertEqual(self.fixture.output.read_text(), "user_owned_existing_output")

    def test_unreviewed_source_frame_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = IdentityFixture(directory, frames=2)
            fixture.change_index(lambda value: value["entries"].pop())
            fixture.approve()
            with self.assertRaisesRegex(ValueError, "Unreviewed source frame"):
                fixture.curate()

    def test_sheet_traversal_and_output_in_other_directory_rejected(self):
        self.fixture.change_index(lambda value: value["entries"][0].update(sheet="../outside.png"))
        confirmation = json.loads(self.fixture.confirmation.read_text())
        confirmation.update(review_index_sha256=curator.digest(self.fixture.index),
                            reviewed_sheets={"../outside.png": "unused"})
        self.fixture.confirmation.write_text(json.dumps(confirmation))
        with self.assertRaisesRegex(ValueError, "sheet changed"):
            self.fixture.curate()
        self.fixture.output = self.fixture.directory/"other"/"output.json"
        with self.assertRaisesRegex(ValueError, "relative JPEG"):
            self.fixture.curate()

    def test_out_of_range_frame_index_fails_closed(self):
        self.fixture.change_index(lambda value: value["entries"][0].update(record_index=3))
        self.fixture.approve()
        with self.assertRaises((ValueError, IndexError, KeyError)):
            self.fixture.curate()
        self.assertFalse(self.fixture.output.exists())

    def test_unknown_native_head_must_not_gain_approved_identity(self):
        self.fixture.change_index(lambda value: value["entries"][0].update(native_id="C1256W999999"))
        self.fixture.approve()
        with self.assertRaises(ValueError):
            self.fixture.curate()
        self.assertFalse(self.fixture.output.exists())

    def test_negative_or_outside_ROI_must_not_gain_approval(self):
        self.fixture.change_index(lambda value: value["entries"][0].update(xyxy=[-2, 10, 500, 22]))
        self.fixture.approve()
        with self.assertRaises(ValueError):
            self.fixture.curate()
        self.assertFalse(self.fixture.output.exists())

    def test_nonfinite_ROI_must_not_gain_approval(self):
        self.fixture.change_index(lambda value: value["entries"][0].update(xyxy=[10, 10, float("nan"), 22]))
        self.fixture.approve()
        with self.assertRaises(ValueError):
            self.fixture.curate()
        self.assertFalse(self.fixture.output.exists())

    def test_explicit_model_use_cannot_be_rewritten_as_prediction_free(self):
        self.fixture.change_index(lambda value: value.update(model_predictions_used=True))
        self.fixture.approve()
        try:
            result = self.fixture.curate()
        except ValueError:
            self.assertFalse(self.fixture.output.exists())
            return  # Rejecting this provenance is also a valid fail-closed fix.
        self.assertTrue(result["CNN_or_detector_predictions_used"])

    def test_known_other_mission_head_is_not_approved_for_this_group(self):
        self.fixture.change_index(lambda value: value["entries"][0].update(native_id="C1256W000077"))
        self.fixture.approve()
        with self.assertRaisesRegex(ValueError, "this mission group"):
            self.fixture.curate()

    def test_negative_boolean_and_noninteger_frame_indices_rejected(self):
        for value in (-1, False, True, 0.0, "0"):
            self.fixture.change_index(lambda review: review["entries"][0].update(record_index=value))
            self.fixture.approve()
            with self.assertRaisesRegex(ValueError, "Invalid frame index"):
                self.fixture.curate()
            self.assertFalse(self.fixture.output.exists())

    def test_absolute_source_JPEG_path_rejected_even_when_hash_matches(self):
        self.fixture.document["records"][0]["image_path"] = str(self.fixture.directory/"frame_00.jpg")
        self.fixture.source.write_text(json.dumps(self.fixture.document))
        self.fixture.change_index(lambda review: review.update(source_sha256=curator.digest(self.fixture.source)))
        self.fixture.approve()
        with self.assertRaisesRegex(ValueError, "Unsafe source JPEG"):
            self.fixture.curate()


if __name__ == "__main__":
    unittest.main()
