#!/usr/bin/env python3
"""Pure/synthetic renderer tests; no real test data or model output is opened."""
import copy
import io
import json
from collections import Counter
from contextlib import redirect_stdout, redirect_stderr
from pathlib import Path
import tempfile
import unittest

from PIL import Image, ImageDraw

import render_traffic_light_pipeline_comparison as renderer

HEAD = "C1256W000016"


def side(bits=(1, 0, 0, 0), reference=(1, 0, 0, 0), target=True, known=True, margin=0):
    observation = {"known": known, "bits": list(bits) if bits is not None else None,
        "crop_xyxy": [10, 10, 50, 24] if not margin else [0, 0, 62, 36],
        "crop_margin_px": margin, "policy": {"corrected_tuples": [["red", "circle", None]]}}
    return {"selected_bbox_xyxy": [10., 10., 50., 24.], "target_identity_correct": target,
        "identity": {"known": target is not None, "native_id": HEAD},
        "selection": {"head_id": HEAD}, "reference": {"known": True, "native_id": HEAD,
            "bits": list(reference), "state": 1, "pixel_visibility_verified": False},
        "classifiers": {"hsv": copy.deepcopy(observation), "cnn": copy.deepcopy(observation)}}


def pair(index, group="A", environment="Sunny13", old="native_agreement", new="native_agreement"):
    return {"manifest_index": index, "source_stamp_ns": 1000000000+index,
        "source": {"image_sha256": "%064x" % index}, "stratum": (group, environment),
        "baseline_status": old, "candidate_status": new}


class RendererContractsTest(unittest.TestCase):
    def test_wrong_identity_cannot_earn_color_agreement_credit(self):
        self.assertEqual(renderer.side_status(side(target=False), "cnn"), "wrong_identity")
        unknown = side(target=None)
        self.assertEqual(renderer.side_status(unknown, "cnn"), "identity_unknown")
        missing = side(); missing["identity"].pop("native_id")
        self.assertEqual(renderer.side_status(missing, "cnn"), "identity_unknown")

    def test_unknown_confident_no_lamps_and_native_unknown_are_distinct(self):
        self.assertEqual(renderer.side_status(side(bits=None, known=False), "cnn"), "classifier_unknown")
        self.assertEqual(renderer.side_status(side(bits=(0, 0, 0, 0)), "cnn"), "raw_mismatch")
        value = side(); value["reference"]["known"] = False
        self.assertEqual(renderer.side_status(value, "cnn"), "native_unknown")
        value = side(); value["reference"]["native_id"] = "C1256W000018"
        self.assertEqual(renderer.side_status(value, "cnn"), "reference_identity_mismatch")
        value = side(); value["selected_bbox_xyxy"] = None
        self.assertEqual(renderer.side_status(value, "cnn"), "no_selection")

    def test_balanced_selection_is_deterministic_includes_failure_and_improvement(self):
        pairs = []
        for group in "ABCD":
            for environment in ("Sunny13", "Foggy15"):
                for offset in range(5):
                    pairs.append(pair(len(pairs), group, environment,
                        old="raw_mismatch" if offset == 0 else "native_agreement"))
        pairs[-1]["candidate_status"] = "wrong_identity"
        selected = renderer.select_examples(pairs)
        reversed_selected = renderer.select_examples(list(reversed(pairs)))
        self.assertEqual([row["manifest_index"] for row, _ in selected],
                         [row["manifest_index"] for row, _ in reversed_selected])
        self.assertEqual(len(selected), 12)
        counts = Counter(row["stratum"] for row, _ in selected)
        self.assertEqual(len(counts), 8)
        self.assertLessEqual(max(counts.values())-min(counts.values()), 1)
        self.assertTrue(any(row["candidate_status"] in renderer.FAILURES for row, _ in selected))
        self.assertTrue(any(row["baseline_status"] == "raw_mismatch" and row["candidate_status"] == "native_agreement" for row, _ in selected))
        for maximum in (0, 13, True):
            with self.assertRaises(ValueError): renderer.select_examples(pairs, maximum)

    def test_recorded_crop_only_no_oracle_substitution_and_inbounds_required(self):
        value = side(margin=12)
        self.assertEqual(renderer.crop_box(value, "cnn", (160, 90)), (0, 0, 62, 36))
        value["classifiers"]["cnn"]["crop_xyxy"] = [-1, 0, 62, 36]
        with self.assertRaises(ValueError): renderer.crop_box(value, "cnn", (160, 90))
        value["selected_bbox_xyxy"] = None
        self.assertIsNone(renderer.crop_box(value, "cnn", (160, 90)))

    def test_prediction_unknown_is_preserved_without_calling_missing_GT_a_failure(self):
        pairs = [pair(0, "A", new="native_unknown"), pair(1, "B", new="classifier_unknown")]
        selected = renderer.select_examples(pairs, 1)
        self.assertEqual(selected[0][0]["manifest_index"], 1)
        self.assertIn("UNKNOWN", selected[0][1])
        self.assertNotIn("native_unknown", renderer.UNSUCCESSFUL)

    def fixture_documents(self, directory):
        image = directory/"source.png"
        canvas = Image.new("RGB", (160, 90), "#7890a0")
        draw = ImageDraw.Draw(canvas); draw.rectangle((10, 10, 50, 24), fill="black")
        draw.ellipse((14, 13, 22, 21), fill="red")
        canvas.save(image)
        manifest = {"status": "complete", "errors": [], "records": []}
        for index in range(2):
            manifest["records"].append({"image_path": image.name, "image_sha256": renderer.digest(image),
                "group_id": "synthetic_C", "environment": "synthetic", "header_stamp": {"secs": 10+index, "nsecs": 123}})
        manifest_path = directory/"manifest.json"; manifest_path.write_text(json.dumps(manifest))
        records = []
        for index, source in enumerate(manifest["records"]):
            records.append({"image_sha256": source["image_sha256"], "pose_group": source["group_id"],
                "environment": source["environment"], "source_stamp_ns": renderer.stamp_ns(source),
                "sequence_frame_key": {"input_document": "/another-machine/manifest.json",
                    "input_row_index": index, "source_stamp_ns": renderer.stamp_ns(source)},
                "legacy_center": side(bits=(1, 0, 1, 0)), "map_association": side(margin=12)})
        records[1]["map_association"]["target_identity_correct"] = False
        pipeline = {"schema_version": 1, "classes_in_bit_order": renderer.CLASSES,
            "sources": {"dataset_sha256": renderer.digest(manifest_path)},
            "parameters": {"imgsz": 640}, "records": records}
        return image, manifest_path, manifest, pipeline

    def test_exact_sequence_alignment_handles_repeated_SHA_and_relocation(self):
        with tempfile.TemporaryDirectory() as directory:
            _, path, manifest, pipeline = self.fixture_documents(Path(directory))
            self.assertEqual(len(renderer.pair_results(pipeline, pipeline, manifest, path)), 2)
            broken = copy.deepcopy(pipeline)
            for row in broken["records"]:
                row.pop("sequence_frame_key"); row.pop("source_stamp_ns")
            with self.assertRaisesRegex(ValueError, "모호"):
                renderer.pair_results(broken, pipeline, manifest, path)
            broken = copy.deepcopy(pipeline); broken["records"].pop()
            with self.assertRaisesRegex(ValueError, "frame 집합"):
                renderer.pair_results(broken, pipeline, manifest, path)
            broken = copy.deepcopy(pipeline); broken["classes_in_bit_order"] = list(reversed(renderer.CLASSES))
            with self.assertRaisesRegex(ValueError, "4bit 순서"):
                renderer.pair_results(broken, pipeline, manifest, path)

    def test_headless_synthetic_render_preserves_originals_and_labels_failures(self):
        try: renderer.load_font()
        except ValueError: self.skipTest("Korean font unavailable in this test environment")
        with tempfile.TemporaryDirectory() as inputs, tempfile.TemporaryDirectory(dir=renderer.ROOT/"artifacts/evaluation") as outputs:
            image, manifest_path, _, pipeline = self.fixture_documents(Path(inputs))
            baseline = Path(inputs)/"baseline.json"; candidate = Path(inputs)/"candidate.json"
            baseline.write_text(json.dumps(pipeline)); pipeline["parameters"]["imgsz"] = 960
            candidate.write_text(json.dumps(pipeline))
            tracked = {path: path.read_bytes() for path in (image, manifest_path, baseline, candidate)}
            output = Path(outputs)/"comparison"
            with redirect_stdout(io.StringIO()):
                status = renderer.main(["--baseline", str(baseline), "--candidate", str(candidate),
                    "--manifest", str(manifest_path), "--output", str(output)])
            self.assertEqual(status, 0)
            index = json.loads((output/"index.json").read_text())
            self.assertEqual(index["selected_example_count"], 2)
            self.assertTrue(index["no_model_inference_ROS_X11_or_image_generation"])
            self.assertTrue(index["not_manual_lamp_GT_or_temporal_latency_certification"])
            self.assertEqual(index["detector_imgsz_baseline_candidate"], [640, 960])
            self.assertEqual({path: path.read_bytes() for path in tracked}, tracked)
            self.assertIn("wrong_identity", [entry["candidate"]["status"] for entry in index["examples"]])
            for entry in index["examples"]:
                self.assertEqual(entry["candidate"]["recorded_margin_px"], 12)
                self.assertEqual(entry["candidate"]["crop_xyxy"], [0, 0, 62, 36])
            for name, sha in index["derived_sheet_SHA256"].items():
                self.assertEqual(renderer.digest(output/name), sha)

    def test_existing_or_outside_output_rejected(self):
        with self.assertRaises(ValueError): renderer.validate_output("/tmp/not_allowed_renderer_output")
        with tempfile.TemporaryDirectory(dir=renderer.ROOT/"artifacts/evaluation") as directory:
            with self.assertRaises(ValueError): renderer.validate_output(directory)


if __name__ == "__main__":
    unittest.main()
