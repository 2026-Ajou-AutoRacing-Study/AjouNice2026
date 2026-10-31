#!/usr/bin/env python3
"""Offline pure/synthetic contract tests, never ROS/RPC/model inference."""
import copy
from datetime import datetime, timezone
import json
from pathlib import Path
import tempfile
import types
import sys
import io
from contextlib import redirect_stdout
from unittest.mock import patch
import unittest

import numpy as np

import evaluate_traffic_light_pipeline as evaluator

T = 1792859969
HEAD = "C1256W000016"
FAR = "C1256W000018"


def readback(identifier=HEAD, code=48, when=.2):
    return {"protocol": "modern", "requested_id": identifier, "readback_success": True,
            "received_at_utc": datetime.fromtimestamp(T+when, timezone.utc).isoformat(),
            "observations": [{"tl_id": identifier, "tl_color": code, "identity_matches_request": True,
                              "color_field_present": True, "color_usable_for_gt": True}]}


def row(identifier=HEAD, code=48):
    return {"image_path": "synthetic.jpg", "image_sha256": "a"*64,
            "group_regulatory_id": 201290, "header_stamp": {"secs": T, "nsecs": 500000000},
            "native_before": [readback(identifier, code, .2)],
            "native_after": [readback(identifier, code, .8)],
            "ego": {"position_xyz_m": [0, 0, 0], "heading_degrees": 1.347}}


def annotation(identifier=HEAD, bbox=(600., 250., 680., 270.)):
    return {"native_id": identifier, "xyxy": list(bbox), "class_id": 0,
            "box_qc_accepted": True, "manual_direct_visual_qc_accepted": True, "proposal_only": False}


def sequence_row(index, seconds_offset=0, code=48):
    source = row(code=code)
    source.update(_input_document="/tmp/synthetic_sequence.json", _input_row_index=index,
                  group_id="synthetic_pose", environment="synthetic_environment")
    source["header_stamp"] = {"secs": T+seconds_offset, "nsecs": 500000000}
    source["native_before"] = [readback(code=code, when=seconds_offset+.2)]
    source["native_after"] = [readback(code=code, when=seconds_offset+.8)]
    return source


class PipelineEvaluationTest(unittest.TestCase):
    def test_sequence_mode_preserves_same_JPEG_at_different_exact_times_default_dedups(self):
        sources = [sequence_row(0), sequence_row(1, 1)]
        original = copy.deepcopy(sources)
        self.assertEqual(len(evaluator.evaluation_rows(sources)), 1)
        preserved = evaluator.evaluation_rows(sources, preserve_sequence_frames=True)
        self.assertEqual(len(preserved), 2)
        keys = [evaluator.sequence_frame_key(source) for source in preserved]
        self.assertNotEqual(keys[0], keys[1])
        self.assertEqual(keys[1]["source_stamp_ns"], (T+1)*10**9+500000000)
        self.assertEqual(keys[1]["input_row_index"], 1)
        self.assertEqual(sources, original)

    def test_sequence_duplicate_stamps_regression_and_missing_metadata_rejected(self):
        first, duplicate = sequence_row(0), sequence_row(1)
        duplicate["image_sha256"] = "b"*64
        with self.assertRaisesRegex(ValueError, "repeated"):
            evaluator.evaluation_rows([first, duplicate], True)
        with self.assertRaisesRegex(ValueError, "regress"):
            evaluator.evaluation_rows([sequence_row(0, 2), sequence_row(1, 1)], True)
        for field in ("_input_document", "_input_row_index", "group_id", "environment", "header_stamp"):
            source = sequence_row(0); source.pop(field)
            with self.assertRaises(ValueError): evaluator.evaluation_rows([source], True)

    def test_sequence_native_references_stay_in_own_frame_not_global_SHA_group(self):
        first, second = sequence_row(0, 0, 1), sequence_row(1, 1, 48)
        legacy = evaluator.index_rows([first, second])
        self.assertFalse(evaluator.head_reference(legacy[first["image_sha256"]], HEAD)["known"])
        for source, expected in ((first, [1, 0, 0, 0]), (second, [0, 0, 1, 1])):
            evidence = evaluator.frame_native_evidence(source, legacy, True)
            self.assertEqual(evaluator.head_reference(evidence, HEAD)["bits"], expected)

    def test_external_native_requires_exact_integer_nanosecond_and_same_SHA(self):
        source = sequence_row(0)
        source.pop("native_before"); source.pop("native_after")
        exact = sequence_row(0, code=1)
        adjacent = copy.deepcopy(exact); adjacent["header_stamp"]["nsecs"] += 1
        wrong_SHA = copy.deepcopy(exact); wrong_SHA["image_sha256"] = "b"*64
        for external in (adjacent, wrong_SHA):
            selected = evaluator.frame_native_evidence(source, evaluator.index_rows([external]), True)
            self.assertEqual(len(selected), 1)
            self.assertFalse(evaluator.head_reference(selected, HEAD)["known"])
        selected = evaluator.frame_native_evidence(source, evaluator.index_rows([exact]), True)
        self.assertEqual(evaluator.head_reference(selected, HEAD)["bits"], [1, 0, 0, 0])

    def test_partial_identity_scope_is_explicit_and_never_complete_detector_GT(self):
        approved = {"annotation_complete": False, "identity_annotations_reviewed": True,
                    "annotation_scope": evaluator.IDENTITY_ONLY_SCOPE, "boxes": [annotation()]}
        contract = evaluator.identity_annotation_contract([approved])
        self.assertTrue(contract["reviewed_for_identity"])
        self.assertFalse(contract["full_frame_detector_training_GT"])
        self.assertEqual(contract["scope"], evaluator.IDENTITY_ONLY_SCOPE)
        for key, value in (("identity_annotations_reviewed", False), ("identity_annotations_reviewed", "true"),
                           ("annotation_scope", "geometric_proposals"), ("annotation_complete", None)):
            rejected = dict(approved, **{key: value})
            self.assertFalse(evaluator.identity_annotation_contract([rejected])["reviewed_for_identity"])
            self.assertFalse(evaluator.identity_annotation_contract([approved, rejected])["reviewed_for_identity"])
        self.assertFalse(evaluator.identity_annotation_contract([{"annotation_complete": False}])["reviewed_for_identity"])

    def test_native_codes_are_explicit_multilabel_not_ordinal_class(self):
        self.assertEqual(evaluator.native_bits(1), (1, 0, 0, 0))
        self.assertEqual(evaluator.native_bits(5), (1, 1, 0, 0))
        self.assertEqual(evaluator.native_bits(33), (1, 0, 0, 1))
        self.assertEqual(evaluator.native_bits(48), (0, 0, 1, 1))
        for code in (0, -1, 64, 49, True, "48"):
            self.assertIsNone(evaluator.native_bits(code))

    def test_exact_head_before_after_and_actual_values_not_requests(self):
        source = row(code=33)
        source.update(requested_state=16, labels={"red": 0, "yellow": 0, "green": 1, "left_arrow": 0})
        reference = evaluator.native_head_reference(source, HEAD)
        self.assertTrue(reference["known"])
        self.assertEqual(reference["bits"], [1, 0, 0, 1])
        self.assertFalse(reference["pixel_visibility_verified"])

    def test_single_forced_C16_never_propagates_GT_to_C18(self):
        source = row()
        self.assertFalse(evaluator.native_head_reference(source, FAR)["known"])
        source["native_before"].append(readback(FAR, 1, .25))
        source["native_after"].append(readback(FAR, 1, .85))
        self.assertEqual(evaluator.native_head_reference(source, FAR)["bits"], [1, 0, 0, 0])
        self.assertEqual(evaluator.native_head_reference(source, HEAD)["bits"], [0, 0, 1, 1])

    def test_wrong_identity_failed_read_unknown_state_and_phase_change_rejected(self):
        sources = []
        for field, value in (("tl_id", FAR), ("identity_matches_request", False),
                             ("color_field_present", False), ("color_usable_for_gt", False), ("tl_color", 64)):
            source = row()
            source["native_before"][0]["observations"][0][field] = value
            sources.append(source)
        source = row(); source["native_before"][0]["readback_success"] = False; sources.append(source)
        source = row(); source["native_after"][0] = readback(code=1, when=.8); sources.append(source)
        for source in sources:
            self.assertFalse(evaluator.native_head_reference(source, HEAD)["known"])

    def test_duplicate_query_ambiguous_and_timestamps_must_bracket_actual_JPEG(self):
        source = row(); source["native_before"].append(readback())
        self.assertFalse(evaluator.native_head_reference(source, HEAD)["known"])
        for before, after in ((.6, .8), (.2, .4), (.2, 8.)):
            source = row()
            source["native_before"] = [readback(when=before)]
            source["native_after"] = [readback(when=after)]
            self.assertFalse(evaluator.native_head_reference(source, HEAD)["known"])
        source = row(); source["header_stamp"] = {"secs": 0, "nsecs": 0}
        self.assertFalse(evaluator.native_head_reference(source, HEAD)["known"])

    def test_old_nested_evidence_requires_actual_bracket_claim(self):
        source = row(code=1)
        old = {"provenance": {"source_header_stamp": source["header_stamp"], "native_gt_evidence": {
            "jpeg_receipt_bracketed": True, "before": {"readback": source["native_before"][0]},
            "after": {"readback": source["native_after"][0]}}}}
        self.assertTrue(evaluator.native_head_reference(old, HEAD)["known"])
        old["provenance"]["native_gt_evidence"]["jpeg_receipt_bracketed"] = False
        self.assertFalse(evaluator.native_head_reference(old, HEAD)["known"])

    def test_conflicting_available_evidence_does_not_choose_convenient_GT(self):
        first, second = row(code=1), row(code=48)
        self.assertFalse(evaluator.head_reference([first, second], HEAD)["known"])
        changed = row(code=48); changed["native_after"] = [readback(code=1, when=.8)]
        self.assertFalse(evaluator.head_reference([first, changed], HEAD)["known"])

    def test_manual_identity_requires_approved_nonproposal_box_not_map_estimate(self):
        source = annotation()
        identity = evaluator.manual_identity([600, 250, 680, 270], [source])
        self.assertTrue(identity["known"])
        self.assertEqual(identity["native_id"], HEAD)
        for key, value in (("box_qc_accepted", False), ("manual_direct_visual_qc_accepted", False), ("proposal_only", True)):
            other = dict(source, **{key: value})
            self.assertFalse(evaluator.manual_identity([600, 250, 680, 270], [other])["known"])
        self.assertFalse(evaluator.manual_identity([600, 250, 680, 270], [source, annotation(FAR)])["known"])
        self.assertFalse(evaluator.manual_identity([0, 0, 10, 10], [source])["known"])

    def test_route_identity_can_join_explicit_native_evidence_but_not_guess_name(self):
        geometry = evaluator.load_geometry(evaluator.DEFAULT_GEOMETRY)
        source = {"group_id": "C20_center"}
        with self.assertRaises(ValueError):
            evaluator.frame_group(source, geometry)
        self.assertEqual(evaluator.frame_group(source, geometry, [{"requested_native_id": HEAD}]), 201290)
        with self.assertRaises(ValueError):
            evaluator.frame_group({"group_regulatory_id": 201397}, geometry)

    def test_current_policy_C_can_mask_arrow_error_and_D_Y_RY_are_separate(self):
        policy = evaluator.SignalPhasePolicy.from_yaml(evaluator.DEFAULT_POLICY)
        reference = {"known": True, "bits": [0, 0, 1, 1]}
        raw = [("green", "circle", None)]
        diagnostic = evaluator.policy_diagnostic(policy, raw, True, reference, 201290)
        self.assertNotEqual(evaluator.bits_from_tuples(raw), tuple(reference["bits"]))
        self.assertTrue(diagnostic["corrected_exact"])
        for bits, expected in (([0, 1, 0, 0], "stop_phase_proxy"),
                               ([1, 1, 0, 0], "go_phase_proxy"),
                               ([0, 0, 1, 0], "stop_phase_proxy"),
                               ([1, 0, 0, 1], "go_phase_proxy")):
            current = evaluator.policy_diagnostic(policy, evaluator.tuples_from_bits(bits), True,
                {"known": True, "bits": bits}, 201326)
            self.assertEqual(current["permission_proxy"], expected)
        self.assertEqual(evaluator.permission_proxy([("yellow", "circle", None)], 201351), "go_phase_proxy")
        self.assertEqual(evaluator.permission_proxy([], 201397), "non_mission")

    def test_unknown_raw_is_not_reported_as_green_even_if_policy_fills_red(self):
        policy = evaluator.SignalPhasePolicy.from_yaml(evaluator.DEFAULT_POLICY)
        diagnostic = evaluator.policy_diagnostic(policy, [], False, {"known": True, "bits": [1, 0, 0, 0]}, 201351)
        self.assertTrue(diagnostic["corrected_exact"])
        self.assertEqual(diagnostic["permission_proxy"], "stop_phase_proxy")
        self.assertFalse(diagnostic["temporal_smoother_evaluated"])

    def test_wrong_target_color_success_and_unknown_are_not_overall_success(self):
        def selected(target, known=True):
            return {"selected_bbox_xyxy": [1, 1, 10, 10], "identity": {"known": target is not None},
                "target_identity_correct": target, "reference": {"known": True, "bits": [0, 0, 1, 1]},
                "classifiers": {"cnn": {"known": known, "bits": [0, 0, 1, 1] if known else None,
                    "policy": {"corrected_exact": True, "permission_proxy_match": True}}}}
        records = [{mode: selected(target, known) for mode in ("legacy_center", "map_association")}
                   for target, known in ((False, True), (None, True), (True, False), (True, True))]
        summary = evaluator.summarize(records)["map_association"]
        self.assertEqual(summary["wrong_mission"], 1)
        self.assertEqual(summary["identity_unknown"], 1)
        cnn = summary["classifiers"]["cnn"]
        self.assertEqual(cnn["native_reference_eligible_correct_identity"], 2)
        self.assertEqual(cnn["unknown"], 1)
        self.assertEqual(cnn["end_to_end_raw_exact_successes"], 1)
        self.assertEqual(cnn["end_to_end_frame_denominator"], 4)
        self.assertEqual(cnn["arrow_recall_including_unknown_as_miss"], .5)

    def test_synthetic_selected_crop_only_CNN_policy_stage_and_source_immutability(self):
        source = row()
        original = copy.deepcopy(source)
        camera = evaluator.load_camera(evaluator.DEFAULT_SENSOR)
        geometry = {"heads": [{"id": HEAD, "type": "car", "group_id": 201290,
            "point": [20, 0, 3.5], "heading": 180., "width": 1.42, "height": .355}]}
        rois, _ = evaluator.project_heads(geometry, camera, evaluator.development_camera_transform(source["ego"], camera), 201290)
        bbox = rois[0]["bbox_xyxy"]
        boxes = [evaluator.make_box(bbox, .9, 1280, 720)]
        from traffic_light_cnn_classifier import observation_from_probabilities
        class FakeCnn:
            def classify_bgr(self, crop):
                self.shape = crop.shape
                return observation_from_probabilities((.01, .01, .99, .99))
        cnn = FakeCnn()
        image = np.zeros((720, 1280, 3), dtype=np.uint8)
        report = evaluator.evaluate_frame(source, image, boxes, geometry, camera,
            evaluator.SignalPhasePolicy.from_yaml(evaluator.DEFAULT_POLICY), cnn, [source],
            [{"annotation_complete": True, "boxes": [annotation(bbox=bbox)]}])
        self.assertTrue(report["map_association"]["target_identity_correct"])
        self.assertEqual(report["map_association"]["classifiers"]["cnn"]["bits"], [0, 0, 1, 1])
        self.assertGreater(cnn.shape[0], int(bbox[3])-int(bbox[1]))
        self.assertEqual(source, original)

    def test_old_archived_C16_actual_GT_is_valid_but_C18_remains_unknown(self):
        path = evaluator.ROOT/"artifacts/evaluation/traffic_light_robustness_261025/crop_probe_dataset_v1.json"
        if not path.exists():
            self.skipTest("optional frozen offline evidence not installed")
        source = json.loads(path.read_text())["records"][0]
        self.assertTrue(evaluator.native_head_reference(source, HEAD)["known"])
        self.assertFalse(evaluator.native_head_reference(source, FAR)["known"])

    def test_output_restricted_to_new_repository_evaluation_JSON(self):
        with self.assertRaises(ValueError): evaluator.validate_output("/tmp/output.json")
        with self.assertRaises(ValueError): evaluator.validate_output(evaluator.ROOT/"source.json")
        with self.assertRaises(ValueError): evaluator.validate_output(evaluator.ROOT/"artifacts/evaluation/a.png")
        with tempfile.TemporaryDirectory(dir=evaluator.ROOT/"artifacts/evaluation") as directory:
            path = Path(directory)/"new.json"
            self.assertEqual(evaluator.validate_output(path), path)
            path.write_text("{}")
            with self.assertRaises(ValueError): evaluator.validate_output(path)

    def test_loader_never_includes_rejected_retry_frames_or_failed_campaign(self):
        self.assertEqual(evaluator._rows({"frames": [dict(row(), accepted_for_sample_quota=False)]}), [])
        accepted = evaluator._rows({"requested_native_id": HEAD,
            "frames": [dict(row(), accepted_for_sample_quota=True)]})
        self.assertEqual(len(accepted), 1)
        self.assertEqual(accepted[0]["requested_native_id"], HEAD)
        with self.assertRaises(ValueError): evaluator._rows({"status": "failed", "records": [row()]})

    def test_loader_keeps_original_row_indices_and_explicit_per_frame_review_refusal(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            (directory/"synthetic.jpg").write_bytes(b"test_fixture_not_decoded")
            accepted = row()
            rejected = dict(row(), accepted_for_sample_quota=False)
            document = {"frames": [rejected, dict(accepted, accepted_for_sample_quota=True)],
                        "annotation_complete": False, "identity_annotations_reviewed": True,
                        "annotation_scope": evaluator.IDENTITY_ONLY_SCOPE}
            path = directory/"input.json"; path.write_text(json.dumps(document))
            loaded = evaluator.load_rows(path)
            self.assertEqual(loaded[0]["_input_row_index"], 1)
            self.assertFalse(loaded[0]["annotation_complete"])
            self.assertTrue(loaded[0]["identity_annotations_reviewed"])
            document["frames"][1]["identity_annotations_reviewed"] = False
            path.write_text(json.dumps(document))
            loaded = evaluator.load_rows(path)
            self.assertFalse(evaluator.identity_annotation_contract(loaded)["reviewed_for_identity"])

    def test_full_CLI_with_fake_detector_serializes_complete_new_artifact(self):
        # Test-only synthetic fixture, no archived images/model/process touched.
        import cv2
        with tempfile.TemporaryDirectory(dir=evaluator.ROOT/"artifacts/evaluation") as directory:
            directory = Path(directory)
            image = directory/"synthetic.jpg"
            self.assertTrue(cv2.imwrite(str(image), np.zeros((720, 1280, 3), dtype=np.uint8)))
            source = row(code=1)
            source.update(image_sha256=evaluator.sha256(image), annotation_complete=True,
                          boxes=[annotation()], environment="synthetic", group_id="synthetic_pose")
            dataset = directory/"input.json"
            dataset.write_text(json.dumps({"records": [source]}))
            output = directory/"output.json"
            detection = types.SimpleNamespace(xyxy=np.array([[600., 250., 680., 270.]]),
                                              conf=np.array([.9]), cls=np.array([0]))
            class FakeDetector:
                device = "cpu"
                names = {0: "housing"}
                def __init__(self, _path): pass
                def predict(self, *_args, **_kwargs):
                    return [types.SimpleNamespace(boxes=[detection])]
            module = types.ModuleType("ultralytics"); module.YOLO = FakeDetector
            actual_sha = evaluator.sha256
            def digest(path):
                return evaluator.BASELINE_SHA256 if Path(path).name == "fake.pt" else actual_sha(path)
            with patch.dict(sys.modules, {"ultralytics": module}), patch.object(evaluator, "sha256", digest), redirect_stdout(io.StringIO()):
                status = evaluator.main(["--dataset", str(dataset), "--housing-qc", str(dataset),
                    "--detector-weights", str(directory/"fake.pt"), "--output", str(output), "--limit", "1"])
            self.assertEqual(status, 0)
            report = json.loads(output.read_text())
            self.assertEqual(report["summary"]["legacy_center"]["frames"], 1)
            self.assertFalse(report["summary"]["legacy_center"]["classifiers"]["cnn"]["evaluated"])
            self.assertEqual(report["summary"]["legacy_center"]["classifiers"]["cnn"]["end_to_end_frame_denominator"], 0)
            self.assertIn("synthetic", report["by_environment"])

    def test_sequence_CLI_same_JPEG_two_states_and_partial_identity_QC_without_models(self):
        import cv2
        with tempfile.TemporaryDirectory(dir=evaluator.ROOT/"artifacts/evaluation") as directory:
            directory = Path(directory)
            image = directory/"synthetic.jpg"
            self.assertTrue(cv2.imwrite(str(image), np.zeros((720, 1280, 3), dtype=np.uint8)))
            sources = [sequence_row(0, 0, 1), sequence_row(1, 1, 16)]
            for source in sources:
                source.update(image_sha256=evaluator.sha256(image), annotation_complete=False,
                    identity_annotations_reviewed=True, annotation_scope=evaluator.IDENTITY_ONLY_SCOPE,
                    boxes=[annotation()])
            dataset = directory/"input.json"; dataset.write_text(json.dumps({"records": sources}))
            output = directory/"output.json"
            detection = types.SimpleNamespace(xyxy=np.array([[600., 250., 680., 270.]]), conf=np.array([.9]), cls=np.array([0]))
            class FakeDetector:
                device, names = "cpu", {0: "housing"}
                def __init__(self, _path): pass
                def predict(self, *_args, **_kwargs): return [types.SimpleNamespace(boxes=[detection])]
            module = types.ModuleType("ultralytics"); module.YOLO = FakeDetector
            actual_sha = evaluator.sha256
            def digest(path):
                return evaluator.BASELINE_SHA256 if Path(path).name == "fake.pt" else actual_sha(path)
            with patch.dict(sys.modules, {"ultralytics": module}), patch.object(evaluator, "sha256", digest), redirect_stdout(io.StringIO()):
                status = evaluator.main(["--dataset", str(dataset), "--housing-qc", str(dataset),
                    "--detector-weights", str(directory/"fake.pt"), "--preserve-sequence-frames", "--output", str(output)])
            self.assertEqual(status, 0)
            report = json.loads(output.read_text())
            self.assertEqual(len(report["records"]), 2)
            self.assertTrue(report["preserve_sequence_frames"])
            self.assertFalse(report["smoother_or_temporal_delay_evaluated"])
            for index, expected in enumerate(([1, 0, 0, 0], [0, 0, 1, 0])):
                result = report["records"][index]
                self.assertEqual(result["sequence_frame_key"]["input_row_index"], index)
                self.assertEqual(result["legacy_center"]["reference"]["bits"], expected)
                self.assertTrue(result["legacy_center"]["target_identity_correct"])
                self.assertFalse(result["identity_annotation_contract"]["full_frame_detector_training_GT"])
            self.assertEqual(report["summary"]["shared_detector"]["named_identity_only_annotation_frames"], 2)


if __name__ == "__main__":
    unittest.main()
