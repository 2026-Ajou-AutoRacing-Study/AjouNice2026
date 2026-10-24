#!/usr/bin/env python3
"""Temporary structural fixtures only; no actual capture datasets are created."""

import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from build_traffic_light_crop_probe_dataset import (
    build_dataset, build_documents, check_environment, labels_from_observed_state, parse_group,
)
from test_sample_traffic_light_bag_frames import jpeg_fixture


class DatasetBuilderTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.mgeo = self.root / "traffic_light_set.json"
        self.native = "C1256W000016"
        self.mgeo.write_text(json.dumps([{"idx": self.native, "point": [-64.1, -25.36, 34.25341],
                                         "heading": -89.678, "width": 1.42, "height": .355, "z_offset": 4.5}]))
        self.matrix = self.root / "matrix"
        self.cell = self.matrix / "sunny13"
        (self.cell / "images").mkdir(parents=True)
        self.payload = jpeg_fixture()
        (self.cell / "images/frame.jpg").write_bytes(self.payload)
        self.frame = {"image_path": "images/frame.jpg", "sha256": hashlib.sha256(self.payload).hexdigest(),
                      "width": 1280, "height": 720, "requested_native_id": self.native,
                      "requested_state": 16, "native_GT_verified": True,
                      "native_gt": {"observed_native_id": self.native, "observed_state": 33},
                      "ego": {"position_xyz_m": [-60.09483, -45.53425, 28.7261], "heading_degrees": 91.663}}
        self.before = {"status": "READBACK_MATCH", "readback_match": True,
                       "actual": {"weather": "Sunny", "hour": 13}}
        self.after = {"status": "READ_ONLY_GET", "actual": {"weather": "Sunny", "hour": 13}}
        self.write_evidence()
        self.spec = "train:C20_center:{}:{}".format(self.native, self.matrix)

    def write_evidence(self, frames=None, status="complete"):
        (self.matrix / "sunny13_environment_before.json").write_text(json.dumps(self.before))
        (self.matrix / "sunny13_environment_after.json").write_text(json.dumps(self.after))
        frames = [self.frame] if frames is None else frames
        (self.cell / "manifest.json").write_text(json.dumps({"status": status, "requested_native_id": self.native,
                                                            "frame_count": len(frames), "frames": frames}))

    def documents(self):
        return build_documents([self.spec], self.mgeo, self.root)

    def test_record_uses_observed_not_requested_labels_and_original_relative_jpeg(self):
        original = (self.cell / "images/frame.jpg").read_bytes()
        dataset, skipped = self.documents()
        self.assertEqual(len(dataset["records"]), 1)
        row = dataset["records"][0]
        self.assertEqual(row["labels"], {"red": 1, "yellow": 0, "green": 0, "left_arrow": 1})
        self.assertEqual(row["environment"], "Sunny13")
        self.assertEqual(row["crop_xyxy"], [473, 174, 532, 196])
        self.assertEqual(row["image_path"], "matrix/sunny13/images/frame.jpg")
        self.assertEqual(row["image_sha256"], hashlib.sha256(original).hexdigest())
        self.assertTrue(row["native_gt_verified"])
        self.assertFalse(row["provenance"]["requested_matches_observed_state"])
        self.assertTrue(dataset["manual_visibility_and_crop_qc_required"])
        self.assertFalse(dataset["manual_visibility_and_crop_qc_performed"])
        self.assertEqual((self.cell / "images/frame.jpg").read_bytes(), original)
        self.assertEqual(skipped["summary"]["skipped_cell_count"], 5)

    def test_partial_failed_cell_is_skipped_not_entire_build(self):
        self.write_evidence(status="failed")
        dataset, skipped = self.documents()
        self.assertEqual(dataset["records"], [])
        self.assertEqual(dataset["summary"]["whole_cell_frames_skipped"], 1)
        self.assertTrue(any(cell.get("reason") == "cell_manifest_not_complete" for cell in skipped["skipped_cells"]))

    def test_explicit_larger_margin_keeps_old_sources_unchanged(self):
        dataset, _ = build_documents([self.spec], self.mgeo, self.root, margin_px=12)
        self.assertEqual(dataset['records'][0]['crop_xyxy'], [465,166,540,204])
        self.assertEqual(dataset['provenance']['margin_px'], 12)
        self.assertFalse(dataset['manual_visibility_and_crop_qc_performed'])
        self.assertEqual(self.documents()[0]['records'][0]['crop_xyxy'], [473,174,532,196])

    def test_invalid_margins_rejected(self):
        for margin in [-1, float('nan'), float('inf'), 33, True]:
            with self.subTest(margin=margin), self.assertRaises(ValueError):
                build_documents([self.spec], self.mgeo, self.root, margin_px=margin)

    def test_environment_pre_post_and_requested_cell_must_match(self):
        for change in ({"weather": "Foggy", "hour": 13}, {"weather": "Sunny", "hour": 11}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                check_environment(self.before, {"status": "READ_ONLY_GET", "actual": change}, "Sunny", 13)
        self.after["actual"]["weather"] = "Foggy"
        self.write_evidence()
        dataset, _ = self.documents()
        self.assertEqual(dataset["records"], [])
        before = dict(self.before, status="READ_ONLY_GET")
        with self.assertRaises(ValueError):
            check_environment(before, {"status": "READ_ONLY_GET", "actual": {"weather": "Sunny", "hour": 13}}, "Sunny", 13)

    def test_unverified_wrong_id_bad_state_and_retry_frames_are_counted(self):
        mutations = [({"native_GT_verified": False}, "native_gt_not_verified"),
                     ({"accepted_for_sample_quota": False}, "retry_not_accepted_for_quota"),
                     ({"requested_native_id": "wrong"}, "requested_native_id_mismatch"),
                     ({"native_gt": {"observed_native_id": "wrong", "observed_state": 1}}, "observed_native_id_mismatch"),
                     ({"native_gt": {"observed_native_id": self.native, "observed_state": -1}}, "unsupported_native_observed_state")]
        for changes, reason in mutations:
            bad = copy.deepcopy(self.frame)
            bad.update(changes)
            self.write_evidence(frames=[bad, self.frame])
            dataset, skipped = self.documents()
            self.assertEqual(len(dataset["records"]), 1)
            self.assertEqual(skipped["summary"]["skipped_frame_reason_counts"], {reason: 1})

    def test_bad_image_hash_and_projection_are_skipped_with_reasons(self):
        self.frame["sha256"] = "0" * 64
        self.write_evidence()
        _, skipped = self.documents()
        self.assertEqual(skipped["summary"]["skipped_frame_reason_counts"], {"image_sha256_mismatch": 1})
        self.frame["sha256"] = hashlib.sha256(self.payload).hexdigest()
        self.frame["ego"]["heading_degrees"] += 180
        self.write_evidence()
        _, skipped = self.documents()
        self.assertEqual(skipped["summary"]["skipped_frame_reason_counts"], {"invalid_projected_roi": 1})

    def test_common_artifact_root_rule_and_unsafe_image_paths(self):
        with self.assertRaises(ValueError):
            build_documents([self.spec], self.mgeo, self.cell)
        self.frame["image_path"] = "../elsewhere.jpg"
        self.write_evidence()
        dataset, skipped = self.documents()
        self.assertEqual(dataset["records"], [])
        self.assertEqual(skipped["summary"]["skipped_frame_reason_counts"], {"unsafe_or_non_jpeg_image_path": 1})

    def test_native_state_mapping_is_exact_and_never_accepts_unknown_or_bool(self):
        expected = {1: [1, 0, 0, 0], 4: [0, 1, 0, 0], 16: [0, 0, 1, 0], 32: [0, 0, 0, 1],
                    33: [1, 0, 0, 1], 48: [0, 0, 1, 1], 5: [1, 1, 0, 0]}
        for state, bits in expected.items():
            self.assertEqual(list(labels_from_observed_state(state).values()), bits)
        for bad in (-1, 0, 20, True, 1., "1", None):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                labels_from_observed_state(bad)

    def test_group_parser_and_duplicate_group_are_rejected(self):
        self.assertEqual(parse_group(self.spec)["group_id"], "C20_center")
        for bad in ("auto:a:b:c", "train::id:dir", "train:a:id", "train:bad group:id:dir"):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                parse_group(bad)
        with self.assertRaises(ValueError):
            build_documents([self.spec, self.spec], self.mgeo, self.root)

    def test_cross_split_identical_jpeg_bytes_are_fatal_before_output(self):
        other = self.root / "other_matrix"
        other_cell = other / "sunny13"
        (other_cell / "images").mkdir(parents=True)
        (other_cell / "images/frame.jpg").write_bytes(self.payload)
        for suffix in ("environment_before.json", "environment_after.json"):
            (other / ("sunny13_" + suffix)).write_bytes((self.matrix / ("sunny13_" + suffix)).read_bytes())
        (other_cell / "manifest.json").write_bytes((self.cell / "manifest.json").read_bytes())
        spec = "test:C40:{}:{}".format(self.native, other)
        output = self.root / "dataset.json"
        with self.assertRaisesRegex(ValueError, "cross train/val/test"):
            build_dataset([self.spec, spec], self.mgeo, output)
        self.assertFalse(output.exists())

    def test_dataset_and_skipped_sidecar_are_both_fresh_only(self):
        output = self.root / "dataset.json"
        dataset, sidecar = build_dataset([self.spec], self.mgeo, output)
        self.assertEqual(json.loads(output.read_text())["records"], dataset["records"])
        self.assertEqual(sidecar.name, "dataset.skipped.json")
        self.assertTrue(sidecar.exists())
        with self.assertRaises(FileExistsError):
            build_dataset([self.spec], self.mgeo, output)
        output.unlink()
        with self.assertRaises(FileExistsError):
            build_dataset([self.spec], self.mgeo, output)

    def test_multiple_groups_produce_exact_training_loader_contract(self):
        from train_traffic_light_crop_probe import load_dataset
        specs = [self.spec]
        for number, (split, states) in enumerate((("train", [4, 16]), ("val", [1]), ("test", [48])), 1):
            matrix = self.root / ("matrix_" + str(number))
            cell = matrix / "sunny13"
            (cell / "images").mkdir(parents=True)
            frames = []
            for index, state in enumerate(states):
                frame = copy.deepcopy(self.frame)
                payload = jpeg_fixture(scan=bytes([number, index, 17]))
                relative = "images/frame_{}.jpg".format(index)
                (cell / relative).write_bytes(payload)
                frame.update(image_path=relative, sha256=hashlib.sha256(payload).hexdigest())
                frame["native_gt"]["observed_state"] = state
                frames.append(frame)
            for filename, evidence in (("sunny13_environment_before.json", self.before),
                                       ("sunny13_environment_after.json", self.after)):
                (matrix / filename).write_text(json.dumps(evidence))
            (cell / "manifest.json").write_text(json.dumps({"status": "complete", "requested_native_id": self.native,
                                                           "frame_count": len(frames), "frames": frames}))
            specs.append("{}:pose{}:{}:{}".format(split, number, self.native, matrix))
        output = self.root / "training_contract.json"
        dataset, _ = build_dataset(specs, self.mgeo, output)
        prepared, metadata = load_dataset(output, size_reader=lambda _path: (1280, 720))
        self.assertEqual(len(prepared), 5)
        self.assertEqual(metadata["split_counts"], {"train": 3, "val": 1, "test": 1})
        checks = dataset["summary"]["structural_trainer_requirements_before_manual_QC"]
        self.assertTrue(checks["all_splits_nonempty"])
        self.assertTrue(checks["at_least_two_train_pose_groups"])
        self.assertTrue(checks["each_train_bit_has_positive_and_negative"])

    def test_png_bytes_are_never_accepted_even_if_manifest_hash_matches(self):
        payload = b"\x89PNG\r\n\x1a\nnot_a_JPEG"
        (self.cell / "images/frame.jpg").write_bytes(payload)
        self.frame["sha256"] = hashlib.sha256(payload).hexdigest()
        self.write_evidence()
        dataset, skipped = self.documents()
        self.assertEqual(dataset["records"], [])
        self.assertEqual(skipped["summary"]["skipped_frame_reason_counts"], {"invalid_jpeg_structure": 1})


if __name__ == "__main__":
    unittest.main()
