#!/usr/bin/env python3
"""Pure policy/geometry/scheduler tests: no Torch, training, GPU or downloads."""

import copy
import importlib.util
import math
from pathlib import Path
import random
import sys
import tempfile
import unittest
from types import SimpleNamespace as NS
from unittest import mock

import train_traffic_light_cnn as cnn

from train_traffic_light_cnn import (
    augmentation_parameters, freeze_batchnorm_running_stats, lr_multiplier,
    quality_gate, selection_split_rows, training_augmentation_contract, validate_config, validation_is_better,
)


def row(split="train"):
    return {"native_gt_verified": True, "manual_qc_accepted": True, "split": split,
            "crop_integer_xyxy": [20, 10, 80, 30], "image_width": 100, "image_height": 50}


class FullCnnPureTests(unittest.TestCase):
    def test_import_is_lazy_and_never_imports_torch_or_torchvision(self):
        before = set(sys.modules)
        path = Path(__file__).with_name("train_traffic_light_cnn.py")
        spec = importlib.util.spec_from_file_location("cnn_lazy_fixture", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        added = set(sys.modules) - before
        self.assertFalse(any(name == "torch" or name.startswith("torch.")
                             or name == "torchvision" or name.startswith("torchvision.") for name in added))

    def test_manual_qc_default_requires_exact_true_and_native_verification(self):
        self.assertEqual(quality_gate([row()])["manual_qc_accepted_count"], 1)
        for flag in (None, False, 1, "true"):
            candidate = row()
            candidate["manual_qc_accepted"] = flag
            with self.subTest(flag=flag), self.assertRaises(ValueError):
                quality_gate([candidate])
        candidate = row()
        candidate["native_gt_verified"] = False
        with self.assertRaises(ValueError):
            quality_gate([candidate], allow_weak_native_labels=True)

    def test_weak_label_opt_in_does_not_override_explicit_qc_rejection(self):
        candidate = row()
        candidate.pop("manual_qc_accepted")
        result = quality_gate([candidate], allow_weak_native_labels=True)
        self.assertEqual(result["unreviewed_record_count"], 1)
        self.assertEqual(result["mode"], "weak_native_label_research_only")
        candidate["manual_qc_accepted"] = False
        with self.assertRaises(ValueError):
            quality_gate([candidate], allow_weak_native_labels=True)

    def test_validation_test_preprocessing_is_exactly_fixed_and_rng_unused(self):
        class ForbiddenRng:
            def uniform(self, *_args):
                raise AssertionError("Validation touched augmentation RNG")
        for split in ("val", "test"):
            parameters = augmentation_parameters(row(split), ForbiddenRng())
            self.assertEqual(parameters["crop_xyxy"], [20, 10, 80, 30])
            self.assertEqual(parameters["brightness"], 1.)
            self.assertEqual(parameters["contrast"], 1.)
            self.assertFalse(parameters["gaussian_blur"])
            self.assertEqual(parameters["padding_fraction_ltrb"], [0., 0., 0., 0.])
            self.assertEqual(parameters["padding_pixels_ltrb"], [0, 0, 0, 0])
            self.assertTrue(parameters["approved_ROI_contained"])
            with self.assertRaises(ValueError):
                augmentation_parameters(row(split), random.Random(0), training=True)

    def test_100_rng_draws_contain_approved_roi_with_no_flip_no_hue(self):
        candidate = row()
        original = copy.deepcopy(candidate)
        for seed in range(100):
            first = augmentation_parameters(candidate, random.Random(seed), training=True)
            second = augmentation_parameters(candidate, random.Random(seed), training=True)
            self.assertEqual(first, second)
            self.assertTrue(all(0 <= value <= .1 for value in first["padding_fraction_ltrb"]))
            left, top, right, bottom = first["crop_xyxy"]
            self.assertTrue(0 <= left <= 20 < 80 <= right <= 100)
            self.assertTrue(0 <= top <= 10 < 30 <= bottom <= 50)
            self.assertTrue(first["approved_ROI_contained"])
            for value, dimension in zip(first["padding_pixels_ltrb"], (60, 20, 60, 20)):
                self.assertTrue(0 <= value <= .1 * dimension)
            self.assertTrue(.9 <= first["brightness"] <= 1.1)
            self.assertTrue(.9 <= first["contrast"] <= 1.1)
            self.assertFalse(first["horizontal_flip"])
            self.assertFalse(first["vertical_flip"])
            self.assertEqual(first["hue_shift"], 0.)
            self.assertFalse(first["gaussian_blur"])
        self.assertEqual(candidate, original)

    def test_crop_clamping_never_empty_even_for_single_pixel_at_boundary(self):
        for box in ([0, 0, 1, 1], [99, 49, 100, 50], [0, 0, 100, 50]):
            candidate = row()
            candidate["crop_integer_xyxy"] = box
            for seed in range(100):
                parameters = augmentation_parameters(candidate, random.Random(seed), training=True)
                left, top, right, bottom = parameters["crop_xyxy"]
                self.assertTrue(0 <= left < right <= 100)
                self.assertTrue(0 <= top < bottom <= 50)
                self.assertTrue(left <= box[0] < box[2] <= right)
                self.assertTrue(top <= box[1] < box[3] <= bottom)
                self.assertTrue(parameters["approved_ROI_contained"])

    def test_expansion_contract_is_shared_and_does_not_advertise_shrink_or_shift(self):
        contract = training_augmentation_contract()
        self.assertEqual(contract["spatial_mode"], "signal_expansion_only_manual_background_fixed")
        self.assertEqual(contract["signal_spatial_mode"], "expansion_only_asymmetric_context_padding")
        self.assertEqual(contract["manual_background_spatial_mode"], "fixed_approved_QC_ROI_no_expansion")
        self.assertEqual(contract["photometric_scope"], "both_signal_and_manual_background_training_records")
        self.assertEqual(contract["independent_per_side_fraction_range"], [0., .1])
        self.assertFalse(contract["approved_ROI_can_be_cut"])
        self.assertEqual(contract["brightness_range"], [.9, 1.1])
        self.assertEqual(contract["contrast_range"], [.9, 1.1])
        self.assertNotIn("center_jitter_fraction", contract)
        self.assertNotIn("scale_range", contract)
        self.assertEqual(contract["blur_probability"], 0.)

    def test_background_100_draws_are_spatially_fixed_but_photometric_is_allowed(self):
        candidate = row()
        candidate.update(annotation_kind="manual_background_no_lamps", native_gt_verified=False,
                         manual_no_lamp_qc_accepted=True, labels=dict.fromkeys(cnn.CLASSES, 0))
        for seed in range(100):
            result = augmentation_parameters(candidate, random.Random(seed), training=True)
            self.assertEqual(result["crop_xyxy"], [20, 10, 80, 30])
            self.assertEqual(result["padding_fraction_ltrb"], [0., 0., 0., 0.])
            self.assertEqual(result["padding_pixels_ltrb"], [0, 0, 0, 0])
            self.assertEqual(result["spatial_mode"], "fixed_approved_manual_background_ROI")
            self.assertTrue(.9 <= result["brightness"] <= 1.1)
            self.assertTrue(.9 <= result["contrast"] <= 1.1)
            self.assertFalse(result["horizontal_flip"])
            self.assertFalse(result["vertical_flip"])
            self.assertEqual(result["hue_shift"], 0.)

        class PhotometricOnlyRng:
            def uniform(self, low, high):
                if (low, high) != (.9, 1.1):
                    raise AssertionError("Background attempted a spatial RNG draw")
                return 1.05
        result = augmentation_parameters(candidate, PhotometricOnlyRng(), training=True)
        self.assertEqual((result["brightness"], result["contrast"]), (1.05, 1.05))

    def test_two_epoch_warmup_then_cosine_reaches_floor(self):
        values = [lr_multiplier(epoch) for epoch in range(30)]
        self.assertEqual(values[:3], [.5, 1., 1.])
        self.assertAlmostEqual(values[-1], .01)
        self.assertTrue(all(left >= right for left, right in zip(values[2:], values[3:])))
        self.assertTrue(all(.01 <= value <= 1. for value in values))
        self.assertEqual(lr_multiplier(0, epochs=1, warmup_epochs=0), 1.)

    def test_selection_uses_only_val_macro_f1_then_lower_bce(self):
        best = {"macro_f1": .8, "bce": .2}
        self.assertTrue(validation_is_better(.8, .19, best))
        self.assertTrue(validation_is_better(.81, 5., best))
        self.assertFalse(validation_is_better(.79, .01, best))
        self.assertFalse(validation_is_better(.8, .2, best))
        self.assertTrue(validation_is_better(0., 1., None))
        with self.assertRaises(ValueError):
            validation_is_better(float("nan"), .1)

    def test_no_positive_support_is_reported_without_division_error_or_fake_f1(self):
        result = cnn.metric_summary([[0, 0, 0, 0]], [[0, 0, 0, 0]])
        self.assertEqual(result["exact_set_accuracy"], 1.)
        self.assertEqual(result["macro_f1"], 0.)
        for item in result["per_class"].values():
            self.assertEqual(item["positive_support"], 0)
            self.assertEqual(item["f1"], 0.)

    def test_training_loop_split_interface_excludes_test(self):
        records = [row("test"), row("train"), row("val")]
        training, validation = selection_split_rows(records)
        self.assertEqual([item["split"] for item in training], ["train"])
        self.assertEqual([item["split"] for item in validation], ["val"])

    def test_batchnorm_eval_only_does_not_freeze_affine_parameters(self):
        class BatchNorm:
            def __init__(self):
                self.training = True
                self.affine = NS(requires_grad=True)
            def eval(self):
                self.training = False
        bn = BatchNorm()
        non_bn = NS(training=True)
        fake_model = NS(modules=lambda: [bn, non_bn])
        fake_torch = NS(nn=NS(modules=NS(batchnorm=NS(_BatchNorm=BatchNorm))))
        freeze_batchnorm_running_stats(fake_model, fake_torch)
        self.assertFalse(bn.training)
        self.assertTrue(bn.affine.requires_grad)
        self.assertTrue(non_bn.training)

    def test_invalid_configuration_does_not_silently_fallback(self):
        validate_config(device="cuda:0")
        for kwargs in ({"epochs": 0}, {"warmup_epochs": 31}, {"batch_size": 0}, {"head_lr": 0},
                       {"backbone_lr": float("nan")}, {"weight_decay": -1}, {"device": "cuda"},
                       {"device": "auto"}, {"blur_probability": .1}, {"dropout": .2}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                validate_config(**kwargs)

    def test_validate_only_and_default_qc_gate_never_call_training(self):
        with tempfile.TemporaryDirectory() as directory:
            weights = Path(directory) / "local.pt"
            weights.write_bytes(b"fixture local weights; never load Torch")
            metadata = {"path": "fixture.json", "sha256": "fixture"}
            with mock.patch.object(cnn, "load_dataset", return_value=([row()], metadata)), \
                    mock.patch.object(cnn, "train", side_effect=AssertionError("Training is forbidden in validation")):
                before = set(sys.modules)
                result = cnn.execute("fixture.json", weights, validate_only=True)
                self.assertFalse(result["training_executed"])
                self.assertFalse(any(name == "torch" or name.startswith("torch.") for name in set(sys.modules) - before))
            unreviewed = row()
            unreviewed.pop("manual_qc_accepted")
            with mock.patch.object(cnn, "load_dataset", return_value=([unreviewed], metadata)):
                with self.assertRaisesRegex(ValueError, "manual_qc_accepted"):
                    cnn.execute("fixture.json", weights, validate_only=True)
                result = cnn.execute("fixture.json", weights, validate_only=True, allow_weak_native_labels=True)
                self.assertEqual(result["quality_gate"]["unreviewed_record_count"], 1)

    def test_new_tool_reuses_existing_split_and_sha_gates(self):
        from train_traffic_light_crop_probe import load_dataset
        import hashlib
        import json
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            rows = []
            for index, (split, group, bits) in enumerate([
                    ("train", "pose1", [1, 0, 0, 0]), ("train", "pose2", [0, 1, 1, 1]),
                    ("val", "pose3", [1, 0, 0, 0]), ("test", "pose4", [0, 1, 1, 1])]):
                image = root / (str(index) + ".jpg")
                image.write_bytes(("fixture-only-" + str(index)).encode())
                rows.append({"image_path": image.name, "image_sha256": hashlib.sha256(image.read_bytes()).hexdigest(),
                             "crop_xyxy": [2, 3, 20, 10], "labels": dict(zip(cnn.CLASSES, bits)),
                             "split": split, "group_id": group, "environment": "Sunny13",
                             "native_gt_verified": True, "manual_qc_accepted": True})
            path, weights = root / "dataset.json", root / "local.pt"
            weights.write_bytes(b"fixture-not-loaded")
            path.write_text(json.dumps({"records": rows}))
            with mock.patch.object(cnn, "load_dataset", side_effect=lambda value, **options: load_dataset(value, size_reader=lambda _path: (100, 50), **options)):
                self.assertFalse(cnn.execute(path, weights, validate_only=True)["training_executed"])
                rows[-1]["group_id"] = rows[0]["group_id"]
                path.write_text(json.dumps({"records": rows}))
                with self.assertRaisesRegex(ValueError, "group.*crosses"):
                    cnn.execute(path, weights, validate_only=True)
                rows[-1]["group_id"] = "pose4"
                rows[0]["image_sha256"] = "0" * 64
                path.write_text(json.dumps({"records": rows}))
                with self.assertRaisesRegex(ValueError, "SHA256 mismatch"):
                    cnn.execute(path, weights, validate_only=True)

    def test_full_cnn_background_quality_gate_preserves_source_kind_and_never_weakens_it(self):
        background = row()
        background.update(native_gt_verified=False, annotation_kind="manual_background_no_lamps",
                          manual_no_lamp_qc_accepted=True, labels=dict.fromkeys(cnn.CLASSES, 0))
        result = quality_gate([row(), background])
        self.assertEqual(result["native_verified_record_count"], 1)
        self.assertEqual(result["manual_background_record_count"], 1)
        self.assertIs(background["native_gt_verified"], False)
        for changes in ({"manual_qc_accepted": None}, {"manual_no_lamp_qc_accepted": None},
                        {"labels": dict.fromkeys(cnn.CLASSES, 1)}, {"annotation_kind": "unknown"}):
            candidate = dict(background, **changes)
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                quality_gate([candidate], allow_weak_native_labels=True)

    def test_full_cnn_loader_explicitly_opts_in_without_enabling_frozen_cli(self):
        with tempfile.TemporaryDirectory() as directory:
            weights = Path(directory) / "local.pt"
            weights.write_bytes(b"fixture; no model load")
            with mock.patch.object(cnn, "load_dataset", return_value=([row()], {})) as loader:
                cnn.execute("fixture.json", weights, validate_only=True)
            loader.assert_called_once_with("fixture.json", allow_manual_background=True)

    def test_prediction_metadata_preserves_background_native_false_with_mock_training_only(self):
        import hashlib
        import json
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            dataset, weights, output = root / "dataset.json", root / "local.pt", root / "new_output"
            dataset.write_bytes(b"{}")
            weights.write_bytes(b"fixture; no model loading")
            background = row()
            background.update(native_gt_verified=False, annotation_kind="manual_background_no_lamps",
                manual_no_lamp_qc_accepted=True, labels=dict.fromkeys(cnn.CLASSES, 0),
                record_index=0, image_path="fixture.jpg", image_sha256="0" * 64,
                crop_xyxy=[20, 10, 80, 30], group_id="pose1", environment="Sunny13", bits=[0, 0, 0, 0])
            metadata = {"path": str(dataset), "sha256": hashlib.sha256(dataset.read_bytes()).hexdigest()}

            def fake_training(_records, _weights, folder, *_args, **_kwargs):
                (folder / "best.pt").write_bytes(b"test artifact; not a trained checkpoint")
                return {"predictions": [[0, 0, 0, 0]], "probabilities": [[.1, .2, .3, .4]],
                        "hsv_predictions": [[0, 0, 1, 0]], "runtime": {},
                        "best_validation": {"epoch": 1, "macro_f1": 0., "bce": .1},
                        "train_pos_weight": dict.fromkeys(cnn.CLASSES, 1.),
                        "test_evaluation_pass_count": 1, "test_batch_forward_count": 1}

            with mock.patch.object(cnn, "load_dataset", return_value=([background], metadata)), \
                    mock.patch.object(cnn, "train", side_effect=fake_training):
                result = cnn.execute(dataset, weights, output)
            prediction = json.loads((output / "predictions.jsonl").read_text())
            self.assertIs(prediction["native_gt_verified"], False)
            self.assertEqual(prediction["annotation_kind"], "manual_background_no_lamps")
            self.assertTrue(prediction["manual_no_lamp_qc_accepted"])
            self.assertEqual(result["provenance"]["annotation_counts"],
                             {"native_verified": 0, "manual_background_no_lamps": 1})


if __name__ == "__main__":
    unittest.main()
