#!/usr/bin/env python3
"""CNN adapter의 순수 계약 테스트. Torch·checkpoint·GPU 추론은 실행하지 않는다."""

import copy
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
import traffic_light_cnn_classifier as cnn


class Shape:
    def __init__(self, *shape):
        self.shape = shape


def checkpoint(dropout=0.):
    final = "fc" if dropout == 0 else "fc.1"
    return {"architecture": "resnet18", "classes": list(cnn.CLASSES),
            "research_only": True, "not_production": True, "threshold": .5,
            "dropout": dropout, "input_size_wh": [224, 128], "normalization": "ImageNet",
            "training_scope": cnn.TRAINING_SCOPE, "preprocessing": cnn.PREPROCESSING,
            "model_state_dict": {"conv1.weight": Shape(64, 3, 7, 7),
                                 final + ".weight": Shape(4, 512), final + ".bias": Shape(4)}}


class ProbabilityTests(unittest.TestCase):
    def test_all_sixteen_combinations_preserved_as_raw_observations(self):
        for mask in range(16):
            probabilities = [.99 if mask & (1 << index) else .01 for index in range(4)]
            out = cnn.observation_from_probabilities(probabilities)
            self.assertTrue(out.known)
            self.assertEqual(out.bits, tuple(int(bool(mask & (1 << index))) for index in range(4)))
            self.assertEqual(out.tuples, tuple(item for index, item in enumerate(cnn.CLASS_TUPLES)
                                              if mask & (1 << index)))

    def test_no_policy_correction_or_direction_assumption(self):
        out = cnn.observation_from_probabilities([.99, .01, .99, .01])
        self.assertEqual(out.tuples, (("red", "circle", None), ("green", "circle", None)))
        arrow = cnn.observation_from_probabilities([.01, .01, .01, .99])
        self.assertEqual(arrow.tuples, (("green", "arrow", True),))

    def test_any_uncertain_bit_abstains_on_entire_phase(self):
        out = cnn.observation_from_probabilities([.99, .01, .5, .01])
        self.assertFalse(out.known)
        self.assertEqual(out.tuples, ())
        self.assertEqual(out.bits, (1, 0, None, 0))
        self.assertIn("green", out.reason)
        self.assertEqual(out.to_dict()["empty_result_kind"], "unknown")

    def test_confident_all_zero_and_unknown_are_distinct_neither_is_green(self):
        dark = cnn.observation_from_probabilities([.01] * 4)
        unknown = cnn.observation_from_probabilities([.5] * 4)
        self.assertTrue(dark.known)
        self.assertFalse(unknown.known)
        self.assertEqual(dark.tuples, unknown.tuples)
        self.assertEqual(dark.reason, "no_lamps")
        self.assertEqual(dark.to_dict()["empty_result_kind"], "no_lamps")

    def test_off_confidence_is_not_probability_of_an_illuminated_arrow(self):
        # A confident false negative cannot be repaired by lowering only the
        # abstention threshold. Phase truth is external, never guessed here.
        for minimum in (.6, .5):
            out = cnn.observation_from_probabilities([.999, .001, .001, .21],
                                                     min_bit_confidence=minimum)
            self.assertTrue(out.known)
            self.assertEqual(out.bits, (1, 0, 0, 0))
            self.assertAlmostEqual(out.confidence, .79)
            self.assertEqual(out.tuples, (("red", "circle", None),))

    def test_class_mapping_and_tie_threshold_are_explicit(self):
        out = cnn.observation_from_probabilities(dict(zip(reversed(cnn.CLASSES), [.99, .01, .01, .01])))
        self.assertEqual(out.tuples, (("green", "arrow", True),))
        tied = cnn.observation_from_probabilities([.5] * 4, min_bit_confidence=.5)
        self.assertEqual(tied.bits, (1, 1, 1, 1))

    def test_invalid_or_nonfinite_probabilities_never_produce_a_phase(self):
        for values in (None, [], [1] * 3, [1] * 5, [float("nan")] * 4,
                       [float("inf")] * 4, [-.1, 0, 0, 0], [1.1, 0, 0, 0],
                       [True, 0, 0, 0], {"red": 1}, ["1", 0, 0, 0]):
            with self.subTest(values=values):
                out = cnn.observation_from_probabilities(values)
                self.assertFalse(out.known)
                self.assertEqual(out.tuples, ())
                self.assertEqual(out.confidence, 0)

    def test_invalid_confidence_configuration_rejected(self):
        for value in (True, None, float("nan"), .49, 1.01):
            with self.assertRaises(ValueError):
                cnn.validate_min_confidence(value)


class MetadataTests(unittest.TestCase):
    def test_strict_supported_checkpoint_contract_and_optional_dropout(self):
        for dropout in (0., .1):
            metadata = cnn.validate_checkpoint_contract(checkpoint(dropout))
            self.assertEqual(metadata["dropout"], dropout)
            self.assertTrue(metadata["not_production"])

    def test_class_order_preprocessing_and_metadata_not_inferred_from_shapes(self):
        changes = {"classes": ["yellow", "red", "green", "left_arrow"],
                   "input_size_wh": [128, 224], "architecture": "resnet50",
                   "threshold": .6, "normalization": "none", "dropout": .2,
                   "preprocessing": "RGB_resize_stretch", "training_scope": "frozen_feature_probe",
                   "research_only": 1, "not_production": False}
        for key, value in changes.items():
            data = checkpoint()
            data[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                cnn.validate_checkpoint_contract(data)

    def test_wrong_or_missing_tensor_shapes_rejected(self):
        for key, shape in (("conv1.weight", (64, 4, 7, 7)), ("fc.weight", (3, 512)), ("fc.bias", (5,))):
            data = checkpoint()
            data["model_state_dict"][key] = Shape(*shape)
            with self.assertRaises(ValueError):
                cnn.validate_checkpoint_contract(data)
        data = checkpoint()
        del data["model_state_dict"]["fc.weight"]
        with self.assertRaises(ValueError):
            cnn.validate_checkpoint_contract(data)

    def test_resize_matches_training_reference_integer_geometry(self):
        self.assertEqual(cnn.letterbox_geometry(120, 70), ((219, 128), (2, 0)))
        self.assertEqual(cnn.letterbox_geometry(100, 50), ((224, 112), (0, 8)))
        self.assertEqual(cnn.letterbox_geometry(224, 128), ((224, 128), (0, 0)))
        with self.assertRaises(ValueError):
            cnn.letterbox_geometry(0, 3)


class StartupTests(unittest.TestCase):
    def test_unconfigured_and_missing_checkpoint_do_not_raise_or_implicit_fallback(self):
        self.assertFalse(cnn.initialize_classifier().ready)
        with tempfile.TemporaryDirectory() as directory:
            before = set(sys.modules)
            startup = cnn.initialize_classifier(Path(directory) / "missing.pt")
            self.assertFalse(startup.ready)
            self.assertIsNone(startup.classifier)
            self.assertIn("FileNotFoundError", startup.reason)
            self.assertTrue(startup.metadata["caller_must_choose_legacy_or_unknown"])
            self.assertFalse(startup.metadata["implicit_fallback"])
            self.assertFalse(any(name == "torch" or name.startswith("torch.") for name in set(sys.modules) - before))

    def test_factory_receives_explicit_load_options_once(self):
        calls = []
        class Fake:
            metadata = {"load_count": 1}
        def factory(path, **kwargs):
            calls.append((path, kwargs))
            return Fake()
        startup = cnn.initialize_classifier("explicit.pt", device="cuda:0", warmup=True,
                                             expected_sha256="a" * 64, classifier_factory=factory)
        self.assertTrue(startup.ready)
        self.assertEqual(len(calls), 1)
        self.assertTrue(calls[0][1]["warmup"])
        self.assertEqual(startup.metadata["load_count"], 1)

    def test_contract_or_dependency_error_becomes_unavailable_not_legacy_inference(self):
        def factory(*args, **kwargs):
            raise ValueError("계약 오류")
        startup = cnn.initialize_classifier("explicit.pt", classifier_factory=factory)
        self.assertFalse(startup.ready)
        self.assertIn("계약 오류", startup.reason)

    def test_classifier_failure_returns_unknown_without_hsv_or_green(self):
        classifier = object.__new__(cnn.TrafficLightCnnClassifier)
        def fail(_):
            raise ValueError("잘못된 crop")
        classifier._input_tensor = fail
        result = classifier.classify_bgr(None)
        self.assertFalse(result.known)
        self.assertEqual(result.tuples, ())
        self.assertIn("inference_error", result.reason)

    def test_device_requires_explicit_supported_name(self):
        for device in ("auto", "0", 0, "cuda", "cuda:-1", "cuda:0;echo"):
            with self.assertRaises(ValueError):
                cnn.validate_device(device)


if __name__ == "__main__":
    unittest.main()
