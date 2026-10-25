"""Tests characterize the existing baseline, including its unfixed limitations.

Synthetic behavior here is not evidence of a MORAI field failure and is not a
claim that the current misclassification mechanisms are desired target behavior.
"""

import json
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

import traffic_light_crop_diagnostics as diagnostics


class BaselineCharacterizationTest(unittest.TestCase):
    def test_neutral_gray_is_negative_and_gt_is_not_invented(self):
        report = diagnostics.diagnose_crop(np.full((10, 10, 3), 100, np.uint8))
        self.assertEqual(report["baseline_observation"], [])
        self.assertEqual(report["baseline_color_counts"]["green"], 0)
        self.assertIsNone(report["human_ground_truth"])
        self.assertEqual(report["ground_truth_status"], "not_provided")
        self.assertIsNone(report["green_mask"]["geometric_border_not_lamp_roi"]["share_of_green_in_border"])

    def test_baseline_limitation_toy_fog_can_cross_green_gate(self):
        base = np.full((10, 10, 3), (20, 40, 20), np.uint8)
        fog = np.full_like(base, 180)
        expected = [[], [["green", "circle", None]], [["green", "circle", None]], []]
        observations = []
        for alpha in (0, 0.15, 0.20, 0.25):
            crop = cv2.addWeighted(base, 1 - alpha, fog, alpha, 0)
            observations.append(diagnostics.diagnose_crop(crop)["baseline_observation"])
        self.assertEqual(observations, expected)

    def test_baseline_limitation_single_green_pixel_is_amplified(self):
        crop = np.zeros((10, 10, 3), np.uint8)
        crop[5, 5] = (0, 255, 0)
        report = diagnostics.diagnose_crop(crop)
        self.assertEqual(report["original_dimensions_wh"], [10, 10])
        self.assertEqual(report["upscaled_dimensions_wh"], [30, 30])
        self.assertGreater(report["baseline_color_counts"]["green"], 1)
        self.assertEqual(report["baseline_observation"], [["green", "circle", None]])

    def test_baseline_limitation_mirrored_arrow_directions_both_left(self):
        mask = np.zeros((40, 60), np.uint8)
        points = np.array([[5, 20], [24, 4], [24, 13], [54, 13], [54, 27], [24, 27], [24, 36]], np.int32)
        cv2.fillPoly(mask, [points], 255)
        for arrow_mask in (mask, cv2.flip(mask, 1)):
            crop = np.zeros((40, 60, 3), np.uint8)
            crop[:, :, 1] = arrow_mask
            report = diagnostics.diagnose_crop(crop)
            self.assertEqual(report["baseline_observation"], [["green", "arrow", True]])

    def test_baseline_limitation_opening_does_not_reject_green_presence(self):
        mask = np.zeros((30, 30), np.uint8)
        mask[1:20:3, 1:20:3] = 255
        result = diagnostics._green_diagnostics(mask, 0.1)
        self.assertEqual(result["pre_open_pixels"], 49)
        self.assertEqual(result["post_open_pixels"], 0)
        self.assertTrue(result["presence_gate_met"])
        self.assertEqual(result["baseline_shape_results"], [["circle", None]])
        self.assertEqual(result["shape_decision_path"], "default_circle_no_contours_after_opening")

    def test_contours_and_geometric_border_statistics(self):
        crop = np.full((10, 30, 3), (51, 70, 51), np.uint8)
        gt = {"annotator": "test_fixture", "state": "unlit_housing"}
        report = diagnostics.diagnose_crop(crop, human_gt=gt)
        self.assertEqual(report["human_ground_truth"], gt)
        self.assertEqual(report["ground_truth_status"], "provided_by_caller_unverified")
        green = report["green_mask"]
        self.assertEqual(green["pre_open_pixels"], 2700)
        contour = green["contours"][0]
        self.assertGreater(contour["area"], 0)
        self.assertGreater(contour["perimeter"], 0)
        self.assertLess(contour["circularity"], diagnostics.BASELINE.CIRCLE_THRESHOLD)
        self.assertEqual(contour["shape_if_analyzed"], "arrow_left")
        border = green["geometric_border_not_lamp_roi"]
        self.assertEqual(border["border_green_occupancy"], 1)
        self.assertEqual(border["interior_green_occupancy"], 1)
        self.assertAlmostEqual(border["share_of_green_in_border"] + border["share_of_green_in_interior"], 1)
        json.dumps(report, allow_nan=False)


class SelectionAndIoTest(unittest.TestCase):
    def test_current_picker_ignores_confidence_and_supports_override(self):
        image = np.zeros((100, 100, 3), np.uint8)
        boxes = [[45.2, 35.4, 55.2, 45.4, 0.99], [46.2, 5.4, 56.2, 15.4, 0.26]]
        report = diagnostics.diagnose_image(image, boxes)
        self.assertEqual(report["baseline_picker_selected_index"], 1)
        self.assertEqual(report["selected_index"], 1)
        self.assertEqual(report["boxes"][1]["xyxy_float_original"], boxes[1][:4])
        self.assertEqual(report["boxes"][1]["xyxy_integer_clamped_as_node"], [46, 5, 56, 15])
        self.assertLess(report["boxes"][1]["selection_score"], report["boxes"][0]["selection_score"])
        override = diagnostics.diagnose_image(image, boxes, selected_index=0)
        self.assertEqual(override["baseline_picker_selected_index"], 1)
        self.assertEqual(override["selected_index"], 0)
        self.assertEqual(override["selection_mode"], "caller_override")

    def test_dictionary_boxes_and_empty_input(self):
        image = np.zeros((10, 10, 3), np.uint8)
        report = diagnostics.diagnose_image(image, [{"xyxy": [-2.5, -1, 8.9, 9.9], "confidence": 0.8, "class_id": 0}])
        self.assertEqual(report["boxes"][0]["xyxy_integer_clamped_as_node"], [0, 0, 8, 9])
        empty = diagnostics.diagnose_image(image, [])
        self.assertIsNone(empty["selected_index"])
        self.assertIsNone(empty["crop_diagnostics"])
        self.assertEqual(empty["baseline_observation"], [])

    def test_invalid_inputs_are_rejected(self):
        with self.assertRaises(ValueError):
            diagnostics.diagnose_crop(np.zeros((0, 2, 3), np.uint8))
        with self.assertRaises(ValueError):
            diagnostics.diagnose_crop(np.zeros((2, 2, 3), np.uint8), border_fraction=0)
        with self.assertRaises(ValueError):
            diagnostics.diagnose_image(np.zeros((2, 2, 3), np.uint8), [[0, 0, 1, 1]], selected_index=1)

    def test_source_hashes_and_exclusive_output(self):
        report = diagnostics.diagnose_crop(np.zeros((2, 2, 3), np.uint8))
        self.assertEqual(len(report["provenance"]["utility_source_sha256"]), 64)
        self.assertEqual(len(report["provenance"]["diagnostic_source_sha256"]), 64)
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "report.json"
            diagnostics.write_json_exclusive(report, output)
            original = output.read_bytes()
            with self.assertRaises(FileExistsError):
                diagnostics.write_json_exclusive({"replacement": True}, output)
            self.assertEqual(output.read_bytes(), original)


if __name__ == "__main__":
    unittest.main()
