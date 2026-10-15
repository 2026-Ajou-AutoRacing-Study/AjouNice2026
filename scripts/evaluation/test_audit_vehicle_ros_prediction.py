#!/usr/bin/env python3
"""Refuse downstream scoring of failed or shifted ROS qualifications."""
import copy
import unittest

from audit_vehicle_ros_prediction import qualify_replay


def fixture():
    row = dict(id="a", label=1, center=[1., 2., 3.])
    outputs = {mode: [dict(stamp=100., objects=[copy.deepcopy(row)])]
               for mode in ("baseline", "guard", "innovation", "combined")}
    golden = dict(reference_frames=1, unpaired_frames=0, identity_set_changed_frames=0,
                  stats={key: dict(maximum=0.) for key in ("stamp_delta", "center_delta")})
    receipt = dict(sent_frames=1, valid_source_frames=1, input_age_preserved=True,
        original_header_preserved=True, original_receipt_schedule_preserved=True,
        private_boundary_barriers=True, outputs={mode: 1 for mode in outputs},
        paired={key: copy.deepcopy(golden) for key in ("baseline_guard", "innovation_combined")})
    return outputs, receipt


class RosPredictionQualificationTest(unittest.TestCase):
    def test_exact_guard_golden_is_accepted(self):
        outputs, receipt = fixture()
        self.assertTrue(qualify_replay(outputs, receipt)["pass_golden"])

    def test_shifted_clock_is_rejected_even_when_summary_claims_pass(self):
        outputs, receipt = fixture()
        outputs["guard"][0]["stamp"] += .001
        with self.assertRaises(ValueError):
            qualify_replay(outputs, receipt)

    def test_altered_center_is_rejected_even_when_summary_claims_pass(self):
        outputs, receipt = fixture()
        outputs["guard"][0]["objects"][0]["center"][0] += 1.e-12
        with self.assertRaises(ValueError):
            qualify_replay(outputs, receipt)

    def test_changed_id_is_rejected(self):
        outputs, receipt = fixture()
        outputs["guard"][0]["objects"][0]["id"] = "b"
        with self.assertRaises(ValueError):
            qualify_replay(outputs, receipt)

    def test_incomplete_native_arrivals_are_rejected(self):
        outputs, receipt = fixture()
        receipt["native_arrival_counts"] = {"baseline": 1, "guard": 0}
        with self.assertRaises(ValueError):
            qualify_replay(outputs, receipt)

    def test_failed_summary_golden_is_rejected(self):
        outputs, receipt = fixture()
        receipt["paired"]["baseline_guard"]["unpaired_frames"] = 1
        with self.assertRaises(ValueError):
            qualify_replay(outputs, receipt)

    def test_lost_header_age_contract_is_rejected(self):
        outputs, receipt = fixture()
        receipt["original_header_preserved"] = False
        with self.assertRaises(ValueError):
            qualify_replay(outputs, receipt)


if __name__ == "__main__":
    unittest.main()
