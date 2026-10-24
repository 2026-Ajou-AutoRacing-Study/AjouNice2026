#!/usr/bin/env python3
"""현행 YAML/보정의 계약. 황색 신규 출발에 관한 규정 승인 테스트가 아니다.

옛 test_signal_policy.py의 v1.0 기대값과 구분한다. 운영 정책/플래닝을
변경하지 않으며 raw→normalize→smoothing 경계를 그대로 검증한다.
"""

from pathlib import Path
import sys
import unittest

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "scripts"))
from signal_phase_policy import SignalPhasePolicy, RESULT_TO_TOKEN
from signal_smoother import SignalSmoother

R, Y, G, L = (("red", "circle", None), ("yellow", "circle", None),
              ("green", "circle", None), ("green", "arrow", True))


class CurrentPolicyTests(unittest.TestCase):
    def setUp(self):
        self.policy = SignalPhasePolicy.from_yaml(str(PACKAGE / "config/traffic_signal_profiles.yaml"))

    def test_all_16_raw_combinations_normalize_to_current_contract(self):
        expected = {
            201351: ["R", "R", "Y", "R", "G", "R", "R", "R", "R", "R", "Y", "R", "G", "R", "R", "R"],
            201365: ["R", "R", "Y", "R", "G", "R", "R", "R", "R", "R", "Y", "R", "G", "R", "R", "R"],
            201290: ["R", "R", "Y", "R", "GL", "R", "R", "R", "GL", "R", "R", "R", "GL", "R", "R", "R"],
            201326: ["R", "R", "R", "RY", "G", "R", "R", "R", "RL", "RL", "R", "R", "G", "R", "R", "R"],
        }
        for group, phases in expected.items():
            for mask, wanted in enumerate(phases):
                raw = [lamp for index, lamp in enumerate((R, Y, G, L)) if mask & (1 << index)]
                normalized, _ = self.policy.apply(raw, group)
                with self.subTest(group=group, mask=mask):
                    self.assertEqual(set(normalized), {lamp for token, lamp in zip("RYGL", (R, Y, G, L)) if token in wanted})

    def test_d2_two_yellows_are_directly_different_not_previous_phase_inference(self):
        for preceding in ([G], [R, L], [R], []):
            self.policy.reset_transition()
            self.policy.apply_transition(preceding, 201326)
            stop, _ = self.policy.apply([Y], 201326)
            preserved, _ = self.policy.apply([R, Y], 201326)
            self.assertEqual(stop, [R])
            self.assertEqual(set(preserved), {R, Y})

    def test_d2_guard_uses_real_normalized_ry_and_expires_without_refreshing(self):
        smoother = SignalSmoother(1)
        def process(raw, stamp):
            normalized, reason = self.policy.apply(raw, 201326)
            return self.policy.process_normalized(normalized, reason, 201326, stamp, 1., smoother)
        self.assertEqual(set(process([R, Y], 0)[0]), {R, Y})
        self.assertEqual(set(process([], 1.0)[0]), {R, Y})
        self.assertEqual(process([], 1.51)[0], [R])
        for stamp in (2.0, 3.0, 4.0, 5.0):
            self.assertEqual(set(process([R, Y], stamp)[0]), {R, Y})
        self.assertEqual(process([R, Y], 6.01)[0], [R])

    def test_unknown_defaults_to_profile_red_not_green(self):
        for group in (0, 201351, 201365, 201290, 201326):
            self.assertEqual(self.policy.apply([], group)[0], [R])
        self.assertEqual(self.policy.apply([], 201397)[0], [])


if __name__ == "__main__":
    unittest.main()
