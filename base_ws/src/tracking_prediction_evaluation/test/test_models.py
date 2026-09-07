import math
import unittest

from tracking_prediction_evaluation.models import (
    CTRAPredictor,
    CTRVPredictor,
    MapBasedP1Predictor,
    MotionState,
)


class _Trajectory:
    def __init__(self, poses, confidence):
        self.poses = poses
        self.confidence = confidence


class _MapNetwork:
    map_sha256 = "test"

    def predict(self, **kwargs):
        horizons = kwargs["horizons"]
        return (
            _Trajectory(tuple((10.0 + t, 2.0, 0.0, 0.0) for t in horizons), 0.2),
            _Trajectory(tuple((20.0 + t, 3.0, 0.0, 0.0) for t in horizons), 0.8),
        )


class PredictionModelTest(unittest.TestCase):
    def test_map_predictor_uses_top_confidence_and_nonvehicle_fallback(self):
        state = MotionState(0.0, 0.0, 0.0, 2.0, 0.0, 0.0)
        predictor = MapBasedP1Predictor(_MapNetwork())
        mapped = predictor.predict_path(state, (1.0, 2.0), "vehicle")
        pedestrian = predictor.predict_path(state, (1.0,), "pedestrian")
        self.assertEqual(mapped, ((21.0, 3.0, 0.0), (22.0, 3.0, 0.0)))
        self.assertAlmostEqual(pedestrian[0][0], 2.0)
        self.assertEqual(predictor.mapped_origin_count, 1)
        self.assertEqual(predictor.fallback_origin_count, 1)

    def test_ctrv_straight(self):
        state = MotionState(1.0, 2.0, 0.0, 3.0, 0.0, 99.0)
        x, y, yaw = CTRVPredictor().predict(state, 2.0)
        self.assertAlmostEqual(x, 7.0)
        self.assertAlmostEqual(y, 2.0)
        self.assertAlmostEqual(yaw, 0.0)

    def test_ctra_straight_acceleration(self):
        state = MotionState(0.0, 0.0, 0.0, 2.0, 0.0, 1.0)
        x, y, _ = CTRAPredictor(acceleration_limit=None).predict(state, 2.0)
        self.assertAlmostEqual(x, 6.0)
        self.assertAlmostEqual(y, 0.0)

    def test_ctrv_circle(self):
        state = MotionState(0.0, 0.0, 0.0, 2.0, 1.0, 0.0)
        x, y, yaw = CTRVPredictor().predict(state, math.pi / 2.0)
        self.assertAlmostEqual(x, 2.0, places=6)
        self.assertAlmostEqual(y, 2.0, places=6)
        self.assertAlmostEqual(yaw, math.pi / 2.0, places=6)

    def test_ctra_zero_acceleration_matches_ctrv(self):
        state = MotionState(4.0, -2.0, 0.4, 5.0, -0.3, 0.0)
        ctrv = CTRVPredictor().predict(state, 1.7)
        ctra = CTRAPredictor().predict(state, 1.7)
        for expected, actual in zip(ctrv, ctra):
            self.assertAlmostEqual(expected, actual, places=9)


if __name__ == "__main__":
    unittest.main()
