import math
import unittest

from tracking_prediction_evaluation.models import (
    CTRAPredictor,
    CTRVPredictor,
    MotionState,
)


class PredictionModelTest(unittest.TestCase):
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
