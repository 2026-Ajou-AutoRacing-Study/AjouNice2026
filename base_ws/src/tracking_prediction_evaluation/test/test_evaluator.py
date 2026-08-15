import unittest

from tracking_prediction_evaluation.evaluator import EvaluationConfig, evaluate_segments
from tracking_prediction_evaluation.models import CTRAPredictor, CTRVPredictor
from tracking_prediction_evaluation.trajectory import TrajectoryPoint, TrajectorySegment


def point(timestamp, x, speed):
    return TrajectoryPoint(
        timestamp=timestamp,
        x=x,
        y=0.0,
        z=0.0,
        yaw=0.0,
        longitudinal_velocity=speed,
        lateral_velocity=0.0,
        longitudinal_acceleration=1.0,
        lateral_acceleration=0.0,
        class_name="vehicle",
        object_id=1,
        asset_name="synthetic",
    )


class EvaluatorTest(unittest.TestCase):
    def test_causal_ctra_beats_ctrv_on_constant_acceleration(self):
        points = []
        dt = 0.05
        acceleration = 1.0
        initial_speed = 2.0
        for index in range(161):
            timestamp = index * dt
            speed = initial_speed + acceleration * timestamp
            x = initial_speed * timestamp + 0.5 * acceleration * timestamp**2
            points.append(point(timestamp, x, speed))
        segment = TrajectorySegment(
            key="vehicle:1:0",
            class_name="vehicle",
            object_id=1,
            asset_name="synthetic",
            points=tuple(points),
        )
        samples = evaluate_segments(
            [segment],
            [CTRVPredictor(), CTRAPredictor(acceleration_limit=None)],
            EvaluationConfig(horizons=(1.0,), evaluation_rate_hz=5.0),
        )
        ctrv = [sample.fde for sample in samples if sample.model == "ctrv"]
        ctra = [sample.fde for sample in samples if sample.model == "ctra"]
        self.assertTrue(ctrv and ctra)
        self.assertLess(sum(ctra) / len(ctra), 1.0e-6)
        self.assertGreater(sum(ctrv) / len(ctrv), 0.4)


if __name__ == "__main__":
    unittest.main()
