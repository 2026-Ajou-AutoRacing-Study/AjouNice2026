#!/usr/bin/env python3

import math
import unittest

from perception_object_pipeline.map_based_prediction import (
    LaneletNetwork,
    LaneletRecord,
    MapPredictionConfig,
    project_point_to_polyline,
    sample_polyline,
)


class MapBasedPredictionCoreTest(unittest.TestCase):
    def _network(self):
        records = [
            LaneletRecord(1, ((0.0, 0.0, 0.0), (10.0, 0.0, 0.0)), (2, 3)),
            LaneletRecord(2, ((10.0, 0.0, 0.0), (20.0, 0.0, 0.0))),
            LaneletRecord(3, ((10.0, 0.0, 0.0), (18.0, 6.0, 0.0))),
        ]
        return LaneletNetwork(
            records,
            MapPredictionConfig(
                maximum_lane_distance_m=2.0,
                maximum_start_lanelets=1,
                maximum_paths=4,
                lateral_convergence_time_sec=1.5,
            ),
            map_sha256="unit-test",
        )

    def test_projection_and_sampling(self):
        line = ((0.0, 0.0, 0.0), (10.0, 0.0, 2.0))
        projection = project_point_to_polyline(line, 3.0, 2.0)
        self.assertAlmostEqual(projection.x, 3.0)
        self.assertAlmostEqual(projection.y, 0.0)
        self.assertAlmostEqual(projection.z, 0.6)
        self.assertAlmostEqual(projection.distance, 2.0)
        self.assertAlmostEqual(sample_polyline(line, 5.0)[0], 5.0)

    def test_branch_paths_and_confidence(self):
        trajectories = self._network().predict(
            x=5.0,
            y=1.0,
            z=0.0,
            yaw=0.0,
            speed=2.0,
            horizons=(0.5, 1.0, 1.5, 2.0, 3.0),
        )
        self.assertEqual(len(trajectories), 2)
        self.assertAlmostEqual(sum(path.confidence for path in trajectories), 1.0)
        self.assertEqual({path.lanelet_ids for path in trajectories}, {(1, 2), (1, 3)})
        # The map path starts continuously at the tracked lateral offset and
        # converges to the centerline instead of snapping in one callback.
        self.assertGreater(trajectories[0].poses[0][1], 0.0)
        self.assertAlmostEqual(trajectories[0].poses[2][1], 0.0, delta=1.0e-6)

    def test_low_speed_and_wrong_direction_fall_back(self):
        network = self._network()
        self.assertEqual(
            network.predict(5.0, 0.0, 0.0, 0.0, 0.1, (1.0, 2.0, 3.0)), ()
        )
        self.assertEqual(
            network.predict(5.0, 0.0, 0.0, math.pi, 2.0, (1.0, 2.0, 3.0)), ()
        )


if __name__ == "__main__":
    unittest.main()
