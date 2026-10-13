#!/usr/bin/env python3

import math
import random
import unittest
from unittest import mock

import perception_object_pipeline.map_based_prediction as map_prediction

from perception_object_pipeline.map_based_prediction import (
    LaneletNetwork,
    LaneletRecord,
    MapPredictionConfig,
    project_point_to_polyline,
    sample_polyline,
)


class MapBasedPredictionCoreTest(unittest.TestCase):
    def test_arc_index_matches_scalar_reference_including_degenerate_segments(self):
        random_source = random.Random(261014)
        lines = [(), ((0.,0.,0.),), ((0.,0.,0.),(0.,0.,0.)),
                 ((0.,0.,0.),(1.e-12,0.,0.),(1.,0.,1.)),
                 ((0.,0.,0.),(1.,0.,0.),(1.,0.,0.),(1.,1.,2.))]
        for _ in range(30):
            points = [(0.,0.,0.)]
            for _ in range(25):
                old = points[-1]
                points.append(tuple(old[i]+random_source.uniform(-5.,5.) for i in range(3)))
            lines.append(tuple(points))
        for line in lines:
            indexed = map_prediction._RoutePolyline(line)
            self.assertEqual(indexed.length,map_prediction.polyline_length(line))
            if not line:
                with self.assertRaises(ValueError):indexed.sample(0.)
                continue
            distances = [-1.,0.,indexed.length,indexed.length+1.]
            distances += indexed.ends
            distances += [random_source.uniform(0.,max(1.,indexed.length)) for _ in range(20)]
            for distance in distances:
                self.assertEqual(indexed.sample(distance),sample_polyline(line,distance))
                self.assertEqual(indexed.tangent(distance),map_prediction.tangent_yaw(line,distance))

    def test_autoware_lanelet2_extension_is_loaded_once_and_globally(self):
        previous = map_prediction._AUTOWARE_LANELET2_EXTENSION_HANDLE
        map_prediction._AUTOWARE_LANELET2_EXTENSION_HANDLE = None
        try:
            with mock.patch.object(map_prediction.ctypes, "CDLL") as load_library:
                handle = object()
                load_library.return_value = handle
                map_prediction._load_autoware_lanelet2_extension()
                map_prediction._load_autoware_lanelet2_extension()
                load_library.assert_called_once_with(
                    "libautoware_lanelet2_extension_lib.so",
                    mode=map_prediction.ctypes.RTLD_GLOBAL,
                )
                self.assertIs(
                    map_prediction._AUTOWARE_LANELET2_EXTENSION_HANDLE, handle
                )
        finally:
            map_prediction._AUTOWARE_LANELET2_EXTENSION_HANDLE = previous

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

    def test_duplicate_start_lane_matches_collapse_per_future_branch(self):
        records = [
            LaneletRecord(1, ((0.0, 0.0, 0.0), (10.0, 0.0, 0.0)), (3, 4)),
            LaneletRecord(2, ((0.0, 0.1, 0.0), (10.0, 0.1, 0.0)), (3, 4)),
            LaneletRecord(3, ((10.0, 0.0, 0.0), (20.0, 0.0, 0.0))),
            LaneletRecord(4, ((10.0, 0.0, 0.0), (18.0, 6.0, 0.0))),
        ]
        network = LaneletNetwork(
            records,
            MapPredictionConfig(
                maximum_lane_distance_m=2.0,
                maximum_start_lanelets=2,
                maximum_paths=6,
                lateral_convergence_time_sec=1.5,
                trajectory_cluster_distance_m=0.75,
            ),
        )

        trajectories = network.predict(
            x=5.0,
            y=0.05,
            z=0.0,
            yaw=0.0,
            speed=2.0,
            horizons=(0.5, 1.0, 1.5, 2.0, 3.0),
        )

        self.assertEqual(len(trajectories), 2)
        self.assertEqual({path.lanelet_ids[-1] for path in trajectories}, {3, 4})
        self.assertAlmostEqual(sum(path.confidence for path in trajectories), 1.0)


if __name__ == "__main__":
    unittest.main()
