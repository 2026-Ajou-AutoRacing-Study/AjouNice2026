#!/usr/bin/env python3

import math
import random
import unittest
import weakref
from concurrent.futures import ThreadPoolExecutor
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
    def test_cached_projection_is_exact_including_ties_and_tiny_segments(self):
        rng = random.Random(26101442)
        lines = [(), ((0., 0., 0.),), ((0., 0., 0.), (0., 0., 0.)),
                 ((0., 0., 0.), (1.e-12, 0., 0.), (2., 0., 4.)),
                 ((0., 0., 0.), (1., 0., 0.), (0., 0., 1.))]
        for _ in range(80):
            line = [(rng.uniform(-1000, 1000), rng.uniform(-1000, 1000), 28.)]
            for _ in range(30):
                point = line[-1]
                line.append(tuple(point[i]+rng.uniform(-20,20) for i in range(3)))
            lines.append(tuple(line))
        for line in lines:
            geometry = map_prediction._ProjectionPolyline(line)
            probes = [(0., 0.), (1., 0.), (1., 1.)]
            probes.extend((p[0], p[1]) for p in line)
            probes.extend((rng.uniform(-1100,1100), rng.uniform(-1100,1100)) for _ in range(30))
            for x, y in probes:
                self.assertEqual(geometry.project(x, y), project_point_to_polyline(line, x, y))

    def test_route_geometry_cache_is_bounded_and_does_not_cache_observations(self):
        network = self._network()
        network._route_geometry_cache_limit = 2
        first = network._route_geometry((1,))
        self.assertIs(first, network._route_geometry((1,)))
        network._route_geometry((1, 2))
        network._route_geometry((1, 3))
        self.assertEqual(len(network._route_geometry_cache), 2)
        self.assertIsNot(first, network._route_geometry((1,)))
        high = network.predict(5., 1., 0., 0., 2., (1., 2., 3.))
        low = network.predict(5., -1., 0., 0., 2., (1., 2., 3.))
        self.assertNotEqual(high[0].poses, low[0].poses)
        self.assertEqual(high, network.predict(5., 1., 0., 0., 2., (1., 2., 3.)))

    def test_route_cache_separates_reference_geometry_class(self):
        network = self._network()
        original = network._route_geometry((1, 2))
        class ScalarGeometry:
            def __init__(self, points):
                self.length = map_prediction.polyline_length(points)
        with mock.patch.object(map_prediction, '_RoutePolyline', ScalarGeometry):
            scalar = network._route_geometry((1, 2))
            self.assertIsInstance(scalar[0], ScalarGeometry)
            self.assertIsNot(scalar, original)
        self.assertIs(network._route_geometry((1, 2)), original)

    def test_map_replacement_invalidates_cached_geometry_and_transition(self):
        network = self._network()
        first_geometry = network._route_geometry((1, 2))
        first_transition = network._transition_score(1, 2)
        first_length = network._lane_length(2)
        network.records[2] = LaneletRecord(2, ((10.,0.,0.), (12.,2.,3.)))
        self.assertIsNot(first_geometry, network._route_geometry((1, 2)))
        self.assertNotEqual(first_transition, network._transition_score(1, 2))
        self.assertNotEqual(first_length, network._lane_length(2))
        candidate = next(c for c in network.lane_candidates(11.,1.,math.pi/4.) if c.lanelet_id == 2)
        self.assertEqual(candidate.projection,
                         project_point_to_polyline(network.records[2].centerline,11.,1.))
        reloaded = self._network()
        self.assertEqual(reloaded._route_geometry_cache, {})
        self.assertEqual(reloaded._transition_scores, {})
        self.assertIsNot(network._projection_geometry[1],reloaded._projection_geometry[1])

    def test_transition_cache_is_bounded(self):
        network = self._network()
        network._transition_score_cache_limit = 2
        for pair in ((1,2), (1,3), (2,3)):
            network._transition_score(*pair)
        self.assertEqual(len(network._transition_scores), 2)

    def test_concurrent_geometry_cache_access_is_exact_and_bounded(self):
        network = self._network()
        network._route_geometry_cache_limit = 1
        network._transition_score_cache_limit = 1
        queries = [(5.,y,0.,0.,speed,(1.,2.,3.))
                   for y in (-1.,0.,1.) for speed in (1.,2.,3.)]
        expected = [network.predict(*query) for query in queries]
        with ThreadPoolExecutor(max_workers=4) as executor:
            actual = list(executor.map(lambda i:network.predict(*queries[i%len(queries)]),range(1000)))
        self.assertEqual(actual,[expected[i%len(queries)]for i in range(1000)])
        self.assertLessEqual(len(network._route_geometry_cache),1)
        self.assertLessEqual(len(network._transition_scores),1)

    def test_route_dfs_does_not_retain_network_in_recursive_closure(self):
        network = self._network()
        network._route_sequences(1, 0., 30.)
        reference = weakref.ref(network)
        del network
        # No gc.collect()/GC disabling: reference-counted ownership ends now.
        self.assertIsNone(reference())

    def test_iterative_route_dfs_preserves_recursive_order_caps_and_loops(self):
        rng = random.Random(2610142026)
        def recursive_reference(network, start_id, start_arc, required_length):
            completed = []
            def visit(ids, available, score, occurrences):
                if len(completed) >= network.config.maximum_paths*3:
                    return
                last = ids[-1]
                if available >= required_length or len(ids) >= network.config.maximum_route_depth:
                    completed.append((tuple(ids),score))
                    return
                followers = [i for i in network.records[last].following_ids
                             if i in network.records and occurrences.get(i,0) < 2]
                if not followers:
                    completed.append((tuple(ids),score))
                    return
                prior = 1./len(followers)
                followers.sort(key=lambda i: network._transition_score(last,i),reverse=True)
                for lane in followers:
                    visited = dict(occurrences)
                    visited[lane] = visited.get(lane,0)+1
                    visit(ids+[lane], available+network.records[lane].length,
                          score*prior*network._transition_score(last,lane),visited)
            visit([start_id],max(0.,network.records[start_id].length-start_arc),1.,{start_id:1})
            completed.sort(key=lambda item:item[1],reverse=True)
            return tuple(completed[:max(1,int(network.config.maximum_paths))])
        for _ in range(80):
            records = []
            for lane in range(1,8):
                angle = rng.uniform(-math.pi,math.pi)
                line = ((0.,0.,0.), (10.*math.cos(angle),10.*math.sin(angle),0.))
                followers = tuple(rng.sample(range(1,9),rng.randrange(4)))
                records.append(LaneletRecord(lane,line,followers))
            network = LaneletNetwork(records, MapPredictionConfig(
                maximum_paths=rng.randrange(1,7),maximum_route_depth=rng.randrange(1,9)))
            for required in (0.,5.,30.,100.):
                self.assertEqual(network._route_sequences(1,2.,required),
                                 recursive_reference(network,1,2.,required))

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
