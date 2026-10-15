import copy
import math
from pathlib import Path
import struct
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest import mock

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import prepare_independent_vehicle_inputs as prepare


def status(uid, x, name="model"):
    return NS(unique_id=uid, position=NS(x=x, y=2., z=3.), name=name)


def frame(*statuses):
    return NS(npc_list=list(statuses), pedestrian_list=[], obstacle_list=[])


def segment(key, rows):
    return NS(key=key, class_name="vehicle", points=[NS(timestamp=t, x=x, y=2., z=3.) for t, x in rows])


class IndependentInputsTests(unittest.TestCase):
    def test_camera_excludes_future_receipt_but_not_already_received_future_header(self):
        rows = [(10.001, 10.10, "not_yet_received"), (9.95, 10.04, "past"),
                (10.02, 10.045, "future_header_received")]
        self.assertEqual(prepare.causal_camera(rows, 10., 10.05)[2], "future_header_received")
        self.assertEqual(prepare.causal_camera(rows, 10., 10.044)[2], "past")
        self.assertIsNone(prepare.causal_camera(rows, 10., 9.9))

    def test_camera_tolerance_and_deterministic_equal_distance_choice(self):
        rows = [(9.9, 10., "before"), (10.1, 10., "after")]
        self.assertEqual(prepare.causal_camera(rows, 10., 10., .1)[2], "before")
        self.assertIsNone(prepare.causal_camera(rows, 10., 10., .05))

    def test_bracketed_moving_pose_and_yaw_wrap(self):
        rows = [(1., dict(x=0., y=2., z=3., yaw=math.radians(179))),
                (1.1, dict(x=4., y=6., z=7., yaw=math.radians(-179)))]
        midpoint = prepare.bracket_pose(rows, 1.05)
        self.assertAlmostEqual(midpoint["x"], 2.)
        self.assertAlmostEqual(midpoint["z"], 5.)
        self.assertAlmostEqual(abs(midpoint["yaw"]), math.pi)
        self.assertEqual(prepare.bracket_pose(rows, 1.), rows[0][1])

    def test_no_pose_extrapolation_or_large_gap(self):
        rows = [(1., dict(x=0., y=0., z=0., yaw=0.)), (1.3, dict(x=3., y=0., z=0., yaw=0.))]
        self.assertIsNone(prepare.bracket_pose(rows, .9))
        self.assertIsNone(prepare.bracket_pose(rows, 1.4))
        self.assertIsNone(prepare.bracket_pose(rows, 1.1))
        self.assertIsNone(prepare.bracket_pose([], 1.))

    def test_local_bottom_and_world_gravity_roundtrip_with_moving_rotated_ego(self):
        ego = dict(x=10., y=20., z=5., yaw=math.pi/2)
        alignment = dict(x=100., y=200., z=30., yaw=-.2)
        point = NS(x=12., y=24., z=8., yaw=.7, length=4., width=2., height=2.)
        translation = np.asarray([1., 0., 2.])
        bottom = prepare.local_bottom_box(point, ego, translation)
        self.assertTrue(np.allclose(bottom[:3], [3., -2., 0.]))
        world_ego = prepare.transform_pose(ego, alignment)
        sensor = np.eye(4)
        sensor[:3, 3] = translation
        world = prepare.world_gravity_box(bottom, prepare.pose_matrix(world_ego) @ sensor)
        expected = prepare.transform_pose(dict(x=point.x, y=point.y, z=point.z, yaw=point.yaw), alignment)
        self.assertTrue(np.allclose(world[:3], [expected[k] for k in "xyz"]))
        self.assertAlmostEqual(world[6], expected["yaw"])

    def test_support_counts_gravity_not_bottom_and_does_not_change_points(self):
        points = np.asarray([[0., 0., 1., 255.], [0., 0., 0., 10.], [2., 0., 1., 90.]])
        saved = points.copy()
        box = [0., 0., 0., 1., 1., 2., 0.]
        self.assertEqual(prepare.count_support(points, box), 2)
        self.assertTrue(np.array_equal(points, saved))
        self.assertEqual(box[2], 0.)

    def test_continuity_preserves_initial_stopped_history_and_reports_interior_gaps(self):
        rows = [dict(raw_lidar_index=i, selected=True) for i in range(5)]
        report = prepare.continuity_report(rows, 1)
        self.assertTrue(report["continuous_for_tracker"])
        self.assertFalse(report["initial_nonmoving_excluded"])
        rows[2]["selected"] = False
        self.assertEqual(prepare.continuity_report(rows, 1)["interior_unselected_raw_indices"], [2])
        self.assertFalse(prepare.continuity_report(rows, 1)["continuous_for_tracker"])
        self.assertFalse(prepare.continuity_report(rows, 2)["continuous_for_tracker"])

    def test_edge_unavailable_is_explicit_not_arbitrary_trim(self):
        rows = [dict(raw_lidar_index=i, selected=i in (1, 2, 3)) for i in range(5)]
        report = prepare.continuity_report(rows, 1)
        self.assertEqual(report["prefix_unselected"], 1)
        self.assertEqual(report["suffix_unselected"], 1)
        self.assertTrue(report["continuous_for_tracker"])

    def test_raw_uid_reuse_increments_generation_not_same_physical_actor(self):
        rows = [(1., 1.01, frame(status(7, 0.))), (1.1, 1.11, frame(status(7, 1.))),
                (2., 2.01, frame(status(7, 50.)))]
        segments = [segment("vehicle:reconstructed:1", [(1., 0.), (1.1, 1.)]),
                    segment("vehicle:reconstructed:2", [(2., 50.)])]
        lineage = prepare.raw_uid_lineage(rows, segments)
        self.assertEqual([row["raw_uid_generation"] for row in lineage[segments[0].key]], [1, 1])
        self.assertEqual(lineage[segments[1].key][0]["raw_uid_generation"], 2)

    def test_physical_actor_raw_uid_change_keeps_reconstructed_actor(self):
        rows = [(1., 1.01, frame(status(7, 0.))), (1.1, 1.11, frame(status(9, 1.)))]
        actor = segment("vehicle:reconstructed:1", [(1., 0.), (1.1, 1.)])
        lineage = prepare.raw_uid_lineage(rows, [actor])[actor.key]
        self.assertEqual([row["raw_uid"] for row in lineage], [7, 9])
        self.assertEqual({row["reconstructed_key"] for row in lineage}, {actor.key})

    def test_duplicate_raw_stamp_uses_most_complete_frame(self):
        rows = [(1., 1.01, frame()), (1., 1.02, frame(status(7, 0.)))]
        actor = segment("vehicle:reconstructed:1", [(1., 0.)])
        self.assertEqual(len(prepare.raw_uid_lineage(rows, [actor])[actor.key]), 1)

    def test_ambiguous_or_missing_raw_identity_fails_closed(self):
        rows = [(1., 1.01, frame(status(7, 0.)))]
        with self.assertRaises(ValueError):
            prepare.raw_uid_lineage(rows, [segment("a", [(1., 0.)]), segment("b", [(1., 0.)])])
        with self.assertRaises(ValueError):
            prepare.raw_uid_lineage(rows, [segment("a", [(1., 10.)])])

    def test_recorded_tf_evidence_map_alignment_is_separate_from_operational_yaw(self):
        def transform(parent, child, x, yaw, stamp):
            return NS(header=NS(frame_id=parent, stamp=NS(to_sec=lambda: stamp)), child_frame_id=child,
                      transform=NS(translation=NS(x=x, y=0., z=0.),
                                   rotation=NS(x=0., y=0., z=math.sin(yaw/2), w=math.cos(yaw/2))))
        fake = mock.MagicMock()
        fake.__enter__.return_value.read_messages.return_value = [
            ("/tf_static", NS(transforms=[transform("map", "morai_map", 100., .2, 0.)]), None),
            ("/tf", NS(transforms=[transform("morai_map", "base_link", 10., .3, 1.)]), None)]
        with mock.patch.dict(sys.modules, {"rosbag": NS(Bag=lambda _: fake)}):
            alignment, dynamic = prepare.read_tf_evidence("unused")
        self.assertAlmostEqual(alignment["yaw"], .2)
        self.assertAlmostEqual(dynamic[0][1]["yaw"], .5)
        self.assertAlmostEqual(dynamic[0][1]["x"], 100+10*math.cos(.2))

    def test_tf_alignment_required_no_guess(self):
        fake = mock.MagicMock()
        fake.__enter__.return_value.read_messages.return_value = []
        with mock.patch.dict(sys.modules, {"rosbag": NS(Bag=lambda _: fake)}), self.assertRaises(ValueError):
            prepare.read_tf_evidence("unused")

    def test_source_hash_change_is_rejected_before_preparation(self):
        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp)
            bag = folder / "raw.bag"
            bag.write_bytes(b"raw")
            (folder / "scenario.json").write_text("{}")
            case = dict(bag=str(bag), bag_sha256=prepare.digest(bag), folder=str(folder))
            self.assertEqual(prepare.validate_case_source(case)[0], bag)
            bag.write_bytes(b"mutated")
            with self.assertRaises(ValueError):
                prepare.validate_case_source(case)

    def test_full_gt_future_outside_model_roi_is_preserved(self):
        key = "vehicle:reconstructed:1"
        actor = NS(key=key, asset_name="model", class_name="vehicle", points=[
            NS(timestamp=1., x=5., y=0., z=1., yaw=0., length=4., width=2., height=2.),
            NS(timestamp=4., x=150., y=50., z=1., yaw=0., length=4., width=2., height=2.)])
        result = prepare.full_gt_case("independent", [actor], {key: [dict(raw_uid=7, raw_uid_generation=2)]},
                                     dict(x=100., y=200., z=0., yaw=0.), {})
        series = result["gt_trajectories"][0]["series"]
        self.assertEqual(series[-1], [4., [250., 250., 1.]])
        self.assertEqual(result["gt"][0][1][-1]["key"], "independent|"+key)
        self.assertEqual(result["gt_trajectories"][0]["raw_uid_lineage"][0]["raw_uid_generation"], 2)
        self.assertFalse(result["provenance"]["model_roi_filter_applied"])

    def test_unknown_actor_near_roi_excludes_scoring_but_far_actor_does_not(self):
        actor = segment("unknown", [(1., 0.)])
        ego = dict(x=0., y=0., z=0., yaw=0.)
        interpolation = lambda points, stamp, gap: points[0]
        self.assertEqual(len(prepare.unresolved_nearby([actor], set(), ego, 1., interpolation, .15)), 1)
        ego["x"] = 106.
        self.assertEqual(prepare.unresolved_nearby([actor], set(), ego, 1., interpolation, .15), [])
        ego["x"] = 0.
        self.assertEqual(prepare.unresolved_nearby([actor], {"unknown"}, ego, 1., interpolation, .15), [])
        self.assertEqual(prepare.unresolved_nearby([actor], set(), ego, 1., lambda *_: None, .15)[0]["reason"],
                         "unbracketed_unknown_actor")

    def test_full_gt_epoch_filter_excludes_previous_scenario_without_losing_loaded_future(self):
        key = "vehicle:reconstructed:1"
        actor = NS(key=key, asset_name="model", class_name="vehicle", points=[
            NS(timestamp=t, x=t, y=0., z=1., yaw=0., length=4., width=2., height=2.) for t in (1., 2., 3.)])
        lineage = {key: [dict(stamp=t, raw_uid=7, raw_uid_generation=1) for t in (1., 2., 3.)]}
        result = prepare.full_gt_case("independent", [actor], lineage, dict(x=0., y=0., z=0., yaw=0.), {}, 2.)
        self.assertEqual([row[0] for row in result["gt_trajectories"][0]["series"]], [2., 3.])
        self.assertEqual([row["stamp"] for row in result["gt_trajectories"][0]["raw_uid_lineage"]], [2., 3.])

    def test_scene_lifecycle_boundary_is_required_and_explicit(self):
        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp)
            with self.assertRaises(ValueError):
                prepare.read_case_events(folder)
            events = [dict(event=name, time="2026-10-14T00:00:0"+str(i)+"+00:00")
                      for i, name in enumerate(("SCENARIO_READY", "CONTROLLER_START", "RUN_FINISHED"))]
            prepare.save(folder / "events.jsonl", {})
            (folder / "events.jsonl").write_text("\n".join(__import__('json').dumps(row) for row in events))
            evidence = prepare.read_case_events(folder)
            self.assertLess(evidence["scene_ready_stamp"], evidence["controller_start_stamp"])
            events.reverse()
            events[0]["time"] = "2026-10-13T23:00:00+00:00"
            (folder / "events.jsonl").write_text("\n".join(__import__('json').dumps(row) for row in events))
            with self.assertRaises(ValueError):
                prepare.read_case_events(folder)


if __name__ == "__main__":
    unittest.main()
