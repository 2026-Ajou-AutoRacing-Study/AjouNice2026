#!/usr/bin/env python3

import importlib.util
import copy
import math
import random
import unittest
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[3]


def load_module(name, relative):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


generator = load_module(
    "dynamic_generator",
    "dataset/morai_od/scripts/generate_dynamic_object_capture_set.py",
)
metadata = load_module(
    "dynamic_metadata",
    "dataset/morai_od/scripts/dynamic_capture_metadata.py",
)
live_verifier = load_module(
    "dynamic_live_verifier",
    "dataset/morai_od/scripts/verify_dynamic_scene_live.py",
)


def vector(x, y, z=0.0):
    return SimpleNamespace(x=x, y=y, z=z)


def actor(unique_id, x, timestamp):
    return SimpleNamespace(
        unique_id=unique_id,
        type=1,
        name="vehicle",
        heading=0.0,
        velocity=vector(2.0, 0.0),
        acceleration=vector(0.0, 0.0),
        size=vector(4.5, 1.9, 1.7),
        position=vector(x, 0.0),
        timestamp=timestamp,
    )


class DynamicCaptureToolsTest(unittest.TestCase):
    def test_pedestrian_waypoint_coordinate_round_trip(self):
        world = {"x": -56.744, "y": -11.614, "z": 28.561}
        internal = generator.world_to_morai_waypoint(world)
        restored = generator.waypoint_to_world(internal)
        for axis in ("x", "y", "z"):
            self.assertAlmostEqual(world[axis], restored[axis], places=6)

    def test_route_pose_before_moves_back_in_arc_length(self):
        route = [
            {"x": float(index), "y": 0.0, "z": 0.0}
            for index in range(21)
        ]
        arc = [float(index) for index in range(21)]
        pose = generator.route_pose_before(
            route, arc, {"x": 15.0, "y": 0.0}, 5.0
        )
        self.assertAlmostEqual(pose["x"], 10.0)
        self.assertAlmostEqual(pose["yaw"], 0.0)

    def test_pedestrian_activation_includes_generated_ego_pose(self):
        scenario = {
            "egoVehicle": {
                "initPosition": {"pos": {"x": 0.0, "y": 0.0}}
            },
            "pedestrianList": [
                {
                    "pos": {"x": 3.0, "y": 4.0},
                    "activeDistance": 2.0,
                }
            ],
        }
        runtime = [{}]
        distance = generator.arm_pedestrian_before_capture(
            scenario, runtime, 5.0
        )
        self.assertAlmostEqual(distance, 5.0)
        self.assertAlmostEqual(
            scenario["pedestrianList"][0]["activeDistance"], 10.0
        )
        self.assertAlmostEqual(runtime[0]["activation_margin_m"], 5.0)

    def test_smoothed_velocity_uses_position_difference_in_mps(self):
        records = []
        for index in range(31):
            timestamp = index * 0.02
            value = actor(7, 2.0 * timestamp, timestamp)
            message = SimpleNamespace(
                num_of_npcs=1,
                num_of_pedestrian=0,
                num_of_obstacle=0,
                npc_list=[value],
                pedestrian_list=[],
                obstacle_list=[],
            )
            serialized = metadata.serialize_objects(message)
            serialized["timestamp"] = timestamp
            records.append(
                {
                    "triggered_unix": timestamp,
                    "ego": None,
                    "objects": serialized,
                }
            )
        metadata.enrich_derived_motion(records)
        middle = records[15]["objects"]["actors"][0]
        self.assertAlmostEqual(middle["derived_speed_mps"], 2.0, places=6)
        self.assertAlmostEqual(
            middle["derived_longitudinal_acceleration_mps2"], 0.0, places=6
        )
        self.assertEqual(middle["motion_phase"], "cruising")

    def test_motion_phase_separates_turning_from_stationary(self):
        self.assertEqual(metadata.motion_phase(0.0, 0.0, 20.0), "stationary")
        self.assertEqual(metadata.motion_phase(3.0, 0.0, 20.0), "turning")

    def test_temporal_split_is_stratified_within_each_family(self):
        values = [generator.split_for_variant(index, 6) for index in range(6)]
        self.assertEqual(values.count("train"), 4)
        self.assertEqual(values.count("val"), 1)
        self.assertEqual(values.count("test"), 1)

    def test_actor_relative_pose_and_velocity_are_ego_aligned(self):
        frames = [
            {
                "triggered_unix": 0.0,
                "ego": {
                    "timestamp": 0.0,
                    "position_m": {"x": 1.0, "y": 2.0, "z": 0.0},
                    "heading_deg": 90.0,
                },
                "objects": {
                    "timestamp": 0.0,
                    "actors": [
                        {
                            "class": "vehicle",
                            "unique_id": 9,
                            "position_m": {"x": 1.0, "y": 4.0, "z": 0.0},
                            "heading_deg": 90.0,
                            "derived_velocity_mps": None,
                        }
                    ],
                },
            }
        ]
        metadata.enrich_derived_motion(frames)
        relative = frames[0]["objects"]["actors"][0]["relative_to_ego"]
        self.assertAlmostEqual(relative["position_m"]["x"], 2.0)
        self.assertAlmostEqual(relative["position_m"]["y"], 0.0)
        self.assertAlmostEqual(relative["heading_deg"], 0.0)

    def test_highway_cruise_uses_three_lane_routes(self):
        links = generator.load_json(
            ROOT
            / "kcity_map/mgeo_map_scenario_runner_2025/link_set.json"
        )
        links_by_id = {item["idx"]: item for item in links}
        route_lengths = []
        for link_ids in generator.HIGHWAY_LANE_LINK_PATHS.values():
            route, arc = generator.route_from_link_path(
                links_by_id, link_ids
            )
            self.assertGreater(len(route), 100)
            route_lengths.append(arc[-1])
        self.assertTrue(all(length > 500.0 for length in route_lengths))

    def test_spawn_and_cruise_families_have_distinct_initial_speed(self):
        config = generator.load_json(
            ROOT / "dataset/morai_od/configs/dynamic_object_capture_v1.json"
        )
        template = generator.load_json(
            ROOT / "configs/scenario/2026_molit_comp_sample_scene.json"
        )
        families = {item["name"]: item for item in config["families"]}
        for spawn_name, cruise_name in (
            ("roundabout_sample_spawn", "roundabout_circulation"),
            ("highway_merge_sample_spawn", "highway_merge_cruise"),
        ):
            spawn_scene = copy.deepcopy(template)
            generator.clear_actor_lists(spawn_scene)
            generator.build_vehicle_scene(
                spawn_scene,
                template,
                families[spawn_name],
                random.Random(7),
                0,
            )
            cruise_scene = copy.deepcopy(template)
            generator.clear_actor_lists(cruise_scene)
            generator.build_vehicle_scene(
                cruise_scene,
                template,
                families[cruise_name],
                random.Random(7),
                0,
            )
            spawn = spawn_scene["spawnPointList"][0]
            cruise = cruise_scene["spawnPointList"][0]
            self.assertLess(
                spawn["MinSpawnVelocity_Custom"],
                spawn["MinDesiredVelocity_Custom"],
            )
            self.assertEqual(
                cruise["MinSpawnVelocity_Custom"],
                cruise["MinDesiredVelocity_Custom"],
            )

    def test_recycled_actor_id_is_split_after_observation_gap(self):
        values = [
            (0.00, 0.0, 0.0, 0.0),
            (0.02, 0.0, 0.0, 0.1),
            (2.00, 10.0, 0.0, 5.0),
            (2.02, 10.1, 0.0, 5.0),
        ]
        segments = live_verifier.split_track_segments(values)
        self.assertEqual(len(segments), 2)
        self.assertEqual([len(item) for item in segments], [2, 2])


if __name__ == "__main__":
    unittest.main()
