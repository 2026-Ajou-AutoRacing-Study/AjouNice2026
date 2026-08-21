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
timing = load_module(
    "dynamic_timing",
    "dataset/morai_od/scripts/dynamic_timing_sidecar.py",
)
live_verifier = load_module(
    "dynamic_live_verifier",
    "dataset/morai_od/scripts/verify_dynamic_scene_live.py",
)
v2_batch = load_module(
    "temporal_v2_batch",
    "dataset/morai_od/scripts/generate_temporal_sequence_v2_batch.py",
)
benchmark_builder = load_module(
    "benchmark_scenario_builder",
    "dataset/morai_od/scripts/build_raw_benchmark_scenarios.py",
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
    def test_benchmark_scenarios_keep_only_mission_spawners(self):
        template = generator.load_json(
            ROOT / "configs/scenario/2026_molit_comp_sample_scene.json"
        )
        expected = {
            "roundabout": {4},
            "highway_merge": {6, 8, 10},
        }
        for mission, expected_ids in expected.items():
            scenario = benchmark_builder.build_scenario(template, mission)
            benchmark_builder.validate_scenario(scenario, mission)
            self.assertEqual(
                {item["UNIQUEID"] for item in scenario["spawnPointList"]},
                expected_ids,
            )
            self.assertEqual(scenario["egoVehicle"]["beforeControlMode"], 0)
            self.assertEqual(scenario["egoVehicle"]["currentControlMode"], 0)
            self.assertEqual(scenario["egoVehicle"]["initPosition"]["gear"], 1)

    def test_timing_sidecar_binds_nearest_samples(self):
        histories = {
            "clock": [
                {
                    "sample_id": "clock:0",
                    "callback_monotonic": 9.9,
                    "callback_unix": 99.9,
                    "message_timestamp": 41.0,
                    "simulation_time": 41.0,
                },
                {
                    "sample_id": "clock:1",
                    "callback_monotonic": 10.1,
                    "callback_unix": 100.1,
                    "message_timestamp": 41.2,
                    "simulation_time": 41.2,
                },
            ],
            "lidar": [],
        }
        triggers = [{"triggered_monotonic": 10.04}]
        timing.bind_trigger_timing(triggers, histories)
        reference = triggers[0]["timing_samples"]["clock"]
        self.assertEqual(reference["sample_id"], "clock:1")
        self.assertAlmostEqual(reference["offset_from_trigger_sec"], 0.06)
        self.assertIsNone(triggers[0]["timing_samples"]["lidar"])

    def test_timing_window_keeps_only_clip_context(self):
        samples = [
            {
                "sample_id": "ego:{}".format(index),
                "callback_monotonic": value,
                "callback_unix": value,
            }
            for index, value in enumerate((1.0, 1.8, 2.0, 3.2, 4.0))
        ]
        selected = timing.samples_in_window(samples, 2.0, 3.0, 0.2)
        self.assertEqual(
            [item["sample_id"] for item in selected],
            ["ego:1", "ego:2", "ego:3"],
        )

    def test_timing_summary_marks_missing_capture_mode_topics(self):
        summary = timing.timing_summary(
            {"clock": [{"message_timestamp": 3.0}], "lidar": []},
            {"clock": "/clock", "lidar": "/sensor/lidar1"},
        )
        self.assertTrue(summary["clock"]["available"])
        self.assertFalse(summary["lidar"]["available"])
        self.assertEqual(summary["lidar"]["sample_count"], 0)

    def test_stable_ego_gate_requires_full_low_acceleration_window(self):
        samples = [
            {
                "callback_monotonic": value,
                "state": {
                    "acceleration_raw": {"x": 0.4, "y": 0.1, "z": 0.0}
                },
            }
            for value in (1.0, 1.25, 1.5)
        ]
        self.assertTrue(timing.stable_ego_motion(samples, 0.5, 1.0))
        samples[-1]["state"]["acceleration_raw"]["x"] = 1.1
        self.assertFalse(timing.stable_ego_motion(samples, 0.5, 1.0))
        self.assertFalse(timing.stable_ego_motion(samples[:2], 0.5, 1.0))

    def test_stable_ego_gate_ignores_steady_turn_lateral_acceleration(self):
        samples = [
            {
                "callback_monotonic": value,
                "state": {
                    "velocity_raw": {"x": 5.0, "y": 0.0, "z": 0.0},
                    "acceleration_raw": {"x": 0.1, "y": 2.0, "z": 0.0},
                },
            }
            for value in (1.0, 1.25, 1.5)
        ]
        self.assertTrue(timing.stable_ego_motion(samples, 0.5, 1.0))
        samples[-1]["state"]["acceleration_raw"]["x"] = 1.1
        self.assertFalse(timing.stable_ego_motion(samples, 0.5, 1.0))

    def test_pedestrian_waypoint_coordinate_round_trip(self):
        world = {"x": -56.744, "y": -11.614, "z": 28.561}
        internal = generator.world_to_morai_waypoint(world)
        restored = generator.waypoint_to_world(internal)
        for axis in ("x", "y", "z"):
            self.assertAlmostEqual(world[axis], restored[axis], places=6)

    def test_temporal_pedestrian_crosswalk_quota_is_location_diverse(self):
        config = generator.load_json(
            ROOT
            / "dataset/morai_od/configs/temporal_sequence_capture_v2_pilot304.json"
        )
        family = next(
            row for row in config["families"]
            if row["name"] == "crosswalk_pedestrian"
        )
        locations = family["crosswalk_locations"]
        counts = {row["crosswalk_id"]: 0 for row in locations}
        for index in range(156):
            selected = generator.weighted_variant_choice(locations, index, 156)
            counts[selected["crosswalk_id"]] += 1
        self.assertEqual(
            counts,
            {
                "CW000000": 39,
                "CW000002": 39,
                "CW000013": 23,
                "CW000015": 24,
                "CW000001": 15,
                "CW000014": 16,
            },
        )
        self.assertAlmostEqual(
            sum(
                row["weight"] for row in locations
                if row["crosswalk_id"] != "CW000000"
            ),
            0.75,
        )

    def test_pedestrian_smoke_covers_every_configured_crosswalk(self):
        config = generator.load_json(
            ROOT
            / "dataset/morai_od/configs/temporal_sequence_capture_v2_pilot304.json"
        )
        family = next(
            row for row in config["families"]
            if row["name"] == "crosswalk_pedestrian"
        )
        self.assertTrue(family["cover_all_crosswalks_when_possible"])
        selected = []
        for index in range(family["count"]):
            if family["count"] <= len(family["crosswalk_locations"]):
                location = family["crosswalk_locations"][
                    index % len(family["crosswalk_locations"])
                ]
            else:
                location = generator.weighted_variant_choice(
                    family["crosswalk_locations"], index, family["count"]
                )
            selected.append(location["crosswalk_id"])
        self.assertEqual(
            set(selected),
            {row["crosswalk_id"] for row in family["crosswalk_locations"]},
        )

    def test_unverified_pedestrian_occlusion_profiles_are_disabled(self):
        config = generator.load_json(
            ROOT
            / "dataset/morai_od/configs/temporal_sequence_capture_v2_pilot304.json"
        )
        family = next(
            row for row in config["families"]
            if row["name"] == "crosswalk_pedestrian"
        )
        weights = {
            row["name"]: float(row["weight"])
            for row in family["motion_profiles"]
        }
        self.assertEqual(weights["occluded_runout"], 0.0)
        self.assertEqual(weights["partial_visibility_transition"], 0.0)
        self.assertAlmostEqual(
            weights["normal_visible"] + weights["fast_crossing"],
            1.0,
        )

    def test_generated_crosswalk_path_extends_beyond_polygon(self):
        catalog = generator.load_crosswalk_catalog(
            ROOT / "kcity_map/mgeo_map_scenario_runner_2025"
        )
        center, start, end = generator.crosswalk_path_endpoints(
            catalog["CW000013"], 4.0
        )
        self.assertGreater(math.hypot(end["x"] - start["x"], end["y"] - start["y"]), 10.0)
        self.assertLess(
            math.hypot(
                (start["x"] + end["x"]) / 2.0 - center["x"],
                (start["y"] + end["y"]) / 2.0 - center["y"],
            ),
            1e-6,
        )

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

    def test_generated_pedestrian_activation_is_deferred_safely(self):
        scenario = {
            "egoVehicle": {
                "initPosition": {"pos": {"x": 0.0, "y": 0.0}}
            },
            "pedestrianList": [
                {
                    "pos": {"x": 8.0, "y": 6.0},
                    "activeDistance": 20.0,
                }
            ],
        }
        runtime = [{"activation_policy": "deferred_ego_approach"}]
        route = [
            {"x": 0.0, "y": 0.0},
            {"x": 10.0, "y": 0.0},
        ]
        distance = generator.configure_pedestrian_activation(
            scenario, runtime, route, deferred_preroll_m=1.5
        )
        self.assertAlmostEqual(distance, 10.0)
        self.assertAlmostEqual(
            scenario["pedestrianList"][0]["activeDistance"], 8.5
        )
        self.assertAlmostEqual(runtime[0]["activation_preroll_m"], 1.5)
        self.assertAlmostEqual(runtime[0]["route_clearance_to_actor_m"], 6.0)

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

    def test_frame_transforms_capture_consecutive_ego_motion(self):
        frames = [
            {
                "ego": {
                    "position_m": {"x": 10.0, "y": 20.0, "z": 1.0},
                    "heading_deg": 90.0,
                }
            },
            {
                "ego": {
                    "position_m": {"x": 10.0, "y": 22.0, "z": 1.0},
                    "heading_deg": 90.0,
                }
            },
        ]
        metadata.enrich_frame_transforms(frames)
        first = frames[0]["transforms"]
        second = frames[1]["transforms"]
        self.assertIsNone(first["previous_ego_to_current_ego"])
        previous_in_current = second["previous_ego_to_current_ego"]
        self.assertAlmostEqual(previous_in_current[0][3], -2.0, places=6)
        self.assertAlmostEqual(previous_in_current[1][3], 0.0, places=6)
        current_in_first = second["current_ego_to_first_ego"]
        self.assertAlmostEqual(current_in_first[0][3], 2.0, places=6)
        self.assertAlmostEqual(current_in_first[1][3], 0.0, places=6)

    def test_normalized_sensor_calibration_is_invertible(self):
        calibration = metadata.normalized_sensor_calibration(
            ROOT / "configs/sensors/Sensor_6.json"
        )
        self.assertEqual(calibration["schema_version"], 1)
        self.assertEqual(set(calibration["cameras"]), {
            "Camera_front",
            "Camera_front_left",
            "Camera_front_right",
            "Camera_back",
        })
        front = calibration["cameras"]["Camera_front"]
        self.assertEqual(front["resolution_px"], [1280, 720])
        self.assertAlmostEqual(front["intrinsic"][0][0], 640.0, places=6)
        product = metadata._matrix_multiply(
            front["sensor_to_ego"], front["ego_to_sensor"]
        )
        for row in range(4):
            for column in range(4):
                self.assertAlmostEqual(
                    product[row][column],
                    1.0 if row == column else 0.0,
                    places=6,
                )

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

    def test_lane_change_route_connects_adjacent_mgeo_centerlines(self):
        links = generator.load_json(
            ROOT / "kcity_map/mgeo_map_scenario_runner_2025/link_set.json"
        )
        links_by_id = {item["idx"]: item for item in links}
        source, source_arc = generator.route_from_link_path(
            links_by_id, generator.HIGHWAY_LANE_LINK_PATHS[6]
        )
        target, target_arc = generator.route_from_link_path(
            links_by_id, generator.HIGHWAY_LANE_LINK_PATHS[8]
        )
        route = generator.build_lane_change_route(
            source,
            source_arc,
            target,
            target_arc,
            start_arc_m=90.0,
            transition_length_m=50.0,
            pre_length_m=25.0,
            post_length_m=30.0,
        )
        self.assertGreater(len(route), 150)
        self.assertLess(
            min(
                math.hypot(
                    route[0]["x"] - point["x"],
                    route[0]["y"] - point["y"],
                )
                for point in source
            ),
            0.2,
        )
        self.assertLess(
            min(
                math.hypot(
                    route[-1]["x"] - point["x"],
                    route[-1]["y"] - point["y"],
                )
                for point in target
            ),
            0.2,
        )
        headings = []
        for before, after in zip(route, route[1:]):
            headings.append(
                math.atan2(after["y"] - before["y"], after["x"] - before["x"])
            )
        heading_jumps = [
            abs(math.atan2(math.sin(b - a), math.cos(b - a)))
            for a, b in zip(headings, headings[1:])
        ]
        self.assertLess(max(heading_jumps), math.radians(3.0))

    def test_temporal_v2_pilot_is_38_eight_frame_clips(self):
        config = generator.load_json(
            ROOT
            / "dataset/morai_od/configs/temporal_sequence_capture_v2_pilot304.json"
        )
        self.assertEqual(config["capture"]["captures_per_scene"], 8)
        self.assertAlmostEqual(
            config["capture"]["capture_period_sec"], 1.0 / 12.0
        )
        self.assertEqual(
            sum(int(family["count"]) for family in config["families"]),
            38,
        )
        self.assertEqual(38 * 8, 304)

    def test_temporal_v2_primary_batch_rounds_to_1504_frames(self):
        self.assertEqual(sum(v2_batch.PRIMARY_COUNTS.values()), 188)
        self.assertEqual(sum(v2_batch.PRIMARY_COUNTS.values()) * 8, 1504)
        self.assertEqual(sum(v2_batch.RESERVE_COUNTS.values()), 28)
        self.assertEqual(
            v2_batch.grouped_counts(v2_batch.PRIMARY_COUNTS),
            {
                "mission_dynamic": 93,
                "static_temporal": 66,
                "hard_transition": 29,
            },
        )
        self.assertEqual(
            v2_batch.grouped_counts(v2_batch.RESERVE_COUNTS),
            {
                "mission_dynamic": 18,
                "static_temporal": 7,
                "hard_transition": 3,
            },
        )
        self.assertEqual(
            v2_batch.PRIMARY_COUNTS["highway_cruise_overtake"], 32
        )
        self.assertEqual(
            v2_batch.PRIMARY_COUNTS["highway_lane_change_overtake"], 4
        )

    def test_highway_cruise_lane_prior_is_twenty_nine_three(self):
        _base, defaults = v2_batch.family_defaults()
        family = defaults["highway_cruise_overtake"]
        family["count"] = 32
        lanes = generator.weighted_lane_schedule(family)
        self.assertEqual(lanes.count(6), 20)
        self.assertEqual(lanes.count(8), 9)
        self.assertEqual(lanes.count(10), 3)
        self.assertEqual(
            generator.ego_lane_speed_range(family, 6), (35.0, 50.0)
        )
        self.assertEqual(
            generator.ego_lane_speed_range(family, 8), (45.0, 55.0)
        )
        self.assertEqual(
            generator.ego_lane_speed_range(family, 10), (50.0, 60.0)
        )

    def test_static_highway_lane_quota_rounds_to_five_two_one(self):
        lanes = [
            generator.static_highway_lane_for_variant(index, 8)
            for index in range(8)
        ]
        self.assertEqual(lanes.count(6), 5)
        self.assertEqual(lanes.count(8), 2)
        self.assertEqual(lanes.count(10), 1)

    def test_weighted_obstacle_deck_matches_documented_quota(self):
        subtypes = (
            ["box"] * 2
            + ["barrel_or_drum"] * 4
            + ["barrier"] * 9
            + ["unoccupied_bicycle_or_scooter"] * 3
            + ["road_sign"] * 10
            + ["fixed_roadside_object", "road_surface"]
        )
        rows = [
            {"data_id": index + 1, "name": "asset{}".format(index), "subtype": subtype}
            for index, subtype in enumerate(subtypes)
        ]
        deck = generator.build_weighted_obstacle_deck(rows)
        self.assertEqual(len(deck), 1415)
        counts = {}
        for row in deck:
            counts[row["data_id"]] = counts.get(row["data_id"], 0) + 1
        self.assertEqual(
            counts,
            {
                row["data_id"]: generator.obstacle_unique_quota(row)
                for row in rows
            },
        )
        self.assertEqual(len({row["data_id"] for row in deck[:215]}), 30)

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

    def test_highway_ego_load_speed_is_below_capture_target(self):
        config = generator.load_json(
            ROOT / "dataset/morai_od/configs/dynamic_object_capture_v1.json"
        )
        for name in (
            "highway_merge_sample_spawn",
            "highway_merge_cruise",
            "highway_cruise_overtake",
        ):
            family = next(item for item in config["families"] if item["name"] == name)
            self.assertIn("ego_initial_speed_kph", family)
            self.assertLessEqual(max(family["ego_initial_speed_kph"]), 20.0)

    def test_dynamic_capture_primes_writer_before_training_clip(self):
        config = generator.load_json(
            ROOT / "dataset/morai_od/configs/dynamic_object_capture_v1.json"
        )
        self.assertTrue(config["capture"]["prime_capture_before_clip"])

    def test_temporal_v2_loads_ego_stationary_before_external_control(self):
        config = generator.load_json(
            ROOT
            / "dataset/morai_od/configs/temporal_sequence_capture_v2_pilot304.json"
        )
        self.assertEqual(
            config["capture"]["scenario_load_initial_speed_kph"],
            0.0,
        )


if __name__ == "__main__":
    unittest.main()
