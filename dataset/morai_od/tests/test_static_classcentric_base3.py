#!/usr/bin/env python3

import random
import math
import json
import tempfile
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "dataset/morai_od/scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import generate_dynamic_object_capture_set as generator
import audit_morai_lidar_gt_visibility as lidar_visibility
import generate_static_classcentric_base3_batch as static_base3
import generate_static_classcentric_base3_pilot as static_base3_pilot
import generate_static_classcentric_base3_smoke as static_base3_smoke
import validate_dynamic_capture_output as capture_validator


class StaticClasscentricBase3Test(unittest.TestCase):
    def test_output_validator_accepts_base3_artifact_schema(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            artifact = root / "LIDAR_5/frame_Instance.bin"
            artifact.parent.mkdir(parents=True)
            artifact.write_bytes(b"test")
            frame = {
                "frame_index": 0,
                "triggered_unix": 1.0,
                "morai_timestamp_stem": "frame",
                "verified": True,
                "quality": {"actor_present": False},
                "new_files": ["LIDAR_5/frame_Instance.bin"],
                "observed_artifacts": {"LIDAR_5": ["_Instance.bin"]},
                "required_artifacts": {"LIDAR_5": ["_Instance.bin"]},
                "missing_artifacts": {},
                "objects": {"actors": []},
            }
            frames_path = root / "frames.jsonl"
            frames_path.write_text(json.dumps(frame) + "\n", encoding="utf-8")
            clip = {
                "clip_id": "base3_schema",
                "dataset_id": "morai_static_classcentric_base3_test",
                "frames_jsonl": "frames.jsonl",
                "frame_count": 1,
            }
            clip_path = root / "clip.json"
            clip_path.write_text(json.dumps(clip), encoding="utf-8")
            result = capture_validator.validate_clip(root, clip_path)
            self.assertEqual([], result["errors"])

    def test_lidar_evidence_tolerance_does_not_change_gt_box(self):
        points = np.asarray([[0.51, 0.0, 0.0, 1.0]], dtype=np.float32)
        box = {
            "center": np.zeros(3, dtype=np.float32),
            "size": np.ones(3, dtype=np.float32),
            "rotation_rpy": [0.0, 0.0, 0.0],
        }
        self.assertFalse(lidar_visibility.points_in_box(points, box).any())
        self.assertTrue(
            lidar_visibility.points_in_box(points, box, 0.02).any()
        )

    def test_campaign_quota_is_exact(self):
        static_base3.validate_global_quota()
        self.assertEqual(3334, static_base3.TOTAL_CLIPS)
        self.assertEqual(10002, static_base3.TOTAL_FRAMES)
        self.assertEqual(
            static_base3.GLOBAL_PRIMARY_COUNTS,
            {
                "stopped_vehicle": 1334,
                "static_obstacle": 1000,
                "standing_pedestrian": 500,
                "sparse_background": 500,
            },
        )

    def test_batches_are_nearly_equal_and_sum_to_campaign(self):
        totals = [sum(row.values()) for row in static_base3.PRIMARY_COUNTS_BY_BATCH]
        self.assertEqual([477, 477, 476, 476, 476, 476, 476], totals)
        self.assertEqual(static_base3.TOTAL_CLIPS, sum(totals))

    def test_static_capture_contract(self):
        base, rows = static_base3.static_defaults()
        capture = base["capture"]
        self.assertEqual(
            "morai_save_sensor_data_rolling_grouped", capture["format"]
        )
        self.assertEqual(3, capture["captures_per_scene"])
        self.assertEqual(3.0, capture["capture_spacing_m"])
        self.assertEqual("static_layout_clip", capture["split_unit"])
        self.assertEqual([1, 4], rows["stopped_vehicle"]["actors_per_scene"])
        self.assertTrue(
            rows["stopped_vehicle"]["controlled_same_lane_front_rear"]
        )
        self.assertTrue(rows["stopped_vehicle"]["snap_static_vehicle_to_mgeo"])
        self.assertTrue(rows["stopped_vehicle"]["same_lane_fallback_only"])
        self.assertEqual(
            [18.0, 21.0, 24.0],
            rows["stopped_vehicle"]["same_lane_front_capture_slots_m"],
        )
        self.assertEqual(
            [-22.0, -26.0, -30.0],
            rows["stopped_vehicle"][
                "same_lane_rear_capture_slots_by_subtype"
            ]["truck"],
        )
        self.assertEqual(
            0.0,
            rows["stopped_vehicle"]["opposite_heading_probability"],
        )
        self.assertEqual(
            0.70,
            rows["stopped_vehicle"]["roundabout_left_vehicle_probability"],
        )
        self.assertEqual(
            0.30,
            rows["stopped_vehicle"][
                "roundabout_second_left_vehicle_probability"
            ],
        )
        self.assertEqual(
            ["0", "1", "2"],
            [
                row["name"]
                for row in rows["stopped_vehicle"][
                    "roundabout_left_actor_count_weights"
                ]
            ],
        )
        self.assertEqual(
            1.0,
            rows["stopped_vehicle"]["highway_side_vehicle_probability"],
        )
        self.assertEqual(
            0.35,
            rows["stopped_vehicle"][
                "highway_second_side_vehicle_probability"
            ],
        )
        self.assertEqual([4, 7], rows["static_obstacle"]["objects_per_scene"])
        self.assertEqual(
            10, len(base["runtime_obstacle_asset_allowlist"])
        )
        self.assertNotIn(
            "Lane_Separation_Bar", base["runtime_obstacle_asset_allowlist"]
        )
        self.assertNotIn(
            "RedBarrel", base["runtime_obstacle_asset_allowlist"]
        )
        self.assertNotIn(
            "WoodBox", base["runtime_obstacle_asset_allowlist"]
        )
        self.assertEqual([3, 5], rows["standing_pedestrian"]["actors_per_scene"])
        self.assertTrue(rows["sparse_background"]["empty_scene"])

    def test_config_has_only_static_classcentric_families(self):
        counts = static_base3.PRIMARY_COUNTS_BY_BATCH[0]
        config = static_base3.build_config(1, "primary", counts, "test", 1000)
        families = {row["name"]: row for row in config["families"]}
        self.assertEqual(set(counts), set(families))
        self.assertTrue(config["use_runtime_vehicle_pool"])
        self.assertEqual(47, families["standing_pedestrian"]["crosswalk_scene_count"])
        self.assertEqual(
            counts["sparse_background"],
            families["sparse_background"]["empty_scene_count"],
        )
        self.assertTrue(
            all(
                row["motion_phase"] == "static_classcentric_singleframe"
                for row in families.values()
            )
        )
        self.assertEqual(
            ["roundabout", "highway_three_lane", "route_general"],
            [
                row["name"]
                for row in families["stopped_vehicle"]["spatial_strata"]
            ],
        )
        self.assertEqual(
            [0.27, 0.30, 0.35, 0.08],
            [
                row["weight"]
                for row in families["stopped_vehicle"]["roundabout_phases"]
            ],
        )

    def test_global_spatial_schedule_preserves_quota_and_balanced_highway_lanes(self):
        family = {
            **static_base3.static_defaults()[1]["stopped_vehicle"],
            "count": static_base3.GLOBAL_PRIMARY_COUNTS["stopped_vehicle"],
        }
        strata = generator.weighted_zone_schedule(family, key="spatial_strata")
        self.assertEqual(1334, len(strata))
        self.assertEqual(267, strata.count("roundabout"))
        self.assertEqual(289, strata.count("highway_three_lane"))
        lane_schedule = generator.weighted_lane_schedule(
            {
                "count": strata.count("highway_three_lane"),
                "source_spawn_ids": [6, 8, 10],
                "ego_lane_weights": {"6": 1, "8": 1, "10": 1},
            }
        )
        lane_counts = {lane: lane_schedule.count(lane) for lane in (6, 8, 10)}
        self.assertEqual(289, sum(lane_counts.values()))
        self.assertLessEqual(max(lane_counts.values()) - min(lane_counts.values()), 1)

    def test_integer_count_range_is_inclusive_and_deterministic(self):
        first = random.Random(7)
        second = random.Random(7)
        a = [generator.integer_count(first, [3, 5]) for _ in range(30)]
        b = [generator.integer_count(second, [3, 5]) for _ in range(30)]
        self.assertEqual(a, b)
        self.assertEqual({3, 4, 5}, set(a))

    def test_route_pose_before_interpolates_sparse_mgeo_vertices(self):
        route = [
            {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "link_idx": "A", "point_idx": 0},
            {"x": 20.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "link_idx": "A", "point_idx": 1},
        ]
        pose = generator.route_pose_before(
            route,
            [0.0, 20.0],
            {"x": 20.0, "y": 0.0, "route_arc_m": 20.0},
            7.0,
        )
        self.assertAlmostEqual(13.0, pose["x"])
        self.assertAlmostEqual(0.0, pose["y"])

    def test_highway_other_lane_schedule_covers_near_far_and_both_sides(self):
        lane6_single = [
            generator.highway_other_lane_order(6, 1, index)[0]
            for index in range(10)
        ]
        lane10_single = [
            generator.highway_other_lane_order(10, 1, index)[0]
            for index in range(10)
        ]
        lane8_single = [
            generator.highway_other_lane_order(8, 1, index)[0]
            for index in range(10)
        ]
        self.assertEqual(7, lane6_single.count(8))
        self.assertEqual(3, lane6_single.count(10))
        self.assertEqual(7, lane10_single.count(8))
        self.assertEqual(3, lane10_single.count(6))
        self.assertEqual({6, 10}, set(lane8_single))
        self.assertEqual([8, 10], generator.highway_other_lane_order(6, 2, 0))
        self.assertEqual([8, 6], generator.highway_other_lane_order(10, 2, 0))
        self.assertEqual([6, 10], generator.highway_other_lane_order(8, 2, 0))
        self.assertEqual([10, 6], generator.highway_other_lane_order(8, 2, 1))

    def test_roundabout_actor_count_schedule_is_exact_and_interleaved(self):
        family = static_base3.static_defaults()[1]["stopped_vehicle"]
        counts = []
        for total in (6, 6, 8):
            schedule = [
                generator.roundabout_left_vehicle_count(
                    family,
                    {
                        "roundabout_phase_total": total,
                        "roundabout_phase_within": index,
                    },
                    random.Random(index),
                )
                for index in range(total)
            ]
            counts.extend(schedule)
            self.assertLessEqual(max(schedule.count(value) for value in (0, 1, 2)), total)
        self.assertEqual(6, counts.count(0))
        self.assertEqual(10, counts.count(1))
        self.assertEqual(4, counts.count(2))

    def test_vehicle_subtype_safety_policy(self):
        family = static_base3.static_defaults()[1]["stopped_vehicle"]
        self.assertEqual(
            5.0,
            generator.static_vehicle_heading_jitter_deg(
                family, "passenger_car"
            ),
        )
        self.assertEqual(
            1.0,
            generator.static_vehicle_heading_jitter_deg(family, "truck"),
        )
        self.assertEqual(
            1.0,
            generator.static_vehicle_heading_jitter_deg(
                family, "passenger_car", "roundabout_left_legal_path"
            ),
        )
        self.assertEqual(
            [22.0, 25.0, 28.0],
            generator.controlled_same_lane_capture_slots(
                family, "suv", "front"
            ),
        )
        self.assertEqual(
            [-16.0, -20.0, -24.0],
            generator.controlled_same_lane_capture_slots(
                family, "suv", "rear"
            ),
        )
        self.assertEqual(
            [0, 1] * 5,
            [generator.same_lane_rear_count(family, index) for index in range(10)],
        )
        self.assertEqual(
            16.0,
            generator.minimum_final_capture_separation_m(family, "bus_van"),
        )

    def test_large_vehicle_pair_requires_body_margin(self):
        occupied = [{
            "class": "vehicle",
            "subtype": "bus_van",
            "position": {"x": 0.0, "y": 0.0, "z": 0.0},
        }]
        self.assertFalse(
            generator.static_vehicle_candidate_is_clear(
                {"x": 12.0, "y": 0.0, "z": 0.0},
                5.5,
                "bus_van",
                occupied,
            )
        )
        self.assertTrue(
            generator.static_vehicle_candidate_is_clear(
                {"x": 15.1, "y": 0.0, "z": 0.0},
                5.5,
                "bus_van",
                occupied,
            )
        )

    def test_obstacle_scene_has_one_current_lane_actor(self):
        family = static_base3.static_defaults()[1]["static_obstacle"]
        for count in range(4, 8):
            for scene_index in range(12):
                roles = generator.static_obstacle_scene_roles(
                    family, count, scene_index
                )
                self.assertEqual(count, len(roles))
                self.assertEqual(1, roles.count("current_lane_front"))

    def test_small_obstacle_range_is_capped_but_long_barrier_is_not(self):
        self.assertEqual(
            14.0,
            generator.static_obstacle_first_capture_distance(
                {"name": "TrafficBarrel", "subtype": "barrel_or_drum"}, 30.0
            ),
        )
        self.assertEqual(
            16.0,
            generator.static_obstacle_first_capture_distance(
                {"name": "WoodBox", "subtype": "box"}, 30.0
            ),
        )
        self.assertEqual(
            30.0,
            generator.static_obstacle_first_capture_distance(
                {"name": "Steel_Barricade"}, 30.0
            ),
        )

    def test_smoke_has_one_representative_clip_per_family(self):
        self.assertEqual(
            set(static_base3.GLOBAL_PRIMARY_COUNTS),
            set(static_base3_smoke.SMOKE_COUNTS),
        )
        self.assertTrue(
            all(count == 1 for count in static_base3_smoke.SMOKE_COUNTS.values())
        )

    def test_balanced_pilot_quota_is_120_clips(self):
        self.assertEqual(120, sum(static_base3_pilot.PILOT_COUNTS.values()))
        self.assertEqual(
            48, static_base3_pilot.PILOT_COUNTS["stopped_vehicle"]
        )
        self.assertEqual(
            {
                "stopped_vehicle": 12,
                "static_obstacle": 9,
                "standing_pedestrian": 5,
                "sparse_background": 4,
            },
            static_base3_pilot.balanced_pilot_counts(30),
        )

    def test_corrective_batch_spatial_quotas_are_exact(self):
        for batch_index in (5, 6, 7):
            counts = static_base3.PRIMARY_COUNTS_BY_BATCH[batch_index - 1]
            config = static_base3.build_config(
                batch_index, "primary", counts, "corrective", 1000
            )
            families = {row["name"]: row for row in config["families"]}
            for family, spatial_counts in (
                static_base3.CORRECTIVE_SPATIAL_COUNTS_BY_BATCH[batch_index].items()
            ):
                schedule = families[family]["global_spatial_stratum_schedule"]
                self.assertEqual(counts[family], len(schedule))
                self.assertEqual(
                    spatial_counts,
                    {name: schedule.count(name) for name in spatial_counts},
                )

    def test_pilot_spatial_deck_covers_corrective_topologies(self):
        for clips in (30, 120):
            family_counts = static_base3_pilot.balanced_pilot_counts(clips)
            for family, total in family_counts.items():
                counts = static_base3_pilot.scaled_spatial_counts(family, total)
                if family == "standing_pedestrian":
                    self.assertIsNone(counts)
                    continue
                self.assertEqual(total, sum(counts.values()))
                if family == "stopped_vehicle":
                    self.assertGreater(counts["highway_exit_split"], 0)
                    self.assertGreater(
                        counts["highway_post_taper_two_usable"], 0
                    )
                    self.assertGreater(
                        counts["route_general_two_lane_right"], 0
                    )

    def test_highway_phase_paths_exclude_restricted_links(self):
        for variants in generator.HIGHWAY_STATIC_PHASE_VARIANTS.values():
            for variant in variants:
                self.assertFalse(
                    set(variant["route"]) & generator.HIGHWAY_RESTRICTED_LINK_IDS
                )
                for side_route in variant.get("side_routes", ()):
                    self.assertFalse(
                        set(side_route) & generator.HIGHWAY_RESTRICTED_LINK_IDS
                    )

    def test_highway_variant_deck_keeps_low_share_exit_branch(self):
        rows = generator.HIGHWAY_STATIC_PHASE_VARIANTS["highway_exit_split"]
        schedule = generator.weighted_row_schedule(
            rows, 8, minimum_each=1
        )
        self.assertEqual(8, len(schedule))
        self.assertEqual(
            set(row["name"] for row in rows),
            set(row["name"] for row in schedule),
        )

    def test_explicit_corrective_routes_are_contiguous(self):
        links = {
            row["idx"]: row
            for row in generator.load_json(generator.DEFAULT_MGEO / "link_set.json")
        }
        paths = []
        for variants in generator.HIGHWAY_STATIC_PHASE_VARIANTS.values():
            for variant in variants:
                paths.append(tuple(variant["route"]))
                paths.extend(tuple(route) for route in variant.get("side_routes", ()))
        for variant in generator.ROUTE_GENERAL_TWO_LANE_VARIANTS:
            paths.extend((tuple(variant["left_route"]), tuple(variant["right_route"])))
        for path in paths:
            route, _arc = generator.route_from_link_path(links, path)
            gaps = [
                math.hypot(after["x"] - before["x"], after["y"] - before["y"])
                for before, after in zip(route, route[1:])
            ]
            self.assertTrue(gaps)
            self.assertLess(max(gaps), 5.0, path)
        self.assertEqual(
            31, sum(static_base3_pilot.balanced_pilot_counts(31).values())
        )


if __name__ == "__main__":
    unittest.main()
