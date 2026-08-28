#!/usr/bin/env python3

import random
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "dataset/morai_od/scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import generate_dynamic_object_capture_set as generator
import generate_static_classcentric_base3_batch as static_base3
import generate_static_classcentric_base3_pilot as static_base3_pilot
import generate_static_classcentric_base3_smoke as static_base3_smoke


class StaticClasscentricBase3Test(unittest.TestCase):
    def test_campaign_quota_is_exact(self):
        static_base3.validate_global_quota()
        self.assertEqual(3334, static_base3.TOTAL_CLIPS)
        self.assertEqual(10002, static_base3.TOTAL_FRAMES)
        self.assertEqual(
            static_base3.GLOBAL_PRIMARY_COUNTS,
            {
                "stopped_vehicle": 1167,
                "static_obstacle": 1167,
                "standing_pedestrian": 500,
                "realistic_mixed_static": 167,
                "sparse_background": 333,
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
        self.assertEqual([3, 5], rows["stopped_vehicle"]["actors_per_scene"])
        self.assertTrue(rows["stopped_vehicle"]["snap_static_vehicle_to_mgeo"])
        self.assertTrue(rows["stopped_vehicle"]["same_lane_fallback_only"])
        self.assertEqual(
            [-18.0, 15.0, 24.0, 33.0, 42.0],
            rows["stopped_vehicle"]["same_lane_fallback_capture_slots_m"],
        )
        self.assertEqual(
            [-28.0, 26.0, 42.0, 58.0, 74.0],
            rows["stopped_vehicle"][
                "same_lane_fallback_capture_slots_by_subtype"
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
            1.0,
            rows["stopped_vehicle"]["highway_left_vehicle_probability"],
        )
        self.assertEqual([5, 8], rows["static_obstacle"]["objects_per_scene"])
        self.assertEqual([4, 7], rows["standing_pedestrian"]["actors_per_scene"])
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
        self.assertEqual(1167, len(strata))
        self.assertEqual(234, strata.count("roundabout"))
        self.assertEqual(253, strata.count("highway_three_lane"))
        lane_schedule = generator.weighted_lane_schedule(
            {
                "count": strata.count("highway_three_lane"),
                "source_spawn_ids": [6, 8, 10],
                "ego_lane_weights": {"6": 1, "8": 1, "10": 1},
            }
        )
        lane_counts = {lane: lane_schedule.count(lane) for lane in (6, 8, 10)}
        self.assertEqual(253, sum(lane_counts.values()))
        self.assertLessEqual(max(lane_counts.values()) - min(lane_counts.values()), 1)

    def test_integer_count_range_is_inclusive_and_deterministic(self):
        first = random.Random(7)
        second = random.Random(7)
        a = [generator.integer_count(first, [3, 5]) for _ in range(30)]
        b = [generator.integer_count(second, [3, 5]) for _ in range(30)]
        self.assertEqual(a, b)
        self.assertEqual({3, 4, 5}, set(a))

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
            [-20.0, 18.0, 28.0, 38.0, 48.0],
            generator.same_lane_capture_slots(family, "suv"),
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

    def test_smoke_has_one_representative_clip_per_family(self):
        self.assertEqual(
            set(static_base3.GLOBAL_PRIMARY_COUNTS),
            set(static_base3_smoke.SMOKE_COUNTS),
        )
        self.assertTrue(
            all(count == 1 for count in static_base3_smoke.SMOKE_COUNTS.values())
        )

    def test_vehicle_pilot_quota_is_120_clips(self):
        self.assertEqual(120, sum(static_base3_pilot.PILOT_COUNTS.values()))
        self.assertEqual(
            108, static_base3_pilot.PILOT_COUNTS["stopped_vehicle"]
        )


if __name__ == "__main__":
    unittest.main()
