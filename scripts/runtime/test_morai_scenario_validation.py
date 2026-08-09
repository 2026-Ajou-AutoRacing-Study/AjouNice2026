#!/usr/bin/env python3
"""Unit tests for MORAI batch-validation logic that does not require ROS."""

import json
import sys
import tempfile
import unittest
from pathlib import Path


RUNTIME_DIR = Path(__file__).resolve().parent
if str(RUNTIME_DIR) not in sys.path:
    sys.path.insert(0, str(RUNTIME_DIR))

from morai_scenario_validation import (  # noqa: E402
    DynamicReactionMonitor,
    make_scenario_load_message,
    object_counts,
    result_table_row,
    scenario_parameters,
)
from summarize_morai_scenario_validation import (  # noqa: E402
    build_aggregate,
    session_summaries,
)


class Value:
    def __init__(self, **values):
        self.__dict__.update(values)


def object_message(npc=(), pedestrian=(), obstacle=()):
    return Value(
        npc_list=list(npc),
        pedestrian_list=list(pedestrian),
        obstacle_list=list(obstacle),
        num_of_npcs=len(npc),
        num_of_pedestrian=len(pedestrian),
        num_of_obstacle=len(obstacle),
    )


def object_at(x, y):
    return Value(position=Value(x=x, y=y))


class MoraiScenarioValidationTest(unittest.TestCase):
    def test_parses_crosswalk_and_vehicle_dimensions(self):
        crosswalk = scenario_parameters(
            Path("dynamic_kid_loc01_obj02_front_crosswalk_ego20_act12_pos001.json")
        )
        self.assertEqual("crosswalk_dynamic_object", crosswalk["kind"])
        self.assertEqual("kid", crosswalk["actor"])
        self.assertEqual(12.0, crosswalk["activation_distance_m"])
        self.assertEqual(20.0, crosswalk["ego_speed_kph"])

        vehicle = scenario_parameters(
            Path("dynamic_car_left_obj30_arrow_tip_ego40_pos001.json")
        )
        self.assertEqual("intersection_vehicle", vehicle["kind"])
        self.assertEqual("car", vehicle["actor"])
        self.assertEqual("arrow_tip", vehicle["trigger"])
        self.assertEqual(40.0, vehicle["ego_speed_kph"])

    def test_pedestrian_counts_are_not_treated_as_missing_objects(self):
        counts = object_counts(object_message(pedestrian=(object_at(3.0, 2.0),)))
        self.assertEqual({"npc": 0, "pedestrian": 1, "obstacle": 0}, counts)
        self.assertEqual(1, sum(counts.values()))

    def test_topic_scenario_load_uses_all_dynamic_object_flags(self):
        message = make_scenario_load_message(
            Value, "dynamic_kid_loc01_obj02_front_crosswalk_ego20_act12_pos001.json"
        )
        self.assertEqual(
            "dynamic_kid_loc01_obj02_front_crosswalk_ego20_act12_pos001.json",
            message.file_name,
        )
        self.assertTrue(message.delete_all)
        self.assertTrue(message.load_ego_vehicle_data)
        self.assertTrue(message.load_surrounding_vehicle_data)
        self.assertTrue(message.load_pedestrian_data)
        self.assertTrue(message.load_obstacle_data)
        self.assertFalse(message.set_pause)

    def test_dynamic_reaction_requires_encounter_stop_and_resume(self):
        monitor = DynamicReactionMonitor(
            encounter_distance_m=18.0,
            stop_speed_threshold_mps=0.3,
            resume_speed_threshold_mps=0.7,
            object_freshness_seconds=1.0,
            require_resume=True,
        )
        monitor.activate(required=True)
        monitor.update_ego((0.0, 0.0), 2.0, 1.0, "t1")
        monitor.update_objects(
            object_message(pedestrian=(object_at(10.0, 0.0),)), 1.1, "t2"
        )
        monitor.update_ego((1.0, 0.0), 0.1, 1.2, "t3")
        monitor.update_ego((1.1, 0.0), 1.0, 1.3, "t4")
        summary = monitor.summary()

        self.assertTrue(summary["object_seen"])
        self.assertTrue(summary["encounter_detected"])
        self.assertTrue(summary["stopped_for_dynamic_object"])
        self.assertTrue(summary["resumed_after_dynamic_object"])
        self.assertTrue(summary["passed"])

    def test_result_row_keeps_dynamic_failure_reason(self):
        row = result_table_row(
            {
                "session_id": "session",
                "scenario": "dynamic_kid_loc01_obj02_front_crosswalk_ego20_act12_pos001.json",
                "passed": False,
                "scenario_parameters": scenario_parameters(
                    Path("dynamic_kid_loc01_obj02_front_crosswalk_ego20_act12_pos001.json")
                ),
                "dynamic_reaction": {
                    "required": True,
                    "object_seen": True,
                    "object_seen_categories": ["pedestrian"],
                    "encounter_detected": True,
                    "minimum_ego_object_distance_m": 2.5,
                    "stopped_for_dynamic_object": False,
                    "resumed_after_dynamic_object": False,
                    "failure_reasons": ["ego_never_stopped_for_dynamic_object"],
                },
            }
        )
        self.assertEqual("pedestrian", row["dynamic_object_categories"])
        self.assertEqual(
            "ego_never_stopped_for_dynamic_object", row["dynamic_failure_reasons"]
        )

    def test_combined_report_reads_all_session_results(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            first = root / "one"
            second = root / "two"
            first.mkdir()
            second.mkdir()
            first.joinpath("summary.json").write_text(
                json.dumps(
                    {
                        "session_id": "one",
                        "results": [{"scenario": "one.json", "passed": True}],
                    }
                ),
                encoding="utf-8",
            )
            second.joinpath("summary.json").write_text(
                json.dumps(
                    {
                        "session_id": "two",
                        "results": [
                            {
                                "scenario": "two.json",
                                "passed": False,
                                "collision_detected": True,
                                "dynamic_reaction": {
                                    "required": True,
                                    "passed": False,
                                },
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )
            aggregate = build_aggregate(list(session_summaries(root)))
        self.assertEqual(2, aggregate["scenarios"])
        self.assertEqual(1, aggregate["passed"])
        self.assertEqual(1, aggregate["collisions"])
        self.assertEqual(1, aggregate["dynamic_reaction_failures"])


if __name__ == "__main__":
    unittest.main()
