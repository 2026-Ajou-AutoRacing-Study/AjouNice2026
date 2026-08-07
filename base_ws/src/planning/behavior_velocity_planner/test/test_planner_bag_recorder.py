#!/usr/bin/env python3

from datetime import datetime
import os
import sys
import tempfile
import unittest


SCRIPTS_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts")
sys.path.insert(0, SCRIPTS_DIR)

from planner_bag_recorder import normalize_topics, timestamped_bag_path  # noqa: E402


class PlannerBagRecorderTest(unittest.TestCase):
    def test_normalize_topics_adds_slash_and_removes_duplicates(self):
        self.assertEqual(
            normalize_topics(["/tf", "tf", "", " /control/trajectory "]),
            ("/tf", "/control/trajectory"),
        )

    def test_timestamped_path_uses_requested_name(self):
        with tempfile.TemporaryDirectory() as output_directory:
            path = timestamped_bag_path(
                output_directory,
                "stopline_debug",
                datetime(2026, 8, 8, 14, 5, 9),
            )
            self.assertEqual(
                path,
                os.path.join(
                    output_directory,
                    "stopline_debug_2026-08-08-14-05-09.bag",
                ),
            )

    def test_timestamped_path_does_not_overwrite_active_recording(self):
        with tempfile.TemporaryDirectory() as output_directory:
            first_path = timestamped_bag_path(
                output_directory,
                "stopline_debug",
                datetime(2026, 8, 8, 14, 5, 9),
            )
            with open(first_path + ".active", "w", encoding="utf-8"):
                pass
            second_path = timestamped_bag_path(
                output_directory,
                "stopline_debug",
                datetime(2026, 8, 8, 14, 5, 9),
            )
            self.assertEqual(
                second_path,
                os.path.join(
                    output_directory,
                    "stopline_debug_2026-08-08-14-05-09_02.bag",
                ),
            )


if __name__ == "__main__":
    unittest.main()
