import importlib.util
import json
import tempfile
import types
import unittest
from pathlib import Path


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts/monitor_ros_capture_progress_v2.py"
)
SPEC = importlib.util.spec_from_file_location("capture_monitor_v2", SCRIPT)
MONITOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MONITOR)


class CaptureMonitorV2Test(unittest.TestCase):
    def test_failure_classification(self):
        cases = {
            "collision reported during timed-grouped": "collision",
            "rosbridge service is stale": "ros_network",
            "missing sensor artifact CAMERA_1": "sensor_artifact",
            "route deviation 1.8 m": "route_safety",
            "Ego did not arm timed-grouped capture": "motion_readiness",
            "object counts {'npc': 0} != {'npc': 1}": "actor_missing",
        }
        for error, expected in cases.items():
            self.assertEqual(MONITOR.classify_failure(error), expected)

    def test_appended_reader_only_consumes_new_lines(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "scenes.jsonl"
            path.write_text('{"id":"a"}\n', encoding="utf-8")
            reader = MONITOR.AppendedLineReader(path)
            self.assertEqual(reader.update(json.loads), [{"id": "a"}])
            self.assertEqual(reader.update(json.loads), [])
            with path.open("a", encoding="utf-8") as stream:
                stream.write('{"id":"b"}\n')
            self.assertEqual(reader.update(json.loads), [{"id": "b"}])
            self.assertEqual(list(reader.rows), [{"id": "a"}, {"id": "b"}])

    def test_console_reader_starts_from_small_tail(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "console.log"
            path.write_text(
                "".join("line-{:03d}\n".format(index) for index in range(100)),
                encoding="utf-8",
            )
            reader = MONITOR.AppendedLineReader(
                path, max_rows=20, initial_tail_bytes=80
            )
            rows = reader.update()
            self.assertNotIn("line-000", rows)
            self.assertEqual(rows[-1], "line-099")

    def test_snapshot_reports_quota_health_and_rates(self):
        manifest = {
            "dataset_id": "demo",
            "scenes": [
                {
                    "id": "drc001",
                    "file_name": "od_rc001",
                    "mission_family": "roundabout_circulation",
                    "split": "train",
                    "sequence_contract": {"clip_length": 8},
                },
                {
                    "id": "dso001",
                    "file_name": "od_so001",
                    "mission_family": "static_obstacle",
                    "split": "val",
                    "sequence_contract": {"clip_length": 8},
                },
                {
                    "id": "drs001",
                    "file_name": "od_rs001",
                    "mission_family": "roundabout_sample_spawn",
                    "split": "test",
                    "sequence_contract": {"clip_length": 8},
                },
            ],
        }
        manifest["by_id"] = {row["id"]: row for row in manifest["scenes"]}
        manifest["by_file"] = {
            row["file_name"]: row for row in manifest["scenes"]
        }
        rows = [
            {
                "id": "drc001",
                "status": "completed",
                "capture_count": 8,
                "metrics": {
                    "scene_duration_sec": 16.0,
                    "trigger_rate_hz": 11.5,
                },
            },
            {
                "id": "dso001",
                "status": "failed",
                "capture_count": 0,
                "error": "collision reported",
                "metrics": {"scene_duration_sec": 12.0},
            },
        ]
        summary = {
            "dataset_id": "demo",
            "requested_scenes": 3,
            "attempted_scenes": 2,
            "completed_scenes": 1,
            "failed_scenes": 1,
            "completed_captures": 8,
            "checkpoint_unix": 1000.0,
            "started_unix": 900.0,
            "finished": False,
        }
        args = types.SimpleNamespace(
            stall_warning_sec=60.0,
            disk_warning_gb=0.0,
            failure_rate_warning=0.10,
        )
        original_time = MONITOR.time.time
        MONITOR.time.time = lambda: 1010.0
        try:
            snapshot = MONITOR.build_snapshot(
                Path(tempfile.gettempdir()),
                summary,
                rows,
                [{"id": "dso001", "error": "collision reported"}],
                [],
                manifest,
                args,
            )
        finally:
            MONITOR.time.time = original_time
        self.assertEqual(snapshot["progress"]["target_frames"], 24)
        self.assertEqual(snapshot["groups"]["mission_dynamic"]["completed"], 1)
        self.assertEqual(snapshot["groups"]["static_temporal"]["failed"], 1)
        self.assertEqual(
            snapshot["families"]["roundabout_circulation"]["completed"], 1
        )
        self.assertEqual(snapshot["health"]["failure_types"], {"collision": 1})
        # Avoid noisy alarms during the first few scenes; failure-rate alerts
        # begin after ten attempts.
        self.assertNotIn("failure_rate_high", snapshot["health"]["warnings"])
        self.assertEqual(snapshot["performance"]["sequence_rate_p50_hz"], 11.5)
        self.assertEqual(snapshot["current"]["scene_id"], "drs001")
        self.assertEqual(
            snapshot["current"]["stage"], "next_or_running_unobserved"
        )

    def test_console_parser_tracks_scene_and_clip_frame(self):
        manifest = {
            "by_file": {
                "od_scene": {"id": "scene001", "file_name": "od_scene"}
            }
        }
        state = MONITOR.parse_console_state(
            [
                "ScenarioLoad ROS published: od_scene (delete_all=false)",
                "Scenario verified: od_scene (npc=1)",
                "Timed-grouped trigger 5/8: sequence=0.3s",
            ],
            manifest,
        )
        self.assertEqual(state["scene_id"], "scene001")
        self.assertEqual(state["stage"], "sensor_capture")
        self.assertEqual(state["capture_index"], 5)
        self.assertEqual(state["capture_total"], 8)


if __name__ == "__main__":
    unittest.main()
