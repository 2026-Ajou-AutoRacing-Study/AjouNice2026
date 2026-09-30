import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/audit_base3_online_bag.py"
SPEC = importlib.util.spec_from_file_location("audit_base3_online_bag", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_build_capture_windows_accepts_sequential_scenes():
    markers = [
        (1.0, {"event": "scene_loading", "scene_id": "a"}),
        (2.0, {"event": "capture_start", "scene_id": "a", "mission_family": "vehicle"}),
        (3.0, {"event": "capture_end", "scene_id": "a"}),
        (4.0, {"event": "capture_start", "scene_id": "b"}),
        (5.0, {"event": "capture_end", "scene_id": "b"}),
    ]
    windows, errors = MODULE.build_capture_windows(markers)
    assert errors == []
    assert [(item["scene_id"], item["start"], item["end"]) for item in windows] == [
        ("a", 2.0, 3.0),
        ("b", 4.0, 5.0),
    ]


def test_build_capture_windows_rejects_overlap_and_unclosed_scene():
    markers = [
        (2.0, {"event": "capture_start", "scene_id": "a"}),
        (2.5, {"event": "capture_start", "scene_id": "b"}),
        (3.0, {"event": "capture_end", "scene_id": "a"}),
    ]
    _, errors = MODULE.build_capture_windows(markers)
    assert any("overlaps active" in error for error in errors)
    assert any("without terminal" in error for error in errors)


def test_build_capture_windows_uses_failure_as_terminal_after_capture_start():
    markers = [
        (2.0, {"event": "capture_start", "scene_id": "a"}),
        (
            4.0,
            {
                "event": "scene_failed",
                "scene_id": "a",
                "status": "failed",
                "error": "legacy SaveSensorData validator rejected output",
            },
        ),
    ]
    windows, errors = MODULE.build_capture_windows(markers)
    assert errors == []
    assert len(windows) == 1
    assert windows[0]["terminal_event"] == "scene_failed"
    assert windows[0]["terminal_error"].startswith("legacy")


def test_sequence_start_rebases_long_warmup_window():
    markers = [
        (2.0, {"event": "capture_start", "scene_id": "a"}),
        (12.0, {"event": "sequence_start", "scene_id": "a"}),
        (13.5, {"event": "capture_end", "scene_id": "a"}),
    ]
    windows, errors = MODULE.build_capture_windows(markers)
    assert errors == []
    assert windows[0]["start"] == 12.0
    assert windows[0]["start_event"] == "sequence_start"


def test_sequence_start_is_sufficient_when_outer_marker_rolled_out():
    markers = [
        (12.0, {"event": "sequence_start", "scene_id": "a"}),
        (13.5, {"event": "capture_end", "scene_id": "a"}),
    ]
    windows, errors = MODULE.build_capture_windows(markers)
    assert errors == []
    assert windows[0]["start"] == 12.0


def test_nearest_delta():
    assert MODULE.nearest_delta(2.1, [1.0, 2.0, 4.0]) == 0.10000000000000009
    assert MODULE.nearest_delta(2.1, []) is None
