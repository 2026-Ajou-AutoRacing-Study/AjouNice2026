import importlib.util
from pathlib import Path

import pytest


SCRIPT = (
    Path(__file__).resolve().parents[1] / "scripts" / "morai_direct_replay_runner.py"
)
SPEC = importlib.util.spec_from_file_location("morai_direct_replay_runner", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_parse_targets_combines_morai_bitmask():
    assert MODULE.parse_targets("ego,npc,pedestrian") == 0x0111
    assert MODULE.parse_targets("npc,ego,npc") == 0x0011


def test_parse_targets_rejects_unknown_values():
    with pytest.raises(ValueError, match="unknown replay target"):
        MODULE.parse_targets("ego,obstacle")


@pytest.mark.parametrize(
    ("start_time_ms", "speed", "expected"),
    [
        (0, 1, 0x0011),
        (1000, 1, 0x0111),
        (0, 2, 0x1011),
        (1000, 2, 0x1111),
    ],
)
def test_build_replay_option(start_time_ms, speed, expected):
    assert MODULE.build_replay_option(
        start_time_ms=start_time_ms, replay_speed=speed
    ) == expected


def test_resolve_bag_uses_launcher_basename(tmp_path):
    bag = tmp_path / "sample.bag"
    bag.write_bytes(b"bag")
    path, service_name = MODULE.resolve_bag("sample.bag", tmp_path)
    assert path == bag.resolve()
    assert service_name == "sample.bag"


def test_resolve_bag_rejects_missing_file(tmp_path):
    with pytest.raises(FileNotFoundError, match="replay bag not found"):
        MODULE.resolve_bag("missing.bag", tmp_path)


class _Status:
    current_mode = 0


def test_require_replay_mode_explains_manual_precondition():
    with pytest.raises(RuntimeError, match="PlayMode > Replay > Rosbag Replay"):
        MODULE.require_replay_mode(_Status())


def test_require_replay_mode_can_probe_launcher_with_zero_status():
    MODULE.require_replay_mode(_Status(), allow_unreported_mode=True)
