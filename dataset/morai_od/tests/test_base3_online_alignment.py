import importlib.util
import math
import sys
import pytest
from pathlib import Path
from types import SimpleNamespace


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts/audit_base3_online_gt_alignment.py"
)
sys.path.insert(0, str(SCRIPT.parent))
SPEC = importlib.util.spec_from_file_location("base3_online_alignment", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def vector(x=0.0, y=0.0, z=0.0):
    return SimpleNamespace(x=float(x), y=float(y), z=float(z))


def actor(unique_id=1, name="vehicle", x=0.0, y=0.0, heading=0.0, vx=0.0):
    return SimpleNamespace(
        unique_id=unique_id,
        name=name,
        position=vector(x, y, 0.0),
        velocity=vector(vx, 0.0, 0.0),
        acceleration=vector(),
        size=vector(4.0, 2.0, 1.5),
        heading=float(heading),
    )


def ego(x=0.0, y=0.0, heading=0.0, vx=0.0):
    return actor(0, "ego", x, y, heading, vx)


def objects(values):
    return SimpleNamespace(
        npc_list=list(values), pedestrian_list=[], obstacle_list=[],
        num_of_npcs=len(values), num_of_pedestrian=0, num_of_obstacle=0,
    )


def test_heading_interpolation_uses_short_arc():
    assert math.isclose(
        MODULE.interpolate_heading_degrees(359.0, 1.0, 0.5), 0.0,
        abs_tol=1e-9,
    )


def test_ego_pose_is_interpolated_to_lidar_time():
    aligned, audit = MODULE.align_ego_state(
        1.5, [(1.0, 1.0, ego(x=0.0)), (2.0, 2.0, ego(x=10.0))]
    )
    assert math.isclose(aligned.position.x, 5.0)
    assert audit["mode"] == "interpolated"
    assert math.isclose(audit["alpha"], 0.5)


def test_ego_short_extrapolation_uses_body_velocity_and_heading():
    aligned, audit = MODULE.align_ego_state(
        1.02, [(1.0, 1.0, ego(heading=90.0, vx=10.0))],
        max_extrapolation_sec=0.05,
    )
    assert abs(aligned.position.x) < 1e-9
    assert math.isclose(aligned.position.y, 0.2, abs_tol=1e-9)
    assert audit["mode"] == "extrapolated"


def test_extrapolation_beyond_limit_is_rejected():
    with pytest.raises(ValueError, match="exceeds"):
        MODULE.align_ego_state(
            1.06, [(1.0, 1.0, ego(vx=10.0))],
            max_extrapolation_sec=0.05,
        )


def test_object_alignment_preserves_nearest_membership_and_interpolates_pose():
    first = objects([actor(7, "car", x=0.0)])
    second = objects([actor(7, "car", x=10.0), actor(8, "new", x=20.0)])
    aligned, audit = MODULE.align_object_state(
        1.4, [(1.0, 1.0, first), (2.0, 2.0, second)]
    )
    assert [item.unique_id for item in aligned.npc_list] == [7]
    assert math.isclose(aligned.npc_list[0].position.x, 4.0)
    assert audit["interpolated_actors"] == 1


def test_actor_yaw_rate_uses_short_arc_across_heading_wrap():
    target = actor(7, "car", heading=0.0)
    samples = [
        (1.0, 1.0, objects([actor(7, "car", heading=359.0)])),
        (1.1, 1.1, objects([actor(7, "car", heading=1.0)])),
    ]
    yaw_rate, audit = MODULE.actor_yaw_rate_at(1.05, samples, target)
    assert audit["valid"]
    assert math.isclose(yaw_rate, math.radians(20.0), rel_tol=1e-9)


def test_bbox_center_velocity_adds_rotational_offset_term():
    from perception.detection3d.bevfusion.tools import (
        build_base3_online_bevfusion_infos as builder,
    )

    value, correction = builder.actor_center_velocity_in_ego_mps(
        actor(7, "car", heading=0.0, vx=0.0),
        ego(heading=0.0),
        {"pivot_to_bbox_center_local_m": [1.0, 0.0, 0.0]},
        yaw_rate_radps=1.0,
    )
    assert value[0] == pytest.approx(0.0)
    assert value[1] == pytest.approx(1.0)
    assert correction.tolist() == pytest.approx([0.0, 1.0])


def test_distance_filter_contracts_are_disjoint():
    from perception.detection3d.bevfusion.tools import build_base3_online_bevfusion_infos as builder

    box = [25.0, 25.0, 0.0, 4.0, 2.0, 1.5, 0.0]
    point_cloud_range = [-30.0, -30.0, -3.0, 30.0, 30.0, 2.0]
    assert builder.keep_box_for_distance(box, "box-range", 30.0, point_cloud_range)
    assert not builder.keep_box_for_distance(box, "radial-0-30", 30.0, point_cloud_range)
    assert builder.keep_box_for_distance(box, "radial-30-plus", 30.0, point_cloud_range)
    boundary = [30.0, 0.0, 0.0, 4.0, 2.0, 1.5, 0.0]
    assert not builder.keep_box_for_distance(
        boundary, "radial-0-30", 30.0, point_cloud_range
    )
    assert builder.keep_box_for_distance(
        boundary, "radial-30-plus", 30.0, point_cloud_range
    )
