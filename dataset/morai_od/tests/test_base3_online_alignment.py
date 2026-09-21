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


def test_ego_heading_correction_rotates_map_delta_into_ego_frame():
    value = MODULE.ego_relative_position(
        ego(heading=91.347), actor(x=0.0, y=10.0),
        ego_heading_correction_deg=-1.347,
    )
    assert value[0] == pytest.approx(10.0, abs=1e-9)
    assert value[1] == pytest.approx(0.0, abs=1e-9)


def test_online_pose_matrix_records_corrected_mr_heading():
    from perception.detection3d.bevfusion.tools import (
        build_base3_online_bevfusion_infos as builder,
    )

    pose = builder.planar_pose_matrix(
        ego(x=3.0, y=4.0, heading=91.347),
        ego_heading_correction_deg=-1.347,
    )
    assert pose[0, 0] == pytest.approx(0.0, abs=1e-9)
    assert pose[1, 0] == pytest.approx(1.0, abs=1e-9)
    assert pose[:2, 3].tolist() == pytest.approx([3.0, 4.0])


def test_catalog_heading_correction_rotates_existing_relative_vector():
    from dataset.morai_od.scripts import build_morai_asset_bbox_offset_catalog as catalog

    value = catalog.corrected_relative_position(
        {"x": math.cos(math.radians(1.347)) * 10.0,
         "y": -math.sin(math.radians(1.347)) * 10.0,
         "z": 2.0},
        -1.347,
    )
    assert value == pytest.approx([10.0, 0.0, 2.0], abs=1e-9)


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
