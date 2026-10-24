"""Map terminals truncate time; search limits and missing matches do not fake exits."""
import math
from perception_object_pipeline.map_based_prediction import (
    LaneletNetwork, LaneletRecord, MapPredictionConfig,
)


def predict(net, cap=8., **kwargs):
    return net.predict(0., 0., 0., 0., 10., tuple(.1*i for i in range(1,int(cap*10)+1)),
                       truncate_at_route_end=True, **kwargs)


def terminal(length):
    return LaneletNetwork([LaneletRecord(1, ((-10.,0.,0.),(length,0.,0.)), (),
                                            (('speed_limit','150'),))])


def test_five_point_six_second_terminal_is_not_replaced_by_three_seconds():
    path = predict(terminal(56.))[0]
    assert len(path.poses) == 56
    assert path.poses[-1][0] == 56.


def test_less_than_three_seconds_stays_map_based_and_not_padded():
    for remaining in (18., 20., 29.8):
        for cap in (3., 8.):
            path = predict(terminal(remaining), cap)[0]
            assert len(path.poses) == math.floor(remaining + 1.e-8)
            assert path.poses[-1][0] <= remaining + 1.e-8
            assert all(a[0] < b[0] for a,b in zip(path.poses,path.poses[1:]))


def test_less_than_one_step_has_current_pose_only_at_ros_conversion_boundary():
    paths = predict(terminal(.04))
    assert len(paths) == 1 and paths[0].poses == ()


def test_depth_limit_is_not_misrepresented_as_npc_disappearance():
    n=LaneletNetwork([
        LaneletRecord(1, ((-10.,0.,0.),(10.,0.,0.)), (2,)),
        LaneletRecord(2, ((10.,0.,0.),(200.,0.,0.)), ()),
    ],MapPredictionConfig(maximum_route_depth=1))
    assert predict(n) == ()


def test_fork_keeps_both_actual_available_path_lengths():
    n=LaneletNetwork([
        LaneletRecord(1, ((-10.,0.,0.),(10.,0.,0.)), (2,3)),
        LaneletRecord(2, ((10.,0.,0.),(20.,0.,0.)), ()),
        LaneletRecord(3, ((10.,0.,0.),(200.,10.,0.)), ()),
    ],MapPredictionConfig(maximum_start_lanelets=1))
    paths = predict(n)
    assert sorted(len(p.poses) for p in paths) == [20,80]
    assert abs(sum(p.confidence for p in paths)-1.) < 1.e-8


def test_reference_core_api_remains_fixed_horizon_unless_opted_in():
    n=terminal(18.)
    assert n.predict(0.,0.,0.,0.,10.,tuple(.1*i for i in range(1,31))) == ()


def test_direction_recovery_never_expands_distance_or_allowed_region():
    n=terminal(200.)
    assert n.recover_lane_direction(0., .1, {1}) == 0.
    assert n.recover_lane_direction(0., 4., {1}) is None
    assert n.recover_lane_direction(0., .1, set()) is None


def test_ambiguous_opposite_road_directions_are_not_forced():
    n=LaneletNetwork([
        LaneletRecord(1, ((-10.,0.,0.),(200.,0.,0.)), ()),
        LaneletRecord(2, ((200.,.2,0.),(-10.,.2,0.)), ()),
    ])
    assert n.recover_lane_direction(0.,.1,{1,2}) is None
