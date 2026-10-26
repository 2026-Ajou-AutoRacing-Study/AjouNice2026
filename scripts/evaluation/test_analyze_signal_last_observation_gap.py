"""Small pure tests: no ROS, model, native RPC, or raw-data writes."""
import pytest
import numpy as np

from analyze_signal_last_observation_gap import interpolate_pose, positive_observation


def test_held_output_does_not_create_observation():
    decision = dict(selection_status='matched', classifier_known=True,
                    observed_tuples=[], final_tuples=[['green', 'circle', None]],
                    observation_timing=dict(output_is_held=True))
    assert not positive_observation(decision)
    decision['observed_tuples'] = [['yellow', 'circle', None]]
    # Raw evidence is independent of downstream permission/hold decision.
    assert positive_observation(decision)
    decision['classifier_known'] = False
    assert not positive_observation(decision)


def test_pose_interpolation_exact_and_bounded():
    poses = [dict(front_point_map_xyz_m=[0., 0., 0.]),
             dict(front_point_map_xyz_m=[2., 0., 0.])]
    xy, gap = interpolate_pose(poses, [0, 100000000], 50000000)
    assert np.allclose(xy, [1., 0.]) and gap == .1
    assert interpolate_pose(poses, [0, 100000000], 0)[1] == 0.
    with pytest.raises(ValueError, match='extrapolation'):
        interpolate_pose(poses, [0, 100000000], -1)
    with pytest.raises(ValueError, match='150ms'):
        interpolate_pose(poses, [0, 200000000], 100000000)
