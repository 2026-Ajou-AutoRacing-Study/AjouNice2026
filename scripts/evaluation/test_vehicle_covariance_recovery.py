from pathlib import Path
import subprocess

import numpy as np
import pytest

from qualify_vehicle_covariance_recovery import (
    compare_motion, evaluate_motion, generated_core, interpolate_gt,
    motion_counterexamples, replay_recovery,
)

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def test_cpp_covariance_projection_and_signed_dynamics(tmp_path):
    binary = tmp_path / 'covariance_recovery_test'
    subprocess.run(['g++', '-std=c++17', '-O1', '-I/usr/include/eigen3', '-I' + str(HERE),
                    str(HERE / 'test_vehicle_covariance_recovery.cpp'), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)


def test_source_overlay_is_isolated_and_native_distance_gate_remains():
    source = ROOT / 'base_ws/src/ekf-multi-object-tracking/src/algorithm/ekf_multi_object_tracking.cpp'
    original = source.read_text()
    decouple = generated_core(original, 'decouple')
    reset = generated_core(original, 'reset')
    assert source.read_text() == original
    assert 'l2_distance > config_.max_association_dist_m' in decouple
    assert 'recovery_pair(meas_idx, track_idx) != 0' in decouple
    assert 'measurement_vec.head<2>().eval(), R.block<2, 2>(0, 0).eval(), false);' in decouple
    assert 'measurement_vec.head<2>().eval(), R.block<2, 2>(0, 0).eval(), true);' in reset
    assert 'if (recovery_pair) K.block<8, 2>(0, 0).setZero();' in reset
    assert 'if (recovery_pair) K.block<8, 2>(0, 0).setZero();' not in decouple
    assert 'both_vehicles && all_tracks_[track_idx].is_init' in decouple


def test_unknown_profile_or_changed_native_anchor_fails_closed():
    with pytest.raises(ValueError):
        generated_core('', 'bad')
    with pytest.raises(ValueError):
        generated_core('', 'decouple')


def test_counterexample_gt_not_mutated_by_bias_or_outlier():
    cases = motion_counterexamples()
    assert len(cases) == 8
    for label in ('localization_step', 'single_outlier', 'two_actor_close_outlier'):
        frames, gt = cases[label]
        measured_actor = min(frames[43]['world_objects'], key=lambda obj: obj['center'][1])
        assert measured_actor['center'][0] != gt['gt'][0][1][43]['center'][0]
    frames, _ = cases['two_actor_close_outlier']
    assert len(frames[44]['world_objects']) == 2
    assert all(len(frame['world_objects']) == 0 for frame in frames[40:43])


def test_signed_reverse_motion_is_not_heading_axis_velocity():
    frames, gt = motion_counterexamples()['born_reverse']
    assert frames[20]['world_objects'][0]['yaw'] == 0.
    p1 = interpolate_gt(*gt['gt'][0], 101.9)
    p2 = interpolate_gt(*gt['gt'][0], 102.1)
    assert np.allclose((p2 - p1) / .2, [-4., 0.])


def test_gt_interpolation_refuses_large_gaps_and_extrapolation():
    points = [dict(center=[0., 0.]), dict(center=[1., 0.])]
    assert interpolate_gt([0., 1.], points, .5) is None
    assert interpolate_gt([0., 1.], points, 2.) is None
    assert np.allclose(interpolate_gt([0., .1], points, .05), [.5, 0.])


def test_motion_gt_not_supplied_to_native_filter_and_pairing_not_id_dependent():
    frames, gt = motion_counterexamples()['born_reverse']
    row = dict(stamp=102., tracks=[dict(id=0, core_class=1, confirmed=True,
                                       center=[-8., 0., .8], velocity=[-4., 0.])])
    summary, samples = evaluate_motion([row], gt)
    assert summary['velocity_error_mps']['maximum'] < 1.e-10
    assert summary['fde_3s_m']['maximum'] < 1.e-10
    assert not summary['opposite_direction']
    changed_id = [dict(samples[0], id=42)]
    paired = compare_motion(samples, changed_id)
    assert paired['paired'] == 1
    assert paired['fde_3s_m']['delta']['maximum'] == 0.


def test_paired_tail_does_not_hide_individual_worsening():
    before = [dict(stamp=float(i), gt=0, fde_3s_m=2.) for i in range(100)]
    after = [dict(row, fde_3s_m=(8. if i == 99 else 1.9)) for i, row in enumerate(before)]
    paired = compare_motion(before, after)['fde_3s_m']
    assert paired['delta']['mean'] < 0.
    assert paired['delta']['maximum'] == 6.
    assert paired['worsened_gt1m'] == 1


def test_scoring_eligibility_filters_only_metrics_not_native_replay_history():
    _, gt = motion_counterexamples()['born_reverse']
    rows = [dict(stamp=stamp, tracks=[dict(id=0, core_class=1, confirmed=True,
                                         center=[-4.*(stamp-100.), 0., .8], velocity=[-4., 0.])])
            for stamp in (102., 102.1)]
    all_summary, all_samples = evaluate_motion(rows, gt)
    summary, samples = evaluate_motion(rows, gt, allowed_stamps={102.1})
    assert len(rows) == 2
    assert len(all_samples) == 2
    assert len(samples) == 1
    assert samples[0]['stamp'] == 102.1
    assert summary['fde_3s_m']['n'] == 1
    assert all_summary['fde_3s_m']['n'] == 2


def test_recovery_diagnostic_restores_caller_environment(tmp_path, monkeypatch):
    path = tmp_path / 'actual_recovery.log'
    monkeypatch.setenv('BASE5_VEHICLE_RECOVERY_TRACE', 'previous_path')
    def fake_replay(binary, frames):
        import os
        assert os.environ['BASE5_VEHICLE_RECOVERY_TRACE'] == str(path)
        path.write_text('100.1 42 1 2 3 4 -5 6 .85\n')
        return ['native_rows']
    monkeypatch.setattr('qualify_vehicle_covariance_recovery.replay', fake_replay)
    rows, events = replay_recovery(Path('/unused'), [], path)
    import os
    assert os.environ['BASE5_VEHICLE_RECOVERY_TRACE'] == 'previous_path'
    assert rows == ['native_rows']
    assert events[0]['id'] == 42
    assert events[0]['prior_velocity'] == [-5., 6.]


def test_recovery_diagnostic_restores_unset_environment_on_failure(tmp_path, monkeypatch):
    monkeypatch.delenv('BASE5_VEHICLE_RECOVERY_TRACE', raising=False)
    def fail_replay(binary, frames):
        raise RuntimeError('test failure')
    monkeypatch.setattr('qualify_vehicle_covariance_recovery.replay', fail_replay)
    with pytest.raises(RuntimeError):
        replay_recovery(Path('/unused'), [], tmp_path / 'actual_recovery.log')
    import os
    assert 'BASE5_VEHICLE_RECOVERY_TRACE' not in os.environ
