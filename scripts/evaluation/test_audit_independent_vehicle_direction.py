import importlib.util
import json
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
spec = importlib.util.spec_from_file_location('independent_direction', HERE/'audit_independent_vehicle_direction.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def truth(times):
    return [(t, dict(key='vehicle:1', class_name='vehicle', center=[2*t, 0., .75],
                     size=[4., 1.8, 1.5], yaw=0., asset='example')) for t in times]


def test_bracket_motion_uses_position_not_status_velocity():
    data = truth([0., .1, .2])
    result = module.velocity_at(([t for t, _ in data], [p for _, p in data]), .1)
    np.testing.assert_allclose(result, [2., 0.])


def test_motion_does_not_extrapolate_birth_or_death():
    data = truth([0., .1, .2])
    series = ([t for t, _ in data], [p for _, p in data])
    assert module.velocity_at(series, 0.) is None
    assert module.velocity_at(series, .2) is None


def test_large_gap_cannot_be_filled_as_continuous_motion():
    data = truth([0., .1, .5])
    assert module.velocity_at(([t for t, _ in data], [p for _, p in data]), .1) is None


def make_frame(velocity, stamp=.1):
    return dict(stamp=stamp, tracks=[dict(id=1, confirmed=True, core_class=1,
        center=[2*stamp, 0., .75], size=[4., 1.8, 1.5], yaw=0., velocity=velocity,
        initialization_time=0., last_measurement_time=stamp, associated=True)])


def test_reverse_detection_is_relative_to_gt_not_positive_body_speed():
    data = truth([0., .1, .2])
    gt = dict(gt=[([t for t, _ in data], [p for _, p in data])])
    forward = module.motion_metrics([make_frame([2., 0.])], gt)
    reverse = module.motion_metrics([make_frame([-2., 0.])], gt)
    assert forward['counts']['opposite_direction'] == 0
    assert reverse['counts']['young_opposite_direction'] == 1
    for point in gt['gt'][0][1]:
        point['center'][0] *= -1
    frame = make_frame([-2., 0.]); frame['tracks'][0]['center'][0] *= -1
    legitimate = module.motion_metrics([frame], gt)
    assert legitimate['counts']['opposite_direction'] == 0


def test_ineligible_gt_keeps_input_but_excludes_statistics():
    data = truth([0., .1, .2])
    gt = dict(gt=[([t for t, _ in data], [p for _, p in data])])
    result = module.motion_metrics([make_frame([-2., 0.])], gt, allowed_stamps=set())
    assert result['counts'] == {'gt_ineligible_frames_excluded': 1}
    assert result['distributions'] == {}


def test_repeat_selection_excludes_preload_and_never_claims_continuity(tmp_path):
    source = tmp_path/'source.json'
    rows = [dict(token=str(i), case='case_a', stamp=i*.1, scene_epoch_valid=i > 0,
                 ego=dict(x=i*.2, y=0.)) for i in range(10)]
    source.write_text(json.dumps(dict(rows=rows, continuous_for_tracker=True)))
    before = source.read_bytes()
    dest = tmp_path/'subset.json'
    module.prepare_repeat_inputs(source, dest, 3)
    payload = json.loads(dest.read_text())
    assert len(payload['rows']) == 3
    assert all(r['scene_epoch_valid'] for r in payload['rows'])
    assert payload['continuous_for_tracker'] is False
    assert source.read_bytes() == before


def test_repeat_selection_does_not_call_stationary_ego_moving(tmp_path):
    source = tmp_path/'source.json'
    source.write_text(json.dumps(dict(rows=[dict(case='a', stamp=i*.1, scene_epoch_valid=True,
        ego=dict(x=0., y=0.)) for i in range(5)])))
    import pytest
    with pytest.raises(ValueError, match='Moving cohort'):
        module.prepare_repeat_inputs(source, tmp_path/'subset.json')


def test_motion_statistics_current_roi_does_not_cut_future_gt():
    points = truth([0., .1, .2])
    gt = dict(gt=[([t for t, _ in points], [p for _, p in points])],
              ego=([0., .1, .2], [dict(center=[500., 0., 0.], yaw=0.) for _ in points]))
    result = module.motion_metrics([make_frame([2., 0.])], gt)
    assert result['counts']['vehicle_gt'] == 0
    assert result['counts']['matched'] == 0
