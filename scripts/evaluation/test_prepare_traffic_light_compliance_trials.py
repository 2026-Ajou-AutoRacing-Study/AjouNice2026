"""Pure preparation helpers: no ROS bag, map binding, simulator or GPU."""
import copy
import json
from pathlib import Path
import types

import pytest

import prepare_traffic_light_compliance_trials as trial


def line_geometry():
    return [(0, 0., 100., [(101, [(0., 0.), (100., 0.)], 100.)])]


def test_all_four_missions_have_focused_trials_including_both_C_heads():
    assert trial.FOCUSED_MISSION_WAYPOINTS == (('A', 0), ('B', 1), ('C', 2), ('D', 3))
    assert {mission for mission, _ in trial.FOCUSED_MISSION_WAYPOINTS} == {
        mission for _, mission, _ in trial.GROUPS}
    assert next(ids for _, mission, ids in trial.GROUPS if mission == 'C') == [
        'C1256W000016', 'C1256W000018']


def test_exact_osm_ref_lines_and_mission_native_ids(tmp_path):
    nodes = '<node id="1" lat="1" lon="2"/><node id="2" lat="3" lon="4"/>'
    way = '<way id="10"><nd ref="1"/><nd ref="2"/></way>'
    relations = ''.join('<relation id="{}"><member type="way" ref="10" role="ref_line"/>'
                        '<tag k="subtype" v="traffic_light"/></relation>'.format(g[0]) for g in trial.GROUPS)
    path = tmp_path/'map.osm'; path.write_text('<osm>'+nodes+way+relations+'</osm>')
    groups = trial.stopline_groups(path, tmp_path/'unused.yaml',
        forward=lambda lat, lon: types.SimpleNamespace(x=lat*10, y=lon*10))
    assert [g['mission'] for g in groups] == ['A', 'B', 'C', 'D']
    assert groups[0]['stop_lines'][0]['points_xy_m'] == [[10., 20.], [30., 40.]]
    assert groups[0]['stop_line_xy'] == groups[0]['stop_lines'][0]['points_xy_m']
    assert groups[0]['stop_line_xy'] is not groups[0]['stop_lines'][0]['points_xy_m']
    assert groups[2]['native_ids'] == ['C1256W000016', 'C1256W000018']
    assert groups[3]['native_ids'] == ['C1256W000052']


def test_stopline_missing_is_not_replaced_by_bulb(tmp_path):
    path = tmp_path/'map.osm'
    path.write_text('<osm><relation id="201351"><tag k="subtype" v="traffic_light"/></relation></osm>')
    with pytest.raises(ValueError, match='stopline is missing'):
        trial.stopline_groups(path, tmp_path/'unused.yaml', forward=lambda *_: None)


def test_multiple_ref_lines_are_rejected_not_concatenated(tmp_path):
    path = tmp_path/'map.osm'
    path.write_text('<osm><node id="1" lat="1" lon="2"/><node id="2" lat="3" lon="4"/>'
        '<way id="10"><nd ref="1"/><nd ref="2"/></way>'
        '<relation id="201351"><member type="way" ref="10" role="ref_line"/>'
        '<member type="way" ref="10" role="ref_line"/><tag k="subtype" v="traffic_light"/></relation></osm>')
    with pytest.raises(ValueError, match='Exactly one ref_line'):
        trial.stopline_groups(path, tmp_path/'unused.yaml',
            forward=lambda lat, lon: types.SimpleNamespace(x=lat, y=lon))


def test_projection_is_finite_segment_not_an_infinite_halfplane():
    distance, progress, length, yaw = trial.project_line([(0., 0.), (10., 0.)], 20., 3.)
    assert distance == pytest.approx((100+9)**.5)
    assert progress == length == 10.
    assert yaw == 0.
    with pytest.raises(ValueError):
        trial.project_line([(0., 0.), (0., 0.)], 1., 0.)


def test_goal_must_be_after_stopline_and_align_with_actual_ordered_route():
    assert not trial.goal_check(dict(x=10., y=0., yaw_deg=0.), line_geometry(), 20., 2.)['valid']
    check = trial.goal_check(dict(x=30., y=0., yaw_deg=0.), line_geometry(), 20., 2.)
    assert check['valid']
    assert check['distance_after_stopline_m'] == 10.
    assert not trial.goal_check(dict(x=30., y=0., yaw_deg=180.), line_geometry(), 20., 2.)['valid']
    assert not trial.goal_check(dict(x=30., y=20., yaw_deg=0.), line_geometry(), 20., 2.)['valid']


def samples(heading=1.347, receipt=.1, ego_x=2.5):
    ego = [dict(receipt_sec=receipt, source_stamp_ns=1, xyz_m=[ego_x, 0., 28.], heading_degrees=heading)]
    loc = [dict(receipt_sec=.1, source_stamp_ns=2, xy_m=[2.5, 0.], yaw_degrees=0.)]
    group = dict(mission='A', stop_lines=[dict(points_xy_m=[[20., -2.], [20., 2.]])])
    return ego, loc, group


@pytest.mark.parametrize('mission', ['A', 'B', 'C', 'D'])
def test_start_uses_paired_actual_pose_and_applies_native_heading_correction_once(mission):
    ego, loc, group = samples()
    group['mission'] = mission
    before = copy.deepcopy((ego, loc))
    result = trial.select_start(ego, loc, line_geometry(), group, 20.,
        dict(translation_xyz_m=[0., 0., 0.], yaw_degrees=0.))
    assert result['native_pose'] == dict(x=2.5, y=0., z=28., yaw=0.)
    assert result['localization_stopline_distance_2d_m'] == 17.5
    assert result['ego_stopline_distance_2d_m'] == 17.5
    assert (ego, loc) == before


@pytest.mark.parametrize('changes', [dict(heading=20.), dict(receipt=.3), dict(ego_x=10.)])
def test_wrong_heading_old_pair_or_coordinate_mismatch_cannot_create_start(changes):
    ego, loc, group = samples(**changes)
    with pytest.raises(ValueError, match='No recorded'):
        trial.select_start(ego, loc, line_geometry(), group, 20.,
            dict(translation_xyz_m=[0., 0., 0.], yaw_degrees=0.))


def test_native_map_transform_is_separate_from_scene_heading_correction():
    assert trial.native_xy_in_map(2., 3., dict(translation_xyz_m=[10., 20., 0.], yaw_degrees=90.)) == pytest.approx((7., 22.))


def test_trial_copy_is_low_volume_and_preserves_full_default_case(tmp_path):
    original = dict(campaign_name='old', scenarios={}, recording={}, upload=dict(enabled=True),
                    stack=dict(pose_source='localization'),
                    default_case=dict(goal=dict(x=1), waypoints=[dict(x=2)], timeout_sec=600))
    before = copy.deepcopy(original)
    full = trial.trial_document(original, tmp_path, 'full_regression', 'full.json')
    assert full['default_case'] == original['default_case']
    assert full['recording']['minimum_free_disk_gib'] == 4.
    assert not full['recording']['video']['enabled']
    assert not any('/camera/' in topic or '/lidar' in topic for topic in full['recording']['selected_topics'])
    for topic in ('/tf', '/tf_static', '/ctrl_cmd', '/rosout', '/traffic_light_node/debug_runtime',
                  '/perception/traffic_light_recognition/traffic_signals',
                  '/evaluation/traffic_light_policy/fault',
                  '/evaluation/traffic_light_policy/source_signals'):
        assert topic in full['recording']['selected_topics']
    focus = trial.trial_document(original, tmp_path, 'A', 'A.json', dict(x=3))
    assert focus['default_case']['waypoints'] == []
    assert focus['default_case']['goal'] == dict(x=3)
    assert original == before


def test_output_must_be_fresh_and_inside_evaluation_root(tmp_path):
    with pytest.raises(ValueError):
        trial.validate_output_directory(tmp_path/'outside')
    with pytest.raises(FileExistsError):
        trial.validate_output_directory(trial.ROOT/'artifacts/evaluation')
