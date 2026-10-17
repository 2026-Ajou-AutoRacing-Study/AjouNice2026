import copy
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from prepare_base5_redbarrel_all_unit_mixed import all_red_unit_geometry, prepare


def fixture():
    actors, raw = [], []
    values = [('RedBarrel', 40100003, 'iso150', 1.5, [.859, .859, 1.379], [0., 0., .6895]),
              ('RedBarrel', 40100003, 'original', 1., [.6746, .6651, 1.6067], [.0084, .0019, .6555]),
              ('CargoBox', 40100001, 'iso125', 1.25, [.936, .936, .543], [0., 0., .306])]
    for index, (name, data_id, profile, factor, size, offset) in enumerate(values):
        position = dict(x=10. + index * 10, y=0., z=7.)
        actors.append(dict(unique_id=8500+index, data_id=data_id, name=name, source_list='objectList',
            motion='static', position=position, estimated_size=size,
            geometric_center_estimate=[position['x'], 0.], placement_yaw_deg=0.,
            planned_center_distance_m=position['x'], planned_surface_clearance_m=position['x']-4.,
            asset_scale=dict(profile_id=profile, scale_xyz=[factor] * 3),
            asset_scale_geometry=dict(placement_pivot_m=offset),
            static_height_initializer=dict(policy='saved-original-unit' if factor == 1. else 'qualified-scaled',
                                           usable=factor != 1., contact_verified=False)))
        raw.append(dict(UNIQUEID=8500+index, DataID=data_id, pos=copy.deepcopy(position),
                        rot=dict(yaw=0.), scale=dict(x=factor, y=factor, z=factor)))
    metadata = dict(spec=dict(asset_scale_plan=dict(source_spec=dict(seed=1, actors=3))),
        runtime_actors=actors, ego=dict(x=0., y=0., z=0., yaw=0.), base=dict(yaw=0.))
    reference = dict(data_id=40100003, bbox_size_m=[.5727, .5724, .9194],
                     pivot_to_bbox_center_local_m=[.007, .002, .4597])
    return dict(objectList=raw), metadata, reference


def test_only_red_geometry_changes_and_source_is_immutable():
    scene, metadata, reference = fixture()
    before = copy.deepcopy((scene, metadata))
    result, current, proofs = all_red_unit_geometry(scene, metadata, reference)
    assert (scene, metadata) == before
    assert len(proofs) == 2
    assert current['runtime_actors'][2] == metadata['runtime_actors'][2]
    assert result['objectList'][2] == scene['objectList'][2]
    assert result['objectList'][2]['scale']['x'] == 1.25
    for actor in current['runtime_actors'][:2]:
        assert actor['estimated_size'] == reference['bbox_size_m']
    assert [result['objectList'][0]['scale'][axis] for axis in 'xyz'] == [1., 1., 1.]


def test_large_conservative_unit_envelope_does_not_lower_existing_unit_pivot():
    scene, metadata, reference = fixture()
    old_unit = copy.deepcopy(metadata['runtime_actors'][1])
    result, current, proofs = all_red_unit_geometry(scene, metadata, reference)
    assert current['runtime_actors'][1]['position'] == old_unit['position']
    assert current['runtime_actors'][1]['static_height_initializer'] == old_unit['static_height_initializer']
    assert result['objectList'][1] == scene['objectList'][1]
    original_proof = next(proof for proof in proofs if proof['parent_uid'] == 8501)
    assert original_proof['original_unit_pose_and_height_preserved']
    assert not original_proof['old_conservative_unit_envelope_is_mesh_ground_truth']


def test_enlarged_red_reuses_qualified_bottom_initializer_not_default_height():
    scene, metadata, reference = fixture()
    _, current, proofs = all_red_unit_geometry(scene, metadata, reference)
    actor = current['runtime_actors'][0]
    assert actor['static_height_initializer']['source_envelope_bottom_preserved']
    assert not actor['static_height_initializer']['contact_verified']
    assert actor['position']['z'] == pytest.approx(7.)
    assert not proofs[0]['original_unit_pose_and_height_preserved']


@pytest.mark.parametrize('repeats', [0, 1, 3, True])
def test_attempt_budget_is_exactly_two_by_two_before_any_source_access(repeats):
    with pytest.raises(ValueError, match='Exactly two layouts'):
        prepare('/missing', '/not-created', '/reference', {}, repeats=repeats)
