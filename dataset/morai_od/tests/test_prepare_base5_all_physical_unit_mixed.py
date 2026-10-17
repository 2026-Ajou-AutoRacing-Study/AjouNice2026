"""All-physical-unit controls preserve existing units, pose and raw inputs."""
import copy
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from prepare_base5_all_physical_unit_mixed import convert


def inputs():
    actors, raw = [], []
    for ordinal, (name, factor) in enumerate((('RedBarrel', 1.), ('CargoBox', 1.25))):
        position = dict(x=10.+ordinal*10, y=0., z=7.)
        actor = dict(unique_id=8500+ordinal, name=name, source_list='objectList',
            motion='static', position=position, estimated_size=[factor]*3,
            geometric_center_estimate=[position['x'], 0.], placement_yaw_deg=0.,
            static_height_initializer=dict(policy='unchanged-original'),
            planned_center_distance_m=position['x'], planned_surface_clearance_m=5.)
        if factor != 1.:
            actor.update(asset_scale=dict(profile_id='iso125', scale_xyz=[factor]*3),
                asset_scale_geometry=dict(placement_pivot_m=[0., 0., factor/2]))
        actors.append(actor)
        raw.append(dict(UNIQUEID=8500+ordinal, pos=copy.deepcopy(position),
            scale=dict(x=factor, y=factor, z=factor), rot=dict(yaw=0.), retained='nongeometry'))
    scene = dict(objectList=raw, egoVehicle=dict(UNIQUEID=-1, retained='ego'))
    meta = dict(runtime_actors=actors, spec=dict(asset_scale_plan=dict(source_spec=dict(seed=9))),
        ego=dict(x=0., y=0., yaw=0.), base=dict(yaw=0.))
    refs = dict(CargoBox=dict(bbox_size_m=[1., 1., 1.], pivot_to_bbox_center_local_m=[0., 0., .5]))
    return scene, meta, refs


def test_all_physical_unit_preserves_source_and_original_unit_raw_pose():
    scene, metadata, refs = inputs()
    before = copy.deepcopy((scene, metadata))
    result, current, proofs = convert(scene, metadata, refs)
    assert (scene, metadata) == before
    assert result['objectList'][0] == scene['objectList'][0]
    assert current['runtime_actors'][0]['position'] == metadata['runtime_actors'][0]['position']
    assert current['runtime_actors'][0]['static_height_initializer'] == metadata['runtime_actors'][0]['static_height_initializer']
    assert result['egoVehicle'] == scene['egoVehicle']
    assert result['objectList'][1]['retained'] == 'nongeometry'
    assert all([row['scale'][axis] for axis in 'xyz'] == [1., 1., 1.] for row in result['objectList'])
    assert current['spec'] == dict(seed=9)
    assert [row['original_unit_pose_and_height_preserved'] for row in proofs] == [True, False]
    assert not any(row['ground_certified'] or row['native_gt_modified'] for row in proofs)


def test_scaled_to_unit_uses_explicit_native_reference_and_preserves_bottom():
    scene, metadata, refs = inputs()
    refs['CargoBox']['pivot_to_bbox_center_local_m'][2] = .4
    result, current, _ = convert(scene, metadata, refs)
    assert current['runtime_actors'][1]['position']['z'] == pytest.approx(7.1)
    assert result['objectList'][1]['pos']['z'] == pytest.approx(7.1)
    assert current['runtime_actors'][1]['static_height_initializer']['source_envelope_bottom_preserved']
    assert not current['runtime_actors'][1]['static_height_initializer']['contact_verified']


def test_missing_scale_source_plan_or_native_reference_is_not_bypassed():
    scene, metadata, refs = inputs()
    with pytest.raises(ValueError, match='pinned native reference'):
        convert(scene, metadata, {})
    metadata['spec'] = {}
    with pytest.raises(ValueError, match='explicit source scale plan'):
        convert(scene, metadata, refs)
