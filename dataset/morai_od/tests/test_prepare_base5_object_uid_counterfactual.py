import copy
from pathlib import Path
import sys
from unittest.mock import patch

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import prepare_base5_object_uid_counterfactual as prep


def fixture():
    actors, objects = [], []
    for ordinal, (name, data_id, factor) in enumerate([('RedBarrel', 40100003, 1.),
                                                      ('CargoBox', 40100001, 1.25)]):
        position = dict(x=10. + 10 * ordinal, y=0., z=7.)
        actors.append(dict(unique_id=8500+ordinal, name=name, data_id=data_id, source_list='objectList',
            population_pool='obstacle_box_barrel', motion='static', position=position,
            placement_yaw_deg=0., estimated_size=[1.,1.,1.],
            asset_scale=dict(profile_id='original' if ordinal == 0 else 'iso125', scale_xyz=[factor] * 3)))
        objects.append(dict(UNIQUEID=8500+ordinal, DataID=data_id, pos=copy.deepcopy(position),
                            rot=dict(yaw=0.), scale=dict(x=factor, y=factor, z=factor)))
    scene = dict(egoVehicle=dict(UNIQUEID=-1), objectList=objects,
                 shadedAreaList=[dict(UNIQUEID=12)], sensor_fixture=dict(m_SensorUniqueID=3))
    metadata = dict(runtime_actors=actors, ego=dict(x=0., y=0., z=7., yaw=0.),
        ground_support_check=dict(required_uids=[8500,8501]),
        spec=dict(asset_scale_plan=dict(source_spec=dict(seed=1, actors=2),
            profile_ids=['original','iso125'], actor_identity=[dict(unique_id=8500),dict(unique_id=8501)])))
    return scene, metadata


def fake_attach(row, actors, policy, profiles):
    return dict(row, asset_scale_plan=dict(source_spec=copy.deepcopy(row), profile_ids=profiles,
        actor_identity=[dict(unique_id=actor['unique_id']) for actor in actors]))


def remap(scene, meta, ordinal=0):
    # Synthetic tests isolate UID bookkeeping; the real four-scene output is
    # separately checked by unmodified validate_entry/unit.validate_entry.
    with patch.object(prep.scale, 'attach_plan', side_effect=fake_attach), \
            patch.object(prep.scale, 'select', side_effect=lambda actor,*a,**kw:actor['asset_scale']):
        return prep.remap_objects(scene, meta, 60000, ordinal, {})


def test_only_object_identity_changes_with_native_qa_ids_rebound():
    scene, metadata = fixture()
    before = copy.deepcopy((scene, metadata))
    raw, current, mapping = remap(scene, metadata)
    assert (scene, metadata) == before
    assert mapping == {8500:60000,8501:60001}
    assert raw['egoVehicle'] == scene['egoVehicle']
    assert raw['shadedAreaList'] == scene['shadedAreaList']
    assert raw['sensor_fixture'] == scene['sensor_fixture']
    assert current['ground_support_check']['required_uids'] == [60000,60001]
    assert [actor['unique_id'] for actor in current['spec']['asset_scale_plan']['actor_identity']] == [60000,60001]
    assert current['spec']['asset_scale_plan']['profile_ids'] == ['original','iso125']
    for old, new in zip(metadata['runtime_actors'], current['runtime_actors']):
        assert {key:value for key,value in old.items() if key!='unique_id'} == \
               {key:value for key,value in new.items() if key!='unique_id'}


def test_four_repeated_poses_receive_globally_unique_object_uid_bands():
    scene, metadata = fixture()
    all_ids = []
    for ordinal in range(4):
        _, current, mapping = remap(scene, metadata, ordinal)
        assert prep.digest(prep.geometry(current)) == prep.digest(prep.geometry(metadata))
        all_ids.extend(mapping.values())
    assert len(set(all_ids)) == len(all_ids) == 8


@pytest.mark.parametrize('problem', ['duplicate','foreign_ground','extra_object','nonobject_collision'])
def test_ambiguous_or_unsafe_identity_is_rejected(problem):
    scene, metadata = fixture()
    if problem == 'duplicate':metadata['runtime_actors'][1]['unique_id']=8500
    if problem == 'foreign_ground':metadata['ground_support_check']['required_uids'].append(999)
    if problem == 'extra_object':scene['objectList'].append(copy.deepcopy(scene['objectList'][0]))
    if problem == 'nonobject_collision':scene['egoVehicle']['UNIQUEID']=60000
    with pytest.raises(ValueError):
        remap(scene,metadata)


@pytest.mark.parametrize('base,ordinal', [(True,0),(9999,0),(65200,0),(60000,-1),(60000,4),(60000,True)])
def test_uid_bounds_are_finite_and_explicit_before_plan_access(base,ordinal):
    scene, metadata = fixture()
    with pytest.raises(ValueError,match='UID bands'):
        prep.remap_objects(scene,metadata,base,ordinal,{})
