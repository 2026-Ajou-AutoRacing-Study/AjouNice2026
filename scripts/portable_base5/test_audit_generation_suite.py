import copy
import json
from pathlib import Path
from unittest.mock import Mock

import pytest
import audit_generation_suite as audit


def row(environment,ident,x=0.,split='train',actor=1):
    return dict(id=environment+':'+ident,environment=environment,split=split,
        meta=dict(ego=dict(x=x,y=0.,z=0.,yaw=0.),scene_group_id=environment+':clip:'+ident,
            spec=dict(family='obstacle',mission=False),runtime_actors=[dict(name='Box',data_id=actor,
                source_list='objectList',position=dict(x=x+10,y=0.,z=0.),placement_yaw_deg=0.,
                estimated_size=[1.,1.,1.],motion='static',planned_center_distance_m=10.)]))


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value)+'\n')
    return path


def test_compaction_preserves_geometry_without_mutating_original_metadata():
    original=row('sunny13','one')['meta']
    original['runtime_actors'][0]['large_ground_evidence']={'unused':list(range(100))}
    before=copy.deepcopy(original)
    result=audit.compact(original,'derived_layout_group')
    assert audit.layout.geometry(result)==audit.layout.geometry(original)
    assert original==before and result['scene_group_id']=='derived_layout_group'
    assert 'large_ground_evidence' not in result['runtime_actors'][0]


def test_joint_exact_duplicate_cannot_hide_behind_different_environment_or_id():
    report=audit.joint_audit([row('sunny13','old'),row('sunny11','new',split='val')],'sunny11')
    assert not report['passed'] and len(report['candidate_duplicate_groups'])==1
    assert len(report['cross_split_groups'])==1


def test_joint_near_duplicate_is_not_mistaken_for_fresh_independent_layout():
    report=audit.joint_audit([row('sunny13','old'),row('sunny11','new',.02)],'sunny11')
    assert not report['passed'] and len(report['near_pairs'])==1


def test_actual_new_geometry_on_same_map_is_allowed():
    report=audit.joint_audit([row('sunny13','old'),row('sunny15','other',1.),
        row('sunny11','new',2.)],'sunny11')
    assert report['passed'] and report['groups']==3
    assert report['known_map_background_overlap_allowed']


def test_historical_duplicate_is_reported_without_rewriting_or_rejecting_new_independent_candidate():
    report=audit.joint_audit([row('sunny13','old'),row('sunny15','old_repeat'),
        row('sunny11','new',2.)],'sunny11')
    assert report['passed'] and len(report['cross_environment_groups'])==1
    assert report['candidate_duplicate_groups']==[]


def test_distribution_separates_parent_layouts_from_visible_gt():
    rows=[row('sunny11',str(i),i) for i in range(3)]
    for value,distance in zip(rows,[19.99,20.,30.]):
        value['meta']['runtime_actors'][0]['planned_center_distance_m']=distance
    rows[-1]['meta']['spec']['family']='shadow'
    rows[-1]['meta']['runtime_actors'][0]['motion']='walking'
    report=audit.distribution(rows,{'Box'})
    assert report['counts']['planned_parents']==3
    assert report['counts']['physical_0_20m']==1
    assert report['counts']['physical_20_30m']==1
    assert report['counts']['physical_30plus_m']==1
    assert report['counts']['shadow_moving_scenes']==1
    assert report['counts_are_scenario_parent_layouts_not_visible_gt']


def test_trusted_package_pickle_is_never_read_before_declared_checksum_validation(tmp_path,monkeypatch):
    path=tmp_path/'infos/512x1056/morai_base5_infos_train.pkl'
    path.parent.mkdir(parents=True);path.write_bytes(b'not a trusted pickle')
    write(tmp_path/'manifest.json',dict(scenes=5000,files={str(path.relative_to(tmp_path)):{'sha256':'0'*64}}))
    load=Mock();monkeypatch.setattr(audit.pickle,'loads',load)
    with pytest.raises(ValueError,match='declared file SHA differs'):
        audit.package_records(tmp_path,'sunny13',{})
    load.assert_not_called()


def test_package_input_symlink_cannot_escape_root_before_pickle_read(tmp_path,monkeypatch):
    root=tmp_path/'dataset';target=tmp_path/'outside.pkl';target.write_bytes(b'outside')
    path=root/'infos/512x1056/morai_base5_infos_train.pkl';path.parent.mkdir(parents=True)
    path.symlink_to(target)
    write(root/'manifest.json',dict(scenes=5000,files={str(path.relative_to(root)):{'sha256':audit.sha256(target)}}))
    load=Mock();monkeypatch.setattr(audit.pickle,'loads',load)
    with pytest.raises(ValueError,match='escaped the root'):
        audit.package_records(root,'sunny13',{})
    load.assert_not_called()


def test_existing_audit_output_is_never_overwritten(tmp_path):
    with pytest.raises(ValueError,match='Fresh audit output required'):
        audit.audit(tmp_path/'no_source',[],tmp_path)
