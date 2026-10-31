import copy
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from base5_scale_capture_contract import native_size_review, validate_entry


def fixture(tmp_path):
    raw=dict(UNIQUEID=8500,DataID=40100001,scale=dict(x=1.5,y=1.5,z=1.5))
    meta=dict(runtime_actors=[dict(unique_id=8500,data_id=40100001,name='CargoBox',
        source_list='objectList',motion='static',estimated_size=[1.5,1.5,1.5])],
        scale_probe=dict(parent_uid=8500,asset='CargoBox',requested_scale=[1.5]*3))
    scene=tmp_path/'scene.json'; metadata=tmp_path/'meta.json'
    scene.write_text(json.dumps(dict(objectList=[raw])));metadata.write_text(json.dumps(meta))
    entry=dict(scenario=str(scene),metadata=str(metadata))
    deck=dict(config=dict(capture=dict(asset_scale_probe=True)),scenes=[entry],status='DIAGNOSTIC_ONLY')
    return raw,meta,entry,deck


def test_bounded_probe_passes(tmp_path):
    _,_,entry,deck=fixture(tmp_path)
    validate_entry(entry,deck)


@pytest.mark.parametrize('change',['no_flag','bulk','false_identity','wrong_scale','wrong_status'])
def test_invalid_probe_fails_before_mr(tmp_path,change):
    _,meta,entry,deck=fixture(tmp_path)
    if change=='no_flag':deck['config']['capture'].clear()
    if change=='bulk':deck['scenes']*=65
    if change=='false_identity':meta['runtime_actors'][0]['name']='Bollard'
    if change=='wrong_scale':meta['scale_probe']['requested_scale']=[2.]*3
    if change=='wrong_status':deck['status']='READY'
    Path(entry['metadata']).write_text(json.dumps(meta))
    with pytest.raises(ValueError):validate_entry(entry,deck)


def test_native_unscaled_dimensions_are_review_not_accept(tmp_path):
    _,meta,_,_=fixture(tmp_path)
    audit=dict(components=[dict(parent_uid=8500,raw_class='Obstacle',size_m=[1.]*3)])
    original=copy.deepcopy(audit)
    result=native_size_review(meta,audit)
    assert result['review_reasons']==['scaled_native_bbox_size_mismatch']
    assert audit==original
    audit['components'][0]['size_m']=[1.5]*3
    assert not native_size_review(meta,audit)['review_reasons']


def test_original_size_keeps_existing_qa(tmp_path):
    _,meta,_,_=fixture(tmp_path)
    meta['scale_probe']['requested_scale']=[1.]*3
    assert native_size_review(meta,dict(components=[]))['actors']==[]


def production_fixture(tmp_path):
    """Synthetic qualified policy with the generator's lean actor metadata."""
    import hashlib
    import base5_asset_scale as scale
    evidence=tmp_path/'evidence.json';evidence.write_text('{"synthetic_test_only":true}')
    policy=scale.candidate_policy([2,0,1])
    profile=policy['assets']['CargoBox']['profiles']['iso125']
    profile.update(status='qualified',native_geometry=dict(bbox_size_m=[1.,1.,.5],
        pivot_to_bbox_center_local_m=[0.,0.,.25],pivot_height_above_ground_m=0.),
        native_qualification=dict(observations=2,semantic_visual_review='accepted',contact_review='passed',
            sensor_gt_review='passed',axis_mapping_verified=True,single_component_obstacle_verified=True,
            evidence=[dict(path=str(evidence),sha256=hashlib.sha256(evidence.read_bytes()).hexdigest())]))
    actor=dict(unique_id=8500,data_id=40100001,name='CargoBox',source_list='objectList',
        motion='static',population_pool='obstacle',estimated_size=[1.,1.,.5])
    row=scale.attach_plan(dict(seed=19,actors=1,family='obstacle',mission=False),[actor],policy,['iso125'])
    assignment=scale.select(scale._identity_asset(actor),row,0,'static',policy,profile_id='iso125')
    actor['asset_scale']=assignment
    actor['static_height_initializer']=scale.height_initializer(assignment,policy,[1.,1.,.5],[0.,0.,.25])
    raw=scale.apply_actor(dict(UNIQUEID=8500,DataID=40100001),assignment,policy)
    scene=tmp_path/'production.json';scene.write_text(json.dumps(dict(objectList=[raw])))
    meta=tmp_path/'production_meta.json';meta.write_text(json.dumps(dict(spec=row,runtime_actors=[actor])))
    entry=dict(scenario=str(scene),metadata=str(meta))
    return entry,dict(config=dict(asset_scale_policy=policy),scenes=[entry])


def test_production_capture_hydrates_catalog_not_missing_runtime_semantic_fields(tmp_path):
    entry,deck=production_fixture(tmp_path)
    before=Path(entry['metadata']).read_bytes()
    assert 'semantic_class' not in json.loads(before)['runtime_actors'][0]
    validate_entry(entry,deck)
    assert Path(entry['metadata']).read_bytes()==before


def test_runtime_whitelist_does_not_override_ineligible_authoritative_catalog(tmp_path,monkeypatch):
    import build_base5_mr_asset_catalog as catalog
    entry,deck=production_fixture(tmp_path)
    invalid=tmp_path/'catalog.json'
    invalid.write_text(json.dumps(dict(assets=[dict(data_id=40100001,name='CargoBox',
        source_list='objectList',semantic_class='vehicle')])))
    monkeypatch.setattr(catalog,'DEFAULT_CATALOG',invalid)
    with pytest.raises(RuntimeError,match='catalog identity'):
        validate_entry(entry,deck)
