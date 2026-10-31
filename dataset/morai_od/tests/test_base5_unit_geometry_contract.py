import copy
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from base5_capture_safety import atomic_json,sha256
import base5_unit_geometry_contract as unit


@pytest.fixture
def sample(tmp_path):
    from build_base5_mr_asset_catalog import DEFAULT_CATALOG
    source=tmp_path/'source.json'
    atomic_json(source,dict(assets={'RedBarrel':dict(data_id=40100003,bbox_size_m=[.5727,.5724,.9194])}))
    origin=dict(path=str(source),sha256=sha256(source),assets=['RedBarrel'])
    policy=tmp_path/'review.json'
    atomic_json(policy,dict(schema=unit.SCHEMA,training_gt_modified=False,sources=[origin],
        catalog_reference=dict(path=str(DEFAULT_CATALOG.resolve()),sha256=sha256(DEFAULT_CATALOG)),assets={
        'RedBarrel':dict(data_id=40100003,source_list='objectList',raw_class='Obstacle',
                         native_size_lwh_m=[.5727,.5724,.9194],reference_source=origin)}))
    ref=dict(path=str(policy),sha256=sha256(policy))
    meta={unit.KEY:ref,'runtime_actors':[dict(name='RedBarrel',data_id=40100003,
        source_list='objectList',motion='static',unique_id=8500,estimated_size=[.67,.66,1.61])]}
    audit=dict(components=[dict(parent_uid=8500,raw_class='Obstacle',size_m=[.5727,.5724,.9194])])
    return meta,audit,policy


def test_unit_native_change_is_review_and_raw_untouched(sample):
    meta,audit,_=sample
    assert not unit.review(meta,audit)['review_reasons']
    audit['components'][0]['size_m']=[v*1.25 for v in audit['components'][0]['size_m']]
    before=copy.deepcopy((meta,audit))
    result=unit.review(meta,audit)
    assert result['review_reasons']==['unit_native_bbox_size_mismatch']
    assert result['actors'][0]['native_size_lwh_m']==audit['components'][0]['size_m']
    assert before==(meta,audit)


def test_placement_envelope_is_not_expected_gt(sample):
    meta,audit,_=sample
    meta['runtime_actors'][0]['estimated_size']=[100.,100.,100.]
    assert unit.review(meta,audit)['actors'][0]['matched']


def test_untagged_legacy_is_unchanged(sample):
    meta,audit,_=sample
    meta.pop(unit.KEY)
    assert unit.review(meta,audit)==dict(enabled=False,actors=[],review_reasons=[])


@pytest.mark.parametrize('change', ['nonunit','moving','different_asset'])
def test_other_geometry_policies_are_not_touched(sample,change):
    meta,audit,_=sample
    if change=='nonunit':meta['runtime_actors'][0]['asset_scale']=dict(scale_xyz=[2.,2.,2.])
    if change=='moving':meta['runtime_actors'][0]['motion']='walking'
    if change=='different_asset':
        meta['runtime_actors'][0].update(name='BabyStroller3',data_id=40200013)
    result=unit.review(meta,audit)
    assert not result['actors'] and not result['review_reasons']


def test_component_change_is_not_merged_to_one_box(sample):
    meta,audit,_=sample
    audit['components'].append(copy.deepcopy(audit['components'][0]))
    assert unit.review(meta,audit)['review_reasons']==['unit_native_component_contract_changed']


def test_wrong_identity_is_contract_error(sample):
    meta,audit,_=sample
    meta['runtime_actors'][0]['data_id']=123
    with pytest.raises(unit.UnitGeometryContractError,match='identity'):unit.review(meta,audit)


@pytest.mark.parametrize('name', ['IgnoredAsset','BabyStroller3','YellowBarrel'])
def test_known_native_data_id_cannot_escape_review_by_renaming(sample,tmp_path,name):
    meta,audit,_=sample
    meta['runtime_actors'][0]['name']=name
    with pytest.raises(unit.UnitGeometryContractError,match='DataID/name identity'):
        unit.review(meta,audit)
    path=tmp_path/'renamed_metadata.json'
    atomic_json(path,meta)
    with pytest.raises(unit.UnitGeometryContractError,match='DataID/name identity'):
        unit.validate_entry(dict(metadata=str(path)),dict(config={unit.KEY:meta[unit.KEY]}))


@pytest.mark.parametrize('inventory', [None,{},'missing'])
def test_tagged_metadata_requires_explicit_actor_inventory(sample,tmp_path,inventory):
    meta,audit,_=sample
    if inventory=='missing':meta.pop('runtime_actors')
    else:meta['runtime_actors']=inventory
    with pytest.raises(unit.UnitGeometryContractError,match='runtime actor inventory'):
        unit.review(meta,audit)
    path=tmp_path/'missing_inventory.json'
    atomic_json(path,meta)
    with pytest.raises(unit.UnitGeometryContractError,match='runtime actor inventory'):
        unit.validate_entry(dict(metadata=str(path)),dict(config={unit.KEY:meta[unit.KEY]}))


def test_reference_tamper_and_tag_removal_fail_preflight(sample,tmp_path):
    meta,audit,policy=sample
    path=tmp_path/'metadata.json'
    atomic_json(path,meta)
    entry=dict(metadata=str(path))
    manifest=dict(config={unit.KEY:meta[unit.KEY]})
    unit.validate_entry(entry,manifest)
    hidden=copy.deepcopy(meta);hidden.pop(unit.KEY);atomic_json(path,hidden)
    with pytest.raises(unit.UnitGeometryContractError,match='removed'):unit.validate_entry(entry,manifest)
    policy.write_text('{}')
    with pytest.raises(unit.UnitGeometryContractError,match='changed'):unit.review(meta,audit)


def test_nonpositive_size_rejected(sample):
    meta,audit,_=sample
    audit['components'][0]['size_m'][0]=-1
    with pytest.raises(unit.UnitGeometryContractError,match='Invalid native'):unit.review(meta,audit)


@pytest.mark.parametrize('field', ['data_id','size','sources','origin'])
def test_rehashed_policy_still_bound_to_catalog_and_native_source(sample,field):
    meta,audit,path=sample
    policy=json.loads(path.read_text())
    if field=='data_id':policy['assets']['RedBarrel']['data_id']=999
    if field=='size':policy['assets']['RedBarrel']['native_size_lwh_m']=[100.,100.,100.]
    if field=='sources':policy['sources']=[]
    if field=='origin':policy['assets']['RedBarrel']['reference_source']={}
    atomic_json(path,policy)
    meta[unit.KEY]['sha256']=sha256(path)
    with pytest.raises(unit.UnitGeometryContractError):unit.review(meta,audit)
