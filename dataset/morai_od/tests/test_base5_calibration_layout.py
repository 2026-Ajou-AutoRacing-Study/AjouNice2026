"""CPU-only fresh calibration detachment and canonical training rebind tests."""
import copy
import json
from pathlib import Path
import shutil
import sys

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import base5_calibration_layout as preparation
import run_base5_quality_loop as quality
from prepare_base5_layout_split import prepare as prepare_layout


def write(path,value):
    preparation.atomic_json(path,value)
    return path


@pytest.fixture
def source(tmp_path):
    catalog=write(tmp_path/'catalog.json',dict(assets=[dict(name='Box',data_id=100,
        source_list='objectList',semantic_class='obstacle')]))
    native=write(tmp_path/'native_units.json',dict(assets={'Box':dict(data_id=100,
        source_list='objectList',scale_xyz=[1.,1.,1.],bbox_size_m=[1.,2.,.5])}))
    origin=dict(path=str(native),sha256=preparation.sha256(native),assets=['Box'])
    policy=write(tmp_path/'unit.json',dict(schema=preparation.unit.SCHEMA,training_gt_modified=False,
        catalog_reference=dict(path=str(catalog),sha256=preparation.sha256(catalog)),sources=[origin],
        assets={'Box':dict(data_id=100,source_list='objectList',raw_class='Obstacle',
            native_size_lwh_m=[1.,2.,.5],reference_source=origin)}))
    reference=dict(path=str(policy),sha256=preparation.sha256(policy));entries=[]
    for i in range(30):
        ego=dict(x=float(i*2),y=0.,z=0.,yaw=0.,link_idx='shared-road')
        actor=dict(name='Box',data_id=100,unique_id=8500,source_list='objectList',motion='static',
            position=dict(x=ego['x']+3.,y=1.,z=0.),placement_yaw_deg=0.,estimated_size=[1.,2.,.5])
        meta=dict(ego=ego,base=ego,split='train',scene_group_id='clip'+str(i),runtime_actors=[actor],
            spec=dict(family='obstacle',mission=False,role='front',actors=1,motion='static',
                distance_bin=0,distance_bounds=[0,20],yaw_limit=3,topology='route_general'),
            **{preparation.unit.KEY:reference})
        scene=dict(objectList=[dict(UNIQUEID=8500,DataID=100,pos=copy.deepcopy(actor['position']),
            scale=dict(x=1.,y=1.,z=1.,_x='1.000',_y='1.000',_z='1.000'))])
        second=copy.deepcopy(actor);second.update(unique_id=8501)
        second['position']['y']=5.;meta['runtime_actors'].append(second)
        raw_second=copy.deepcopy(scene['objectList'][0]);raw_second.update(UNIQUEID=8501,pos=copy.deepcopy(second['position']))
        scene['objectList'].append(raw_second);meta['spec']['actors']=2
        sp=write(tmp_path/'source'/(str(i)+'.json'),scene);mp=write(sp.with_name(str(i)+'_metadata.json'),meta)
        entries.append(dict(id='source'+str(i),scenario=str(sp),metadata=str(mp),scenario_sha256=preparation.sha256(sp),split='train'))
    full=write(tmp_path/'source/manifest.json',dict(count=30,scenes=entries,
        config={preparation.unit.KEY:reference,'capture':dict(natural_setup_budget_sec=6)},provenance={}))
    final=prepare_layout(full,tmp_path/'candidate');deck=preparation.read(final['manifest'])
    entry=deck['scenes'][5];deck.update(count=1,scenes=[entry])
    return dict(root=tmp_path,deck=deck,entry=entry,meta=preparation.read(entry['metadata']))


def prepared(source,kind='calibration',name='prepared'):
    folder=source['root']/name;deck=copy.deepcopy(source['deck']);entry=copy.deepcopy(source['entry'])
    scene=preparation.read(entry['scenario']);meta=copy.deepcopy(source['meta'])
    if kind=='calibration':
        scene['objectList']=[];meta.update(calibration_only=True,runtime_actors=[])
        meta['spec'].update(family='background',actors=0,motion='static')
        meta['ego']=dict(meta['ego'],x=meta['ego']['x']-1.)
    elif kind=='height':
        scene['objectList'][0]['pos']['z']+=.02;meta['runtime_actors'][0]['position']['z']+=.02
    elif kind=='qa':
        meta['ground_reference_test']=dict(measured=True)
    sp=folder/'scenario.json';mp=folder/'metadata.json'
    if kind in ('qa','bound'):sp.parent.mkdir(parents=True);shutil.copyfile(entry['scenario'],sp)
    else:write(sp,scene)
    write(mp,meta);entry.update(scenario=str(sp),metadata=str(mp),scenario_sha256=preparation.sha256(sp))
    if kind!='bound':entry['id']+='_'+kind
    deck['scenes']=[entry]
    manifest=write(folder/'manifest.json',deck)
    return manifest


def normalize(source,manifest,output='normalized',**kwargs):
    return preparation.normalize_prepared_deck(manifest,source['deck'],source['entry'],
        source['root']/output,'test-stage',**kwargs)


def test_calibration_detaches_only_derived_training_contract_and_preserves_every_input(source):
    manifest=prepared(source);before={p:p.read_bytes() for p in source['root'].rglob('*') if p.is_file()}
    result=normalize(source,manifest);deck=preparation.read(result);entry=deck['scenes'][0];meta=preparation.read(entry['metadata'])
    assert 'split_overlay' not in deck and 'training_split_overlay' not in meta
    assert 'derived_training_split' not in entry and 'derived_layout_group_id' not in entry
    assert deck['calibration_only'] and not deck['training_quota_eligible'] and not deck['bulk_ready']
    assert meta['calibration_only'] and meta[preparation.unit.KEY]==source['meta'][preparation.unit.KEY]
    proof=meta[preparation.KEY]
    assert proof['training_quota_eligible'] is False and proof['training_overlay_detached'] is True
    assert proof['independent_terrain_preparation_claimed'] is False
    assert proof['source_entry']==source['entry']
    assert proof['helper_sha256']==preparation.sha256(preparation.__file__)
    assert entry['scenario_sha256']==preparation.sha256(entry['scenario'])
    assert all(p.read_bytes()==data for p,data in before.items())
    assert normalize(source,manifest)==result


def test_already_bound_training_input_is_not_rebound_or_given_new_ordinal(source):
    manifest=prepared(source,'bound');root=source['root']/'normalized'
    assert normalize(source,manifest)==manifest
    assert not root.exists()


def test_final_geometry_uses_real_canonical_api_not_overlay_tag_copy(source):
    manifest=prepared(source,'height');context=quality.replacement_contract(source['deck'],source['entry'],source['meta'])
    result=normalize(source,manifest,bind_training=quality.bind_replacement_contract,context=context)
    deck=preparation.read(result);entry=deck['scenes'][0];meta=preparation.read(entry['metadata'])
    assert entry['id']!=source['entry']['id'] and entry['source_cell_id']==entry['id']
    assert meta['fresh_completion_source']['source_cell_id']==source['entry']['id']
    assert meta['quality_replacement_contract']['root_entry']==source['entry']
    assert deck['layout_split_finalization_pending'] and not deck['bulk_ready']
    assert meta[preparation.unit.KEY]==source['meta'][preparation.unit.KEY]
    assert preparation.layout.validate_manifest_overlay(deck,allow_subset=True)['errors']==[]
    assert normalize(source,manifest,bind_training=quality.bind_replacement_contract,context=context)==result


def test_metadata_only_same_payload_uses_new_pretag_pin_and_same_source_identity(source):
    manifest=prepared(source,'qa');result=normalize(source,manifest)
    deck=preparation.read(result);entry=deck['scenes'][0];meta=preparation.read(entry['metadata'])
    assert entry['id']==source['entry']['id'] and entry['scenario_sha256']==source['entry']['scenario_sha256']
    assert meta['ground_reference_test']==dict(measured=True)
    assert meta['training_split_overlay']['source_metadata_path']!=source['meta']['training_split_overlay']['source_metadata_path']
    assert preparation.layout.validate_manifest_overlay(deck,allow_subset=True)['errors']==[]


def test_calibration_can_keep_a_valid_stationary_source_subset(source):
    manifest=prepared(source);deck=preparation.read(manifest);entry=deck['scenes'][0]
    meta=preparation.read(entry['metadata']);scene=preparation.read(source['entry']['scenario'])
    meta['runtime_actors']=copy.deepcopy(source['meta']['runtime_actors'][:1]);scene['objectList']=scene['objectList'][:1]
    write(Path(entry['scenario']),scene);write(Path(entry['metadata']),meta)
    entry['scenario_sha256']=preparation.sha256(entry['scenario']);write(manifest,deck)
    result=normalize(source,manifest)
    actual=preparation.read(preparation.read(result)['scenes'][0]['metadata'])
    assert actual['runtime_actors']==meta['runtime_actors']
    assert actual[preparation.KEY]['training_quota_eligible'] is False


@pytest.mark.parametrize('kind',['foreign_uid','entry_split','metadata_split'])
def test_calibration_source_actor_and_capture_split_cannot_be_substituted(source,kind):
    manifest=prepared(source);deck=preparation.read(manifest);entry=deck['scenes'][0]
    meta=preparation.read(entry['metadata']);scene=preparation.read(source['entry']['scenario'])
    meta['runtime_actors']=copy.deepcopy(source['meta']['runtime_actors'])
    if kind=='foreign_uid':
        scene['objectList'][0]['UNIQUEID']=9999;meta['runtime_actors'][0]['unique_id']=9999
    if kind=='entry_split':entry['split']='val'
    if kind=='metadata_split':meta['split']='val'
    write(Path(entry['scenario']),scene);write(Path(entry['metadata']),meta)
    entry['scenario_sha256']=preparation.sha256(entry['scenario']);write(manifest,deck)
    with pytest.raises(preparation.CalibrationLayoutError):normalize(source,manifest)


@pytest.mark.parametrize('kind',['order','uid_position','micro_coordinate','ego'])
def test_same_payload_qa_requires_exact_ordered_actor_and_ego_metadata(source,kind):
    manifest=prepared(source,'qa');deck=preparation.read(manifest);meta=preparation.read(deck['scenes'][0]['metadata'])
    actors=meta['runtime_actors']
    if kind=='order':actors.reverse()
    if kind=='uid_position':actors[0]['unique_id'],actors[1]['unique_id']=actors[1]['unique_id'],actors[0]['unique_id']
    if kind=='micro_coordinate':actors[0]['position']['x']+=1e-8
    if kind=='ego':meta['ego']['x']+=1e-8
    write(Path(deck['scenes'][0]['metadata']),meta)
    with pytest.raises(preparation.CalibrationLayoutError,match='ordered actor|exact Ego'):
        normalize(source,manifest,bind_training=quality.bind_replacement_contract,
            context=quality.replacement_contract(source['deck'],source['entry'],source['meta']))


def test_already_bound_fast_path_does_not_accept_rounding_hidden_actor_changes(source):
    manifest=prepared(source,'bound');deck=preparation.read(manifest);entry=deck['scenes'][0]
    meta=preparation.read(entry['metadata']);meta['runtime_actors'][0]['position']['x']+=1e-8
    body={k:v for k,v in meta.items() if k!='training_split_overlay'}
    pretag=write(manifest.parent/'new_source_metadata.json',body)
    meta['training_split_overlay'].update(source_metadata_path=str(pretag),source_metadata_sha256=preparation.sha256(pretag))
    write(Path(entry['metadata']),meta)
    assert preparation.layout.validate_manifest_overlay(deck,allow_subset=True)['errors']==[]
    with pytest.raises(preparation.CalibrationLayoutError,match='exact Ego/ordered actor'):
        normalize(source,manifest)


@pytest.mark.parametrize('kind',['spec','base','scene_group_id','logical_quota_slot_id','entry_quota'])
def test_already_bound_fast_path_preserves_frozen_spec_and_source_quota(source,kind):
    manifest=prepared(source,'bound');deck=preparation.read(manifest);entry=deck['scenes'][0]
    meta=preparation.read(entry['metadata'])
    if kind=='entry_quota':entry['family']='other-family';write(manifest,deck)
    elif kind=='spec':meta['spec']['role']='rear'
    elif kind=='base':meta['base']['x']+=1e-8
    else:meta[kind]='other-quota'
    pretag=write(manifest.parent/'new_source_metadata.json',{k:v for k,v in meta.items() if k!='training_split_overlay'})
    meta['training_split_overlay'].update(source_metadata_path=str(pretag),source_metadata_sha256=preparation.sha256(pretag))
    write(Path(entry['metadata']),meta)
    assert preparation.layout.validate_manifest_overlay(deck,allow_subset=True)['errors']==[]
    with pytest.raises(preparation.CalibrationLayoutError,match='frozen spec/base/source quota|frozen source entry quota'):
        normalize(source,manifest)


@pytest.mark.parametrize('kind',['raw_scale','mirror','missing_object_scale','unit','actor','moving_calibration',
    'missing_source_tag','foreign_overlay','config','missing_callback'])
def test_invalid_preparation_never_publishes_an_eligible_capture_deck(source,kind):
    manifest=prepared(source,'height' if kind=='missing_callback' else 'calibration')
    deck=preparation.read(manifest);entry=deck['scenes'][0]
    meta=preparation.read(entry['metadata']);scene=preparation.read(entry['scenario'])
    if kind in ('raw_scale','mirror','missing_object_scale','actor'):
        scene=preparation.read(source['entry']['scenario']);meta['runtime_actors']=copy.deepcopy(source['meta']['runtime_actors'])
        if kind=='raw_scale':scene['objectList'][0]['scale']['x']=1.25
        if kind=='mirror':scene['objectList'][0]['scale']['_x']='1.25'
        if kind=='missing_object_scale':scene['objectList'][0].pop('scale')
        if kind=='actor':meta['runtime_actors'][0]['name']='HiddenBox'
    if kind=='unit':meta.pop(preparation.unit.KEY)
    if kind=='moving_calibration':
        scene=preparation.read(source['entry']['scenario']);meta['runtime_actors']=copy.deepcopy(source['meta']['runtime_actors'])
        meta['runtime_actors'][0]['motion']='walking'
    if kind=='missing_source_tag':meta.pop('training_split_overlay')
    if kind=='foreign_overlay':meta['training_split_overlay']['sha256']='0'*64
    if kind=='config':deck['config']['capture']['unrelated_change']=True
    write(Path(entry['scenario']),scene);write(Path(entry['metadata']),meta);entry['scenario_sha256']=preparation.sha256(entry['scenario']);write(manifest,deck)
    with pytest.raises((RuntimeError,ValueError,KeyError)):normalize(source,manifest)
    assert not (source['root']/'normalized/manifest.json').exists()


@pytest.mark.parametrize('kind',['prepared_input','cached_output','stage','receipt',
    'omit_manifest_pin','omit_metadata_pin','extra_output_file','missing_output_file'])
def test_cached_normalization_is_read_only_and_fails_changed_contract(source,kind):
    manifest=prepared(source);result=normalize(source,manifest)
    if kind=='prepared_input':
        deck=preparation.read(manifest);write(Path(deck['scenes'][0]['metadata']),{})
    if kind=='cached_output':write(result,{})
    if kind=='receipt':write(result.parent/'normalization.json',{})
    if kind in ('omit_manifest_pin','omit_metadata_pin'):
        receipt=result.parent/'normalization.json';saved=preparation.read(receipt)
        name='manifest.json' if kind=='omit_manifest_pin' else 'metadata.json'
        saved['output_hashes'].pop(str((result.parent/name).resolve()))
        write(result.parent/name,{});write(receipt,saved)
    if kind=='extra_output_file':write(result.parent/'untracked.json',{})
    if kind=='missing_output_file':(result.parent/'scenario.json').unlink()
    with pytest.raises((RuntimeError,ValueError,KeyError)):
        if kind=='stage':
            preparation.normalize_prepared_deck(manifest,source['deck'],source['entry'],result.parent,'other-stage')
        else:normalize(source,manifest)


def test_untagged_legacy_workflow_is_unchanged(tmp_path):
    manifest=write(tmp_path/'legacy.json',{})
    assert preparation.normalize_prepared_deck(manifest,{}, {},tmp_path/'out','legacy')==manifest
    assert not (tmp_path/'out').exists()
