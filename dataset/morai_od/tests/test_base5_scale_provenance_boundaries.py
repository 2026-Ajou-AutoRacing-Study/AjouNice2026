"""Scaled RAW/audit/resume/conversion/portable boundaries, CPU-only fixtures."""
import copy
import hashlib
import importlib.util
import itertools
import json
from pathlib import Path
import pickle
import shutil
import sys

import numpy as np
import pytest

REPO=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(REPO/'dataset/morai_od/scripts'))
import base5_scale_capture_contract as contract
from audit_morai_lidar_gt_visibility import parse_instances
from base5_native_box_corners import load_matching_corners,derive_geometry
from base5_capture_safety import prepare_session
from report_base5_compound_evidence import _validate_native_geometry
from build_base5_bevfusion_infos import convert


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value)+'\n')
    return path


def fixture(tmp_path):
    root=tmp_path/'source'/'capture';lidar=root/'LIDAR_5';lidar.mkdir(parents=True)
    actor=dict(unique_id=8500,data_id=40100001,name='CargoBox',source_list='objectList',
        motion='static',estimated_size=[1.5,1.5,.75])
    meta=dict(runtime_actors=[actor],split='train',scene_group_id='group',
        spec=dict(family='obstacle'),
        scale_probe=dict(parent_uid=8500,asset='CargoBox',requested_scale=[1.5]*3))
    scene=dict(objectList=[dict(UNIQUEID=8500,DataID=40100001,scale=dict(x=1.5,y=1.5,z=1.5))])
    write(root/'scenario.json',scene);write(root/'metadata.json',meta)
    center=np.array([10.,0.,-1.]);size=np.array([1.5,1.5,.75])
    corners=np.array(list(itertools.product((-1.,1.),repeat=3)))*size/2+center
    annotation=lidar/'stamp_Instance.txt'
    annotation.write_text('Obstacle '+' '.join(map(str,[*center,0,0,0,1,1,.5,0,0,0,0,8500]))+'\n')
    (lidar/'stamp_Instance_8Points.txt').write_text('Obstacle '+' '.join(map(str,[*corners.reshape(-1),0,0,0,0,0,0,0,8500]))+'\n')
    box=parse_instances(annotation)[0]
    derived=derive_geometry(box,load_matching_corners(annotation)[(8500,'Obstacle',0)],allow_dimension_change=True)
    component=dict(component_id='8500:Obstacle:0',parent_uid=8500,raw_class='Obstacle',
        center_lidar_m=derived['center'],size_m=derived['size'],rotation_rpy=derived['rotation_rpy'],
        native_corner_geometry=derived['evidence'],native_summary_size_m=box['size'].tolist(),
        training_class='obstacle',asset='CargoBox',lidar_points=1,positive_lidar1_camera0=True)
    audit=dict(components=[component],review_reasons=[])
    write(root/'audit.json',audit)
    np.array([[10.,0.,-1.,1.]],dtype=np.float32).tofile(lidar/'stamp_Intensity.bin')
    for i in range(1,5):
        path=root/f'CAMERA_{i}'/'stamp_Intensity.png';path.parent.mkdir();path.write_bytes(b'fixture-rgb')
    for name in ('telemetry.json','collision_history.json'):write(root/name,{})
    entry=dict(id='s1',capture_root=str(root),scenario=str(root/'scenario.json'),metadata=str(root/'metadata.json'),
        scenario_sha256=hashlib.sha256((root/'scenario.json').read_bytes()).hexdigest(),status='accepted_candidate')
    deck=dict(config=dict(capture=dict(asset_scale_probe=True)),scenes=[entry],status='DIAGNOSTIC_ONLY')
    return root,meta,scene,audit,entry,deck


@pytest.mark.parametrize('kind',['missing','unit','wrong_uid','wrong_scale','duplicate','vehicle_scale'])
def test_raw_nonunit_cannot_be_hidden_by_metadata(tmp_path,kind):
    root,meta,scene,_,_,_=fixture(tmp_path)
    if kind=='missing':meta.pop('scale_probe')
    if kind=='unit':meta['scale_probe']['requested_scale']=[1.]*3
    if kind=='wrong_uid':meta['runtime_actors'][0]['unique_id']=8501
    if kind=='wrong_scale':meta['scale_probe']['requested_scale']=[1.25]*3
    if kind=='duplicate':meta['runtime_actors']*=2
    if kind=='vehicle_scale':scene['vehicleList']=scene.pop('objectList');write(root/'scenario.json',scene)
    with pytest.raises(ValueError):contract.registered_variants(root,meta)


def test_raw_unit_legacy_remains_noop_and_missing_variant_scenario_fails(tmp_path):
    root,meta,scene,_,_,_=fixture(tmp_path)
    scene['objectList'][0]['scale']=dict.fromkeys(('x','y','z'),1.)
    meta.pop('scale_probe');write(root/'scenario.json',scene)
    assert not contract.registered_variants(root,meta)
    (root/'scenario.json').unlink()
    assert not contract.registered_variants(root,meta)
    meta['spec']['asset_scale_plan']={}
    with pytest.raises(ValueError):contract.registered_variants(root,meta)


@pytest.mark.parametrize('kind',['drop_markers','partial_markers','wrong_size','missing_component','raw_corner_tamper'])
def test_scaled_native_validation_requires_exact_measured_proof(tmp_path,kind):
    root,_,_,audit,_,_=fixture(tmp_path);component=audit['components'][0]
    _validate_native_geometry(root,audit['components'])
    if kind=='drop_markers':
        component['size_m']=component.pop('native_summary_size_m');component.pop('native_corner_geometry')
    if kind=='partial_markers':component.pop('native_summary_size_m')
    if kind=='wrong_size':component['size_m'][0]*=2
    if kind=='missing_component':audit['components']=[]
    if kind=='raw_corner_tamper':
        p=root/'LIDAR_5'/'stamp_Instance_8Points.txt';p.write_text(p.read_text().replace('8500','8501'))
    with pytest.raises((ValueError,RuntimeError)):_validate_native_geometry(root,audit['components'])


def test_explicit_relocation_preserves_original_proof_and_rejects_other_changes(tmp_path):
    root,_,_,audit,_,_=fixture(tmp_path);original=copy.deepcopy(audit)
    relocated=tmp_path/'portable'/'data'/'s1';shutil.copytree(root,relocated)
    with pytest.raises(ValueError):_validate_native_geometry(relocated,audit['components'])
    _validate_native_geometry(relocated,audit['components'],original_capture_root=root)
    assert audit==original
    for changed in ('wrong_root','forged_bytes','forged_hash','wrong_relative_name','forged_uid'):
        value=copy.deepcopy(audit);old=root
        pin=value['components'][0]['native_corner_geometry']['source_files']['corners']
        if changed=='wrong_root':old=root.parent
        if changed=='forged_bytes':pin['bytes']+=1
        if changed=='forged_hash':pin['sha256']='0'*64
        if changed=='wrong_relative_name':pin['path']=str(root/'LIDAR_6'/Path(pin['path']).name)
        if changed=='forged_uid':value['components'][0]['native_corner_geometry']['parent_uid']=8501
        with pytest.raises((ValueError,RuntimeError)):
            _validate_native_geometry(relocated,value['components'],original_capture_root=old)


@pytest.mark.parametrize('kind',['valid','metadata_missing_scale','missing_marker','changed_source_spec'])
def test_completed_resume_rebinds_actual_scale_and_native_proof(tmp_path,kind):
    root,meta,_,audit,entry,deck=fixture(tmp_path)
    original_meta=write(tmp_path/'expected_metadata.json',meta);entry['metadata']=str(original_meta)
    manifest=write(tmp_path/'manifest.json',deck);network=write(tmp_path/'network.json',{});sensor=write(tmp_path/'sensor.json',{})
    output=tmp_path/'session';prepare_session(output,manifest,network,sensor)
    write(output/'ledger.jsonl',entry)
    if kind=='metadata_missing_scale':meta.pop('scale_probe');write(root/'metadata.json',meta)
    if kind=='missing_marker':audit['components'][0].pop('native_corner_geometry');write(root/'audit.json',audit)
    if kind=='changed_source_spec':meta['spec']['seed']=9;write(root/'metadata.json',meta)
    if kind=='valid':assert prepare_session(output,manifest,network,sensor)[1]['s1']==entry
    else:
        with pytest.raises((ValueError,RuntimeError)):prepare_session(output,manifest,network,sensor)


@pytest.mark.parametrize('kind',['valid','metadata_missing_scale','missing_marker','wrong_geometry','scenario_hash'])
def test_common_converter_cannot_bypass_scaled_raw_checks(tmp_path,kind):
    root,meta,_,audit,entry,_=fixture(tmp_path)
    if kind=='metadata_missing_scale':meta.pop('scale_probe');write(root/'metadata.json',meta)
    if kind=='missing_marker':audit['components'][0].pop('native_corner_geometry');write(root/'audit.json',audit)
    if kind=='wrong_geometry':audit['components'][0]['size_m'][0]*=2;write(root/'audit.json',audit)
    if kind=='scenario_hash':entry['scenario_sha256']='0'*64
    ledger=write(tmp_path/'selected.jsonl',entry)
    args=(ledger,tmp_path/'infos',REPO/'configs/sensors/Sensor_competition_local.json')
    if kind=='valid':assert convert(*args)['splits']['train']==1
    else:
        with pytest.raises((ValueError,RuntimeError)):convert(*args)


def test_portable_export_retains_scaled_proof_and_verifies_byte_exact_move(tmp_path):
    root,_,_,_,entry,_=fixture(tmp_path)
    ledger=write(tmp_path/'selected.jsonl',entry);infos=tmp_path/'infos'
    convert(ledger,infos,REPO/'configs/sensors/Sensor_competition_local.json')
    path=REPO/'perception/detection3d/bevfusion/tools/prepare_base5_portable_dataset.py'
    spec=importlib.util.spec_from_file_location('scale_portable_fixture',path)
    portable=importlib.util.module_from_spec(spec);spec.loader.exec_module(portable)
    output=tmp_path/'portable';result=portable.build(ledger,{'512':infos},output,1)
    assert result['provenance']['s1']['native_corner_relocation_policy']
    moved=tmp_path/'moved';output.rename(moved);portable.validate(moved,verify_hashes=True)
    assert (root/'audit.json').read_bytes()==(moved/'data/s1/audit.json').read_bytes()
    # Even when generic inventory hashing is not requested, semantic proof is
    # checked against real measured native geometry, not package assertions.
    p=moved/'data/s1/LIDAR_5/stamp_Instance_8Points.txt'
    p.write_text(p.read_text().replace('8500','8501'))
    with pytest.raises((ValueError,RuntimeError)):portable.validate(moved,verify_hashes=False)
