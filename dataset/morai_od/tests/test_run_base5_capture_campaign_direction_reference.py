"""Final-direction reference는 새 입력 준비이며 원본 QA/GT를 바꾸지 않는다."""
import argparse
import copy
import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import run_base5_capture_campaign_direction_reference as driver


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,indent=2)+'\n')
    return path


def source(tmp_path,*,yaw=10.,target_z=28.,change=180):
    target=dict(x=3.,y=4.,z=target_z)
    points=[dict(x=1.,y=4.,z=target_z),dict(x=5.,y=4.,z=target_z)]
    actor=dict(unique_id=8500,data_id=1,name='Ped_Pet1',source_list='pedestrianList',
        motion='running',position=copy.deepcopy(points[0]),yaw=yaw,
        motion_path=dict(points=points,target_position=target,target_tolerance_m=.4))
    raw=dict(UNIQUEID=8500,DataID=1,pos=copy.deepcopy(points[0]),rot=dict(yaw=str(yaw)),speed=3.,
        isWaypointMode=True,waypointEventList=[])
    scenario=dict(pedestrianList=[raw],objectList=[],vehicleList=[],aiPedstrianList=[],waypointDataList=[])
    meta=dict(spec=dict(family='shadow',shadow_moving=True),split='train',runtime_actors=[actor],
        ego=dict(x=0.,y=0.,z=0.,yaw=0.))
    sp=write(tmp_path/'source'/'scene.json',scenario);mp=write(tmp_path/'source'/'metadata.json',meta)
    entry=dict(id='canonical',scenario=str(sp),metadata=str(mp),scenario_sha256=driver.sha256(sp),split='train',motion='running')
    manifest=write(tmp_path/'source'/'manifest.json',dict(count=1,scenes=[entry],config={}))
    preview=copy.deepcopy(meta);preview['runtime_actors'][0]['motion_path']['target_position']['z']-=.2
    path=preview['runtime_actors'][0]['motion_path']
    path['points']=[dict(x=5.,y=4.,z=27.8),dict(x=1.,y=4.,z=27.8)]
    path['ground_path_evidence']=dict(terrain_selected_direction_change_deg=change)
    ps=copy.deepcopy(scenario);ps['pedestrianList'][0]['rot']['yaw']='180.000'
    psp=write(tmp_path/'preview'/'scene.json',ps);pmp=write(tmp_path/'preview'/'metadata.json',preview)
    pe=dict(entry,scenario=str(psp),metadata=str(pmp),scenario_sha256=driver.sha256(psp))
    pm=write(tmp_path/'preview'/'manifest.json',dict(count=1,scenes=[pe],config={}))
    return manifest,pm


def test_final_yaw_reference_restores_original_pivot_not_preliminary_z(tmp_path):
    original,preview=source(tmp_path)
    before=driver.file_inventory(original.parent)
    result=driver.write_direction_reference(original,preview,tmp_path/'matched')
    deck,_,meta,scene=driver.single(result)
    actor=meta['runtime_actors'][0];raw=scene['pedestrianList'][0]
    assert actor['position']==dict(x=3.,y=4.,z=28.)
    assert float(raw['pos']['z'])==28. and float(raw['rot']['yaw'])==180.
    assert actor['yaw']==180. and actor['motion']=='standing' and 'motion_path' not in actor
    assert meta['calibration_only'] and meta['final_direction_reference']['original_target_xyz_restored']
    assert scene['waypointDataList']==[] and raw['speed']==0.
    assert driver.file_inventory(original.parent)==before
    assert deck['count']==1


def test_reference_overwrite_and_wrong_selected_yaw_are_fatal(tmp_path):
    original,preview=source(tmp_path)
    driver.write_direction_reference(original,preview,tmp_path/'matched')
    with pytest.raises(driver.DirectionReferenceContractError,match='Fresh'):
        driver.write_direction_reference(original,preview,tmp_path/'matched')
    _,_,meta,scene=driver.single(preview);scene['pedestrianList'][0]['rot']['yaw']='30'
    with pytest.raises(driver.DirectionReferenceContractError,match='path and raw'):
        driver.selected_yaws(driver.single(original)[2],meta,scene)


def test_three_actual_frozen_prepare_hooks_compile_without_executing_ros():
    contract=dict(quality_source_sha256=driver.sha256(driver.SOURCE))
    fn=driver.compiled_execute(contract)
    assert fn.__name__=='execute' and fn is not driver.quality.execute
    assert fn.__globals__['__file__']==driver.quality.__file__


def test_ast_source_changed_or_hook_count_changed_fails(tmp_path):
    contract=dict(quality_source_sha256='0'*64)
    with pytest.raises(driver.DirectionReferenceContractError,match='source changed'):
        driver.compiled_execute(contract)
    source=tmp_path/'changed.py';source.write_text('def execute():\n    return 1\n')
    with pytest.raises(driver.DirectionReferenceContractError,match='three'):
        driver.compiled_execute(dict(quality_source_sha256=driver.sha256(source)),source)


def test_installed_is_reversible_even_when_campaign_raises(monkeypatch):
    original=driver.quality.execute;replacement=lambda:None
    monkeypatch.setattr(driver,'compiled_execute',lambda c:replacement)
    with pytest.raises(RuntimeError):
        with driver.installed({}):
            assert driver.quality.execute is replacement
            raise RuntimeError('stop')
    assert driver.quality.execute is original


@pytest.mark.parametrize('margin',[1.,1.4])
def test_envelope_requires_more_independent_support_than_static_corners(margin):
    corners=np.asarray([[0.,0.,0.],[1.,0.,0.],[1.,1.,0.],[0.,1.,0.]])
    points=driver.envelope_points(corners,margin)
    assert points[:,0].min()==pytest.approx(-margin)
    assert points[:,0].max()==pytest.approx(1.+margin)


@pytest.mark.parametrize('margin',[.75,1.5,float('nan')])
def test_envelope_policy_is_bounded(margin):
    with pytest.raises(driver.DirectionReferenceContractError):
        driver.envelope_points(np.zeros((8,3)),margin)


def test_support_deficit_preserves_original_near_probe_error_contract():
    error=driver.CompoundEnvelopeUnsupported(dict(uid=1))
    assert isinstance(error,ValueError) and str(error)=='Static pivot terrain unsupported'
    assert error.details['uid']==1


def test_file_inventory_rejects_symlink_and_detects_added_file(tmp_path):
    write(tmp_path/'raw.json',{})
    before=driver.file_inventory(tmp_path)
    write(tmp_path/'added.json',{})
    assert driver.file_inventory(tmp_path)!=before
    (tmp_path/'link').symlink_to(tmp_path/'raw.json')
    with pytest.raises(driver.DirectionReferenceContractError,match='symlink'):
        driver.file_inventory(tmp_path)


def test_record_index_detects_rewrite_and_keeps_contract(tmp_path):
    contract=dict(capture_output=str(tmp_path))
    write(tmp_path/driver.RECORDS,dict(schema=driver.SCHEMA,contract_sha256=driver.digest(contract),records={}))
    value=dict(schema=driver.SCHEMA,policy=driver.POLICY)
    path=tmp_path/'direction_reference_preparations'/'x'/'recipe.json'
    driver.record(path,value,contract)
    index=driver.read(tmp_path/driver.RECORDS)
    assert index['records']=={str(path):driver.sha256(path)}
    with pytest.raises(driver.DirectionReferenceContractError,match='New campaign-local'):
        driver.record(path,value,contract)


@pytest.mark.parametrize('setting',['disabled','bulk','too_many','duplicate'])
def test_prototype_cannot_bypass_authority_or_cell_bound(tmp_path,setting):
    manifest=write(tmp_path/'source'/'manifest.json',dict(count=5,scenes=[dict(id=str(i)) for i in range(5)]))
    args=argparse.Namespace(manifest=manifest,output=tmp_path/'out',enable_final_direction_reference=True,
        approve_bulk_capture=False,indices='0',limit=1,resume=False,quota_rounds=1,seed_capture=None)
    if setting=='disabled':args.enable_final_direction_reference=False
    if setting=='bulk':args.approve_bulk_capture=True
    if setting=='too_many':args.indices='0,1,2,3,4'
    if setting=='duplicate':args.indices='0,0'
    with pytest.raises(driver.DirectionReferenceContractError):
        driver.run(args)
    assert not args.output.exists()


def test_input_pins_support_subset_with_original_full_specs(tmp_path,monkeypatch):
    original,_=source(tmp_path)
    full=write(tmp_path/'full_specs.json',[dict(x=i) for i in range(5000)])
    deck=driver.read(original);deck['config'].update(frozen_specs_path=str(full),frozen_specs_sha256=driver.sha256(full))
    write(original,deck)
    monkeypatch.setattr(driver.pins,'_default_references',lambda d:{})
    checked=driver.input_pins(original)
    assert checked[str(full)]==driver.sha256(full)
    write(full,[])
    with pytest.raises(driver.DirectionReferenceContractError,match='SHA differs'):
        driver.input_pins(original)


def test_verify_record_rechecks_runtime_source_and_native_session(tmp_path):
    raw=write(tmp_path/'session'/'raw.json',{})
    pin=write(tmp_path/'pin.json',{})
    path=write(tmp_path/'receipt.json',dict(schema=driver.SCHEMA,policy=driver.POLICY,
        input_hashes={str(pin):driver.sha256(pin)},runtime_hashes={str(pin):driver.sha256(pin)},
        session_inventories={str(raw.parent):driver.file_inventory(raw.parent)}))
    driver.verify_record(path)
    write(raw,dict(changed=True))
    with pytest.raises(driver.DirectionReferenceContractError,match='inventory changed'):
        driver.verify_record(path)


def test_near_probe_reentry_reuses_one_final_direction_reference(tmp_path,monkeypatch):
    original,preview=source(tmp_path)
    base=tmp_path/'campaign'/'cell';base.mkdir(parents=True)
    runtime=write(tmp_path/'runtime.json',{})
    contract=dict(capture_output=str(tmp_path/'campaign'),runtime_hashes={str(runtime):driver.sha256(runtime)})
    write(tmp_path/'campaign'/driver.RECORDS,dict(schema=driver.SCHEMA,contract_sha256=driver.digest(contract),records={}))
    terrain=tmp_path/'terrain';reference=tmp_path/'reference';near=tmp_path/'near'
    for path in (terrain,reference,near):write(path/'evidence.json',{})
    monkeypatch.setattr(driver,'input_pins',lambda path:{str(original):driver.sha256(original)})
    def calibration(path,manifest=None):
        path=Path(path)
        return dict(root=str(path),session=str(path),inventory=driver.file_inventory(path))
    monkeypatch.setattr(driver,'calibration_session',calibration)
    monkeypatch.setattr(driver,'corners_by_uid',lambda root:{8500:[np.zeros((8,3)),np.ones((8,3))]})
    def prepare(source,terrain,reference,output,additional_terrain=None):
        output=Path(output);assert not output.exists()
        deck,entry,meta,scene=driver.single(preview)
        sp=write(output/'scene.json',scene);mp=write(output/'metadata.json',meta)
        entry.update(scenario=str(sp),metadata=str(mp),scenario_sha256=driver.sha256(sp))
        return write(output/'manifest.json',dict(deck,scenes=[entry]))
    monkeypatch.setattr(driver.ground,'prepare',prepare)
    capture_calls=[]
    def capture(manifest,root,cell,kind):
        capture_calls.append(kind);write(Path(root)/'native.json',dict(observed=True))
        return dict(status='accepted_candidate',audit={})
    def prepared(path,make):return path if path.exists() else make()
    expansions=[]
    def expand(pivot,reference,sessions,output):
        expansions.append(list(sessions))
        if len(expansions)==1:raise driver.CompoundEnvelopeUnsupported(dict(uid=8500))
        deck,entry,meta,scene=driver.single(pivot)
        sp=write(Path(output)/'scene.json',scene);mp=write(Path(output)/'metadata.json',meta)
        entry.update(scenario=str(sp),metadata=str(mp),scenario_sha256=driver.sha256(sp))
        return write(Path(output)/'manifest.json',dict(deck,scenes=[entry]))
    monkeypatch.setattr(driver,'expand_compound_support',expand)
    arguments=dict(prepared_callback=prepared,capture_callback=capture,cell='canonical',contract=contract)
    with pytest.raises(ValueError,match='Static pivot terrain unsupported'):
        driver.direction_prepare(original,terrain,reference,base/'motion_ground_deck',**arguments)
    result=driver.direction_prepare(original,terrain,reference,base/'motion_ground_deck',additional_terrain=[near],**arguments)
    assert Path(result).is_file()
    assert capture_calls==['motion_final_direction_static_reference']
    assert len(list((base/'direction_reference_preparations').glob('*')))==2
    assert len(list((base/'final_direction_references').glob('*')))==1
    assert list((base/'direction_reference_preparations').rglob('envelope_deficit_receipt.json'))


def test_without_native_compound_old_prepare_is_called_unchanged(tmp_path,monkeypatch):
    original,_=source(tmp_path)
    reference=tmp_path/'ref';write(reference/'native.json',{})
    monkeypatch.setattr(driver,'calibration_session',lambda p:dict(root=str(reference)))
    monkeypatch.setattr(driver,'corners_by_uid',lambda p:{8500:[np.zeros((8,3))]})
    runtime=write(tmp_path/'pin.json',{});calls=[]
    monkeypatch.setattr(driver.ground,'prepare',lambda *args,**kw:calls.append((args,kw)) or 'original-result')
    result=driver.direction_prepare(original,tmp_path/'terrain',reference,tmp_path/'out',
        prepared_callback=None,capture_callback=None,cell='canonical',contract=dict(runtime_hashes={str(runtime):driver.sha256(runtime)}))
    assert result=='original-result' and len(calls)==1
    assert calls[0][1]==dict(additional_terrain=None)


def test_completed_verifier_rejects_nonaccepted_quota_before_native_use(tmp_path,monkeypatch):
    original,_=source(tmp_path);out=tmp_path/'out'
    write(out/driver.GUARD,dict(indices=[0]))
    write(out/'status.json',dict(stage='complete_with_deficit'))
    monkeypatch.setattr(driver,'verify_campaign',lambda *a,**kw:dict(passed=True))
    with pytest.raises(driver.DirectionReferenceContractError,match='completed approved'):
        driver.verify_completed_campaign(original,0,out)


def test_completed_proof_accepts_registered_two_cell_campaign_shape_before_native_validation(tmp_path,monkeypatch):
    original,_=source(tmp_path);deck=driver.read(original)
    deck['scenes'].append(dict(deck['scenes'][0],id='second'));deck['count']=2;write(original,deck)
    out=tmp_path/'out'
    write(out/driver.GUARD,dict(indices=[0,1]));write(out/'status.json',dict(stage='complete'))
    write(out/'quota.json',dict(accepted=2,deficit=0))
    (out/'selected_ledger.jsonl').write_text(json.dumps(dict(id='canonical',status='review'))+'\n'+
        json.dumps(dict(id='second',status='review'))+'\n')
    monkeypatch.setattr(driver,'verify_campaign',lambda *a,**kw:dict(passed=True))
    # 2셀 캠페인을 1셀이라고 거부하지 않고 해당 quota의 실제 승인을 검사한다.
    with pytest.raises(driver.DirectionReferenceContractError,match='capture_root'):
        try:driver.verify_completed_campaign(original,1,out)
        except KeyError as error:raise driver.DirectionReferenceContractError(str(error))


def test_integrity_hash_failure_is_fatal_not_quality_valueerror(tmp_path):
    path=write(tmp_path/'pin.json',{})
    with pytest.raises(driver.DirectionReferenceContractError):
        driver.verify_hashes({str(path):'0'*64})


def test_runtime_closure_contains_adapter_and_all_original_capture_preparation_files():
    values=driver.runtime_hashes()
    names={Path(path).name for path in values}
    assert {'run_base5_capture_campaign_direction_reference.py','run_base5_quality_loop.py',
        'run_base5_capture_campaign_spatial.py','base5_motion_ground.py','base5_native_box_corners.py',
        'base5_ground_probe.py','base5_ground_support.py','base5_calibration_layout.py',
        'run_base5_capture.py'}<=names
