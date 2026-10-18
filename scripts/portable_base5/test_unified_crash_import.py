"""CPU-only halted-prefix fixtures. No ROS, simulator, or production writes."""
import copy
import fcntl
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

import unified_crash_import as crash


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value)+'\n')
    return path


def ledger(path,rows):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(''.join(json.dumps(row)+'\n' for row in rows))


@pytest.fixture
def sample(tmp_path):
    old,new,scripts = tmp_path/'old',tmp_path/'new',tmp_path/'runtime'
    old.mkdir();scripts.mkdir()
    driver = write(scripts/'run_base5_batch_campaign.py',{'frozen':'driver'})
    write(scripts/'base5_campaign_replenishment.py',{'frozen':'replenishment'})
    write(scripts/'base5_campaign_import.py',{'frozen':'import'})
    entries,native,selected = [],[],[]
    for index in range(8):
        ident = 'scene_%02d'%index
        scenario = write(tmp_path/'source'/(ident+'.json'),{'index':index})
        metadata = write(tmp_path/'source'/(ident+'_meta.json'),{'calibration_only':False})
        entries.append(dict(id=ident,scenario=str(scenario),scenario_sha256=crash.sha(scenario),metadata=str(metadata)))
        capture = old/('chunk_%04d'%(index//4))/'capture/captures'/ident
        write(capture/'scenario.json',{'index':index});write(capture/'metadata.json',{'calibration_only':False})
        write(capture/'audit.json',{'review_reasons':[]});(capture/'sensor.bin').write_bytes(b'original native bytes')
        row = dict(id=ident,status='accepted_candidate',audit={'review_reasons':[]},capture_root=str(capture),
            scenario_sha256=crash.sha(capture/'scenario.json'),between_scene_park={'park_verified':True})
        native.append(row);selected.append(dict(copy.deepcopy(row),source_cell_id=ident))
    source = write(tmp_path/'source/manifest.json',{'count':8,'scenes':entries})
    safe = old/'chunk_0000/capture'
    write(old/'chunk_0000/manifest.json',{'count':4,'scenes':entries[:4]})
    write(safe/'session_contract.json',{'runner_options':{'shared_ros_batch':True}})
    write(safe/'status.json',{'stage':'complete'})
    write(safe/'shutdown_safety.json',dict(park_verified=True,gear=1,brake_held_during_requests=True,
        request_fields={'option':2,'gear':1}))
    native[1]['status'] = native[2]['status'] = native[3]['status'] = 'review'
    ledger(safe/'ledger.jsonl',native[:4])
    selected = selected[:3]
    deficits = [dict(id='scene_03',index=3,reasons=[{'reason':'quality_exhausted'}])]
    ledger(old/'selected_ledger.jsonl',selected)
    write(old/'quota.json',dict(target=8,accepted=3,deficit=1,pending=4,cells=deficits))
    write(old/'status.json',dict(stage='halted',error='Batch final Park unverified',failed=1,current=None,
        requested=8,finished=4,checkpoint_finished=4,accepted_candidate=3,review=1,elapsed_sec=1234.567))
    write(old/'.batch.lock',{'lock':True})
    for index in (1,2,3):
        recovery = old/'chunk_0000/recovery'/('scene_%02d'%index)
        write(recovery/'status.json',{'stage':'complete_with_deficit' if index == 3 else 'complete'})
        ledger(recovery/'selected_ledger.jsonl',[] if index == 3 else [selected[index]])
        write(recovery/'quota.json',dict(accepted=int(index!=3),deficit=int(index==3),pending=0,
            cells=deficits[0]['reasons'] if index == 3 else []))
        for ordinal in range(2 if index == 3 else 1):
            write(recovery/('scene_%02d'%index)/('round_%d'%ordinal)/'report.json',
                dict(stage='complete',attempts=[{'original_round':ordinal}]))
    suffix = old/'chunk_0001';unsafe = suffix/'capture'
    chunk = {'count':4,'scenes':entries[4:]}
    write(suffix/'manifest.json',chunk);execution = write(suffix/'execution_source/manifest.json',chunk)
    write(unsafe/'session_contract.json',dict(manifest_sha256=crash.sha(execution),
        runner_options=dict(shared_ros_batch=True,selected_scene_ids=[row['id'] for row in entries[4:]])))
    rows = copy.deepcopy(native[4:7]);rows[-1].update(status='failed',error=crash.STALE,between_scene_park={'park_verified':False})
    ledger(unsafe/'ledger.jsonl',rows)
    write(unsafe/'status.json',dict(stage='halted_exit_safety',current=None,error='Park not confirmed: '+crash.STALE,
        requested=4,finished=3,accepted_candidate=2,review=0,failed=1))
    write(unsafe/'shutdown_safety.json',dict(park_verified=False,gear=None,errors=[crash.STALE],
        brake_held_during_requests=True,request_fields={'option':2,'gear':1}))
    legacy = dict(manifest=crash.sha(source),indices=list(range(8)),rounds=2,code={})
    contract = dict(copy.deepcopy(legacy),batch_driver_sha256=crash.sha(driver),chunk_size=4,bulk=True,
        replenishment_driver_sha256=crash.sha(scripts/'base5_campaign_replenishment.py'))
    write(old/'batch_contract.json',contract)
    calls = []
    batch = SimpleNamespace(__file__=str(driver),campaign=SimpleNamespace(contract=lambda *a:copy.deepcopy(legacy)),
        inspect_batch=lambda *a:copy.deepcopy(native[:4]),verify_recovery=lambda *a:0,
        is_collision_failure=lambda row:False,is_stationary_failure=lambda row:False)
    path = Path(__file__).resolve().parents[2]/'dataset/morai_od/scripts/base5_campaign_import.py'
    spec = importlib.util.spec_from_file_location('_original_import_for_crash_test',path)
    importer = importlib.util.module_from_spec(spec);spec.loader.exec_module(importer)
    callbacks = dict(verify_context=lambda *a:calls.append(('context',a)),
        verify_regular=lambda *a:calls.append(('regular',a)),
        verify_recovery_extra=lambda *a:calls.append(('recovery',a)))
    return SimpleNamespace(base=tmp_path,old=old,new=new,source=source,entries=entries,
        selected=selected,deficits=deficits,batch=batch,importer=importer,callbacks=callbacks,
        unsafe=unsafe,suffix=suffix,safe=safe,calls=calls,scripts=scripts)


def run(f,**kwargs):
    return crash.export(f.source,f.old,f.new,batch=f.batch,importer=f.importer,**dict(f.callbacks,**kwargs))


def test_plan_only_preserves_original_halt_and_false_park(sample):
    f = sample;before = crash.paused.historical.tree_snapshot(f.old)
    result = crash.plan(f.source,f.old,f.new)
    assert (result['selected_count'],result['deficit_count'],result['pending']) == (3,1,4)
    assert result['unsafe_suffix']['ignored_accepted_rows'] == 2
    assert result['unsafe_suffix']['imported_rows'] == 0
    assert not f.new.exists() and result['native_capture_executed'] is False
    assert crash.paused.historical.tree_snapshot(f.old) == before


def test_export_exact_prefix_deficits_elapsed_history_and_unsafe_raw_unchanged(sample):
    f = sample;before = crash.paused.historical.tree_snapshot(f.old)
    result = run(f);receipt = crash.read(f.new/'imported_prefix.json')
    assert f.importer.verify(f.new/'imported_prefix.json',f.source,list(range(8))) == (f.selected,f.deficits,1234.567)
    assert len(receipt['selected']) == 3 and len(receipt['deficits']) == 1
    assert len(receipt['recovery_history']) == 3 and len(receipt['recovery_history'][-1]['reports']) == 2
    assert result['unsafe_suffix']['ignored_accepted_rows'] == 2
    assert result['original_halted_status_retained'] and result['original_false_park_retained']
    assert crash.read(f.unsafe/'shutdown_safety.json')['park_verified'] is False
    assert crash.read(f.old/'status.json')['stage'] == 'halted'
    assert crash.paused.historical.tree_snapshot(f.old) == before
    assert [kind for kind,_ in f.calls] == ['context','regular','regular','recovery','recovery','recovery']


@pytest.mark.parametrize('key,value',[('stage','paused'),('error','Unknown error'),('failed',0),
    ('current','active'),('checkpoint_finished',3),('finished',5),('accepted_candidate',4),
    ('elapsed_sec',float('nan')),('elapsed_sec',-1)])
def test_wrong_parent_incident_or_checkpoint_refused(sample,key,value):
    f = sample;data = crash.read(f.old/'status.json');data[key] = value;write(f.old/'status.json',data)
    with pytest.raises(RuntimeError):run(f)
    assert not f.new.exists()


@pytest.mark.parametrize('kind',['duplicate','hole','suffix_selected','quota','deficit_index','review_selected'])
def test_authoritative_prefix_cannot_be_reinterpreted(sample,kind):
    f = sample
    if kind in ('duplicate','hole','suffix_selected','review_selected'):
        rows = copy.deepcopy(f.selected)
        if kind == 'duplicate':rows[1] = rows[0]
        elif kind == 'hole':rows[1]['source_cell_id'] = 'scene_07'
        elif kind == 'suffix_selected':rows[1]['capture_root'] = str(f.unsafe/'captures/scene_05')
        else:rows[1]['audit']['review_reasons'] = ['review']
        ledger(f.old/'selected_ledger.jsonl',rows)
    else:
        data = crash.read(f.old/'quota.json')
        if kind == 'quota':data['pending'] = 3
        else:data['cells'][0]['index'] = 2
        write(f.old/'quota.json',data)
    with pytest.raises(RuntimeError):run(f)
    assert not f.new.exists()


@pytest.mark.parametrize('kind',['later_chunk','extra_session','recovery','native_stage','no_stale','park_true',
    'row_order','early_unsafe','failed_kind','native_counts','source_chunk','execution','source_ids'])
def test_only_exact_next_stale_native_suffix_can_be_isolated(sample,kind):
    f = sample
    if kind == 'later_chunk':(f.old/'chunk_0002').mkdir()
    elif kind == 'extra_session':write(f.suffix/'other/session_contract.json',{})
    elif kind == 'recovery':write(f.suffix/'recovery/scene_04/status.json',{'stage':'complete'})
    elif kind == 'source_chunk':
        data = crash.read(f.suffix/'manifest.json');data['scenes'].reverse();write(f.suffix/'manifest.json',data)
    elif kind == 'execution':write(f.suffix/'execution_source/manifest.json',{'changed':'source'})
    elif kind == 'source_ids':
        data = crash.read(f.unsafe/'session_contract.json');data['runner_options']['selected_scene_ids'].reverse()
        write(f.unsafe/'session_contract.json',data)
    elif kind in ('row_order','early_unsafe','failed_kind'):
        rows = crash.strict_rows(f.unsafe/'ledger.jsonl')
        if kind == 'row_order':rows.reverse()
        elif kind == 'early_unsafe':rows[0]['between_scene_park']['park_verified'] = False
        else:rows[-1]['error'] = 'Unknown native failure'
        ledger(f.unsafe/'ledger.jsonl',rows)
    elif kind == 'park_true':
        data = crash.read(f.unsafe/'shutdown_safety.json');data['park_verified'] = True;write(f.unsafe/'shutdown_safety.json',data)
    else:
        data = crash.read(f.unsafe/'status.json')
        if kind == 'native_stage':data['stage'] = 'complete'
        elif kind == 'no_stale':data['error'] = 'Different failure'
        else:data['accepted_candidate'] = 3
        write(f.unsafe/'status.json',data)
    with pytest.raises(RuntimeError):run(f)
    assert not f.new.exists()


@pytest.mark.parametrize('kind',['prefix_park','recovery_incomplete','recovery_bounds','wrong_batch','torn','symlink','stop'])
def test_safe_prefix_and_completed_recoveries_still_require_all_original_proofs(sample,kind):
    f = sample
    if kind == 'prefix_park':
        data = crash.read(f.safe/'shutdown_safety.json');data['park_verified'] = False;write(f.safe/'shutdown_safety.json',data)
    elif kind == 'recovery_incomplete':write(f.old/'chunk_0000/recovery/scene_03/status.json',{'stage':'running'})
    elif kind == 'recovery_bounds':
        data = crash.read(f.old/'quota.json');data['cells'][0]['reasons'] = ['reset attempts'];write(f.old/'quota.json',data)
    elif kind == 'wrong_batch':
        data = crash.read(f.old/'batch_contract.json');data['code'] = {'new':'code'};write(f.old/'batch_contract.json',data)
    elif kind == 'torn':
        p = f.safe/'ledger.jsonl';p.write_bytes(p.read_bytes().rstrip(b'\n'))
    elif kind == 'symlink':(f.old/'unrelated_symlink').symlink_to(f.source)
    else:(f.old/'STOP').touch()
    with pytest.raises(RuntimeError):run(f)
    assert not f.new.exists()


def test_active_lock_refused_without_writes(sample):
    f = sample
    with (f.old/'.batch.lock').open('rb') as handle:
        fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
        with pytest.raises(RuntimeError,match='still active'):run(f)
    assert not f.new.exists()


@pytest.mark.parametrize('kind',['missing_context','write','network','process'])
def test_callbacks_must_remain_readonly_and_all_frozen_proofs_required(sample,kind):
    f = sample;before = crash.paused.historical.tree_snapshot(f.old)
    if kind == 'missing_context':callback = None
    else:
        def callback(*args):
            if kind == 'write':(f.old/'status.json').write_text('{}')
            elif kind == 'network':
                import socket;socket.socket()
            else:
                import subprocess;subprocess.Popen(['true'])
    with pytest.raises(RuntimeError):run(f,verify_context=callback)
    assert not f.new.exists() and crash.paused.historical.tree_snapshot(f.old) == before


def test_original_static_profile_callbacks_cannot_be_dropped(sample):
    f = sample;write(f.old/'static_attitude_batch_contract.json',{'original':'profile'})
    with pytest.raises(RuntimeError,match='all original extra'):run(f,verify_regular=None)
    assert not f.new.exists()


def test_changed_unsafe_and_selected_raw_after_handoff_rejected(sample):
    f = sample;run(f)
    filename = f.unsafe/'shutdown_safety.json';filename.write_text('{"park_verified":true}')
    with pytest.raises(ValueError,match='evidence changed'):f.importer.verify(f.new/'imported_prefix.json',f.source,list(range(8)))


@pytest.mark.parametrize('document,key,value',[('status.json','failed',True),('status.json','finished',4.0),
    ('quota.json','accepted',3.0),('quota.json','deficit',True)])
def test_boolean_or_float_checkpoint_counts_cannot_be_reinterpreted(sample,document,key,value):
    f = sample;data = crash.read(f.old/document);data[key] = value;write(f.old/document,data)
    with pytest.raises(RuntimeError,match='integer'):run(f)
    assert not f.new.exists()


def test_nested_import_ancestor_ids_hashes_and_elapsed_are_preserved(sample):
    f = sample;ancestor = f.base/'ancestor';ancestor.mkdir()
    original = Path(f.selected[0]['capture_root']);target = ancestor/'captures/scene_00';target.mkdir(parents=True)
    for filename in original.iterdir():(target/filename.name).write_bytes(filename.read_bytes())
    inherited = copy.deepcopy(f.selected[0]);inherited['capture_root'] = str(target)
    f.selected[0] = inherited;ledger(f.old/'selected_ledger.jsonl',f.selected)
    native = f.batch.inspect_batch(None,None)[1:]
    f.batch.inspect_batch = lambda *args:copy.deepcopy(native)
    ledger(f.safe/'ledger.jsonl',native)
    nested = dict(schema=1,original_output=str(ancestor),manifest_sha256=crash.sha(f.source),
        selected=[inherited],deficits=[],elapsed_sec=20.0,
        hashes={str(path):crash.sha(path) for path in target.iterdir()},sensor_stats={})
    write(f.old/'imported_prefix.json',nested)
    contract = crash.read(f.old/'batch_contract.json')
    contract.update(imported_prefix_sha256=crash.sha(f.old/'imported_prefix.json'),
        import_driver_sha256=crash.sha(f.scripts/'base5_campaign_import.py'))
    write(f.old/'batch_contract.json',contract)
    result = run(f);receipt = crash.read(f.new/'imported_prefix.json')
    assert receipt['selected'] == f.selected and receipt['deficits'] == f.deficits
    assert receipt['elapsed_sec'] == 1234.567 and len(result['import_lineage']) == 1
    assert set(nested['hashes']) <= set(receipt['hashes'])
    assert result['import_lineage'][0]['elapsed_sec'] == 20.0
    (target/'sensor.bin').write_bytes(b'ancestor corruption')
    with pytest.raises(ValueError,match='evidence changed'):
        f.importer.verify(f.new/'imported_prefix.json',f.source,list(range(8)))


def test_arbitrary_extra_unsafe_session_is_never_silently_skipped(sample):
    f = sample;write(f.old/'unrelated/session_contract.json',{'unsafe':'unrelated'})
    with pytest.raises(RuntimeError,match='Unexpected native session'):run(f)
    assert not f.new.exists()
