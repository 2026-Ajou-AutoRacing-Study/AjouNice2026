"""CPU-only incident fixtures; no real collection output or ROS is modified."""
import copy
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

import local_sunny15_crash_import as handoff


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value)+'\n')
    return path


def ledger(path,rows):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(''.join(json.dumps(row)+'\n' for row in rows))
    return path


@pytest.fixture
def incident(tmp_path,monkeypatch):
    old,new = tmp_path/'old',tmp_path/'new'
    old.mkdir()
    entries,native = [],[]
    for index in range(4):
        ident = 'scene_%04d' % index
        scenario = write(tmp_path/'source'/(ident+'.json'),dict(scene=index))
        metadata = write(tmp_path/'source'/(ident+'.meta.json'),dict(split='train'))
        entries.append(dict(id=ident,scenario=str(scenario),metadata=str(metadata),
            scenario_sha256=handoff.sha(scenario)))
        capture = old/('chunk_%04d' % (index//2))/'capture/captures'/ident
        write(capture/'scenario.json',dict(scene=index))
        write(capture/'metadata.json',dict(split='train',calibration_only=False))
        audit = dict(review_reasons=[] if index != 1 else ['primary_no_sensor_evidence'])
        write(capture/'audit.json',audit)
        (capture/'sensor.bin').write_bytes(b'original native bytes')
        native.append(dict(id=ident,status='review' if index == 1 else 'accepted_candidate',
            audit=audit,capture_root=str(capture),scenario_sha256=handoff.sha(capture/'scenario.json'),
            between_scene_park=dict(park_verified=True)))
    source = write(tmp_path/'source/manifest.json',dict(count=4,scenes=entries))
    for ordinal in range(2):
        folder = old/('chunk_%04d' % ordinal)
        write(folder/'manifest.json',dict(count=2,scenes=entries[ordinal*2:ordinal*2+2]))
        write(folder/'capture/session_contract.json',dict(original=True))
        write(folder/'capture/status.json',dict(stage='complete' if ordinal == 0 else 'halted_exit_safety'))
        write(folder/'capture/shutdown_safety.json',dict(park_verified=ordinal == 0))
        ledger(folder/'capture/ledger.jsonl',native[ordinal*2:ordinal*2+2])
        (folder/'capture/.runner.lock').write_bytes(b'')
    selected = [dict(native[0],source_cell_id=entries[0]['id'])]
    deficit = dict(id=entries[1]['id'],index=1,reasons=[dict(id=entries[1]['id'],reasons=['exhausted'])])
    recovery = old/'chunk_0000/recovery'/entries[1]['id']
    write(recovery/'status.json',dict(stage='complete_with_deficit'))
    write(recovery/'quota.json',dict(accepted=0,deficit=1,cells=deficit['reasons']))
    ledger(recovery/'selected_ledger.jsonl',[])
    write(old/'status.json',dict(stage='halted',error=handoff.ERROR,current=None,failed=1,
        requested=4,finished=2,checkpoint_finished=2,accepted_candidate=1,review=1,elapsed_sec=123.5))
    write(old/'quota.json',dict(target=4,accepted=1,deficit=1,pending=2,cells=[deficit]))
    ledger(old/'selected_ledger.jsonl',selected)
    write(old/'deficit_manifest.json',dict(count=1,scenes=[entries[1]]))
    for name in ('batch_contract.json','static_attitude_batch_contract.json',handoff.original.CONTRACT):
        write(old/name,dict(original=name))
    for name in ('.batch.lock','.static_attitude_batch.lock'):
        (old/name).write_bytes(b'')
    release = write(tmp_path/'source/release.json',dict(original=True))
    qualification = write(tmp_path/'source/qualification.json',dict(passed=True))
    fast_contract = write(tmp_path/'source/fast_contract.json',dict(original=True))
    attachment = write(tmp_path/'source/attachment.json',dict(original=True))
    for name,value in dict(OLD=old,NEW=new,IMPORT=new/'imported_prefix.json',HANDOFF=new/'crash_handoff.json',
            EXPECTED_SELECTED=1,EXPECTED_DEFICITS=1,EXPECTED_FINISHED=2,SOURCE_COUNT=4,CHUNK_SIZE=2).items():
        monkeypatch.setattr(handoff,name,value)
    for name,value in dict(OUTPUT=old,MANIFEST=source,MANIFEST_SHA=handoff.sha(source),RELEASE=release,
            RELEASE_SHA=handoff.sha(release),QUALIFICATION=qualification).items():
        monkeypatch.setattr(handoff.original,name,value)
    monkeypatch.setattr(handoff.fast,'CONTRACT',fast_contract)
    monkeypatch.setattr(handoff.fast,'ATTACHMENT',attachment)
    calls = []
    def inspect(root,manifest):
        calls.append(('inspect',root))
        if not handoff.read(root/'shutdown_safety.json')['park_verified']:
            raise ValueError('Original native Park unverified')
        return copy.deepcopy(native[:2])
    def verify_recovery(source,index,root):
        calls.append(('recovery',index,root));return 0
    batch = SimpleNamespace(inspect_batch=inspect,verify_recovery=verify_recovery)
    parent = SimpleNamespace(batch=batch,GUARD='static_attitude_batch_contract.json',
        verify_regular=lambda *args:calls.append(('regular',*args)),
        verify_strict_recovery=lambda *args:calls.append(('strict',*args)),
        profiles=lambda:dict(isolated_recovery=dict(original=True)),runtime_hashes=lambda:dict(original='frozen'))
    importer_path = Path(handoff.__file__).resolve().parents[2]/'dataset/morai_od/scripts/base5_campaign_import.py'
    spec = importlib.util.spec_from_file_location('_crash_original_importer_fixture',importer_path)
    importer = importlib.util.module_from_spec(spec);spec.loader.exec_module(importer)
    monkeypatch.setattr(handoff,'_modules',lambda:(parent,batch,importer))
    monkeypatch.setattr(handoff,'_original_context',lambda:None)
    return SimpleNamespace(old=old,new=new,source=source,selected=selected,deficit=deficit,
        native=native,calls=calls,parent=parent,batch=batch,importer=importer,recovery=recovery)


def test_plan_is_readonly_and_requires_exact_incident(incident):
    before = handoff._inventory()
    result = handoff.plan()
    assert result['selected_count'] == 1 and result['deficit_count'] == 1
    assert result['files_written'] is False and not incident.new.exists()
    assert handoff._inventory() == before and incident.calls == []


def test_prepare_imports_only_authoritative_prefix_preserves_unsafe_final_session(incident):
    before = handoff._inventory()
    result = handoff.prepare()
    assert result['selected_count'] == 1 and result['unsafe_chunk_imported'] is False
    assert sorted(path.name for path in incident.new.iterdir()) == ['crash_handoff.json','imported_prefix.json']
    receipt = handoff.read(handoff.IMPORT)
    assert receipt['selected'] == incident.selected and receipt['deficits'] == [incident.deficit]
    assert receipt['elapsed_sec'] == 123.5
    assert handoff.read(incident.old/'status.json')['stage'] == 'halted'
    assert handoff.read(incident.old/'chunk_0001/capture/shutdown_safety.json')['park_verified'] is False
    assert handoff._inventory() == before
    assert len([call for call in incident.calls if call[0] == 'inspect']) == 1
    assert len([call for call in incident.calls if call[0] == 'strict']) == 1
    assert handoff.verify() == result


def test_torn_historical_ledger_is_rejected_without_repair(incident):
    target = incident.old/'chunk_0001/capture/ledger.jsonl'
    target.write_bytes(target.read_bytes().rstrip(b'\n'))
    before = handoff._inventory()
    with pytest.raises(RuntimeError,match='Torn'):
        handoff.prepare()
    assert handoff._inventory() == before and not incident.new.exists()


@pytest.mark.parametrize('field,value',[('stage','paused'),('error','different'),('finished',3),
    ('accepted_candidate',2),('elapsed_sec',float('nan'))])
def test_incident_status_change_fails_closed(incident,field,value):
    filename = incident.old/'status.json';document = handoff.read(filename)
    document[field] = value;write(filename,document)
    with pytest.raises(ValueError):
        handoff.prepare()
    assert not incident.new.exists()


def test_selected_from_unsafe_chunk_cannot_enter_authoritative_prefix(incident):
    row = dict(incident.native[2],source_cell_id='scene_0002')
    ledger(incident.old/'selected_ledger.jsonl',[row])
    with pytest.raises(ValueError,match='exact completed prefix'):
        handoff.prepare()
    assert not incident.new.exists()


def test_safe_prefix_final_park_is_not_waived(incident):
    write(incident.old/'chunk_0000/capture/shutdown_safety.json',dict(park_verified=False))
    before = handoff._inventory()
    with pytest.raises(ValueError,match='Park'):
        handoff.prepare()
    assert handoff._inventory() == before and not incident.new.exists()


def test_authoritative_selected_tamper_rejected_by_original_native_proof(incident):
    row = dict(incident.selected[0],changed=True)
    ledger(incident.old/'selected_ledger.jsonl',[row])
    with pytest.raises(ValueError,match='completed recovery proof'):
        handoff.prepare()
    assert not incident.new.exists()


def test_deficit_budget_or_reasons_cannot_be_reset(incident):
    value = handoff.read(incident.recovery/'quota.json');value['cells'] = []
    write(incident.recovery/'quota.json',value)
    with pytest.raises(ValueError,match='recovery bounds changed'):
        handoff.prepare()
    assert not incident.new.exists()


def test_original_validator_cannot_write_old_tree(incident):
    filename = incident.old/'status.json';before = filename.read_bytes()
    incident.parent.verify_regular = lambda *args:filename.write_text('unsafe repair')
    with pytest.raises(RuntimeError,match='forbids file writes'):
        handoff.prepare()
    assert filename.read_bytes() == before and not incident.new.exists()


def test_old_raw_changes_after_handoff_fail_original_import_verify(incident):
    handoff.prepare()
    raw = Path(incident.selected[0]['capture_root'])/'sensor.bin'
    raw.write_bytes(b'different')
    with pytest.raises(ValueError,match='Historical tree'):
        handoff.verify()


def test_handoff_rejects_import_mutation_and_additional_old_evidence(incident):
    handoff.prepare()
    original = handoff.IMPORT.read_bytes()
    with handoff.IMPORT.open('ab') as stream:stream.write(b'\n')
    with pytest.raises(ValueError,match='Historical tree'):
        handoff.verify()
    handoff.IMPORT.write_bytes(original)
    write(incident.old/'new-evidence.json',dict(not_original=True))
    with pytest.raises(ValueError,match='Historical tree'):
        handoff.verify()


def test_new_parent_progress_does_not_change_historical_handoff(incident):
    handoff.prepare()
    write(incident.new/'status.json',dict(stage='running',finished=3))
    assert handoff.verify()['old_halted_status_retained'] is True


def test_parent_and_native_locks_must_be_idle(incident):
    import fcntl
    for filename in (incident.old/'.batch.lock',incident.old/'chunk_0001/capture/.runner.lock'):
        with filename.open('rb') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            with pytest.raises(BlockingIOError):
                handoff.prepare()
    assert not incident.new.exists()


def test_existing_or_symlink_new_output_is_never_overwritten(incident,tmp_path):
    incident.new.mkdir()
    with pytest.raises(ValueError,match='already exists'):
        handoff.prepare()
    incident.new.rmdir();incident.new.symlink_to(incident.old,target_is_directory=True)
    with pytest.raises(ValueError,match='nonsymlink'):
        handoff.prepare()


def test_selected_completed_strict_recovery_is_proven_before_import(incident):
    # A regular review may already have an authoritative accepted replacement.
    incident.native[0]['status'] = 'review'
    incident.native[0]['audit'] = dict(review_reasons=['original review'])
    recovery = incident.old/'chunk_0000/recovery/scene_0000'
    write(recovery/'status.json',dict(stage='complete'))
    write(recovery/'quota.json',dict(accepted=1,deficit=0,cells=[]))
    ledger(recovery/'selected_ledger.jsonl',incident.selected)
    result = handoff.prepare()
    assert result['completed_recoveries_verified'] == 2
    assert handoff.read(handoff.IMPORT)['selected'] == incident.selected


def test_failed_strict_recovery_verification_never_creates_output(incident):
    def reject(*args):
        raise ValueError('Actual strict direction/native support proof failed')
    incident.parent.verify_strict_recovery = reject
    with pytest.raises(ValueError,match='strict direction'):
        handoff.prepare()
    assert not incident.new.exists()


def test_raw_content_is_checked_even_if_size_and_mtime_are_restored(incident):
    import os
    handoff.prepare()
    raw = Path(incident.selected[0]['capture_root'])/'sensor.bin'
    info = raw.stat()
    raw.write_bytes(b'X'*info.st_size)
    os.utime(raw,ns=(info.st_atime_ns,info.st_mtime_ns))
    # Metadata-only preservation snapshots are not substituted for raw SHA.
    assert handoff._inventory() == handoff.read(handoff.HANDOFF)['historical_inventory']
    with pytest.raises(ValueError,match='Imported evidence changed'):
        handoff.verify()


def test_duplicate_authoritative_quota_slot_fails_closed(incident):
    value = handoff.read(incident.old/'quota.json')
    value['cells'][0]['id'] = incident.selected[0]['source_cell_id']
    write(incident.old/'quota.json',value)
    with pytest.raises(ValueError,match='exact completed prefix'):
        handoff.prepare()
    assert not incident.new.exists()
