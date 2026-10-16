"""Temporary fixtures and mocked children only; no ROS, MR or sensor writes."""
import json
from pathlib import Path
from types import SimpleNamespace
import sys

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import verify_base5_direct_resume as verifier


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value)+'\n')
    return path


def fixture(tmp_path,monkeypatch):
    output=tmp_path/'capture';capture=output/'captures'/'a';capture.mkdir(parents=True)
    scenario=write(tmp_path/'source'/'scene.json',{})
    metadata=write(tmp_path/'source'/'metadata.json',{})
    write(capture/'scenario.json',{})
    (capture/'native.bin').write_bytes(b'unchanged-native')
    row=dict(id='a',capture_root=str(capture),status='accepted_candidate',scenario_sha256=verifier.sha256(scenario))
    entry=dict(id='a',scenario=str(scenario),metadata=str(metadata),scenario_sha256=row['scenario_sha256'])
    manifest=write(tmp_path/'manifest.json',dict(scenes=[entry],config={}))
    write(output/'ledger.jsonl',row)
    write(output/'session_contract.json',dict(safety_code_sha256={},ego_support_map_sha256={}))
    write(output/'shutdown_safety.json',dict(park_verified=True))
    write(output/'timing.json',dict(total_sec=12.))
    (output/'event_cmd_requests.jsonl').write_text('preserved-event\n')
    (output/'.runner.lock').write_text('')
    write(output/'status.json',dict(stage='complete',current=None,error=None,requested=1,finished=1,
        accepted_candidate=1,review=0,failed=0,updated_at=1.,results=[verifier.status_row(row)]))
    network=write(tmp_path/'network.json',{})
    scripts=tmp_path/'scripts';scripts.mkdir()
    for name in ('optimized.py','collector.py','sensor.json'):(scripts/name).write_text('{}\n')
    monkeypatch.setattr(verifier,'SCRIPT',scripts/'optimized.py')
    monkeypatch.setattr(verifier,'SOURCE',scripts/'collector.py')
    monkeypatch.setattr(verifier,'SENSOR',scripts/'sensor.json')
    calls=[]
    def prepare(out,man,net,sensor,options):
        assert options['shared_ros_batch'] is False
        assert options['selected_scene_ids']==['a']
        assert options['optimized_adapter']['profile']=='static-ready'
        return [row],{'a':row}
    monkeypatch.setattr(verifier,'prepare_session',prepare)
    def run(command,**kwargs):
        calls.append(command)
        assert command[1]=='-c' and 'rospy' in command[2] and 'socket.__new__' in command[2]
        assert command[command.index('--setup-profile')+1]=='static-ready'
        assert '--resume' not in command and '--prepare-only' not in command
        status=verifier.read(output/'status.json');status['updated_at']=2.;write(output/'status.json',status)
        return SimpleNamespace(returncode=0,stdout=verifier.GUARD_MARKER+json.dumps(dict(ros_imports=0,sockets=0))+'\n')
    return output,capture,manifest,network,calls,run


def test_complete_resume_preserves_raw_timing_ledger_and_eventcmd(tmp_path,monkeypatch):
    output,capture,manifest,network,calls,run=fixture(tmp_path,monkeypatch)
    receipt=tmp_path/'receipt'
    result=verifier.verify(manifest,output,receipt,1,network=network,run=run)
    assert result['passed'] and result['actual_new_captures']==0 and len(calls)==1
    assert verifier.read(receipt/'before.json')==verifier.read(receipt/'after.json')
    assert verifier.read(output/'status.json')['updated_at']==2.
    assert any(path.endswith('event_cmd_requests.jsonl') for path in verifier.read(receipt/'before.json'))
    assert (capture/'native.bin').read_bytes()==b'unchanged-native'


@pytest.mark.parametrize('kind',['incomplete','unsafe_park','torn_ledger','failed_row','count_mismatch','result_mismatch','wrong_sha','missing_eventlog'])
def test_incomplete_or_unverified_session_never_launches_child(tmp_path,monkeypatch,kind):
    output,_,manifest,network,calls,run=fixture(tmp_path,monkeypatch)
    status=verifier.read(output/'status.json')
    if kind=='incomplete':status['stage']='stopped'
    if kind=='unsafe_park':write(output/'shutdown_safety.json',dict(park_verified=False))
    if kind=='torn_ledger':(output/'ledger.jsonl').write_text('{"id":')
    if kind=='failed_row':
        row=json.loads((output/'ledger.jsonl').read_text());row['status']='failed';write(output/'ledger.jsonl',row)
    if kind=='count_mismatch':status['finished']=0
    if kind=='result_mismatch':status['results'][0]['status']='review'
    if kind=='wrong_sha':
        deck=verifier.read(manifest);deck['scenes'][0]['scenario_sha256']='0'*64;write(manifest,deck)
    if kind=='missing_eventlog':(output/'event_cmd_requests.jsonl').unlink()
    write(output/'status.json',status)
    with pytest.raises((ValueError,KeyError)):
        verifier.verify(manifest,output,tmp_path/'receipt',1,network=network,run=run)
    assert not calls


@pytest.mark.parametrize('kind',['native','ledger','eventcmd','timing','source','runtime','status','guard','child_failure'])
def test_any_protected_change_or_live_attempt_fails_receipt(tmp_path,monkeypatch,kind):
    output,capture,manifest,network,_,original_run=fixture(tmp_path,monkeypatch)
    def run(command,**kwargs):
        result=original_run(command,**kwargs)
        targets=dict(native=capture/'native.bin',ledger=output/'ledger.jsonl',
            eventcmd=output/'event_cmd_requests.jsonl',timing=output/'timing.json',source=manifest,runtime=verifier.SCRIPT)
        if kind in targets:
            with targets[kind].open('ab') as handle:handle.write(b'changed')
        if kind=='status':
            status=verifier.read(output/'status.json');status['unexpected']='new';write(output/'status.json',status)
        if kind=='guard':result.stdout=verifier.GUARD_MARKER+json.dumps(dict(ros_imports=1,sockets=0))+'\n'
        if kind=='child_failure':result.returncode=1
        return result
    receipt=tmp_path/'receipt'
    with pytest.raises(ValueError):verifier.verify(manifest,output,receipt,1,network=network,run=run)
    assert verifier.read(receipt/'report.json')['passed'] is False


def test_receipt_must_be_fresh_and_outside_capture(tmp_path,monkeypatch):
    output,_,manifest,network,calls,run=fixture(tmp_path,monkeypatch)
    with pytest.raises(ValueError):verifier.verify(manifest,output,output/'receipt',1,network=network,run=run)
    receipt=tmp_path/'receipt';receipt.mkdir()
    with pytest.raises(FileExistsError):verifier.verify(manifest,output,receipt,1,network=network,run=run)
    assert not calls
