"""CPU-only fresh crash continuation fixtures; never ROS/MORAI/native capture."""
from contextlib import nullcontext
import copy
import json
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

import recover_unified_fresh as recovery

common = recovery.common


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value)+'\n')
    return path


def snapshot(root):
    return {str(p):p.read_bytes() for p in root.rglob('*') if p.is_file()}


@pytest.fixture
def case(tmp_path, monkeypatch):
    old = tmp_path/'collections/bulk';old.mkdir(parents=True)
    output = old.with_name('continued')
    source = write(tmp_path/'source.json', dict(count=5000))
    release = write(tmp_path/'release.json', dict(source_authority=True))
    proof = write(tmp_path/'pilot/proof.json', dict(park_verified=True))
    pilot = write(tmp_path/'pilot/manifest.json', dict(count=5))
    previous = dict(schema=common.SCHEMA, policy=common.POLICY, environment='foggy13',
        manifest=str(source), manifest_sha256=common.sha(source), release=str(release),
        release_sha256=common.sha(release), old_output=str(old.with_name(old.name+'_fresh_origin')),
        output=str(old), fresh_start=True, fresh_origin_schema=common.FRESH_ORIGIN_SCHEMA,
        helper_pins=common.helpers(fresh_start=True), native_qa_unchanged=True,
        common_static_attitude=True, common_final_direction_reference=True,
        automatic_completion=False, fixed_pins={str(source):common.sha(source)},
        source_authority=dict(original_authority=dict(capture_output=str(old))),
        runtime_hashes=dict(original='same'), import_sha256='0'*64)
    profile = write(tmp_path/'profiles/original.json', previous)
    target = tmp_path/'profiles/continued.json'
    qvalue = dict(schema='base5_unified_native_profile_qualification_v1', passed=True,
        policy=common.POLICY, environment='foggy13', manifest_sha256=common.sha(source),
        helper_pins=previous['helper_pins'], captures=5, approved=5,
        main_output_native_captures=0, source_indices=[1,2,3,4,5],
        actual_pause_resume=True, completed_replay_no_new_capture=True,
        pilot_manifest=str(pilot), pilot_manifest_sha256=common.sha(pilot),
        proof_hashes={str(proof):common.sha(proof)})
    qpath = write(old/'unified_live_qualification.json', qvalue)
    write(old/'status.json', dict(stage='halted', failed=1, error='Batch final Park unverified'))
    parent,direction = SimpleNamespace(),SimpleNamespace()
    boundary = SimpleNamespace(installed=lambda **kwargs:nullcontext(), verify=Mock())
    monkeypatch.setattr(common, 'modules', lambda:(parent,direction))
    monkeypatch.setattr(common, 'boundary_for', Mock(return_value=boundary))
    monkeypatch.setattr(common, 'route_compatibility', lambda doc:nullcontext())
    authority = Mock(side_effect=lambda doc,p:copy.deepcopy(previous['source_authority']))
    monkeypatch.setattr(common, 'source_authority', authority)
    original_prefix = Mock()
    monkeypatch.setattr(common, 'verify_fresh_prefix', original_prefix)
    monkeypatch.setattr(common, 'warm_boundary', Mock())
    verifiers = dict(verify_context=Mock(), verify_regular=Mock(), verify_recovery_extra=Mock())
    monkeypatch.setattr(common, 'historical_verifiers', Mock(return_value=verifiers))
    monkeypatch.setattr(common, 'qualified_handoff', lambda *args:nullcontext())
    def initialize(parent,doc,authority):
        write(Path(doc['output'])/'status.json', dict(stage='prepared_waiting_for_resume'))
    monkeypatch.setattr(common, 'initialize', Mock(side_effect=initialize))
    def export(source, old, output, **kwargs):
        output.mkdir()
        prefix = write(output/'imported_prefix.json', dict(selected=['safe'],deficits=['kept']))
        handoff = dict(schema=recovery.crash.SCHEMA, original_output=str(old), output=str(output),
            selected_count=2043, deficit_count=21, finished=2064, elapsed_sec=123.5,
            imported_prefix_sha256=common.sha(prefix), authoritative_rows_unchanged=True,
            original_import_verifier_passed=True, unsafe_suffix=dict(imported_rows=0))
        write(output/'unified_crash_handoff.json',handoff)
        return handoff
    monkeypatch.setattr(recovery.crash, 'export', Mock(side_effect=export))
    importer = SimpleNamespace(verify=Mock(return_value=([None]*2043,[None]*21,123.5)))
    monkeypatch.setattr(common.handoff, '_modules', lambda:(None,importer))
    s = recovery.selection(profile,target,output)
    return SimpleNamespace(root=tmp_path,s=s,old=old,output=output,profile=profile,target=target,
        previous=previous,source=source,proof=proof,pilot=pilot,qpath=qpath,qvalue=qvalue,
        authority=authority,original_prefix=original_prefix,boundary=boundary,
        verifiers=verifiers,importer=importer,parent=parent)


def test_prepare_preserves_original_and_only_imports_safe_prefix(case):
    c=case;before=snapshot(c.old);profile=c.profile.read_bytes()
    result=recovery.prepare(c.s)
    assert result['selected']==2043 and result['deficits']==21 and result['finished']==2064
    assert result['captures_started']==0 and not result['unsafe_suffix_imported']
    assert snapshot(c.old)==before and c.profile.read_bytes()==profile
    doc,receipt=recovery.prepared(c.s)
    assert doc['fresh_start'] is True and doc['old_output']==c.previous['old_output']
    assert doc['canonical_output']==str(c.old) and doc['output']==str(c.output)
    assert doc['source_authority']==c.previous['source_authority']
    assert doc['helper_pins']==c.previous['helper_pins']
    assert (c.output/'unified_live_qualification.json').read_bytes()==c.qpath.read_bytes()
    assert receipt['new_native_qualification_claimed'] is False
    historical=common.historical_verifiers.call_args.args[0]
    assert historical['old_output']==str(c.old)
    c.original_prefix.assert_called_once_with(c.previous)
    c.authority.assert_called_once_with(c.previous,c.parent)


@pytest.mark.parametrize('field,value',[('fresh_start',False),('canonical_output','already-continuation'),
    ('crash_continuation',{}),('automatic_completion',True),('environment','other')])
def test_only_original_unchanged_fresh_profile_supported(case,field,value):
    c=case;doc=copy.deepcopy(c.previous);doc[field]=value;write(c.profile,doc)
    with pytest.raises((RuntimeError,ValueError)):
        recovery.selection(c.profile,c.target,c.output)


@pytest.mark.parametrize('kind',['same','nested','external_profile'])
def test_paths_cannot_alias_or_enter_historical_output(case,kind):
    c=case
    output=c.old if kind=='same' else c.old/'nested' if kind=='nested' else c.output
    target=c.old/'profile.json' if kind=='external_profile' else c.target
    with pytest.raises(RuntimeError):recovery.selection(c.profile,target,output)


@pytest.mark.parametrize('field,value',[('approved',4),('environment','sunny13'),
    ('actual_pause_resume',False),('completed_replay_no_new_capture',False),
    ('proof_hashes',{}),('main_output_native_captures',1),('helper_pins',{}),
    ('source_indices',[1,2,3,4,4])])
def test_qualification_cannot_be_weakened_or_faked(case,field,value):
    c=case;doc=copy.deepcopy(c.qvalue);doc[field]=value;write(c.qpath,doc)
    with pytest.raises(RuntimeError):recovery.qualification(c.previous)


@pytest.mark.parametrize('which',['proof','pilot'])
def test_qualification_inputs_rehashed(case,which):
    c=case;write(getattr(c,which),dict(changed=True))
    with pytest.raises(RuntimeError):recovery.qualification(c.previous)


@pytest.mark.parametrize('which',['output','profile'])
def test_prepare_never_overwrites_existing(case,which):
    c=case
    if which=='output':c.output.mkdir()
    else:write(c.target,dict(operator_file=True))
    with pytest.raises(RuntimeError):recovery.prepare(c.s)
    recovery.crash.export.assert_not_called()


@pytest.mark.parametrize('field',['source_authority','canonical_output','old_output','fresh_start',
    'helper_pins','crash_continuation','fixed_pins'])
def test_continuation_profile_tampering_rejected(case,field):
    c=case;recovery.prepare(c.s)
    doc=common.read(c.target);doc[field]={} if type(doc[field]) is dict else 'changed';write(c.target,doc)
    with pytest.raises(RuntimeError):recovery.prepared(c.s)


@pytest.mark.parametrize('field',['unsafe_suffix_imported','new_native_qualification_claimed',
    'captures_started_during_prepare','source_profile_sha256','qualification_sha256','handoff_sha256'])
def test_receipt_tampering_rejected(case,field):
    c=case;recovery.prepare(c.s);path=c.output/'unified_fresh_crash_continuation.json'
    doc=common.read(path);doc[field]='changed';write(path,doc)
    with pytest.raises(RuntimeError):recovery.prepared(c.s)


def test_adapter_uses_original_authority_and_empty_origin_but_verifies_import(case):
    c=case;recovery.prepare(c.s);doc,_=recovery.prepared(c.s)
    original=common.source_authority
    with recovery.installed(c.s):
        assert common.source_authority(doc,c.parent)==c.previous['source_authority']
        common.verify_fresh_prefix(doc)
        other=dict(doc,output='/another')
        with pytest.raises(RuntimeError):common.source_authority(other,c.parent)
    assert common.source_authority is original
    assert all(call.args[0]==c.previous for call in c.authority.call_args_list)
    c.importer.verify.assert_called_once()


def test_adapter_restores_ownership_after_exception(case):
    c=case;original=common.worker_payload
    with pytest.raises(ValueError):
        with recovery.installed(c.s):raise ValueError('stop')
    assert common.worker_payload is original


def test_worker_command_only_retargets_exact_original_dispatch(case,monkeypatch):
    c=case;calls=[]
    monkeypatch.setattr(recovery.subprocess,'Popen',lambda command,*a,**kw:calls.append((command,kw)))
    original=[recovery.sys.executable,'-B',str(Path(common.__file__).absolute()),'--worker','/target',
        '--session-fd','3','--parent-pid','12','--parent-start-ticks','15']
    with recovery.installed(c.s):
        recovery.subprocess.Popen(original,pass_fds=(3,))
        recovery.subprocess.Popen(['unrelated','--worker'])
    assert calls[0][0][2]==str(Path(recovery.__file__).absolute())
    assert calls[0][0][3:]==original[3:] and calls[0][1]['pass_fds']==(3,)
    assert calls[1][0]==['unrelated','--worker']
    assert original[2]==str(Path(common.__file__).absolute())


def test_payload_preserves_original_capability_checks_and_restores_file(case,monkeypatch):
    c=case;recovery.prepare(c.s);doc,_=recovery.prepared(c.s)
    original_file=common.__file__;observed=[]
    def original(fd,parent):
        observed.append((fd,parent,common.__file__))
        return dict(profile=doc)
    monkeypatch.setattr(common,'worker_payload',original)
    with recovery.installed():
        assert common.worker_payload(7,dict(pid=12))['profile']==doc
        assert common.__file__==original_file
    assert observed==[(7,dict(pid=12),str(Path(recovery.__file__).absolute()))]
    assert common.worker_payload is original


def test_unbound_worker_profile_rejected(case,monkeypatch):
    c=case;recovery.prepare(c.s);doc,_=recovery.prepared(c.s)
    doc['fixed_pins'].pop(str(Path(recovery.__file__).absolute()))
    monkeypatch.setattr(common,'worker_payload',lambda *args:dict(profile=doc))
    with recovery.installed():
        with pytest.raises(RuntimeError,match='pinned explicit'):
            common.worker_payload(7,dict(pid=12))


def test_worker_identity_change_detected_and_restored(case,monkeypatch):
    c=case;original=common.__file__
    def invalid(*args):common.__file__='changed';return {}
    monkeypatch.setattr(common,'worker_payload',invalid)
    with recovery.installed():
        with pytest.raises(RuntimeError,match='ownership'):common.worker_payload(7,{})
    assert common.__file__==original


def test_actual_original_worker_payload_accepts_only_pinned_wrapper_and_sealed_capability(case,monkeypatch):
    c=case;recovery.prepare(c.s);doc,_=recovery.prepared(c.s)
    original_read=Path.read_bytes
    proc=Path('/proc')/str(os.getppid())/'cmdline'
    command=[recovery.sys.executable,'-B',str(Path(recovery.__file__).absolute()),'--run']
    monkeypatch.setattr(Path,'read_bytes',lambda self:
        b'\0'.join(os.fsencode(v) for v in command)+b'\0' if self==proc else original_read(self))
    fd=common.sealed_payload(dict(profile=doc,parent=common.identity(os.getppid())))
    original_file=common.__file__
    try:
        with recovery.installed():
            assert common.worker_payload(fd,common.identity(os.getppid()))['profile']==doc
            assert common.__file__==original_file
            command[-1]='--prepare'
            with pytest.raises(RuntimeError,match='mode identity'):
                common.worker_payload(fd,common.identity(os.getppid()))
    finally:os.close(fd)


def test_actual_common_router_and_continuation_keep_sealed_boundary_and_target_scope(case,monkeypatch):
    c=case;recovery.prepare(c.s);doc,_=recovery.prepared(c.s)
    calls=[]
    def base(command,*args,**kwargs):
        fd=int(command[command.index('--session-fd')+1])
        assert fd in kwargs['pass_fds']
        value=json.loads(os.pread(fd,os.fstat(fd).st_size,0))
        assert value['profile']==doc and value['boundary']==dict(verified=True)
        calls.append(command)
    monkeypatch.setattr(recovery.subprocess,'Popen',base)
    target=common.SCRIPTS/'run_base5_capture_static_attitude.py'
    output=c.output/'chunk_0129/capture'
    command=[recovery.sys.executable,str(target),'--manifest',str(output.parent/'execution_source/manifest.json'),
        '--output',str(output),'--enable-static-surface-attitude','--allow-static-pedestrians',
        '--setup-profile','static-ready','--shared-ros-batch','--limit','16']
    with recovery.installed(c.s),common.routed_workers(doc,SimpleNamespace(snapshot=lambda:dict(verified=True))):
        recovery.subprocess.Popen(command)
        foreign=list(command);foreign[foreign.index('--output')+1]='/foreign/capture'
        with pytest.raises(RuntimeError,match='escaped'):
            recovery.subprocess.Popen(foreign)
    assert len(calls)==1 and calls[0][2]==str(Path(recovery.__file__).absolute())
    assert '--worker' in calls[0] and str(target) in calls[0]


@pytest.mark.parametrize('argv',[['--run'],['--prepare','--confirmed-environment','foggy13'],
    ['--stop-after-chunks','1']])
def test_cli_confirmation_is_not_implicit(case,argv):
    c=case
    with pytest.raises(RuntimeError):
        recovery.main(argv+['--source-profile',str(c.profile),'--profile',str(c.target),'--output',str(c.output)])


def test_documented_runtime_search_path_loads_the_actual_frozen_importers():
    """Run enters prefix verification before common.modules populates sys.path."""
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PYTHONPATH=os.pathsep.join(
        (str(Path(recovery.__file__).parent), str(common.SCRIPTS))))
    code = (
        'from pathlib import Path; import unified_paused_import as handoff; '
        'batch, importer = handoff._modules(); '
        'assert Path(batch.__file__).resolve() == Path(__import__("sys").argv[1])/"run_base5_batch_campaign.py"; '
        'assert Path(importer.__file__).resolve() == Path(__import__("sys").argv[1])/"base5_campaign_import.py"'
    )
    subprocess.run([sys.executable, '-B', '-c', code, str(common.SCRIPTS)],
                   env=env, check=True, capture_output=True, timeout=20)
