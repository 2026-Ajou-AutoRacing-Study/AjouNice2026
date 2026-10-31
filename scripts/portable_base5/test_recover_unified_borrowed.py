"""Crash continuation wrapper 검증. 원격·ROS·native 촬영은 실행하지 않는다."""
from contextlib import nullcontext,redirect_stdout
import copy
import io
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import recover_unified_borrowed as recovery
import unified_capture as common


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True,mode=0o700)
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n' if not isinstance(value,str) else value)
    path.chmod(0o600)
    return path


def snapshot(root):
    return {str(path):path.read_bytes() for path in root.rglob('*') if path.is_file()}


@pytest.fixture
def fixture(tmp_path,monkeypatch):
    workspace=tmp_path/'home/ajounice_base5_capture'
    root=workspace/recovery.RUNTIME_REL
    wrapper=write(root/'recover_unified_borrowed.py','# separately deployed wrapper\n')
    crash_path=write(root/'unified_crash_import.py','# separately deployed CPU importer\n')
    helpers=root/'checks';helpers.mkdir(mode=0o700)
    helper=write(helpers/'unified_capture.py','# exact pinned common driver\n')
    helper_pins={str(helper):common.sha(helper)}
    monkeypatch.setattr(common,'helpers',lambda:helper_pins)
    monkeypatch.setattr(recovery,'__file__',str(wrapper))
    source=write(workspace/'source/manifest.json',dict(count=5000))
    release=write(workspace/'source/release.json',dict(original_qualification=True))
    older=workspace/'collections/sunny13_5k_recovery_v2';older.mkdir(parents=True,mode=0o700)
    old=workspace/'collections/sunny13_5k_unified_v1';old.mkdir(mode=0o700)
    write(old/'status.json',dict(stage='halted',error='Batch final Park unverified',accepted_candidate=224))
    pilot=workspace/'collections/sunny13_5k_unified_v1_qualification_example'
    pilot_manifest=write(pilot/'source/manifest.json',dict(count=5))
    proof=write(pilot/'capture/shutdown_safety.json',dict(park_verified=True))
    qualification=dict(schema='base5_unified_native_profile_qualification_v1',passed=True,
        policy=common.POLICY,manifest_sha256=common.sha(source),environment='sunny13',
        helper_pins=helper_pins,pilot_manifest=str(pilot_manifest),pilot_manifest_sha256=common.sha(pilot_manifest),
        source_indices=[0,1,2,3,4],pilot_output=str(pilot/'capture'),captures=5,approved=5,
        actual_pause_resume=True,completed_replay_no_new_capture=True,
        proof_hashes={str(proof):common.sha(proof)},main_output_native_captures=0)
    qualification_path=write(old/'unified_live_qualification.json',qualification)
    previous=dict(schema=common.SCHEMA,policy=common.POLICY,environment='sunny13',
        manifest=str(source),manifest_sha256=common.sha(source),release=str(release),release_sha256=common.sha(release),
        old_output=str(older),output=str(old),helper_pins=helper_pins,native_qa_unchanged=True,
        common_static_attitude=True,common_final_direction_reference=True,automatic_completion=False,
        fixed_pins={str(source):common.sha(source),str(release):common.sha(release)},
        runtime_hashes=helper_pins,source_authority=dict(original_source_qualification=True),import_sha256='a'*64)
    profile=write(root/'sunny13_profile.json',previous)
    ctx=dict(workspace=workspace,paths=dict(profile=profile,checks=helpers,old=older,output=old),
        intent=dict(manifest=str(source),release=str(release)))
    boundary=SimpleNamespace(installed=Mock(side_effect=lambda **kwargs:nullcontext()),verify=Mock())
    parent,direction=SimpleNamespace(),SimpleNamespace()
    monkeypatch.setattr(common,'modules',lambda:(parent,direction))
    monkeypatch.setattr(common,'boundary_for',Mock(return_value=boundary))
    monkeypatch.setattr(common,'route_compatibility',lambda doc:nullcontext())
    monkeypatch.setattr(common,'source_authority',Mock(return_value=dict(original_source_qualification=True)))
    monkeypatch.setattr(common,'warm_boundary',Mock())
    verifiers=dict(verify_context=Mock(),verify_regular=Mock(),verify_recovery_extra=Mock())
    monkeypatch.setattr(common,'historical_verifiers',Mock(return_value=verifiers))
    monkeypatch.setattr(common,'qualified_handoff',Mock(side_effect=lambda *args:nullcontext()))
    def initialize(parent,doc,authority):
        write(Path(doc['output'])/'status.json',dict(stage='prepared_waiting_for_resume',accepted_candidate=224))
    monkeypatch.setattr(common,'initialize',Mock(side_effect=initialize))
    def export(source,old,output,**kwargs):
        output.mkdir(mode=0o700)
        write(output/'imported_prefix.json',dict(selected=224,deficits=2,old_preserved=True))
        return dict(selected_count=224,deficit_count=2,finished=226)
    crash=SimpleNamespace(__file__=str(crash_path),export=Mock(side_effect=export),plan=Mock(return_value=dict(captures_requested=0)))
    def namespace(ctx,command):
        return ['bwrap','--ro-bind','/','/','--chdir','/home/luv/AjouNice2026','--',*command]
    borrowed=SimpleNamespace(namespace=namespace,load=Mock(return_value=ctx),supervise=Mock(return_value=0))
    monkeypatch.setattr(recovery,'load_modules',lambda root:(borrowed,common,crash))
    selection=recovery.scope(ctx,common)
    return SimpleNamespace(workspace=workspace,root=root,source=source,release=release,old=old,
        profile=profile,previous=previous,ctx=ctx,selection=selection,borrowed=borrowed,crash=crash,
        boundary=boundary,parent=parent,direction=direction,verifiers=verifiers,
        qualification_path=qualification_path,qualification=qualification,proof=proof,pilot_manifest=pilot_manifest)


def prepare(f):
    return recovery.prepare_inside(common,f.crash,f.selection)


def test_scope_defaults_to_separate_sibling_output_and_profile(fixture):
    f=fixture;s=f.selection
    assert s['source']==f.profile and s['old']==f.old
    assert s['output']==f.old.with_name(f.old.name+'_after_crash_v1')
    assert s['profile']==f.root/'crash-continuation-v1/sunny13_profile.json'


@pytest.mark.parametrize('field',['source_profile','profile','output'])
def test_scope_rejects_path_escape(fixture,field):
    f=fixture
    kwargs={field:f.workspace.parent/'outside.json'}
    with pytest.raises((ValueError,RuntimeError,OSError)):
        recovery.scope(f.ctx,common,**kwargs)


@pytest.mark.parametrize('field,value',[('environment','sunny15'),('manifest','/different/manifest.json'),
    ('release','/different/release.json'),('native_qa_unchanged',False),('common_static_attitude',False),
    ('common_final_direction_reference',False),('automatic_completion',True),('helper_pins',{})])
def test_source_or_qa_changes_are_not_crash_recovery(fixture,field,value):
    f=fixture;doc=copy.deepcopy(f.previous);doc[field]=value;write(f.profile,doc)
    with pytest.raises((ValueError,RuntimeError)):
        recovery.scope(f.ctx,common)


def test_original_qualification_proofs_are_reverified_and_not_claimed_as_new_captures(fixture):
    f=fixture;before=snapshot(f.workspace)
    path,value=recovery.validate_qualification(common,f.previous)
    assert path==f.qualification_path and value==f.qualification
    assert before==snapshot(f.workspace)


@pytest.mark.parametrize('kind',['missing_proof','changed_proof','changed_pilot','missing_qualification',
    'qa_pin_changed','empty_proofs','pause_not_checked','terminal_replay_not_checked'])
def test_invalid_or_missing_native_qualification_cannot_be_reused(fixture,kind):
    f=fixture;value=copy.deepcopy(f.qualification)
    if kind=='missing_proof':f.proof.unlink()
    elif kind=='changed_proof':write(f.proof,dict(park_verified=False))
    elif kind=='changed_pilot':write(f.pilot_manifest,dict(count=0))
    elif kind=='missing_qualification':f.qualification_path.unlink()
    elif kind=='qa_pin_changed':value['helper_pins']={}
    elif kind=='empty_proofs':value['proof_hashes']={}
    elif kind=='pause_not_checked':value['actual_pause_resume']=False
    else:value['completed_replay_no_new_capture']=False
    if f.qualification_path.exists():write(f.qualification_path,value)
    with pytest.raises((ValueError,RuntimeError,OSError)):
        recovery.validate_qualification(common,f.previous)


def test_prepare_preserves_halted_original_and_qualifier_bytes_without_native_capture(fixture,monkeypatch):
    f=fixture;old=snapshot(f.old);source=f.profile.read_bytes();proof=snapshot(f.proof.parent)
    spawn=Mock(side_effect=AssertionError('CPU handoff cannot capture'))
    monkeypatch.setattr(recovery.subprocess,'Popen',spawn)
    result=prepare(f)
    assert result['captures_started']==0 and not result['unsafe_suffix_imported']
    assert snapshot(f.old)==old and f.profile.read_bytes()==source and snapshot(f.proof.parent)==proof
    assert (f.selection['output']/'unified_live_qualification.json').read_bytes()==f.qualification_path.read_bytes()
    assert common.read(f.old/'status.json')['stage']=='halted'
    f.crash.export.assert_called_once_with(f.source,f.old,f.selection['output'],**f.verifiers)
    spawn.assert_not_called();f.boundary.verify.assert_called_once_with()
    doc,receipt=recovery.prepared(common,f.selection)
    assert doc['old_output']==str(f.old) and doc['output']==str(f.selection['output'])
    assert receipt['new_native_qualification_claimed'] is False


def test_prepare_preserves_even_noncanonical_original_qualification_bytes(fixture):
    f=fixture
    payload=('\n '+json.dumps(f.qualification,ensure_ascii=False,indent=4)+'\n\n').encode()
    f.qualification_path.write_bytes(payload)
    prepare(f)
    copied=f.selection['output']/'unified_live_qualification.json'
    assert copied.read_bytes()==payload
    assert copied.stat().st_mode&0o777==0o600
    assert f.qualification_path.read_bytes()==payload


def test_qualification_copy_never_overwrites_existing_bytes_or_follows_destination_symlink(fixture):
    f=fixture
    destination=write(f.root/'existing_qualification.json','operator evidence')
    with pytest.raises(FileExistsError):
        recovery.copy_new(f.qualification_path,destination)
    assert destination.read_text()=='operator evidence'
    alias=f.root/'aliased_qualification.json';alias.symlink_to(destination)
    with pytest.raises(FileExistsError):
        recovery.copy_new(f.qualification_path,alias)
    assert destination.read_text()=='operator evidence'


@pytest.mark.parametrize('kind',['existing_profile','existing_output'])
def test_prepare_never_overwrites_partial_or_existing_continuation(fixture,kind):
    f=fixture
    if kind=='existing_profile':write(f.selection['profile'],dict(operator_data=True))
    else:f.selection['output'].mkdir(mode=0o700)
    before=snapshot(f.workspace)
    with pytest.raises(ValueError,match='already exists'):
        prepare(f)
    assert snapshot(f.workspace)==before
    f.crash.export.assert_not_called()


@pytest.mark.parametrize('field',['schema','source_profile','source_profile_sha256','profile_sha256',
    'qualification_reused_unchanged','unsafe_suffix_imported','captures_started_during_prepare'])
def test_prepared_receipt_detects_required_identity_and_capture_claim_tampering(fixture,field):
    f=fixture;prepare(f)
    path=f.selection['output']/'unified_crash_continuation.json'
    doc=common.read(path);doc[field]='changed';write(path,doc)
    with pytest.raises((ValueError,RuntimeError)):
        recovery.prepared(common,f.selection)


@pytest.mark.parametrize('field',['qualification_source','qualification_sha256','original_output','output',
    'profile','native_qa_unchanged','new_native_qualification_claimed'])
def test_prepared_receipt_must_bind_original_qualification_and_exact_destination(fixture,field):
    f=fixture;prepare(f)
    path=f.selection['output']/'unified_crash_continuation.json'
    doc=common.read(path);doc[field]='changed';write(path,doc)
    with pytest.raises((ValueError,RuntimeError)):
        recovery.prepared(common,f.selection)


@pytest.mark.parametrize('which',['reused_copy','original_report','original_proof'])
def test_prepared_continuation_rechecks_qualification_and_proof_bytes(fixture,which):
    f=fixture;prepare(f)
    path={'reused_copy':f.selection['output']/'unified_live_qualification.json',
          'original_report':f.qualification_path,'original_proof':f.proof}[which]
    write(path,dict(changed=True))
    with pytest.raises((ValueError,RuntimeError,OSError,KeyError)):
        recovery.prepared(common,f.selection)


def test_namespace_makes_failed_output_and_both_profiles_readonly_then_restores_original(fixture):
    f=fixture;original=f.borrowed.namespace
    write(f.selection['profile'],dict(existing=True))
    with recovery.preserved_namespace(f.borrowed,f.selection):
        argv=f.borrowed.namespace(f.ctx,['cpu-helper'])
        mounts=[argv[index+1:index+3] for index,value in enumerate(argv) if value=='--ro-bind']
        for path in (f.old,f.profile,f.selection['profile'],f.root/'recover_unified_borrowed.py',
                     f.root/'unified_crash_import.py'):
            assert [str(path),str(path)] in mounts
    assert f.borrowed.namespace is original
    with pytest.raises(RuntimeError,match='intentional'):
        with recovery.preserved_namespace(f.borrowed,f.selection):
            raise RuntimeError('intentional')
    assert f.borrowed.namespace is original


def test_inside_cpu_plan_does_not_reinterpret_host_ros_pids(fixture):
    f=fixture
    write(f.workspace/'checks/population_recovery_intent.json',f.ctx['intent'])
    with redirect_stdout(io.StringIO()):
        assert recovery.main(['plan','--inside'])==0
    f.borrowed.load.assert_not_called();f.borrowed.supervise.assert_not_called()
    f.crash.plan.assert_called_once_with(f.source,f.old,f.selection['output'])


def test_run_is_separate_unmodified_common_process_and_pause_target_is_new_output(fixture):
    f=fixture;prepare(f)
    with redirect_stdout(io.StringIO()):
        assert recovery.main(['run','--confirmed-environment','sunny13'])==0
    ctx,action,argv=f.borrowed.supervise.call_args.args
    assert action=='run' and ctx['paths']['output']==f.selection['output']
    assert ctx['paths']['old']==f.old and ctx['paths']['profile']==f.selection['profile']
    assert argv[:4]==['/usr/bin/python3','-B','-u',str(f.ctx['paths']['checks']/'unified_capture.py')]
    assert '--run' in argv and '--inside' not in argv
    assert argv[argv.index('--profile')+1]==str(f.selection['profile'])


@pytest.mark.parametrize('argv',[['run'],['plan','--confirmed-environment','sunny13'],
    ['run','--inside','--confirmed-environment','sunny13']])
def test_missing_confirmation_or_inside_native_invocation_is_rejected_before_any_dispatch(fixture,argv):
    f=fixture
    with pytest.raises(ValueError):
        recovery.main(argv)
    f.borrowed.load.assert_not_called();f.borrowed.supervise.assert_not_called()
