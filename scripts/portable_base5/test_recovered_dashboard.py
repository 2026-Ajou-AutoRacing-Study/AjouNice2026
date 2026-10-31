"""Crash 복구 대시보드는 검증된 새 출력만 관측하며 기존 서버를 건드리지 않는다."""
from contextlib import redirect_stdout
import copy
import io
import json
import os
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import recovered_dashboard as recovered


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True,mode=0o700)
    path.write_text(json.dumps(value)+'\n' if not isinstance(value,str) else value)
    path.chmod(0o600)
    return path


def snapshot(root):
    return {str(p):p.read_bytes() for p in root.rglob('*') if p.is_file()}


@pytest.fixture
def fixture(tmp_path,monkeypatch):
    workspace=tmp_path/'home/ajounice_base5_capture'
    root=workspace/recovered.recovery.RUNTIME_REL
    helper=write(root/'recovered_dashboard.py','# separately deployed observer\n')
    monkeypatch.setattr(recovered,'__file__',str(helper))
    old=workspace/'collections/sunny13_5k_unified_v1';old.mkdir(parents=True,mode=0o700)
    new=workspace/'collections/sunny13_5k_unified_v1_after_crash_v1';new.mkdir(mode=0o700)
    earlier=workspace/'collections/sunny13_5k_recovery_v2';earlier.mkdir(mode=0o700)
    write(old/'status.json',dict(stage='halted',accepted_candidate=2688))
    write(new/'status.json',dict(stage='prepared_waiting_for_resume',accepted_candidate=2688))
    source=write(workspace/'source/manifest.json',dict(count=5000))
    previous=dict(schema='base5_unified_capture_v1',policy='fixed-input-boundaries-and-common-native-profiles-v1',
        environment='sunny13',manifest=str(source),manifest_sha256='a'*64,release=str(workspace/'source/release.json'),
        release_sha256='b'*64,helper_pins={'common.py':'c'*64},runtime_hashes={'native.py':'d'*64},
        output=str(old),old_output=str(earlier))
    doc=dict(previous,output=str(new),old_output=str(old))
    profile=write(root/'crash-continuation-v1/sunny13_profile.json',doc)
    source_profile=write(root/'sunny13_profile.json',previous)
    mr=workspace.parent/'MoraiLauncher_Lin';mr.mkdir(mode=0o700)
    ctx=dict(workspace=workspace,mr=mr)
    selection=dict(root=root,profile=profile,source=source_profile,old=old,output=new,previous=previous)
    borrowed=SimpleNamespace(load=Mock(return_value=ctx),
        namespace=lambda ctx,argv:['bwrap','--ro-bind','/','/','--bind',str(workspace),str(workspace),
            '--bind',str(mr),str(mr),'--chdir','/home/luv/AjouNice2026','--',*argv],
        environment=lambda ctx:dict(PATH='/usr/bin',PYTHONPATH='/pinned/helpers'))
    base=dict(doc=previous,ctx=ctx,profile=source_profile,root=old,old=earlier,manifest=source,
        state=root/'original_dashboard_state',history=root/'original_dashboard_state/history.jsonl',
        port=8075,script=workspace/'work/web-dashboard-sunny13-v1/base5_capture_dashboard.py',
        borrowed=True,borrowed_module=borrowed)
    def configuration(port=8075,borrowed=True):
        recovered.observer.require(type(port) is int and 1024<=port<=65535,'Unprivileged observer port required')
        return dict(base,port=port)
    monkeypatch.setattr(recovered.recovery,'load_modules',Mock(return_value=(borrowed,object(),object())))
    monkeypatch.setattr(recovered.recovery,'scope',Mock(return_value=selection))
    monkeypatch.setattr(recovered.recovery,'prepared',Mock(return_value=(doc,{'verified':True})))
    monkeypatch.setattr(recovered.observer,'configuration',Mock(side_effect=configuration))
    monkeypatch.setattr(recovered.observer,'check_port',Mock())
    return SimpleNamespace(workspace=workspace,root=root,new=new,old=old,earlier=earlier,
        source=source,profile=profile,source_profile=source_profile,doc=doc,previous=previous,
        selection=selection,borrowed=borrowed,base=base,ctx=ctx)


def call(argv=()):
    out=io.StringIO()
    with redirect_stdout(out):result=recovered.main(list(argv))
    return result,out.getvalue()


def test_recovered_configuration_binds_verified_successor_not_old_halted_dashboard(fixture):
    f=fixture;c=recovered.configuration(8077)
    assert c['root']==f.new and c['old']==f.old and c['profile']==f.profile
    assert c['doc']==f.doc and c['manifest']==f.source and c['port']==8077
    assert c['state']==f.root/'recovered-dashboard-state-port8077'
    assert c['history']==c['state']/'history.jsonl'
    f.borrowed.load.assert_called_once_with(verify=False)
    recovered.recovery.prepared.assert_called_once()
    recovered.observer.configuration.assert_called_once_with(port=8077,borrowed=True)


def test_default_plan_does_not_create_state_start_server_or_modify_capture(fixture,monkeypatch):
    f=fixture;before=snapshot(f.workspace)
    spawn=Mock();kill=Mock()
    monkeypatch.setattr(recovered.subprocess,'Popen',spawn);monkeypatch.setattr(recovered.os,'kill',kill)
    result,text=call(['--port','8077']);plan=json.loads(text)
    assert result==0 and plan['capture_root']==str(f.new) and plan['profile']==str(f.profile)
    assert not plan['files_written'] and not plan['process_started'] and not plan['capture_controls']
    assert not Path(plan['writable_state_only']).exists()
    assert before==snapshot(f.workspace)
    spawn.assert_not_called();kill.assert_not_called()


@pytest.mark.parametrize('field',['schema','policy','environment','manifest','manifest_sha256',
    'release','release_sha256','helper_pins','runtime_hashes'])
def test_changed_source_or_runtime_identity_is_rejected_before_observer_start(fixture,field,monkeypatch):
    f=fixture;f.doc[field]='changed'
    spawn=Mock();monkeypatch.setattr(recovered.subprocess,'Popen',spawn)
    with pytest.raises(ValueError,match='same frozen source/runtime/UI'):
        call(['--run'])
    spawn.assert_not_called()


def test_invalid_continuation_receipt_is_not_bypassed_for_dashboard(fixture,monkeypatch):
    monkeypatch.setattr(recovered.recovery,'prepared',Mock(side_effect=ValueError('qualification receipt changed')))
    with pytest.raises(ValueError,match='qualification receipt changed'):
        recovered.configuration()
    recovered.observer.configuration.assert_not_called()


@pytest.mark.parametrize('field',['output','old_output'])
def test_arbitrary_collection_output_override_is_forbidden(fixture,field):
    fixture.doc[field]=str(fixture.earlier)
    with pytest.raises(ValueError,match='explicitly verified crash continuation'):
        recovered.configuration()


def test_foreign_workspace_and_local_undeployed_script_are_rejected(fixture,monkeypatch):
    f=fixture
    f.base['ctx']=dict(f.ctx,workspace=f.workspace.parent/'foreign')
    with pytest.raises(ValueError,match='same frozen'):
        recovered.configuration()
    monkeypatch.setattr(recovered,'__file__',str(f.workspace.parent/'elsewhere/recovered_dashboard.py'))
    with pytest.raises(ValueError,match='approved borrowed-host runtime'):
        recovered.configuration()


def test_effective_namespace_keeps_workspace_morai_and_capture_readonly_with_state_only_writable(fixture):
    f=fixture;c=recovered.configuration()
    argv=recovered.observer.namespace_command(c)
    effective={}
    for index,value in enumerate(argv):
        if value in ('--bind','--ro-bind'):effective[argv[index+2]]=value
    assert effective[str(f.workspace)]=='--ro-bind'
    assert effective[str(f.ctx['mr'])]=='--ro-bind'
    assert {name for name,mode in effective.items() if mode=='--bind'}=={str(c['state'])}
    assert argv[argv.index('--capture-root')+1]==str(f.new)
    assert argv[argv.index('--storage-root')+1]==str(f.old)
    assert '--run' not in argv and '--prepare' not in argv and '--complete' not in argv


def test_port8075_conflict_never_stops_existing_server_or_creates_state(fixture,monkeypatch):
    f=fixture
    def occupied(port):
        assert port==8075
        raise ValueError('기존 대시보드 터미널에서 Ctrl+C')
    monkeypatch.setattr(recovered.observer,'check_port',occupied)
    spawn=Mock();kill=Mock()
    monkeypatch.setattr(recovered.subprocess,'Popen',spawn);monkeypatch.setattr(recovered.os,'kill',kill)
    before=snapshot(f.workspace)
    with pytest.raises(ValueError,match='Ctrl\\+C'):
        call(['--port','8075','--run'])
    assert before==snapshot(f.workspace)
    assert not (f.root/'recovered-dashboard-state-port8075').exists()
    spawn.assert_not_called();kill.assert_not_called()


def test_run_creates_only_private_observer_state_and_waits_for_web_child(fixture,monkeypatch):
    f=fixture;before=snapshot(f.workspace)
    child=Mock();child.wait.return_value=17
    spawn=Mock(return_value=child);monkeypatch.setattr(recovered.subprocess,'Popen',spawn)
    old_umask=os.umask(0o022);os.umask(old_umask)
    result,_=call(['--run'])
    assert result==17 and before==snapshot(f.workspace)
    state=f.root/'recovered-dashboard-state-port8075'
    assert state.is_dir() and state.stat().st_mode&0o777==0o700
    argv=spawn.call_args.args[0];env=spawn.call_args.kwargs['env']
    assert '--capture-root' in argv and str(f.new) in argv
    assert env['HOME']==str(state) and 'ROS_MASTER_URI' not in env
    child.wait.assert_called_once_with()
    restored=os.umask(old_umask);assert restored==old_umask


@pytest.mark.parametrize('port',[0,80,65536])
def test_invalid_ports_are_not_used(fixture,port):
    with pytest.raises(ValueError,match='Unprivileged'):
        recovered.configuration(port)


def test_storage_lineage_includes_all_nested_import_roots_and_deduplicates_them(fixture):
    f=fixture
    earliest=f.workspace/'collections/sunny13_5k';earliest.mkdir(mode=0o700)
    receipt=recovered.recovery.prepared.return_value[1]
    receipt['imported']=dict(import_lineage=[
        dict(output=str(f.old),original_output=str(f.earlier)),
        dict(output=str(f.earlier),original_output=str(earliest)),
        dict(output=str(f.old),original_output=str(f.earlier)),
    ])
    c=recovered.configuration()
    assert c['storage_ancestors']==sorted({f.old,f.earlier,earliest})
    assert len(c['storage_ancestors'])==3


def test_external_storage_ancestry_is_rejected_even_if_directory_exists(fixture):
    f=fixture
    recovered.recovery.prepared.return_value[1]['imported']=dict(import_lineage=[
        dict(output=str(f.old),original_output=str(f.root))])
    with pytest.raises(ValueError,match='inside preserved borrowed collections'):
        recovered.configuration()


def test_missing_storage_ancestor_is_not_scanned_or_created(fixture):
    f=fixture;missing=f.workspace/'collections/missing_original'
    recovered.recovery.prepared.return_value[1]['imported']=dict(import_lineage=[
        dict(output=str(f.old),original_output=str(missing))])
    with pytest.raises(ValueError,match='inside preserved borrowed collections'):
        recovered.configuration()
    assert not missing.exists()


def test_symlink_storage_ancestor_cannot_escape_collection_scope(fixture):
    f=fixture;alias=f.workspace/'collections/aliased_original'
    alias.symlink_to(f.root,target_is_directory=True)
    recovered.recovery.prepared.return_value[1]['imported']=dict(import_lineage=[
        dict(output=str(f.old),original_output=str(alias))])
    with pytest.raises(ValueError,match='nonsymlink'):
        recovered.configuration()


def test_namespace_passes_each_preserved_storage_root_once_after_readonly_mounts(fixture):
    f=fixture
    recovered.recovery.prepared.return_value[1]['imported']=dict(import_lineage=[
        dict(output=str(f.old),original_output=str(f.earlier)),
        dict(output=str(f.old),original_output=str(f.earlier)),
    ])
    c=recovered.configuration();argv=recovered.namespace_command(c)
    roots=[argv[index+1] for index,value in enumerate(argv) if value=='--storage-root']
    assert roots.count(str(f.old))==1 and roots.count(str(f.earlier))==1
    assert set(roots)=={str(f.old),str(f.earlier)}
    assert argv[-2:]==['--storage-root',str(f.earlier)]
    assert argv.index('--storage-root')>argv.index('--chdir')
    writable=[argv[index+2] for index,value in enumerate(argv) if value=='--bind']
    assert str(f.old) not in writable and str(f.earlier) not in writable
