"""공통 수집 대시보드의 CPU-only 계획·포트·RO namespace 검증."""
from contextlib import redirect_stdout
import io
import json
import os
from pathlib import Path
import socket
import stat
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import unified_dashboard as dashboard


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True,mode=0o700)
    path.write_text(json.dumps(value)+'\n' if not isinstance(value,str) else value)
    path.chmod(0o600)
    return path


@pytest.fixture
def fixture(tmp_path,monkeypatch):
    root=tmp_path/'capture_new';root.mkdir(mode=0o700)
    old=tmp_path/'capture_old';old.mkdir(mode=0o700)
    write(root/'status.json',dict(stage='prepared_waiting_for_resume',accepted_candidate=123))
    write(old/'status.json',dict(stage='paused',accepted_candidate=123))
    source=write(tmp_path/'source/manifest.json',dict(count=5000))
    scripts=tmp_path/'repo/dataset/morai_od/scripts'
    runtime={}
    for name in ('base5_fresh_dashboard.py','base5_capture_dashboard.py'):
        path=write(scripts/name,'# exact frozen dashboard\n')
        runtime[str(path)]=dashboard.sha(path)
    html=write(scripts/'base5_capture_dashboard.html','<html>frozen UI</html>')
    monkeypatch.setattr(dashboard,'HTML_SHA',dashboard.sha(html))
    profile=write(tmp_path/'work/sunny15_profile.json',dict(schema=dashboard.SCHEMA,
        policy=dashboard.POLICY,environment='sunny15',output=str(root),old_output=str(old),
        manifest=str(source),manifest_sha256=dashboard.sha(source),fixed_pins=runtime,runtime_hashes={}))
    monkeypatch.setattr(dashboard,'LOCAL_SCRIPTS',scripts)
    monkeypatch.setattr(dashboard,'check_port',lambda port:None)
    return SimpleNamespace(root=root,old=old,source=source,scripts=scripts,profile=profile,base=tmp_path)


def snapshot(root):
    return {str(path):path.read_bytes() for path in root.rglob('*') if path.is_file()}


def call(f,args=()):
    out=io.StringIO()
    with redirect_stdout(out):
        result=dashboard.main(['--profile',str(f.profile),*args])
    return result,out.getvalue()


def test_local_default_plan_tracks_new_output_not_preserved_old_root_without_any_writes(fixture,monkeypatch):
    f=fixture;before=snapshot(f.base)
    spawn=Mock();monkeypatch.setattr(dashboard.subprocess,'Popen',spawn)
    result,text=call(f);plan=json.loads(text)
    assert result==0 and plan['capture_root']==str(f.root)
    assert not plan['files_written'] and not plan['process_started'] and not plan['capture_controls']
    assert str(f.root) not in plan['history_file']
    assert not Path(plan['writable_state_only']).exists()
    assert before==snapshot(f.base)
    spawn.assert_not_called()


def test_command_is_only_frozen_observer_with_separate_history_and_imported_raw_storage(fixture):
    f=fixture;c=dashboard.configuration(f.profile)
    argv=dashboard.dashboard_command(c)
    assert argv[:3]==['/usr/bin/python3','-B',str(f.scripts/'base5_capture_dashboard.py')]
    for flag,value in (('--capture-root',f.root),('--manifest',f.source),
        ('--history-file',c['state']/'history.jsonl'),('--storage-root',f.old)):
        assert argv[argv.index(flag)+1]==str(value)
    assert '--run' not in argv and '--prepare' not in argv and '--complete' not in argv


@pytest.mark.parametrize('field',['schema','policy','environment','manifest_sha256'])
def test_invalid_or_changed_common_profile_fails_closed(fixture,field):
    f=fixture;doc=dashboard.read(f.profile);doc[field]='invalid';write(f.profile,doc)
    with pytest.raises(ValueError):
        dashboard.configuration(f.profile)


def test_changed_frozen_dashboard_source_is_not_executed(fixture):
    f=fixture;write(f.scripts/'base5_capture_dashboard.py','# edited')
    with pytest.raises(ValueError,match='Frozen dashboard source changed'):
        dashboard.configuration(f.profile)


def test_full_fixed_inventory_is_used_when_parent_runtime_closure_omits_dashboard(fixture):
    f=fixture;doc=dashboard.read(f.profile)
    assert not doc['runtime_hashes']
    assert str(f.scripts/'base5_capture_dashboard.py') in doc['fixed_pins']
    assert dashboard.configuration(f.profile)['script']==f.scripts/'base5_capture_dashboard.py'


def test_explicit_runtime_pin_fallback_and_conflicting_pin_rejection(fixture):
    f=fixture;doc=dashboard.read(f.profile)
    doc['runtime_hashes']=doc.pop('fixed_pins')
    write(f.profile,doc)
    dashboard.configuration(f.profile)
    doc['fixed_pins']={str(f.scripts/'base5_capture_dashboard.py'):'0'*64}
    write(f.profile,doc)
    with pytest.raises(ValueError,match='Conflicting frozen'):
        dashboard.configuration(f.profile)


def test_profile_must_not_put_observer_state_in_capture_or_source(fixture):
    f=fixture
    for parent in (f.root,f.old,f.source.parent):
        copied=write(parent/'observer_profile.json',dashboard.read(f.profile))
        with pytest.raises(ValueError,match='outside raw/source'):
            dashboard.configuration(copied)


def test_run_creates_only_private_separate_state_and_waits_for_observer(fixture,monkeypatch):
    f=fixture;before=snapshot(f.base)
    child=Mock();child.wait.return_value=17
    spawn=Mock(return_value=child);monkeypatch.setattr(dashboard.subprocess,'Popen',spawn)
    result,_=call(f,['--run'])
    assert result==17
    c=dashboard.configuration(f.profile)
    assert c['state'].is_dir() and stat.S_IMODE(c['state'].stat().st_mode)==0o700
    assert before==snapshot(f.base)
    env=spawn.call_args.kwargs['env']
    assert env['HOME']==str(c['state']) and env['TMPDIR']==str(c['state'])
    assert 'ROS_MASTER_URI' not in env and 'SSH_AUTH_SOCK' not in env
    child.wait.assert_called_once_with()


def test_occupied_port_rejects_before_creating_state_or_spawning_and_never_kills(fixture,monkeypatch):
    f=fixture;spawn=Mock();kill=Mock()
    monkeypatch.setattr(dashboard.subprocess,'Popen',spawn);monkeypatch.setattr(dashboard.os,'kill',kill)
    def occupied(port):
        raise ValueError('기존 대시보드 터미널에서 Ctrl+C')
    monkeypatch.setattr(dashboard,'check_port',occupied)
    before=snapshot(f.base)
    with pytest.raises(ValueError,match='Ctrl\\+C'):
        call(f,['--run'])
    assert before==snapshot(f.base)
    assert not dashboard.configuration(f.profile)['state'].exists()
    spawn.assert_not_called();kill.assert_not_called()


def test_actual_port_collision_check_only_reports_and_keeps_existing_listener(tmp_path):
    # 실제 localhost socket만 열며 ROS나 HTTP 서버는 시작하지 않는다.
    spec=__import__('importlib.util',fromlist=['spec_from_file_location'])
    module_spec=spec.spec_from_file_location('_dashboard_port_check',Path(__file__).with_name('unified_dashboard.py'))
    module=spec.module_from_spec(module_spec);module_spec.loader.exec_module(module)
    with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as listener:
        listener.bind(('127.0.0.1',0));listener.listen(1)
        port=listener.getsockname()[1]
        with pytest.raises(ValueError,match='Ctrl\\+C'):
            module.check_port(port)
        assert listener.getsockname()[1]==port


@pytest.mark.parametrize('kind',['symlink','hardlink','foreign_mode'])
def test_separate_state_never_aliases_capture_files(fixture,kind):
    f=fixture;c=dashboard.configuration(f.profile);c['state'].mkdir(mode=0o700)
    if kind=='symlink':
        (c['state']/'history.jsonl').symlink_to(f.root/'status.json')
    elif kind=='hardlink':
        os.link(f.root/'status.json',c['state']/'history.jsonl')
    else:
        c['state'].chmod(0o777)
    with pytest.raises(ValueError):
        dashboard.state_directory(c)


@pytest.fixture
def borrowed_fixture(fixture,monkeypatch):
    f=fixture;workspace=f.base/'home/ajounice_base5_capture';workspace.mkdir(parents=True,mode=0o700)
    runtime=workspace/'work/unified-runtime-v1'
    logical=Path('/home/luv/AjouNice2026/dataset/morai_od/scripts')
    asset_root=workspace/'work/web-dashboard-sunny13-v1'
    for name in ('base5_capture_dashboard.py','base5_capture_dashboard.html'):
        write(asset_root/name,(f.scripts/name).read_text())
    profile=write(runtime/'sunny13_profile.json',dict(dashboard.read(f.profile),environment='sunny13',
        fixed_pins={str(logical/'base5_capture_dashboard.py'):dashboard.sha(asset_root/'base5_capture_dashboard.py')},
        runtime_hashes={}))
    ctx=dict(workspace=workspace,mr=f.base/'home/MoraiLauncher_Lin',paths=dict(profile=profile))
    ctx['mr'].mkdir(mode=0o700)
    def namespace(ctx,command):
        return ['bwrap','--ro-bind','/','/','--bind',str(workspace),str(workspace),
            '--bind',str(ctx['mr']),str(ctx['mr']),'--chdir','/home/luv/AjouNice2026','--',*command]
    module=SimpleNamespace(LOGICAL_SCRIPTS=logical,
        namespace=namespace,environment=lambda ctx:dict(PATH='/usr/bin',PYTHONPATH='/pinned/checks',
            ROS_MASTER_URI='http://127.0.0.1:11311',ROS_HOME='/old'))
    monkeypatch.setattr(dashboard,'load_borrowed',lambda:(module,ctx))
    return SimpleNamespace(f=f,ctx=ctx,module=module,profile=profile)


def test_borrowed_namespace_is_workspace_and_morai_readonly_with_state_only_writable(borrowed_fixture):
    f=borrowed_fixture;c=dashboard.configuration(borrowed=True)
    argv=dashboard.namespace_command(c);index=argv.index('--chdir')
    assert argv[index-9:index]==['--ro-bind',str(f.ctx['workspace']),str(f.ctx['workspace']),
        '--ro-bind',str(f.ctx['mr']),str(f.ctx['mr']),'--bind',str(c['state']),str(c['state'])]
    effective={}
    for index,value in enumerate(argv):
        if value in ('--bind','--ro-bind'):
            effective[argv[index+2]]=value
    assert {name for name,mode in effective.items() if mode=='--bind'}=={str(c['state'])}
    assert not dashboard.inside(c['state'],f.ctx['workspace']/'work/unified-runtime-v1')
    env=dashboard.environment(c)
    assert env['PYTHONPATH']=='/pinned/checks'
    assert not any(name.startswith('ROS_') for name in env)
    assert dashboard.dashboard_command(c)[2]==str(f.ctx['workspace']/'work/web-dashboard-sunny13-v1/base5_capture_dashboard.py')


@pytest.mark.parametrize('asset',['base5_capture_dashboard.py','base5_capture_dashboard.html'])
def test_borrowed_requires_original_paired_ui_assets_without_modifying_frozen_overlay(borrowed_fixture,asset):
    f=borrowed_fixture
    write(f.ctx['workspace']/'work/web-dashboard-sunny13-v1'/asset,'changed')
    with pytest.raises(ValueError,match='Frozen dashboard'):
        dashboard.configuration(borrowed=True)


def test_borrowed_profile_override_is_forbidden(borrowed_fixture):
    with pytest.raises(ValueError,match='profile path is fixed'):
        dashboard.configuration(borrowed_fixture.f.profile,borrowed=True)


def test_borrowed_runtime_and_plan_add_no_files_to_frozen_helpers(borrowed_fixture,monkeypatch):
    f=borrowed_fixture;before=snapshot(f.ctx['workspace'])
    spawn=Mock();monkeypatch.setattr(dashboard.subprocess,'Popen',spawn)
    with redirect_stdout(io.StringIO()):
        assert dashboard.main(['--borrowed'])==0
    assert before==snapshot(f.ctx['workspace'])
    spawn.assert_not_called()


@pytest.mark.parametrize('port',[0,80,65536,True])
def test_unprivileged_explicit_ports_only(fixture,port):
    with pytest.raises(ValueError):
        dashboard.configuration(fixture.profile,port=port)
