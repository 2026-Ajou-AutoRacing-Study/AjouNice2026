"""Pure CPU observer tests: no SSH, MORAI, ROS, captures or HTTP server."""
from contextlib import redirect_stdout
import io
import json
import os
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import borrowed_completion_dashboard as dashboard
import unified_dashboard as observer


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    path.write_text(json.dumps(value)+'\n')
    path.chmod(0o600)
    return path


def snapshot(root):
    return {str(p):p.read_bytes() for p in root.rglob('*') if p.is_file()}


@pytest.fixture
def setup(tmp_path, monkeypatch):
    workspace = tmp_path/'home/ajounice_base5_capture'
    root = workspace/dashboard.OBSERVER_REL
    script = write(root/'borrowed_completion_dashboard.py', {})
    monkeypatch.setattr(dashboard, '__file__', str(script))
    completion_root = workspace/dashboard.COMPLETION_REL
    completion_root.mkdir(mode=0o700)
    old, seed, new, earlier = [workspace/'collections'/name for name in ('main','seed','flexible','earlier')]
    for path in (old, seed, new, earlier):
        path.mkdir(parents=True, mode=0o700)
    manifest = write(workspace/'source/manifest.json', {'count':5000})
    doc = dict(schema=observer.SCHEMA, policy=observer.POLICY, environment='sunny13',
        manifest=str(manifest),manifest_sha256='a'*64,release='release',release_sha256='b'*64,
        helper_pins={'helper':'c'*64}, runtime_hashes={'runtime':'d'*64},output=str(old),old_output=str(earlier))
    profile = write(workspace/'work/unified-runtime-v1/crash-continuation-v1/sunny13_profile.json',doc)
    write(old/'imported_prefix.json',dict(schema=1,manifest_sha256='a'*64,original_output=str(earlier)))
    write(old/'status.json',dict(stage='complete_with_deficit',accepted_candidate=4944))
    write(new/'status.json',dict(stage='running',accepted_candidate=4945))
    mr = workspace.parent/'MoraiLauncher_Lin'; mr.mkdir(mode=0o700)
    ctx = dict(workspace=workspace,mr=mr)
    borrowed = SimpleNamespace(namespace=lambda ctx,argv:['bwrap','--ro-bind','/','/',
        '--bind',str(workspace),str(workspace),'--bind',str(mr),str(mr),'--chdir','/','--',*argv],
        environment=lambda ctx:dict(PATH='/usr/bin',ROS_MASTER_URI='http://localhost:11311'))
    base = dict(ctx=ctx,doc=doc,profile=profile,root=old,old=earlier,manifest=manifest,
        borrowed=True,borrowed_module=borrowed,script=workspace/'work/ui/base5_capture_dashboard.py')
    original_configuration = Mock(side_effect=lambda port,borrowed:dict(base,port=port))
    monkeypatch.setattr(observer,'configuration',original_configuration)
    monkeypatch.setattr(observer,'check_port',Mock())
    scope = dict(root=completion_root,workspace=workspace,profile=profile,previous=doc,
        baseline=old,seed=seed,output=new)
    monkeypatch.setattr(dashboard,'load_components',Mock(return_value=(observer,scope)))
    return SimpleNamespace(workspace=workspace,root=root,old=old,seed=seed,new=new,earlier=earlier,
        doc=doc,scope=scope,base=base,borrowed=borrowed,profile=profile,mr=mr)


def call(argv=()):
    stream = io.StringIO()
    with redirect_stdout(stream):
        result = dashboard.main(list(argv))
    return result, stream.getvalue()


def test_default_plan_never_creates_files_or_processes(setup,monkeypatch):
    before = snapshot(setup.workspace)
    spawn = Mock(); monkeypatch.setattr(dashboard.subprocess,'Popen',spawn)
    code,text = call(); plan=json.loads(text)
    assert code == 0 and plan['capture_root'] == str(setup.new)
    assert plan['url'] == 'http://127.0.0.1:8077'
    assert not plan['files_written'] and not plan['process_started'] and not plan['capture_controls']
    assert before == snapshot(setup.workspace) and not Path(plan['writable_state_only']).exists()
    spawn.assert_not_called()


def test_configuration_uses_pinned_source_and_whole_preserved_storage_chain(setup):
    _,config = dashboard.configuration()
    assert config['profile'] == setup.profile and config['root'] == setup.new
    assert config['storage_ancestors'] == sorted({setup.old,setup.seed,setup.earlier})
    assert config['state'] == setup.root/'state-port8077'
    observer.configuration.assert_called_once_with(port=8077,borrowed=True)


def test_namespace_has_only_state_writable_not_collections_morai_or_workspace(setup):
    _,config = dashboard.configuration(); argv=dashboard.namespace_command(observer,config)
    effective={argv[i+2]:flag for i,flag in enumerate(argv) if flag in ('--bind','--ro-bind')}
    assert effective[str(setup.workspace)] == effective[str(setup.mr)] == '--ro-bind'
    assert {path for path,flag in effective.items() if flag=='--bind'} == {str(config['state'])}
    roots=[argv[i+1] for i,value in enumerate(argv) if value=='--storage-root']
    assert len(roots)==len(set(roots))==3 and set(roots)==set(map(str,config['storage_ancestors']))
    assert '--run' not in argv and '--prepare' not in argv and '--complete' not in argv
    assert '--capture-root' in argv and str(setup.new) in argv


@pytest.mark.parametrize('key',dashboard.IDENTITY)
def test_source_identity_change_is_rejected(setup,key):
    setup.base['doc']=dict(setup.doc,**{key:'changed'})
    with pytest.raises(ValueError,match='same frozen source/runtime/UI'):
        dashboard.configuration()


@pytest.mark.parametrize('kind',['foreign','missing','cyclic','changed_manifest','symlink'])
def test_storage_lineage_cannot_escape_or_rewrite_identity(setup,kind):
    value=dict(schema=1,manifest_sha256='a'*64,original_output=str(setup.earlier))
    if kind=='foreign':value['original_output']=str(setup.root)
    elif kind=='missing':value['original_output']=str(setup.old.parent/'missing')
    elif kind=='cyclic':value['original_output']=str(setup.old)
    elif kind=='changed_manifest':value['manifest_sha256']='x'*64
    else:
        alias=setup.old.parent/'alias';alias.symlink_to(setup.earlier,target_is_directory=True)
        value['original_output']=str(alias)
    write(setup.old/'imported_prefix.json',value)
    with pytest.raises(ValueError):dashboard.configuration()


def test_port_conflict_leaves_existing_server_and_captures_untouched(setup,monkeypatch):
    observer.check_port.side_effect=ValueError('port already in use')
    spawn=Mock();monkeypatch.setattr(dashboard.subprocess,'Popen',spawn)
    before=snapshot(setup.workspace)
    with pytest.raises(ValueError,match='already in use'):call(['--run'])
    assert before==snapshot(setup.workspace) and not (setup.root/'state-port8077').exists()
    spawn.assert_not_called()


def test_run_creates_only_private_state_and_waits_for_observer(setup,monkeypatch):
    before=snapshot(setup.workspace)
    child=Mock();child.wait.return_value=7
    spawn=Mock(return_value=child);monkeypatch.setattr(dashboard.subprocess,'Popen',spawn)
    code,_=call(['--run']);state=setup.root/'state-port8077'
    assert code==7 and state.is_dir() and state.stat().st_mode&0o777==0o700
    assert before==snapshot(setup.workspace)
    env=spawn.call_args.kwargs['env']
    assert env['HOME']==env['TMPDIR']==str(state) and 'ROS_MASTER_URI' not in env
    child.wait.assert_called_once_with()


def test_invalid_observer_location_is_rejected_before_loading(setup,monkeypatch):
    monkeypatch.setattr(dashboard,'__file__',str(setup.workspace/'checks/elsewhere.py'))
    with pytest.raises(ValueError,match='separate approved'):dashboard.configuration()
    dashboard.load_components.assert_not_called()


def test_original_observer_source_is_checked_before_import(tmp_path,monkeypatch):
    root=tmp_path/'ajounice_base5_capture'/dashboard.OBSERVER_REL
    write(root.parents[1]/dashboard.ORIGINAL_REL,{'not':'original'})
    loader=Mock();monkeypatch.setattr(dashboard,'load_file',loader)
    with pytest.raises(ValueError,match='implementation changed'):dashboard.load_components(root)
    loader.assert_not_called()
