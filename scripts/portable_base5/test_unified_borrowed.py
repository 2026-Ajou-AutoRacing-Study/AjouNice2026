"""원격 접근·ROS·MORAI 없이 새 borrowed namespace와 명령 경계를 검사한다."""
from contextlib import redirect_stdout
import io
import json
import os
from pathlib import Path
import signal
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import unified_borrowed as borrowed


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    path.write_text(json.dumps(value)+'\n' if not isinstance(value, str) else value)
    path.chmod(0o600)
    return path


@pytest.fixture
def fixture(tmp_path, monkeypatch):
    home = tmp_path/'home'; workspace = home/borrowed.WORKSPACE
    workspace.mkdir(parents=True, mode=0o700)
    p = borrowed.paths(workspace)
    graph = dict(release=dict(source='/home/luv/AjouNice2026/release.json'),
        manifest=dict(source='/home/luv/AjouNice2026/manifest.json'), files={})
    originals = ['legacy_'+str(index)+'.py' for index in range(360)]
    for name in originals+sorted(borrowed.EXTRA_SCRIPTS):
        path = write(p['scripts']/name, '# immutable '+name+'\n')
        if name in originals:
            graph['files'][name] = dict(source=str(borrowed.LOGICAL_SCRIPTS/name),
                sha256=borrowed.sha(path), roles=['runtime'])
    for index in range(67):
        graph['files']['external_'+str(index)] = dict(
            source='/home/luv/AjouNice2026/perception/detection3d/bevfusion/model_'+str(index)+'.py',
            sha256='b'*64,roles=['runtime'])
    for name in borrowed.REQUIRED_HELPERS:
        write(p['checks']/name, '# deployed '+name+'\n')
    for name in borrowed.OLD_OUTPUTS:
        (workspace/'collections'/name).mkdir(parents=True, mode=0o700)
    witness = write(workspace/'checks/canonical_route_v1.json', dict(readonly=True))
    intent = dict(workspace=str(workspace), environment='sunny13', output=str(p['old']),
        scripts=str(borrowed.LOGICAL_SCRIPTS), manifest=graph['manifest']['source'],
        release=graph['release']['source'], witness=str(witness), witness_sha256=borrowed.sha(witness))
    write(p['intent'], intent)
    session = workspace/'work/ros-session-v2'
    session.mkdir(mode=0o700)
    for name in ('host_contract.json', 'qualification.json'):
        write(session/name, dict(preserved=True))
    source_runtime = {entry['source']:entry['sha256'] for entry in graph['files'].values()}
    source_runtime.update({str(borrowed.LOGICAL_SCRIPTS/name):borrowed.sha(p['scripts']/name)
                           for name in borrowed.EXTRA_SCRIPTS})
    deployment = dict(schema=borrowed.SCHEMA, workspace=str(workspace), environment='sunny13',
        runtime_hashes={str(path):borrowed.sha(path) for path in p['scripts'].iterdir()},
        source_runtime_hashes=source_runtime,
        helper_hashes={str(path):borrowed.sha(path) for path in p['checks'].iterdir()},
        recovery_intent_sha256=borrowed.sha(p['intent']))
    write(p['deployment'], deployment)
    ctx = dict(workspace=workspace, bundle=workspace/'capture_bundle', graph=graph,
        mr=home/'MoraiLauncher_Lin', session=session, home=workspace/'work/ros-home-v2')
    old = SimpleNamespace(load=Mock(return_value=ctx), ready=Mock(), verify_qualification=Mock(),
        namespace=Mock(side_effect=lambda ctx, command:['bwrap', '--ro-bind', '/', '/',
            '--bind', str(workspace), str(workspace), '--chdir', '/home/luv/AjouNice2026', '--', *command]),
        environment=Mock(return_value=dict(PYTHONPATH='/run/base5-bundle/python', PATH='/usr/bin')),
        port_open=Mock(return_value=True), service_owned=Mock(return_value=True),
        signal_inner_app=Mock(), write_new=lambda path,value:write(path,value))
    monkeypatch.setattr(Path, 'home', lambda:home)
    monkeypatch.setattr(borrowed, '__file__', str(p['checks']/'unified_borrowed.py'))
    monkeypatch.setattr(borrowed, 'load_original', lambda workspace:old)
    return SimpleNamespace(workspace=workspace, paths=p, graph=graph, old=old,
        deployment=deployment, ctx=ctx, intent=intent)


def profile(f):
    return write(f.paths['profile'], dict(environment='sunny13', old_output=str(f.paths['old']),
        output=str(f.paths['output']), manifest=f.intent['manifest'], release=f.intent['release'],
        route_witness=dict(path=f.intent['witness'], sha256=f.intent['witness_sha256'])))


def test_load_reuses_original_native_host_qualification_without_writes(fixture):
    f = fixture
    before = {str(path):path.read_bytes() for path in f.workspace.rglob('*') if path.is_file()}
    ctx = borrowed.load()
    assert ctx['old_live'] is f.old
    f.old.load.assert_called_once_with(f.workspace, verify=True)
    f.old.ready.assert_called_once_with(f.ctx)
    f.old.verify_qualification.assert_called_once_with(f.ctx)
    assert before == {str(path):path.read_bytes() for path in f.workspace.rglob('*') if path.is_file()}


@pytest.mark.parametrize('kind', ['changed_script', 'extra_script', 'missing_script', 'changed_helper',
    'extra_helper', 'writable_helper', 'symlink_helper', 'changed_intent', 'changed_witness'])
def test_changed_deployment_inventory_or_old_evidence_fails_closed(fixture, kind):
    f = fixture
    if kind == 'changed_script':
        write(f.paths['scripts']/'legacy_0.py', '# changed')
    elif kind == 'extra_script':
        write(f.paths['scripts']/'unexpected.py', '# extra')
    elif kind == 'missing_script':
        (f.paths['scripts']/'legacy_0.py').unlink()
    elif kind == 'changed_helper':
        write(f.paths['checks']/'unified_capture.py', '# changed')
    elif kind == 'extra_helper':
        write(f.paths['checks']/'unexpected.py', '# extra')
    elif kind == 'writable_helper':
        (f.paths['checks']/'unified_capture.py').chmod(0o666)
    elif kind == 'symlink_helper':
        path = f.paths['checks']/'unified_capture.py'
        path.unlink();path.symlink_to(f.paths['checks']/'unified_borrowed.py')
    elif kind == 'changed_intent':
        write(f.paths['intent'], dict(changed=True))
    else:
        write(Path(f.intent['witness']), dict(changed=True))
    with pytest.raises(ValueError):
        borrowed.load()


@pytest.mark.parametrize('update_source_pin',[False,True])
def test_updated_overlay_pin_still_cannot_replace_original_427_bytes(fixture,update_source_pin):
    f = fixture
    path = write(f.paths['scripts']/'legacy_0.py', '# new but not historical')
    deployment = dict(f.deployment)
    deployment['runtime_hashes'][str(path)] = borrowed.sha(path)
    if update_source_pin:
        deployment['source_runtime_hashes'][str(borrowed.LOGICAL_SCRIPTS/'legacy_0.py')]=borrowed.sha(path)
    write(f.paths['deployment'], deployment)
    with pytest.raises(ValueError, match='preserve all 427|371 qualified scripts'):
        borrowed.load()


def test_namespace_overlays_scripts_readonly_and_protects_all_old_output_trees(fixture):
    f = fixture;ctx = borrowed.load()
    argv = borrowed.namespace(ctx, ['python3', '-B', 'common.py'])
    readonly = [(argv[index+1], argv[index+2]) for index,value in enumerate(argv) if value == '--ro-bind']
    assert (str(f.paths['scripts']), str(borrowed.LOGICAL_SCRIPTS)) in readonly
    for key in ('scripts', 'checks', 'deployment'):
        assert (str(f.paths[key]), str(f.paths[key])) in readonly
    for name in borrowed.OLD_OUTPUTS:
        root = str(f.workspace/'collections'/name)
        assert (root,root) in readonly
    assert (str(f.paths['root']),str(f.paths['root'])) not in readonly
    assert (str(f.paths['profile']),str(f.paths['profile'])) not in readonly
    assert argv[-3:] == ['python3', '-B', 'common.py']
    assert '--unshare-net' not in argv


def test_environment_adds_only_pinned_helpers_to_original_isolated_pythonpath(fixture):
    ctx = borrowed.load()
    env = borrowed.environment(ctx)
    assert env['PYTHONPATH'].split(':') == [str(fixture.paths['checks']), '/run/base5-bundle/python']
    assert 'SSH_AUTH_SOCK' not in env and 'DISPLAY' not in env


def test_prepare_command_is_cpu_only_fixed_scope_and_never_reprepares(fixture):
    f = fixture;ctx = borrowed.load()
    argv = borrowed.command(ctx, 'prepare')
    assert argv[:3] == ['/usr/bin/python3', '-B', str(f.paths['checks']/'unified_capture.py')]
    assert '--prepare' in argv and '--run' not in argv and '--complete' not in argv
    for flag, key in (('--profile','profile'), ('--old-output','old'), ('--output','output')):
        assert argv[argv.index(flag)+1] == str(f.paths[key])
    assert argv[argv.index('--manifest')+1] == f.intent['manifest']
    profile(f)
    with pytest.raises(ValueError, match='Fresh unified handoff'):
        borrowed.command(ctx, 'prepare')


def test_bulk_and_qualification_commands_have_separate_explicit_capture_scopes(fixture):
    f = fixture;ctx = borrowed.load();profile(f)
    run = borrowed.command(ctx, 'run', stop_after_chunks=1)
    assert '--run' in run and '--indices' not in run and '--manifest' not in run
    assert run[run.index('--stop-after-chunks')+1] == '1'
    qualify = borrowed.command(ctx, 'qualify', selected='0,2,4999')
    assert '--qualify-prototype' in qualify and '--run' not in qualify
    assert qualify[qualify.index('--indices')+1] == '0,2,4999'
    assert '--complete' not in run+qualify


def test_refresh_command_only_requests_cpu_profile_refresh_at_fixed_profile(fixture):
    f=fixture;ctx=borrowed.load();profile(f)
    argv=borrowed.command(ctx,'refresh')
    assert argv == ['/usr/bin/python3','-B',str(f.paths['checks']/'unified_capture.py'),
        '--profile',str(f.paths['profile']),'--refresh-prepared-profile']
    assert '--confirmed-environment' not in argv and '--run' not in argv
    for selected,chunk_limit in (('1',0),(None,1)):
        with pytest.raises(ValueError):
            borrowed.command(ctx,'refresh',selected,chunk_limit)


def test_refresh_uses_host_capture_lock_without_ros_or_ui_capture_authority(fixture,monkeypatch):
    f=fixture;profile(f)
    child=Mock();child.wait.return_value=0;child.poll.return_value=0
    spawn=Mock(return_value=child);monkeypatch.setattr(borrowed.subprocess,'Popen',spawn)
    assert borrowed.main(['refresh'])==0
    argv=spawn.call_args.args[0]
    assert '--refresh-prepared-profile' in argv
    assert '--run' not in argv and '--qualify-prototype' not in argv
    assert (f.ctx['session']/'capture.lock').is_file()
    f.old.port_open.assert_not_called();f.old.service_owned.assert_not_called()


@pytest.mark.parametrize('value', ['', '-1', '5000', '1,1', '1, 2', '1;reboot', '1.0',
    ','.join(map(str, range(33))), '１'])
def test_unbounded_or_malformed_qualification_indices_are_rejected(value):
    with pytest.raises(ValueError):
        borrowed.indices(value)


@pytest.mark.parametrize('field', ['environment', 'old_output', 'output', 'manifest', 'release', 'route_witness'])
def test_prepared_profile_cannot_select_arbitrary_user_paths(fixture, field):
    f = fixture;ctx = borrowed.load();path = profile(f)
    doc = borrowed.read(path);doc[field] = 'foreign'
    write(path,doc)
    with pytest.raises((ValueError, AttributeError)):
        borrowed.command(ctx, 'run')


def test_main_plan_and_status_never_spawn_or_change_files(fixture, monkeypatch):
    f = fixture;profile(f)
    spawn = Mock();monkeypatch.setattr(borrowed.subprocess,'Popen',spawn)
    before = {str(p):p.read_bytes() for p in f.workspace.rglob('*') if p.is_file()}
    for action in ('plan','status'):
        with redirect_stdout(io.StringIO()):
            assert borrowed.main([action]) == 0
    spawn.assert_not_called()
    assert before == {str(p):p.read_bytes() for p in f.workspace.rglob('*') if p.is_file()}


@pytest.mark.parametrize('argv', [['run'], ['qualify','--indices','1'], ['plan','--confirmed-environment','sunny13'],
    ['prepare','--stop-after-chunks','1'], ['run','--confirmed-environment','sunny13','--indices','1'],
    ['refresh','--confirmed-environment','sunny13'],['refresh','--indices','1'],
    ['refresh','--stop-after-chunks','1']])
def test_capture_requires_confirmation_and_readonly_actions_cannot_receive_capture_flags(argv, monkeypatch):
    load = Mock();monkeypatch.setattr(borrowed,'load',load)
    with pytest.raises(ValueError):
        borrowed.main(argv)
    load.assert_not_called()


def test_run_needs_both_original_owned_services_and_no_unconfined_fallback(fixture, monkeypatch):
    f = fixture;ctx = borrowed.load();profile(f)
    f.old.service_owned.return_value = False
    spawn = Mock();monkeypatch.setattr(borrowed.subprocess,'Popen',spawn)
    with pytest.raises(ValueError, match='approved ros'):
        borrowed.supervise(ctx,'run',borrowed.command(ctx,'run'))
    spawn.assert_not_called()


def test_pause_marker_targets_only_new_output(fixture):
    f = fixture;ctx = borrowed.load()
    f.paths['output'].mkdir(mode=0o700)
    marker = borrowed.request_pause(ctx)
    assert marker == f.paths['output']/'PAUSE' and marker.is_file()
    for name in borrowed.OLD_OUTPUTS:
        assert not (f.workspace/'collections'/name/'PAUSE').exists()


def test_ctrl_c_waits_for_current_common_chunk_and_never_signals_ros_bridge(fixture, monkeypatch):
    f = fixture;ctx = borrowed.load();profile(f);f.paths['output'].mkdir(mode=0o700)
    handlers = {}
    def installed(signum, handler):
        previous = handlers.get(signum)
        handlers[signum] = handler
        return previous
    monkeypatch.setattr(borrowed.signal,'signal',installed)
    child = Mock();child.poll.return_value=None
    def wait():
        handlers[signal.SIGINT](signal.SIGINT,None)
        child.poll.return_value = 0
        return 0
    child.wait.side_effect = wait
    monkeypatch.setattr(borrowed.subprocess,'Popen',Mock(return_value=child))
    assert borrowed.supervise(ctx,'run',borrowed.command(ctx,'run')) == 0
    assert (f.paths['output']/'PAUSE').exists()
    f.old.signal_inner_app.assert_not_called()
    assert all(value is None for value in handlers.values())


def test_qualification_interrupt_targets_only_the_exact_inner_application(fixture, monkeypatch):
    f = fixture;ctx = borrowed.load();profile(f)
    handlers = {}
    def installed(signum, handler):
        previous = handlers.get(signum)
        handlers[signum] = handler
        return previous
    monkeypatch.setattr(borrowed.signal,'signal',installed)
    child = Mock();child.poll.return_value=None
    def wait():
        handlers[signal.SIGTERM](signal.SIGTERM,None)
        child.poll.return_value = 0
        return 0
    child.wait.side_effect = wait
    monkeypatch.setattr(borrowed.subprocess,'Popen',Mock(return_value=child))
    argv = borrowed.command(ctx,'qualify',selected='1,2')
    assert borrowed.supervise(ctx,'qualify',argv) == 0
    f.old.signal_inner_app.assert_called_once_with(child,argv,signal.SIGINT)
    assert not (f.paths['output']/'PAUSE').exists()


def test_probe_is_import_only_and_does_not_start_services_or_native_capture(fixture, monkeypatch):
    f = fixture
    run = Mock(return_value=SimpleNamespace(returncode=0))
    monkeypatch.setattr(borrowed.subprocess,'run',run)
    assert borrowed.main(['probe']) == 0
    command = run.call_args.args[0]
    assert 'unified_capture as common; common.modules()' in command[-1]
    assert '--prepare' not in command and '--run' not in command
    f.old.port_open.assert_not_called()
    f.old.service_owned.assert_not_called()


def test_original_live_loader_keeps_its_exact_path_and_refuses_modified_bytes(tmp_path):
    # fixture의 대역이 아니라 실제 동결 stdlib-only module import를 검사한다.
    import hashlib
    import importlib.util
    source = Path(__file__).with_name('unified_borrowed.py')
    spec = importlib.util.spec_from_file_location('_unified_original_loader_fixture',source)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    live_source = Path(__file__).with_name('live.py')
    assert hashlib.sha256(live_source.read_bytes()).hexdigest() == module.LIVE_SHA
    path = tmp_path/'workspace/checks/live_v2.py'
    write(path,live_source.read_text())
    original = module.load_original(tmp_path/'workspace')
    assert original.__file__ == str(path)
    assert all(hasattr(original,name) for name in ('load','ready','namespace','environment','verify_qualification'))
    write(path,'# altered')
    with pytest.raises(ValueError,match='live_v2 bytes changed'):
        module.load_original(tmp_path/'workspace')
