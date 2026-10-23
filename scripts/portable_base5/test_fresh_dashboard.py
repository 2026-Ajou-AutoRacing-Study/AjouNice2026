"""원격 접근 없이 새 환경 observer의 RO 범위·정합·무실행 기본값을 검증한다."""
from contextlib import redirect_stdout
import io
import json
import os
from pathlib import Path
import socket
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import fresh_borrowed as fresh
import fresh_dashboard as dashboard


def write(path, value):
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    path.write_text(value if isinstance(value, str) else json.dumps(value)+'\n')
    path.chmod(0o600)
    return path


def snapshot(root):
    return {str(p):p.read_bytes() for p in root.rglob('*') if p.is_file()}


@pytest.fixture
def fixture(tmp_path, monkeypatch):
    workspace = tmp_path/'home/ajounice_base5_capture'
    runtime = workspace/'work/fresh-runtime-v1/sunny11'
    output = workspace/'collections/sunny11_5k_fresh'
    output.mkdir(mode=0o700, parents=True)
    old = workspace/'collections/sunny13_complete'; old.mkdir(mode=0o700)
    mr = tmp_path/'home/MoraiLauncher_Lin'; mr.mkdir(mode=0o700)
    source = write(runtime/'source/manifest.json', dict(count=5000, config=dict(environment_tag='sunny11')))
    logical = fresh.RAW/'new_sunny11/manifest.json'
    script = write(workspace/'work/web-dashboard-sunny13-v1/base5_capture_dashboard.py', '# original dashboard\n')
    html = write(script.with_suffix('.html'), '<html>original dashboard</html>')
    monkeypatch.setattr(dashboard, 'SCRIPT_SHA', dashboard.sha(script))
    monkeypatch.setattr(dashboard, 'HTML_SHA', dashboard.sha(html))
    key = str(fresh.SCRIPTS/'base5_capture_dashboard.py')
    paths = dict(root=runtime, profile=runtime/'profile.json', checks=runtime/'checks',
                 scripts=runtime/'scripts', output=output, deployment=runtime/'deployment.json')
    dep = dict(environment='sunny11', manifest=str(logical), manifest_sha256=fresh.sha(source),
        output=str(output), release='/release.json', release_sha256='a'*64,
        route_witness=dict(path='/witness.json', sha256='b'*64),
        source_runtime_hashes={key:dashboard.sha(script)}, input_hashes={str(logical):fresh.sha(source)})
    write(paths['deployment'], dep)
    doc = dict(schema='base5_unified_capture_v1', policy='fixed-input-boundaries-and-common-native-profiles-v1',
        fresh_start=True, environment='sunny11', manifest=str(logical), manifest_sha256=fresh.sha(source),
        release=dep['release'], release_sha256=dep['release_sha256'], output=str(output),
        route_witness=dep['route_witness'])
    write(paths['profile'], doc)
    write(output/'status.json', dict(stage='capturing', accepted_candidate=25))
    ctx = dict(workspace=workspace, mr=mr, paths=paths, deployment=dep, environment='sunny11', graph={})
    def namespace(ctx, command):
        return ['bwrap', '--ro-bind', '/', '/', '--bind', str(workspace), str(workspace),
                '--bind', str(mr), str(mr), '--chdir', str(fresh.PROJECT), '--', *command]
    module = SimpleNamespace(validate_profile=fresh.validate_profile, logical=fresh.logical, sha=fresh.sha,
        mapped=lambda ctx, name:source, SCRIPTS=fresh.SCRIPTS, namespace=namespace)
    loader = Mock(return_value=(module, ctx))
    monkeypatch.setattr(dashboard, 'load_fresh', loader)
    monkeypatch.setattr(dashboard, 'check_port', lambda port:None)
    return SimpleNamespace(base=tmp_path, workspace=workspace, runtime=runtime, output=output,
        old=old, mr=mr, source=source, script=script, paths=paths, dep=dep, doc=doc,
        ctx=ctx, module=module, loader=loader, key=key)


def configuration(f, **kwargs):
    return dashboard.configuration(f.paths['deployment'], f.ctx['environment'], **kwargs)


def call(f, args=()):
    output = io.StringIO()
    with redirect_stdout(output):
        result = dashboard.main(['--deployment', str(f.paths['deployment']),
            '--environment', f.ctx['environment'], *args])
    return result, output.getvalue()


def test_default_plan_writes_nothing_and_never_starts_or_controls_processes(fixture, monkeypatch):
    f = fixture; before = snapshot(f.base); spawn = Mock(); kill = Mock()
    monkeypatch.setattr(dashboard.subprocess, 'Popen', spawn); monkeypatch.setattr(dashboard.os, 'kill', kill)
    result, text = call(f); plan = json.loads(text)
    assert result == 0 and plan['capture_root'] == str(f.output)
    assert not plan['files_written'] and not plan['process_started'] and not plan['capture_controls']
    assert not Path(plan['writable_state_only']).exists() and before == snapshot(f.base)
    spawn.assert_not_called(); kill.assert_not_called()
    f.loader.assert_called_once_with(f.paths['deployment'], 'sunny11')


@pytest.mark.parametrize('environment', dashboard.ENVIRONMENTS)
def test_six_environments_use_the_same_profile_and_observer_path(fixture, environment):
    f = fixture; f.ctx['environment'] = f.dep['environment'] = f.doc['environment'] = environment
    output = f.workspace/'collections'/(environment+'_5k_fresh'); output.mkdir(exist_ok=True, mode=0o700)
    f.paths['output'] = output; f.dep['output'] = f.doc['output'] = str(output)
    write(f.paths['profile'], f.doc)
    assert configuration(f)['root'] == output


@pytest.mark.parametrize('field', ['environment', 'manifest', 'manifest_sha256', 'output', 'release',
    'release_sha256', 'route_witness', 'fresh_start', 'schema', 'policy'])
def test_changed_profile_fails_closed(fixture, field):
    f = fixture; f.doc[field] = 'changed'; write(f.paths['profile'], f.doc)
    with pytest.raises(ValueError): configuration(f)


@pytest.mark.parametrize('asset', ['script', 'html'])
def test_changed_ui_bytes_are_rejected(fixture, asset):
    f = fixture; write(f.script if asset == 'script' else f.script.with_suffix('.html'), 'changed')
    with pytest.raises(ValueError, match='Frozen dashboard'): configuration(f)


def test_conflicting_ui_pin_is_rejected(fixture):
    f = fixture; f.doc['fixed_pins'] = {f.key:'c'*64}; write(f.paths['profile'], f.doc)
    with pytest.raises(ValueError, match='pin'): configuration(f)


def test_missing_ui_closure_uses_only_exact_original_graph_pin(fixture):
    f = fixture; f.dep['source_runtime_hashes'] = {}
    with pytest.raises(ValueError, match='pin'): configuration(f)
    f.ctx['graph'] = dict(files={'ui':dict(source=f.key, sha256=dashboard.SCRIPT_SHA)})
    assert configuration(f)['script'] == f.script


def test_namespace_covers_workspace_morai_and_keeps_only_separate_state_writable(fixture):
    f = fixture; c = configuration(f); command = dashboard.namespace_command(c)
    at = command.index('--chdir')
    assert command[at-9:at] == ['--ro-bind', str(f.workspace), str(f.workspace),
        '--ro-bind', str(f.mr), str(f.mr), '--bind', str(c['state']), str(c['state'])]
    effective = {}
    for i, value in enumerate(command):
        if value in ('--bind', '--ro-bind'): effective[command[i+2]] = value
    assert {p for p, mode in effective.items() if mode == '--bind'} == {str(c['state'])}
    env = dashboard.environment(c)
    assert env['HOME'] == str(c['state']) and env['TMPDIR'] == str(c['state'])
    assert not any(k.startswith(('ROS_', 'WANDB_', 'SSH_')) for k in env)


def test_storage_roots_are_explicit_deduplicated_direct_collections_and_do_not_change_capture_root(fixture):
    f = fixture; c = configuration(f, storage=[f.old, f.old]); argv = dashboard.dashboard_command(c)
    assert c['storage'] == [f.old] and argv.count('--storage-root') == 1
    assert argv[argv.index('--capture-root')+1] == str(f.output)
    assert argv[argv.index('--storage-root')+1] == str(f.old)


@pytest.mark.parametrize('kind', ['external', 'missing', 'capture', 'symlink'])
def test_invalid_storage_scopes_fail_closed(fixture, kind):
    f = fixture
    path = f.base if kind == 'external' else f.workspace/'collections/missing' if kind == 'missing' else f.output
    if kind == 'symlink':
        path = f.workspace/'collections/alias'; path.symlink_to(f.old, target_is_directory=True)
    with pytest.raises((OSError, ValueError)): configuration(f, storage=[path])


@pytest.mark.parametrize('kind', ['symlink', 'hardlink', 'mode'])
def test_state_cannot_alias_protected_raw_or_sources(fixture, kind):
    f = fixture; c = configuration(f); c['state'].mkdir(mode=0o700)
    if kind == 'symlink': (c['state']/'history.jsonl').symlink_to(f.output/'status.json')
    elif kind == 'hardlink': os.link(f.output/'status.json', c['state']/'history.jsonl')
    else: c['state'].chmod(0o777)
    with pytest.raises(ValueError): dashboard.state_directory(c)


def test_occupied_port_never_creates_state_or_terminates_old_server(fixture, monkeypatch):
    f = fixture; spawn = Mock(); kill = Mock()
    monkeypatch.setattr(dashboard.subprocess, 'Popen', spawn); monkeypatch.setattr(dashboard.os, 'kill', kill)
    def occupied(port): raise ValueError('기존 대시보드 Ctrl+C')
    monkeypatch.setattr(dashboard, 'check_port', occupied)
    with pytest.raises(ValueError, match='Ctrl\\+C'): call(f, ['--run'])
    assert not configuration(f)['state'].exists(); spawn.assert_not_called(); kill.assert_not_called()


def test_run_and_snapshot_only_start_frozen_observer_and_keep_profile_raw_unchanged(fixture, monkeypatch):
    f = fixture; before = snapshot(f.base); child = Mock(); child.wait.return_value = 19
    spawn = Mock(return_value=child); monkeypatch.setattr(dashboard.subprocess, 'Popen', spawn)
    result, _ = call(f, ['--run', '--once']); c = configuration(f)
    assert result == 19 and c['state'].stat().st_mode & 0o777 == 0o700
    command = spawn.call_args.args[0]
    assert command[-1] == '--once' and '--run' not in command and '--prepare' not in command
    assert str(f.script) in command and before == snapshot(f.base)
    child.wait.assert_called_once_with()


@pytest.mark.parametrize('port', [0, 80, 65536, True])
def test_only_unprivileged_integer_ports(fixture, port):
    with pytest.raises(ValueError): configuration(fixture, port=port)


def test_observer_must_not_be_added_to_frozen_helpers(tmp_path, monkeypatch):
    dep = write(tmp_path/'runtime/deployment.json', {})
    monkeypatch.setattr(dashboard, '__file__', str(dep.parent/'checks/fresh_dashboard.py'))
    with pytest.raises(ValueError, match='outside frozen checks'):
        dashboard.load_fresh(dep, 'sunny11')
