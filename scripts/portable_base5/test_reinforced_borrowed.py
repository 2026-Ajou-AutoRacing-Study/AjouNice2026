from pathlib import Path
from types import SimpleNamespace
import importlib.util
import json
import os
import shutil
import subprocess
import sys

import pytest

import reinforced_borrowed as adapter


def test_environment_only_adds_pinned_logical_path(monkeypatch):
    monkeypatch.setattr(adapter, 'BASE_ENVIRONMENT', lambda _: dict(PYTHONPATH='/new/checks:/old/platform', OTHER='unchanged'))
    result = adapter.environment({})
    assert result == dict(PYTHONPATH=str(adapter.LOGICAL_HELPERS)+':/new/checks:/old/platform', OTHER='unchanged')


@pytest.mark.parametrize('action', ('prepare', 'plan', 'qualify', 'run'))
def test_common_main_and_worker_start_from_logical_path(monkeypatch, action):
    ctx = dict(paths=dict(checks=Path('/new/checks')))
    original = ['/usr/bin/python3', '-B', '/new/checks/unified_capture.py', '--profile', '/new/profile.json', '--run']
    monkeypatch.setattr(adapter, 'BASE_COMMAND', lambda context, chosen, selected: list(original))
    result = adapter.command(ctx, action, None)
    assert result == original[:2]+[str(adapter.LOGICAL_HELPERS/'unified_capture.py')]+original[3:]
    assert original[2] == '/new/checks/unified_capture.py'


def test_probe_stays_read_only_and_foreign_command_is_not_rerouted(monkeypatch):
    ctx = dict(paths=dict(checks=Path('/new/checks')))
    command = ['/usr/bin/python3', '-B', '-c', 'read_only_probe']
    monkeypatch.setattr(adapter, 'BASE_COMMAND', lambda *args: list(command))
    assert adapter.command(ctx, 'probe') == command
    with pytest.raises(ValueError, match='command shape'):
        adapter.command(ctx, 'run')


def fake_context(monkeypatch):
    checks = Path('/approved/work/fresh/checks')
    names = set(adapter.LOGICAL_REQUIRED) | {'reinforced_borrowed.py'}
    own = adapter.original.physical(adapter.__file__)
    pins = {str(checks/name): 'a'*64 for name in names}
    pins[str(own)] = 'a'*64
    inputs = {str(adapter.LOGICAL_HELPERS/name): 'a'*64 for name in adapter.LOGICAL_REQUIRED}
    ctx = dict(paths=dict(checks=checks), deployment=dict(helper_hashes=pins, input_hashes=inputs))
    monkeypatch.setattr(adapter.original, 'sha', lambda _: 'a'*64)
    monkeypatch.setattr(adapter.original, 'mapped', lambda ctx, value: Path(value))
    return ctx


def test_both_path_copies_must_be_exact(monkeypatch):
    adapter.validate_helpers(fake_context(monkeypatch))


@pytest.mark.parametrize('kind', ('adapter', 'logical', 'physical'))
def test_incomplete_path_binding_rejected(monkeypatch, kind):
    ctx = fake_context(monkeypatch)
    if kind == 'adapter':
        ctx['deployment']['helper_hashes'].pop(str(adapter.original.physical(adapter.__file__)))
    elif kind == 'logical':
        ctx['deployment']['input_hashes'].pop(str(adapter.LOGICAL_HELPERS/'reinforcement_profile.py'))
    else:
        ctx['deployment']['helper_hashes'].pop(str(ctx['paths']['checks']/'reinforcement_profile.py'))
    with pytest.raises(ValueError):
        adapter.validate_helpers(ctx)


def test_no_bulk_or_arbitrary_process_in_workflow_actions():
    assert 'run' not in adapter.WORKFLOW_PHASES
    assert 'bulk' not in adapter.WORKFLOW_PHASES
    assert adapter.NATIVE_PHASES <= adapter.WORKFLOW_PHASES


def test_actual_main_dispatch_installs_and_restores_exact_adapters(monkeypatch):
    ctx = dict(paths=dict(checks=Path('/approved/checks')), deployment=dict(helper_hashes={
        '/approved/checks/reinforced_environment_workflow.py': 'a'*64}))
    calls = []
    previous_env, previous_command = adapter.original.environment, adapter.original.command
    monkeypatch.setattr(adapter.original, 'load', lambda *a, **k: ctx)
    monkeypatch.setattr(adapter, 'validate_helpers', lambda context: None)
    monkeypatch.setattr(adapter.original, 'sha', lambda _: 'a'*64)

    def original_main(argv):
        assert adapter.original.environment is adapter.environment
        assert adapter.original.command is adapter.command
        calls.append(('original_main', argv))
        return 0

    monkeypatch.setattr(adapter.original, 'main', original_main)
    monkeypatch.setattr(adapter.original, 'supervise', lambda context, action, argv: calls.append((action, argv)) or 0)
    prefix = ['--deployment', '/approved/deployment.json', '--environment', 'foggy15']
    assert adapter.main(prefix+['plan']) == 0
    assert adapter.main(prefix+['workflow', '--', 'register', '--build-extension', '/logical/extension']) == 0
    assert adapter.main(prefix+['workflow', '--', 'first16', '--run', '--confirmed-environment', 'foggy15']) == 0
    assert calls[0][0] == 'original_main'
    assert calls[1][0] == 'prepare'
    assert calls[2][0] == 'prototype'
    assert calls[2][1][:5] == ['/usr/bin/python3', '-B', str(adapter.LOGICAL_HELPERS/'reinforced_environment_workflow.py'),
                              '--environment', 'foggy15']
    assert adapter.original.environment is previous_env
    assert adapter.original.command is previous_command


def test_fresh_profile_does_not_require_physical_check_helper_paths(tmp_path):
    """Fresh profile validation binds source/output, not a physical helper prefix."""
    logical = Path(adapter.__file__).resolve().parent
    checks = tmp_path/'checks'
    checks.mkdir(mode=0o700)
    for name in ('unified_capture.py', 'fresh_route_compat.py', 'fresh_environment_qualification.py'):
        shutil.copyfile(logical/name, checks/name)
    spec = importlib.util.spec_from_file_location('_exact_deployed_common_helper_identity', checks/'unified_capture.py')
    common = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(common)
    pins = common.helpers(fresh_start=True)
    assert str(checks/'unified_capture.py') in pins
    assert str(checks/'fresh_route_compat.py') in pins
    assert str(checks/'fresh_environment_qualification.py') in pins
    assert str(logical/'session_input_boundary.py') in pins
    assert str(logical/'unified_paused_import.py') in pins
    assert str(logical/'recovery_import.py') in pins
    assert str(logical/'recovery_population.py') in pins
    profile = tmp_path/'profile.json'
    document = dict(fresh_start=True, environment='foggy15', manifest='/logical/full/manifest.json',
        manifest_sha256='a'*64, release='/new/release.json', release_sha256='b'*64,
        output=str(tmp_path/'collection'), route_witness=dict(path='/logical/witness.json', sha256='c'*64),
        helper_pins=pins)
    profile.write_text(json.dumps(document)); profile.chmod(0o600)
    ctx = dict(environment='foggy15', paths=dict(profile=profile, output=tmp_path/'collection'),
        deployment=dict(document))
    assert adapter.original.validate_profile(ctx) == document
    assert common.profile_helpers(document) == pins


def test_logical_main_has_all_logical_helpers_for_unchanged_worker_contract():
    import unified_capture as common
    logical = Path(adapter.__file__).resolve().parent
    assert Path(common.__file__).resolve() == logical/'unified_capture.py'
    pins = common.helpers(fresh_start=True)
    assert pins and all(Path(name).parent == logical for name in pins)
    assert str(logical/'unified_capture.py') in pins
    assert str(logical/'fresh_environment_qualification.py') in pins


def test_actual_direct_python_startup_uses_script_directory_not_check_pythonpath(tmp_path):
    """Real CLI --help imports only; it never reads a profile or starts capture."""
    logical = Path(adapter.__file__).resolve().parent
    checks = tmp_path/'checks'
    checks.mkdir()
    for name in adapter.LOGICAL_REQUIRED:
        shutil.copyfile(logical/name, checks/name)
    env = dict(os.environ, PYTHONPATH=str(checks)+':'+str(logical), PYTHONDONTWRITEBYTECODE='1')
    # Interactive inspection follows argparse's help exit. Python drops the
    # __file__ global here, so inspect the real function code filename instead.
    probe = ("print('IDENTITY=' + json.dumps(dict(main=helpers.__code__.co_filename, "
             "argv=sys.argv[0], modules=[boundary_module.__file__, handoff.__file__, "
             "handoff.historical.__file__, recovery_population.__file__])))\nexit()\n")
    result = subprocess.run([sys.executable, '-B', '-i', str(logical/'unified_capture.py'), '--help'],
        input=probe, text=True, capture_output=True, env=env, timeout=15, check=True)
    records = [json.loads(line[len('IDENTITY='):]) for line in result.stdout.splitlines() if line.startswith('IDENTITY=')]
    assert len(records) == 1
    record = records[0]
    assert record['main'] == record['argv'] == str(logical/'unified_capture.py')
    assert all(Path(filename).parent == logical for filename in record['modules'])
