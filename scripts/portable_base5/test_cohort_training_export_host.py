"""CPU-only isolated namespace, final-5K and immutable deployment tests."""
import json
import os
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

import cohort_training_export_host as host


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value) + '\n')
    path.chmod(0o600)
    return path


@pytest.fixture
def setup(tmp_path, monkeypatch):
    monkeypatch.setattr(host.Path, 'home', staticmethod(lambda: tmp_path))
    workspace = tmp_path / 'ajounice_base5_capture'
    root = workspace / 'work/sunny11-training-export-runtime-v1'
    root.mkdir(parents=True)
    root.chmod(0o700)
    for name in host.FILES:
        write(root / name, dict(fixture=name))
    monkeypatch.setattr(host, '__file__', str(root / 'cohort_training_export_host.py'))
    fresh_root = workspace / 'work/fresh-sunny11-v3'
    helper = write(fresh_root / 'checks/fresh_borrowed.py', dict(fixture='fresh helper'))
    helper.parent.chmod(0o700)
    fresh = write(fresh_root / 'deployment-capture.json', dict(environment='sunny11',
        workspace=str(workspace), output=str(workspace / 'collections/sunny11_5k_v160_v1'),
        helper_hashes={str(helper): host.sha(helper)}))
    source = workspace / 'collections/sunny11_5k_flexible_v1'
    for name in host.CONTROL_FILES:
        write(source / name, dict(fixture=name))
    source.chmod(0o700)
    baseline = workspace / 'collections/sunny11_5k_v160_v1'
    baseline.mkdir(mode=0o700)
    write(source / 'status.json', dict(stage='complete', accepted_candidate=5000,
        review=0, failed=0, error=None, current=None, selected_not_certified=False))
    write(source / 'quota.json', dict(deficit=0, cells=[]))
    export = workspace / 'work/sunny11-training-export-v1'
    export.mkdir()
    export.chmod(0o700)
    (export / 'tmp').mkdir()
    doc = dict(schema=host.SCHEMA, workspace=str(workspace), environment='sunny11',
        files={name: host.sha(root / name) for name in host.FILES},
        fresh_deployment=str(fresh), fresh_deployment_sha256=host.sha(fresh),
        selected_root=str(source), export_root=str(export),
        usage_roots=[str(source), str(baseline)],
        source_pins={name: host.sha(source / name) for name in host.CONTROL_FILES},
        archive_name='base5_sunny11_5k_jpeg90_upright3_261019.tar.zst')
    deployment = write(root / 'deployment.json', doc)
    return SimpleNamespace(root=root, workspace=workspace, source=source, export=export,
        fresh=fresh, helper=helper, doc=doc, deployment=deployment)


def save(setup):
    write(setup.deployment, setup.doc)


def test_scope_accepts_only_frozen_same_environment_complete_native_collection(setup):
    scoped = host.scope(setup.root, 'sunny11')
    assert scoped['source'] == setup.source and scoped['export'] == setup.export
    assert scoped['fresh_deployment'] == setup.fresh


@pytest.mark.parametrize('change', ['helper_sha', 'extra_python', 'source_pin', 'source_pin_missing',
    'wrong_env', 'wrong_fresh_env', 'wrong_fresh_sha', 'not_complete', 'partial', 'review',
    'failed', 'error', 'current', 'uncertified', 'quota', 'quota_cells', 'wrong_archive',
    'source_prefix', 'export_prefix', 'group_writable', 'symlink', 'budget_duplicate',
    'budget_missing_baseline', 'budget_other_environment'])
def test_unsafe_or_changed_scope_is_rejected_before_any_process(setup, change):
    if change == 'helper_sha':
        write(setup.root / 'pack_training.py', dict(changed=True))
    elif change == 'extra_python':
        write(setup.root / 'unapproved.py', dict(extra=True))
    elif change == 'source_pin':
        write(setup.source / 'selected_ledger.jsonl', dict(changed=True))
    elif change == 'source_pin_missing':
        setup.doc['source_pins'].pop('status.json')
    elif change == 'wrong_env':
        setup.doc['environment'] = 'sunny13'
    elif change == 'wrong_fresh_env':
        fresh = host.read(setup.fresh)
        fresh['environment'] = 'sunny13'
        write(setup.fresh, fresh)
        setup.doc['fresh_deployment_sha256'] = host.sha(setup.fresh)
    elif change == 'wrong_fresh_sha':
        write(setup.fresh, dict(changed=True))
    elif change in ('not_complete', 'partial', 'review', 'failed', 'error', 'current', 'uncertified'):
        status = host.read(setup.source / 'status.json')
        key, value = dict(not_complete=('stage', 'running'), partial=('accepted_candidate', 4999),
            review=('review', 1), failed=('failed', 1), error=('error', 'failed'),
            current=('current', {'work': 'active'}), uncertified=('selected_not_certified', True))[change]
        status[key] = value
        write(setup.source / 'status.json', status)
        setup.doc['source_pins']['status.json'] = host.sha(setup.source / 'status.json')
    elif change in ('quota', 'quota_cells'):
        write(setup.source / 'quota.json', dict(deficit=int(change == 'quota'),
            cells=[dict(id='pending')] if change == 'quota_cells' else []))
        setup.doc['source_pins']['quota.json'] = host.sha(setup.source / 'quota.json')
    elif change == 'wrong_archive':
        setup.doc['archive_name'] = 'base5_sunny13_5k_jpeg90_upright3_261019.tar.zst'
    elif change == 'source_prefix':
        setup.doc['selected_root'] = str(setup.source.parent / 'sunny13_5k')
        Path(setup.doc['selected_root']).mkdir()
    elif change == 'export_prefix':
        setup.doc['export_root'] = str(setup.export.parent / 'sunny13-training-export-v1')
        Path(setup.doc['export_root']).mkdir()
    elif change == 'group_writable':
        (setup.root / 'pack_training.py').chmod(0o660)
    elif change == 'symlink':
        filename = setup.root / 'pack_training.py'
        other = setup.root / 'pack_training.txt'
        filename.rename(other)
        filename.symlink_to(other)
    elif change == 'budget_duplicate':
        setup.doc['usage_roots'].append(str(setup.source))
    elif change == 'budget_missing_baseline':
        setup.doc['usage_roots'] = [str(setup.source)]
    elif change == 'budget_other_environment':
        other = setup.source.parent / 'sunny13_5k_v1'
        other.mkdir(mode=0o700)
        setup.doc['usage_roots'].append(str(other))
    save(setup)
    with pytest.raises(ValueError):
        host.scope(setup.root, 'sunny11')


@pytest.mark.parametrize('action', ['plan', 'qualify', 'pack', 'validate'])
def test_command_uses_same_final_selection_identity_and_only_explicit_mutation(setup, action):
    scoped = host.scope(setup.root, 'sunny11')
    argv = host.command(scoped, action)
    assert argv[:3] == ['/usr/bin/python3', '-B', '-u']
    assert argv[argv.index('--cohort') + 1] == 'sunny11'
    assert argv[argv.index('--selected') + 1] == str(setup.source / 'layout_split_final/derived_selected_ledger.jsonl')
    assert ('--run' in argv) == (action in ('qualify', 'pack'))
    assert argv.count('--usage-root') == 2
    assert str(setup.workspace / 'collections') not in argv
    assert not any('delete' in word or 'capture.py' in word for word in argv)


def platform(setup, monkeypatch, subprocess_result=0):
    scoped = host.scope(setup.root, 'sunny11')
    session = setup.workspace / 'session'
    session.mkdir()
    write(session / 'capture.lock', dict(lock=True))
    ctx = dict(session=session, mr=setup.workspace / 'morai')
    fresh = SimpleNamespace(namespace=Mock(side_effect=lambda ctx, command:
        ['bwrap', '--bind', str(setup.workspace / 'work'), str(setup.workspace / 'work'),
         '--chdir', '/logical', '--', *command]),
        environment=Mock(return_value=dict(PYTHONPATH='/frozen/python')))
    monkeypatch.setattr(host, 'load_platform', Mock(return_value=(fresh, ctx)))
    monkeypatch.setattr(host.os, 'nice', Mock())
    runner = Mock(return_value=SimpleNamespace(returncode=subprocess_result))
    monkeypatch.setattr(host.subprocess, 'run', runner)
    return scoped, fresh, ctx, runner


def test_final_mounts_make_workspace_morai_selection_readonly_and_only_export_writable(setup, monkeypatch):
    scoped, fresh, ctx, runner = platform(setup, monkeypatch)
    argv = host.isolated_namespace(scoped, fresh, ctx, 'plan')
    end = argv.index('--chdir')
    assert '--unshare-net' in argv[:end]
    assert argv[end-3:end] == ['--bind', str(setup.export), str(setup.export)]
    for protected in (setup.workspace, setup.source, ctx['mr']):
        assert ['--ro-bind', str(protected), str(protected)] in [argv[i:i+3] for i in range(end)]
    assert not runner.called


def test_host_plan_keeps_source_pins_and_private_umask_and_uses_original_flock(setup, monkeypatch):
    _, _, _, runner = platform(setup, monkeypatch)
    before = {name: (setup.source / name).read_bytes() for name in host.CONTROL_FILES}
    previous = os.umask(0o077)
    os.umask(previous)
    assert host.execute('plan', 'sunny11') == 0
    after = os.umask(0o077)
    os.umask(after)
    assert previous == after
    assert before == {name: (setup.source / name).read_bytes() for name in host.CONTROL_FILES}
    env = runner.call_args.kwargs['env']
    assert env['TMPDIR'] == str(setup.export / 'tmp')
    assert env['PYTHONPATH'].startswith(str(setup.root) + ':')
    assert env['OMP_NUM_THREADS'] == env['OPENBLAS_NUM_THREADS'] == '1'


def test_failed_original_cpu_process_is_not_retried_or_reclassified(setup, monkeypatch):
    _, _, _, runner = platform(setup, monkeypatch, subprocess_result=7)
    assert host.execute('qualify', 'sunny11') == 7
    runner.assert_called_once()


def test_source_control_change_during_new_export_stops_with_error(setup, monkeypatch):
    _, _, _, runner = platform(setup, monkeypatch)
    def changed(*args, **kwargs):
        write(setup.source / 'selected_ledger.jsonl', dict(changed=True))
        return SimpleNamespace(returncode=0)
    runner.side_effect = changed
    with pytest.raises(ValueError, match='changed during CPU export'):
        host.execute('qualify', 'sunny11')
    runner.assert_called_once()
