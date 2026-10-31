"""Fresh-host completion adapter: CPU-only scope and namespace tests."""
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import json

import pytest
import fresh_completion_borrowed as wrapper


def write(path, value):
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    path.write_text(value if isinstance(value, str) else json.dumps(value)+'\n')
    path.chmod(0o600)
    return path


@pytest.fixture
def deployment(tmp_path, monkeypatch):
    home = tmp_path/'home'
    workspace = home/'ajounice_base5_capture'
    root = workspace/'work/completion-v1'
    fresh = workspace/'work/fresh-v1'
    profile = write(fresh/'profile.json', dict(environment='sunny11',
        output=str(workspace/'collections/sunny11_main'), manifest='/logical/manifest.json',
        manifest_sha256='a'*64, release='/logical/release.json', release_sha256='b'*64))
    helper = write(fresh/'checks/fresh_borrowed.py', '# frozen\n')
    original = write(fresh/'deployment-capture.json', dict(environment='sunny11',
        workspace=str(workspace), manifest='/logical/manifest.json', manifest_sha256='a'*64,
        release='/logical/release.json', release_sha256='b'*64,
        helper_hashes={str(helper):wrapper.sha(helper)}))
    for name in wrapper.FILES:
        write(root/name, '# '+name+'\n')
    doc = dict(schema=wrapper.SCHEMA, environment='sunny11', workspace=str(workspace),
        fresh_deployment=str(original), fresh_deployment_sha256=wrapper.sha(original),
        source_profile=str(profile), source_profile_sha256=wrapper.sha(profile),
        baseline=str(workspace/'collections/sunny11_main'),
        seed=str(workspace/'collections/sunny11_seed'), output=str(workspace/'collections/sunny11_flexible'),
        helper_hashes={str(root/name):wrapper.sha(root/name) for name in wrapper.FILES})
    write(root/'deployment.json', doc)
    monkeypatch.setattr(wrapper, '__file__', str(root/'fresh_completion_borrowed.py'))
    monkeypatch.setattr(wrapper.pwd, 'getpwuid', lambda uid:SimpleNamespace(pw_dir=str(home)))
    return SimpleNamespace(root=root, fresh=fresh, profile=profile, original=original,
        helper=helper, doc=doc, workspace=workspace)


def scope(d):
    return wrapper.scope_for(d.root, 'sunny11')


def test_valid_scope_preserves_original_profile(deployment):
    d = deployment; s = scope(d)
    assert s['profile'] == d.profile
    assert s['baseline'] != s['seed'] != s['output']
    assert s['checks'] == d.fresh/'checks'


def test_isolated_session_home_does_not_change_account_scope(deployment, monkeypatch):
    monkeypatch.setenv('HOME', '/isolated/session')
    assert scope(deployment)['workspace'] == deployment.workspace


@pytest.mark.parametrize('name', sorted(wrapper.FILES))
def test_changed_helper_rejected(deployment, name):
    write(deployment.root/name, '# changed\n')
    with pytest.raises(ValueError, match='inventory/SHA'):
        scope(deployment)


def test_extra_helper_rejected(deployment):
    write(deployment.root/'extra.py', '# unpinned\n')
    with pytest.raises(ValueError, match='inventory/SHA'):
        scope(deployment)


@pytest.mark.parametrize('name', ['profile.json', 'deployment-capture.json', 'checks/fresh_borrowed.py'])
def test_original_frozen_input_change_rejected(deployment, name):
    write(deployment.fresh/name, '{}\n')
    with pytest.raises(ValueError):
        scope(deployment)


@pytest.mark.parametrize('key,value', [
    ('output', 'same'), ('seed', 'same'), ('output', 'outside'), ('seed', 'wrong_environment')])
def test_completion_outputs_cannot_alias_or_escape(deployment, key, value):
    d = deployment; doc = dict(d.doc)
    doc[key] = {'same':doc['baseline'], 'outside':str(d.workspace/'elsewhere/sunny11_flexible'),
                'wrong_environment':str(d.workspace/'collections/sunny15_seed')}[value]
    write(d.root/'deployment.json', doc)
    with pytest.raises(ValueError, match='Independent same-environment'):
        scope(d)


@pytest.mark.parametrize('environment', ['sunny13', 'sunny15', 'foggy11', 'foggy13', 'foggy15'])
def test_explicit_environment_must_match_frozen_source(deployment, environment):
    with pytest.raises(ValueError, match='matching environment'):
        wrapper.scope_for(deployment.root, environment)


@pytest.mark.parametrize('arguments', [
    ['--environment', 'sunny11', 'run'],
    ['--environment', 'sunny11', 'run', '--confirmed-environment', 'sunny13'],
    ['--environment', 'sunny11', 'prepare', '--confirmed-environment', 'sunny11'],
    ['--environment', 'sunny11', 'status', '--confirmed-environment', 'sunny11'],
])
def test_native_requires_exact_environment_confirmation(arguments):
    with pytest.raises(ValueError, match='matching environment'):
        wrapper.main(arguments)


@pytest.mark.parametrize('option,value', [('--attempts', '0'), ('--attempts', '6'),
    ('--time-budget-sec', '29'), ('--time-budget-sec', '7201'), ('--stop-after-cells', '-1')])
def test_invalid_budgets_rejected_before_any_input_load(option, value):
    with pytest.raises(ValueError, match='Finite completion budgets'):
        wrapper.main(['--environment', 'sunny11', 'prepare', option, value])


def test_namespace_preserves_baseline_read_only_and_seed_control_read_only(deployment):
    s = scope(deployment)
    for name in ('state.json', 'status.json', 'quota.json', 'selected_ledger.jsonl',
                 'adopted_manifest.json', 'zero_capture_seed.json'):
        write(s['seed']/name, '{}\n')
    original = Mock(return_value=['bwrap', '--chdir', '/logical', '--', 'child'])
    fresh = SimpleNamespace(namespace=original)
    with wrapper.namespace_for_completion(fresh, s):
        argv = fresh.namespace({}, ['child'])
        for path in (s['root'], s['profile'], s['baseline']):
            index = argv.index(str(path)); assert argv[index-1] == '--ro-bind'
        index = argv.index(str(s['seed'])); assert argv[index-1] == '--bind'
        for name in ('state.json', 'status.json', 'quota.json', 'selected_ledger.jsonl',
                     'adopted_manifest.json', 'zero_capture_seed.json'):
            index = argv.index(str(s['seed']/name)); assert argv[index-1] == '--ro-bind'
            assert index < argv.index('--chdir')
    assert fresh.namespace is original


def test_namespace_restored_on_failure(deployment):
    original = Mock(return_value=['bwrap', '--chdir', '/logical', '--', 'child'])
    fresh = SimpleNamespace(namespace=original)
    with pytest.raises(RuntimeError, match='failure'):
        with wrapper.namespace_for_completion(fresh, scope(deployment)):
            raise RuntimeError('failure')
    assert fresh.namespace is original


def test_seed_missing_is_not_created_by_namespace(deployment):
    s = scope(deployment)
    fresh = SimpleNamespace(namespace=Mock(return_value=['bwrap', '--chdir', '/', '--']))
    with wrapper.namespace_for_completion(fresh, s):
        argv = fresh.namespace({}, [])
        assert str(s['seed']) not in argv
    assert not s['seed'].exists()


def test_outer_run_targets_new_output_for_pause_not_parent(deployment, monkeypatch):
    s = scope(deployment)
    ctx = dict(paths=dict(root=deployment.fresh, output=s['baseline']))
    fresh = SimpleNamespace(load=Mock(return_value=ctx), validate_profile=Mock(return_value=s['previous']),
        namespace=Mock(), supervise=Mock(return_value=0))
    monkeypatch.setattr(wrapper, 'load_components', lambda scope:(fresh, None, None, None))
    assert wrapper.main(['--environment', 'sunny11', 'run', '--confirmed-environment', 'sunny11']) == 0
    updated, action, command = fresh.supervise.call_args.args
    assert updated['paths']['output'] == s['output']
    assert action == 'run'
    assert '--inside' in command
    assert command[command.index('--confirmed-environment')+1] == 'sunny11'
    assert ctx['paths']['output'] == s['baseline']


def test_plan_never_supervises_capture(deployment, monkeypatch):
    s = scope(deployment); write(s['baseline']/'status.json', dict(accepted_candidate=4944, review=56))
    fresh = SimpleNamespace(load=Mock(return_value={}), validate_profile=Mock(return_value=s['previous']),
        supervise=Mock())
    monkeypatch.setattr(wrapper, 'load_components', lambda scope:(fresh, None, None, None))
    assert wrapper.main(['--environment', 'sunny11', 'plan']) == 0
    fresh.supervise.assert_not_called()
    assert not s['seed'].exists() and not s['output'].exists()


@pytest.mark.parametrize('raises', [False, True])
def test_prepare_protects_new_files_and_restores_parent_umask(deployment, monkeypatch, raises):
    s = scope(deployment)
    def supervise(*args):
        current = wrapper.os.umask(0o077)
        assert current == 0o077
        if raises:
            raise RuntimeError('supervisor failed')
        return 0
    fresh = SimpleNamespace(load=Mock(return_value=dict(paths={})),
        validate_profile=Mock(return_value=s['previous']), namespace=Mock(), supervise=supervise)
    monkeypatch.setattr(wrapper, 'load_components', lambda scope:(fresh, None, None, None))
    previous = wrapper.os.umask(0o022)
    try:
        if raises:
            with pytest.raises(RuntimeError, match='supervisor failed'):
                wrapper.main(['--environment', 'sunny11', 'prepare'])
        else:
            assert wrapper.main(['--environment', 'sunny11', 'prepare']) == 0
        current = wrapper.os.umask(0o022)
        assert current == 0o022
    finally:
        wrapper.os.umask(previous)
