from argparse import Namespace
import copy
import json
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import run_base5_batch_static_attitude as driver
from base5_capture_safety import atomic_json, sha256


def source(root, count=1):
    entries = []
    for i in range(count):
        scenario, metadata = root/'source'/f'{i}.json', root/'source'/f'{i}_metadata.json'
        atomic_json(scenario, dict(index=i))
        atomic_json(metadata, dict(split='train', runtime_actors=[],
            spec=dict(family='background', mission=False, role='front', distance_bounds=[0, 100])))
        entries.append(dict(id='scene_'+str(i), scenario=str(scenario), metadata=str(metadata),
            scenario_sha256=sha256(scenario), split='train', family='background', role='front',
            distance_bounds=[0, 100], subfamily=None, motion='static'))
    manifest = root/'source'/'manifest.json'
    atomic_json(manifest, dict(count=count, scenes=entries, config=dict(environment_tag='sunny15')))
    return manifest


def options(manifest, output, **kwargs):
    values = dict(manifest=manifest, output=output, environment='sunny15', confirmed_environment=None,
        qualified_release=None, limit=64, chunk_size=16, run=False, prepare_only=False,
        approve_bulk_capture=False, resume=False, stop_after_chunks=0)
    values.update(kwargs)
    return Namespace(**values)


@pytest.fixture
def pinned_runtime(monkeypatch):
    value = {str(Path(driver.__file__).resolve()):sha256(driver.__file__)}
    monkeypatch.setattr(driver, 'runtime_hashes', lambda:value)
    return value


def test_regular_chunk_command_is_independent_bounded_and_has_exact_optin_flags():
    command = driver.regular_command(Path('/manifest'), Path('/output'), 16)
    assert str(driver.CHILD) in command
    assert '--enable-static-surface-attitude' in command
    assert '--allow-static-pedestrians' in command
    assert command[command.index('--setup-profile')+1] == 'static-ready'
    assert '--shared-ros-batch' in command
    assert '--approve-bulk-capture' not in command
    for count in (0, 17, 5000, True):
        with pytest.raises(ValueError):driver.regular_command(Path('/m'), Path('/o'), count)


def test_recovery_is_explicit_stricter_spatial_not_silent_optin_fallback():
    command = driver.strict_recovery_command(Path('/manifest'), 3, Path('/output'), Path('/seed.json'))
    assert str(driver.RECOVERY) in command
    assert '--seed-capture' in command
    assert '--enable-final-direction-reference' in command
    assert '--enable-static-surface-attitude' not in command
    assert '--approve-bulk-capture' not in command
    assert not driver.profiles()['isolated_recovery']['enable_static_surface_attitude']
    assert driver.profiles()['isolated_recovery']['setup_profile'] == 'baseline'
    assert driver.profiles()['isolated_recovery']['actual_compound_corner_and_arrival_support_envelope']


def test_default_calls_real_batch_cpu_prepare_only_without_ros_or_child(tmp_path, monkeypatch, pinned_runtime):
    manifest = source(tmp_path)
    popen = Mock(side_effect=AssertionError('CPU prepare must not start a child'))
    monkeypatch.setattr(driver.subprocess, 'Popen', popen)
    output = tmp_path/'prepared'
    assert driver.run(options(manifest, output)) == 0
    assert driver.read(output/'status.json')['stage'] == 'prepared_waiting_for_resume'
    contract = driver.read(output/driver.GUARD)
    assert contract['prototype_only'] and contract['qualified_release'] is None
    assert contract['bulk_training_observations_at_qualification'] == 0
    assert contract['runtime_profile']['regular_chunk']['allow_static_pedestrians']
    assert contract['legacy_batch_contract'] == driver.read(output/'batch_contract.json')
    assert popen.call_count == 0


def test_parent_pins_source_map_sensor_network_and_resumes_same_prototype(tmp_path, monkeypatch, pinned_runtime):
    manifest = source(tmp_path);output = tmp_path/'prepared'
    assert driver.run(options(manifest, output)) == 0
    contract = driver.read(output/driver.GUARD)
    assert any(path.endswith('link_set.json') for path in contract['input_hashes'])
    assert any(path.endswith('lane_boundary_set.json') for path in contract['input_hashes'])
    assert any(path.endswith('Sensor_competition_local.json') for path in contract['input_hashes'])
    assert driver.run(options(manifest, output, resume=True)) == 0
    with pytest.raises(ValueError, match='contract changed'):
        driver.run(options(manifest, output, resume=True, chunk_size=8))


def test_old_output_cannot_gain_new_profile_or_upgrade_in_place(tmp_path, pinned_runtime):
    manifest = source(tmp_path);output = tmp_path/'legacy';output.mkdir()
    atomic_json(output/'batch_contract.json', dict(legacy=True))
    before = (output/'batch_contract.json').read_bytes()
    with pytest.raises(ValueError, match='old output'):
        driver.run(options(manifest, output, resume=True))
    assert (output/'batch_contract.json').read_bytes() == before
    assert not (output/driver.GUARD).exists()


@pytest.mark.parametrize('kwargs,pattern', [
    (dict(environment='sunny11'), 'environment mismatch'),
    (dict(run=True), 'explicitly confirm'),
    (dict(run=True, prepare_only=True), 'Choose'),
    (dict(chunk_size=17), 'Chunk size'),
])
def test_bad_operational_request_fails_before_output(tmp_path, pinned_runtime, kwargs, pattern):
    manifest = source(tmp_path);output = tmp_path/'new'
    with pytest.raises(ValueError, match=pattern):driver.run(options(manifest, output, **kwargs))
    assert not output.exists()


def test_disk_gate_prevents_any_live_prototype_before_output(tmp_path, monkeypatch, pinned_runtime):
    manifest = source(tmp_path);output = tmp_path/'new'
    monkeypatch.setattr(driver.shutil, 'disk_usage', lambda *a:SimpleNamespace(free=7*2**30))
    with pytest.raises(ValueError, match='8 GiB'):
        driver.run(options(manifest, output, run=True, confirmed_environment='sunny15'))
    assert not output.exists()


def test_bounded_prototype_uses_8gib_but_does_not_reduce_5k_storage_budget(tmp_path, monkeypatch, pinned_runtime):
    assert driver.storage_budget_gib(1) == driver.storage_budget_gib(64) == 8
    assert driver.storage_budget_gib(65) == driver.storage_budget_gib(5000) == 55
    for count in (0, 5001, True):
        with pytest.raises(ValueError):driver.storage_budget_gib(count)
    manifest = source(tmp_path);output = tmp_path/'bounded'
    monkeypatch.setattr(driver.shutil, 'disk_usage', lambda *a:SimpleNamespace(free=45*2**30))
    execute = Mock(return_value=0);monkeypatch.setattr(driver.batch, 'execute', execute)
    assert driver.run(options(manifest, output, run=True, confirmed_environment='sunny15')) == 0
    assert driver.read(output/driver.GUARD)['storage_gate']['minimum_gib'] == 8
    assert execute.call_count == 1


def test_bulk_request_cannot_start_without_deep_release_verifier(tmp_path, monkeypatch, pinned_runtime):
    manifest = source(tmp_path, 5000);output = tmp_path/'new'
    monkeypatch.setattr(driver.importlib, 'import_module', Mock(side_effect=ImportError('not ready')))
    release = tmp_path/'release.json';atomic_json(release, {})
    with pytest.raises(ValueError, match='bulk remains forbidden'):
        driver.run(options(manifest, output, limit=5000, run=True,
            confirmed_environment='sunny15', approve_bulk_capture=True, qualified_release=release))
    assert not output.exists()


def qualified_fixture(tmp_path, monkeypatch, runtime):
    manifest = source(tmp_path);output = (tmp_path/'future_capture').resolve()
    release = tmp_path/'release.json';atomic_json(release, dict(qualified=True))
    verifier = tmp_path/'finalizer.py';verifier.write_text('# immutable fixture verifier\n')
    gates = {}
    for name in driver.GATES:
        path = tmp_path/(name+'.json');atomic_json(path, dict(actual_fixture=True))
        gates[name] = dict(passed=True, path=str(path), sha256=sha256(path))
    context = dict(passed=True, bulk_ready=True, environment='sunny15', manifest_sha256=sha256(manifest),
        capture_output=str(output), runtime_profile=driver.profiles(), native_exact_observations=32,
        bulk_training_observations=0, runtime_hashes=runtime,
        input_hashes={str(manifest):sha256(manifest)}, gate_evidence=gates)
    module = SimpleNamespace(__file__=str(verifier), qualified_release_context=Mock(return_value=context))
    monkeypatch.setattr(driver.importlib, 'import_module', lambda *a:module)
    return manifest, output, release, context, module


def test_qualified_bulk_context_demands_all_actual_evidence_pins(tmp_path, monkeypatch, pinned_runtime):
    manifest, output, release, context, module = qualified_fixture(tmp_path, monkeypatch, pinned_runtime)
    result = driver.qualified_release(release, 'sunny15', manifest, output, pinned_runtime)
    assert result['bulk_ready'] and result['native_exact_observations'] == 32
    assert result['bulk_training_observations'] == 0
    module.qualified_release_context.assert_called_once()
    context['gate_evidence'].pop('independent_shadow')
    with pytest.raises(ValueError, match='Every native'):
        driver.qualified_release(release, 'sunny15', manifest, output, pinned_runtime)


@pytest.mark.parametrize('change,pattern', [
    ('profile', 'profiles differ'), ('runtime', 'complete new driver'),
    ('native', 'not 5000'), ('forged5000', 'not 5000'), ('gate_bytes', 'evidence changed'),
])
def test_qualified_context_cannot_mix_runtime_or_count_pilot_as_full_collection(tmp_path, monkeypatch, pinned_runtime, change, pattern):
    manifest, output, release, context, _ = qualified_fixture(tmp_path, monkeypatch, pinned_runtime)
    if change == 'profile':context['runtime_profile']['regular_chunk']['enable_static_surface_attitude'] = False
    if change == 'runtime':context['runtime_hashes'] = {}
    if change == 'native':context['native_exact_observations'] = 14
    if change == 'forged5000':context['bulk_training_observations'] = 5000
    if change == 'gate_bytes':atomic_json(Path(context['gate_evidence']['native_exact']['path']), dict(changed=True))
    with pytest.raises(ValueError, match=pattern):
        driver.qualified_release(release, 'sunny15', manifest, output, pinned_runtime)


def test_callbacks_use_independent_process_and_restore_legacy_functions(tmp_path, monkeypatch, pinned_runtime):
    manifest = source(tmp_path);output = tmp_path/'new';child_root = output/'chunk_0000'/'capture'
    original_child, original_recover = driver.batch.run_child, driver.batch.recover
    popen, wait = Mock(return_value=object()), Mock(return_value=0)
    monkeypatch.setattr(driver.subprocess, 'Popen', popen)
    monkeypatch.setattr(driver.batch, 'wait_child', wait)
    monkeypatch.setattr(driver.shutil, 'disk_usage', lambda *a:SimpleNamespace(free=100*2**30))
    def execute(*args, **kwargs):
        assert kwargs['prepare_only'] is False
        assert driver.batch.run_child(manifest, child_root, output/'STOP', lambda *a:None) == 0
        return 7
    monkeypatch.setattr(driver.batch, 'execute', execute)
    assert driver.run(options(manifest, output, run=True, confirmed_environment='sunny15')) == 7
    assert popen.call_args.kwargs['start_new_session'] is True
    assert '--approve-bulk-capture' not in popen.call_args.args[0]
    command = popen.call_args.args[0]
    execution = Path(command[command.index('--manifest')+1])
    assert execution == child_root.parent/driver.EXECUTION_SOURCE/'manifest.json'
    assert execution.read_bytes() == manifest.read_bytes()
    assert execution.parent not in child_root.parents
    assert driver.verify_execution_source(manifest, child_root) == execution
    assert driver.batch.run_child is original_child and driver.batch.recover is original_recover


def test_execution_source_preserves_exact_bytes_and_satisfies_unchanged_child_path_guard(tmp_path):
    manifest = source(tmp_path);root = tmp_path/'chunk'/'capture'
    original = manifest.read_bytes()
    execution = driver.prepare_execution_source(manifest, root)
    assert execution.read_bytes() == manifest.read_bytes() == original
    assert execution != manifest and execution.parent not in root.parents
    assert not root.exists()
    receipt = driver.read(root.parent/driver.EXECUTION_RECEIPT)
    assert receipt['original_manifest_sha256'] == receipt['execution_manifest_sha256'] == sha256(manifest)
    assert receipt['capture_output'] == str(root.resolve())
    before = (execution.stat().st_ino, execution.stat().st_mtime_ns)
    assert driver.prepare_execution_source(manifest, root) == execution
    assert (execution.stat().st_ino, execution.stat().st_mtime_ns) == before


def test_actual_static_child_cpu_prepare_accepts_sibling_manifest_without_ros(tmp_path):
    manifest = source(tmp_path);root = tmp_path/'chunk'/'capture'
    execution = driver.prepare_execution_source(manifest, root)
    result = driver.subprocess.run(driver.regular_command(execution, root, 1)+['--prepare-only'],
        capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stderr
    assert driver.read(root/'status.json')['stage'] == 'prepared_waiting_for_MR_reconnect'
    assert driver.read(root/'selected_plan.json')['manifest'] == str(execution)
    assert not (root/'session_contract.json').exists()
    assert not (root/'event_cmd_requests.jsonl').exists()
    assert driver.verify_execution_source(manifest, root) == execution


@pytest.mark.parametrize('change', ['execution_bytes','original_bytes','receipt','missing_receipt','missing_execution','extra_file'])
def test_execution_source_resume_never_repairs_changed_or_missing_evidence(tmp_path, change):
    manifest = source(tmp_path);root = tmp_path/'chunk'/'capture'
    execution = driver.prepare_execution_source(manifest, root)
    receipt = root.parent/driver.EXECUTION_RECEIPT
    if change == 'execution_bytes':execution.write_bytes(execution.read_bytes()+b' ')
    if change == 'original_bytes':manifest.write_bytes(manifest.read_bytes()+b' ')
    if change == 'receipt':atomic_json(receipt, dict(forged=True))
    if change == 'missing_receipt':receipt.unlink()
    if change == 'missing_execution':execution.unlink()
    if change == 'extra_file':atomic_json(execution.parent/'extra.json', {})
    with pytest.raises(ValueError):driver.prepare_execution_source(manifest, root)
    if change == 'missing_receipt':assert not receipt.exists()
    if change == 'missing_execution':assert not execution.exists()


def test_execution_source_symlink_cannot_redirect_new_copy(tmp_path):
    manifest = source(tmp_path);root = tmp_path/'chunk'/'capture'
    root.parent.mkdir();other = tmp_path/'other';other.mkdir()
    (root.parent/driver.EXECUTION_SOURCE).symlink_to(other, target_is_directory=True)
    with pytest.raises(ValueError, match='symlink'):
        driver.prepare_execution_source(manifest, root)
    assert not any(other.iterdir())


def test_strict_recovery_rejects_a_new_adjudicated_child_profile(tmp_path, monkeypatch):
    root = tmp_path/'recovery';contract = dict(profile='strict', source_manifest='/manifest', index=3)
    root.mkdir()
    atomic_json(driver.parent_recovery_guard(root), contract)
    direction = Mock(return_value=dict(passed=True, native_qa_unchanged=True))
    monkeypatch.setattr(driver.importlib, 'import_module', lambda *a:SimpleNamespace(verify_campaign=direction))
    session = root/'capture';session.mkdir()
    atomic_json(session/'session_contract.json', dict(runner_options=dict(static_surface_attitude_adapter={})))
    atomic_json(root/'report.json', dict(attempts=[dict(kind='same_cell_replacement', capture_root=str(session))]))
    with pytest.raises(ValueError, match='silently changed'):
        driver.verify_strict_recovery(root, contract)
    atomic_json(session/'session_contract.json', dict(runner_options=dict(shared_ros_batch=False)))
    driver.verify_strict_recovery(root, contract)
    direction.assert_called_once_with(root, Path('/manifest'), [3])
    atomic_json(root/'report.json', dict(attempts=[dict(kind='seeded_batch_review', capture_root=str(session))]))
    with pytest.raises(ValueError, match='actual original seed'):
        driver.verify_strict_recovery(root, contract)


def test_regular_resume_checks_actual_optin_session_and_sidecars(tmp_path, monkeypatch):
    root = tmp_path/'capture';manifest = source(tmp_path)
    driver.prepare_execution_source(manifest, root)
    expected = dict(enabled=True, actual='test-profile')
    monkeypatch.setattr(driver.attitude, 'configuration', lambda *a:expected)
    verifier = Mock();monkeypatch.setattr(driver.attitude, 'verify_capture', verifier)
    atomic_json(root/'static_surface_attitude_runtime_contract.json', expected)
    atomic_json(root/'session_contract.json', dict(runner_options=dict(static_surface_attitude_adapter=expected)))
    (root/'ledger.jsonl').write_text(json.dumps(dict(status='accepted_candidate', capture_root='/native'))+'\n')
    driver.verify_regular(root, manifest)
    verifier.assert_called_once_with('/native', expected)
    atomic_json(root/'session_contract.json', dict(runner_options={}))
    with pytest.raises(ValueError, match='omitted'):
        driver.verify_regular(root, manifest)


def test_terminal_replay_uses_new_driver_and_preserves_actual_snapshot_contract(tmp_path, monkeypatch):
    manifest = source(tmp_path);capture = tmp_path/'capture'
    atomic_json(capture/'status.json', dict(stage='complete', elapsed_sec=12.))
    atomic_json(capture/driver.GUARD, dict(manifest_sha256=sha256(manifest), indices=[0],
        environment='sunny15', chunk_size=16))
    snapshot = dict(raw_files={'actual':dict(sha256='test', inode=1, mtime_ns=2, size=3)},
        event_logs={'actual_service':'test'}, selected_sha256='test', cumulative_elapsed_sec=12.)
    monkeypatch.setattr(driver, 'completed_snapshot', lambda *a:copy.deepcopy(snapshot))
    run = Mock(return_value=0);monkeypatch.setattr(driver, 'run', run)
    popen = driver.subprocess.Popen
    receipt = driver.verify_completed_resume(manifest, capture, tmp_path/'receipt.json')
    saved = driver.read(receipt)
    assert saved['schema'] == 'base5_static_attitude_batch_terminal_resume_v1'
    assert saved['passed'] and saved['before'] == saved['after']
    assert saved['additional_native_captures'] == 0
    assert not saved['new_subprocesses_permitted']
    assert run.call_args.args[0].resume and run.call_args.args[0].run
    assert driver.subprocess.Popen is popen


def test_terminal_replay_cannot_start_any_child_or_accept_changed_raw(tmp_path, monkeypatch):
    manifest = source(tmp_path);capture = tmp_path/'capture'
    atomic_json(capture/'status.json', dict(stage='complete', elapsed_sec=12.))
    atomic_json(capture/driver.GUARD, dict(manifest_sha256=sha256(manifest), indices=[0],
        environment='sunny15', chunk_size=16))
    snapshot = dict(raw_files={'native':1}, event_logs={'service':1}, selected_sha256='a', cumulative_elapsed_sec=12.)
    monkeypatch.setattr(driver, 'completed_snapshot', lambda *a:copy.deepcopy(snapshot))
    monkeypatch.setattr(driver, 'run', lambda *a:driver.subprocess.Popen(['must-not-run']))
    before = driver.subprocess.Popen
    with pytest.raises(RuntimeError, match='forbids every new subprocess'):
        driver.verify_completed_resume(manifest, capture, tmp_path/'forbidden.json')
    assert driver.subprocess.Popen is before
    monkeypatch.setattr(driver, 'run', Mock(return_value=0))
    later = copy.deepcopy(snapshot);later['selected_sha256'] = 'changed'
    monkeypatch.setattr(driver, 'completed_snapshot', Mock(side_effect=[snapshot, later]))
    with pytest.raises(ValueError, match='changed raw'):
        driver.verify_completed_resume(manifest, capture, tmp_path/'changed.json')
    assert not (tmp_path/'changed.json').exists()
