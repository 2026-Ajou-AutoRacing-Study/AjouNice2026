"""CPU-only recovery facade/dispatch fixtures; no remote, ROS or capture."""
from contextlib import contextmanager, redirect_stdout
import importlib.util
import io
import json
import os
from pathlib import Path
import shutil
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import recovery_runtime as runtime
import recovery_population

SOURCE = Path(__file__).parent


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    path.write_text(json.dumps(value)+'\n');path.chmod(0o600)
    return path


@pytest.fixture
def fixture(tmp_path, monkeypatch):
    workspace = tmp_path/'home/ajounice_base5_capture';checks = workspace/'checks'
    checks.mkdir(parents=True, mode=0o700)
    for name in ('live_route_compat', 'route_dispatch', 'route_compat', 'control', 'status',
                 'finish_collection', 'finish', 'recovery_population', 'recovery_import', 'recovery_runtime'):
        p = checks/(name+'.py');shutil.copyfile(SOURCE/(name+'.py'), p);p.chmod(0o600)
    (checks/'recovery_qualification.py').write_text('# CPU fixture only\n')
    (checks/'recovery_qualification.py').chmod(0o600)
    shutil.copyfile(SOURCE/'live.py', checks/'live_v2.py');(checks/'live_v2.py').chmod(0o600)
    monkeypatch.setattr(runtime, '__file__', str(checks/'recovery_runtime.py'))
    with monkeypatch.context() as patch:
        patch.setattr(runtime.historical, '__file__', str(checks/'live_route_compat.py'))
        facade = runtime.Facade()
    old = facade.old
    mr = workspace.parent/'MoraiLauncher_Lin';mr.mkdir(mode=0o700)
    bundle = workspace/'capture_bundle';bundle.mkdir(mode=0o700)
    graph = dict(files={}, bindings=[], release=dict(source='/home/luv/AjouNice2026/release.json'),
        manifest=dict(source='/home/luv/AjouNice2026/manifest.json'))
    ctx = dict(workspace=workspace, bundle=bundle, mr=mr, graph=graph,
        session=workspace/'work/ros-session-v2', home=workspace/'work/ros-home-v2')
    old.prepare(ctx)
    smoke = workspace/'collections/smoke_v1'
    write(smoke/'status.json', dict(stage='complete', accepted_candidate=3, review=0, failed=0))
    write(smoke/'ledger.jsonl', dict(id='retained-smoke'))
    write(smoke/'shutdown_safety.json', dict(park_verified=True))
    proof = dict(host_contract=old.contract(ctx), native_frames=3, native_files={})
    for field, name in (('ledger_sha256', 'ledger.jsonl'), ('status_sha256', 'status.json'),
                        ('shutdown_sha256', 'shutdown_safety.json')):
        proof[field] = old.digest(smoke/name);proof['native_files'][name] = old.digest(smoke/name)
    native_qualification = write(ctx['session']/'qualification.json', proof)
    witness = write(checks/'canonical_route_v1.json', dict(cpu_fixture_only=True))
    parent_names = ('live_v2.py', 'live_route_compat.py', 'route_dispatch.py', 'route_compat.py',
                    'control.py', 'status.py', 'finish_collection.py', 'finish.py', 'canonical_route_v1.json')
    files = {str(checks/name):old.digest(checks/name) for name in parent_names}
    for path in (ctx['session']/'host_contract.json', native_qualification):files[str(path)] = old.digest(path)
    parent = dict(schema=runtime.historical.SCHEMA, workspace=str(workspace), scripts=str(runtime.route.LOGICAL_SCRIPTS),
        release=graph['release']['source'], manifest=graph['manifest']['source'], environment='sunny13',
        output=str(workspace/'collections/sunny13_5k'), witness=str(witness), witness_sha256=old.digest(witness),
        files=files, source_runtime_count=427, new_live_smoke_executed=False,
        host_contract_sha256=old.digest(ctx['session']/'host_contract.json'),
        qualification_sha256=old.digest(native_qualification))
    parent_intent = write(checks/'route_compat_intent.json', parent)
    old_root = workspace/'collections/sunny13_5k'
    write(old_root/'status.json', dict(stage='halted', accepted_candidate=69))
    output = workspace/'collections'/runtime.OUTPUT_NAME
    receipt = write(output/'imported_prefix.json', dict(schema=1,
        manifest_sha256=runtime.compat.CANONICAL_MANIFEST_SHA256, deficits=[],
        selected=[dict(source_cell_id='source_%s' % i) for i in range(69)]))
    qualification = write(workspace/'work/recovery-qualification/v1/qualification.json', dict(
        schema='base5_portable_recovery_cpu_qualification_v1',
        passed=True, captures_requested=0, native_capture_executed=False,
        problem_00082_candidate_pass=True, unexpected_contract_stops=0, bounded_exhaustions=0,
        source_inputs_unchanged=True, native_qa_unchanged=True, parent_intent_sha256=old.digest(parent_intent),
        matrix=dict(all_branches_selected=True), source_inventory=dict(passed=True, source_scenes=5000),
        replacement_count=2,
        population_policy=recovery_population.POLICY, manifest_sha256=runtime.compat.CANONICAL_MANIFEST_SHA256,
        helper_hashes={str(checks/name):old.digest(checks/name)
            for name in ('recovery_population.py', 'recovery_qualification.py')}))
    doc = dict(parent, schema=runtime.SCHEMA, previous_output=str(old_root), output=str(output),
        qa_changed=False, recovery_policy=recovery_population.POLICY,
        parent_intent_sha256=old.digest(parent_intent), files={str(checks/name):old.digest(checks/name)
            for name in runtime.HELPERS}, import_receipt=str(receipt), import_receipt_sha256=old.digest(receipt),
        cpu_qualification=str(qualification), cpu_qualification_sha256=old.digest(qualification))
    write(facade.intent_path(ctx), doc)
    monkeypatch.setattr(facade, 'load', lambda *args, **kwargs:ctx)
    monkeypatch.setattr(runtime, 'Facade', lambda:facade)
    return SimpleNamespace(facade=facade, ctx=ctx, doc=doc, output=output, old=old_root, checks=checks)


def invoke(f, args):
    out = io.StringIO()
    with redirect_stdout(out):result = runtime.main(['--workspace', str(f.ctx['workspace']), *args])
    return result, out.getvalue()


def test_profile_and_readonly_capture_plan(fixture, monkeypatch):
    f = fixture;spawn = Mock();ports = Mock()
    monkeypatch.setattr(runtime.subprocess, 'Popen', spawn)
    monkeypatch.setattr(f.facade.old, 'port_open', ports)
    before = {str(p):p.read_bytes() for p in f.ctx['workspace'].rglob('*') if p.is_file()}
    for action in ('capture', 'pause', 'resume-paused'):
        result, text = invoke(f, [action]);doc = json.loads(text)
        assert result == 0 and not doc['files_written'] and not doc['capture_started']
    assert before == {str(p):p.read_bytes() for p in f.ctx['workspace'].rglob('*') if p.is_file()}
    spawn.assert_not_called();ports.assert_not_called()


@pytest.mark.parametrize('field', ['output', 'previous_output', 'scripts', 'manifest', 'witness',
                                  'recovery_policy', 'source_runtime_count', 'qa_changed',
                                  'host_contract_sha256', 'qualification_sha256'])
def test_wrong_profile_scope_refused(fixture, field):
    f = fixture;doc = dict(f.doc);doc[field] = 'foreign'
    write(f.facade.intent_path(f.ctx), doc)
    with pytest.raises(ValueError):f.facade.intent(f.ctx)


@pytest.mark.parametrize('which', ['helper', 'qualification', 'import', 'parent'])
def test_changed_helper_receipt_or_parent_refused(fixture, which):
    f = fixture
    path = {'helper':f.checks/'recovery_population.py', 'qualification':Path(f.doc['cpu_qualification']),
            'import':Path(f.doc['import_receipt']), 'parent':f.checks/'route_compat_intent.json'}[which]
    path.write_text('{}\n')
    with pytest.raises(ValueError):f.facade.intent(f.ctx)


@pytest.mark.parametrize('field,value', [('problem_00082_candidate_pass', False),
    ('unexpected_contract_stops', 1), ('bounded_exhaustions', 1), ('replacement_count', 0),
    ('matrix', {'all_branches_selected':False}), ('source_inventory', {'passed':False})])
def test_boolean_pass_without_actual_branch_gate_is_refused(fixture, field, value):
    f = fixture;path = Path(f.doc['cpu_qualification']);report = json.loads(path.read_text())
    report[field] = value;write(path, report)
    doc = dict(f.doc, cpu_qualification_sha256=f.facade.old.digest(path))
    write(f.facade.intent_path(f.ctx), doc)
    with pytest.raises(ValueError):f.facade.intent(f.ctx)


def test_capture_command_uses_new_output_and_real_new_executor(fixture):
    f = fixture;command = f.facade.capture_command(f.ctx)
    assert command[:3] == ['/usr/bin/python3', '-B', f.facade.__file__]
    assert command[command.index('--intent')+1] == str(f.facade.intent_path(f.ctx))
    assert command[command.index('--output')+1] == str(f.output)
    assert '--complete' not in command
    assert f.facade.old.__file__.endswith('/checks/live_v2.py')


def test_old_collection_is_readonly_in_every_new_namespace(fixture):
    f = fixture;command = f.facade.old.namespace(f.ctx, ['python3', '-B', 'fixture.py'])
    assert ['--ro-bind', str(f.old), str(f.old)] in [command[i:i+3] for i in range(len(command)-2)]
    index = command.index('--chdir')
    assert command[index-3:index] == ['--ro-bind', str(f.old), str(f.old)]


def test_shared_capture_lock_and_signal_pause_target_new_output(fixture, monkeypatch):
    f = fixture;seen = {}
    original_pause = f.facade.old.request_pause
    def supervised(self, ctx, command):
        seen['marker'] = self.old.request_pause(ctx)
        seen['ctx'] = ctx
        return 0
    monkeypatch.setattr(runtime.historical.Facade, 'supervise_capture', supervised)
    assert f.facade.supervise_capture(f.ctx, ['fixture']) == 0
    assert seen['marker'] == f.output/'PAUSE' and seen['marker'].exists()
    assert not (f.old/'PAUSE').exists()
    assert f.facade.old.request_pause is original_pause
    assert seen['ctx']['session'] == f.ctx['session']


def test_control_and_finish_paths_are_rebound_without_old_file_rewrite(fixture):
    f = fixture
    control = f.facade.helper('control')
    assert control.output_root(f.ctx) == f.output
    finish = f.facade.helper('finish_collection')
    paths = finish.paths(f.ctx)
    assert paths['collection'] == f.output and paths['selected'] == f.output/'selected_ledger.jsonl'
    assert paths['output'] == f.ctx['workspace']/'finished'/runtime.OUTPUT_NAME
    assert paths['session'] == f.ctx['workspace']/'work/finish-session-recovery-v2'
    assert finish.__file__ == str(f.checks/'finish_collection.py')


def test_status_reads_new_69_not_old_halted_snapshot(fixture):
    f = fixture
    write(f.output/'status.json', dict(stage='prepared_waiting_for_resume', requested=5000,
        accepted_candidate=69, review=0, elapsed_sec=610.38, eta_sec=None,
        current=None, checkpoint_only=True, updated_at=1000))
    write(f.output/'quota.json', dict(target=5000, accepted=69, deficit=0, pending=4931))
    result, text = invoke(f, ['status'])
    assert result == 0 and '시작 대기' in text and '승인 69 / 5,000장' in text
    assert '안전 중단' not in text
    assert json.loads((f.old/'status.json').read_text())['stage'] == 'halted'


def test_guard_only_created_on_fresh_import_and_never_retrofit(fixture):
    f = fixture;intent = f.facade.intent_path(f.ctx)
    runtime.collection_guard(f.doc, intent, create=True)
    runtime.collection_guard(f.doc, intent)
    assert (f.output/'population_recovery_contract.json').exists()
    other = f.ctx['workspace']/'collections/unapproved';write(other/'status.json', {})
    with pytest.raises(ValueError):runtime.collection_guard(dict(f.doc, output=str(other)), intent, create=True)
    changed = dict(f.doc, import_receipt_sha256='0'*64)
    with pytest.raises(ValueError):runtime.collection_guard(changed, intent)


def test_router_propagates_new_executor_to_each_exact_leaf(fixture, monkeypatch):
    f = fixture;spawn = Mock();monkeypatch.setattr(runtime.subprocess, 'Popen', spawn)
    original = runtime.route.dispatcher_command
    with runtime.routed_children(f.facade.intent_path(f.ctx), f.doc):
        for name in ('run_base5_capture.py', 'run_base5_capture_optimized.py', 'run_base5_capture_campaign_spatial.py'):
            command = [runtime.sys.executable, str(runtime.route.LOGICAL_SCRIPTS/name), '--output', str(f.output/'child')]
            runtime.subprocess.Popen(command)
            actual = spawn.call_args[0][0]
            assert actual[2] == str(Path(runtime.__file__).absolute()) and '--role' in actual
    assert runtime.route.dispatcher_command is original


def test_actual_capture_requires_ui_and_both_owned_services(fixture, monkeypatch):
    f = fixture;supervise = Mock();monkeypatch.setattr(f.facade, 'supervise_capture', supervise)
    with pytest.raises(ValueError):invoke(f, ['capture', '--run'])
    monkeypatch.setattr(f.facade.old, 'port_open', lambda port:True)
    monkeypatch.setattr(f.facade, 'service_owned', lambda ctx, role:False)
    with pytest.raises(ValueError):invoke(f, ['capture', '--run', '--confirmed-environment', 'sunny13'])
    supervise.assert_not_called()


def test_finish_plan_never_postprocesses_or_captures(fixture, monkeypatch):
    f = fixture;spawn = Mock();monkeypatch.setattr(runtime.subprocess, 'Popen', spawn)
    result, text = invoke(f, ['finish']);doc = json.loads(text)
    assert result == 0 and doc['collection'] == str(f.output)
    assert not doc['files_written'] and not doc['capture_started']
    spawn.assert_not_called()
