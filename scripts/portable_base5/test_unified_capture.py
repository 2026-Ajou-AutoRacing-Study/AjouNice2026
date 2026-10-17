"""Offline unified-driver safety fixtures; never connects ROS or MORAI."""
import argparse
from contextlib import contextmanager, nullcontext
import fcntl
import json
import os
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

import unified_capture as capture


@pytest.fixture
def profile(tmp_path, monkeypatch):
    old, output = tmp_path/'old', tmp_path/'new'
    source, release = tmp_path/'manifest.json', tmp_path/'release.json'
    source.write_text(json.dumps({'scenes':[{'id':'scene%d'%i} for i in range(32)]}))
    release.write_text(json.dumps({'input_hashes':{},'runtime_hashes':{}}));old.mkdir()
    monkeypatch.setattr(capture, 'helpers', lambda:{})
    return dict(schema=capture.SCHEMA, policy=capture.POLICY, environment='sunny15',
        manifest=str(source), manifest_sha256=capture.sha(source), release=str(release),
        release_sha256=capture.sha(release), old_output=str(old), output=str(output),
        helper_pins={}, native_qa_unchanged=True, common_static_attitude=True,
        common_final_direction_reference=True, automatic_completion=False,
        runtime_hashes={'code':'fixed'}, source_authority={'source':'original'}, fixed_pins={})


def test_profile_preserves_both_native_supplements_and_no_auto_completion(profile):
    assert capture.validate_profile(profile) is profile


@pytest.mark.parametrize('field', ['native_qa_unchanged', 'common_static_attitude',
                                  'common_final_direction_reference', 'automatic_completion'])
def test_profile_cannot_disable_qa_or_enable_automatic_completion(profile, field):
    profile[field] = not profile[field]
    with pytest.raises(RuntimeError, match='QA/safety'):capture.validate_profile(profile)


@pytest.mark.parametrize('change', ['same', 'inside_old', 'contains_old', 'manifest_inside_new'])
def test_profile_never_reuses_or_nests_old_output(profile, change, tmp_path):
    old, new = Path(profile['old_output']), Path(profile['output'])
    if change == 'same':profile['output'] = str(old)
    elif change == 'inside_old':profile['output'] = str(old/'new')
    elif change == 'contains_old':profile['output'] = str(old.parent)
    else:profile['manifest'] = str(new/'manifest.json')
    with pytest.raises(RuntimeError, match='separate'):capture.validate_profile(profile)


def test_changed_helper_or_unknown_environment_blocks_profile(profile):
    profile['helper_pins'] = {'unrecorded':'change'}
    with pytest.raises(RuntimeError, match='profile/helper'):capture.validate_profile(profile)
    profile['helper_pins'] = {};profile['environment'] = 'cloudy17'
    with pytest.raises(RuntimeError, match='profile/helper'):capture.validate_profile(profile)


def test_qualified_handoff_keeps_source_authority_separate_and_restores(profile):
    original = Mock(side_effect=AssertionError('Do not requalify arbitrary source as current runtime'))
    parent = SimpleNamespace(qualified_release=original, profiles=lambda:{'strict':'same'})
    authority = {'original_native_pilot':32}
    with capture.qualified_handoff(parent, profile, authority):
        result = parent.qualified_release(Path(profile['release']), profile['environment'],
            Path(profile['manifest']), Path(profile['output']), profile['runtime_hashes'])
        assert result['original_native_pilot'] == 32
        assert result['original_source_qualification_only'] is True
        assert result['live_profile_qualification_separate'] is True
        assert result['runtime_profile'] == {'strict':'same'}
    assert parent.qualified_release is original
    original.assert_not_called()


@pytest.mark.parametrize('field', ['release', 'environment', 'manifest', 'output', 'runtime'])
def test_handoff_rejects_every_unbound_argument(profile, field, tmp_path):
    parent = SimpleNamespace(qualified_release=Mock(), profiles=lambda:{})
    values = dict(release=Path(profile['release']), environment=profile['environment'],
        manifest=Path(profile['manifest']), output=Path(profile['output']), runtime=profile['runtime_hashes'])
    values[field] = {} if field == 'runtime' else ('sunny13' if field == 'environment' else tmp_path/'wrong')
    with capture.qualified_handoff(parent, profile, {}):
        with pytest.raises(RuntimeError, match='destination/source/runtime'):parent.qualified_release(**values)


def test_handoff_adapter_ownership_violation_is_not_silent(profile):
    original = Mock();parent = SimpleNamespace(qualified_release=original)
    with pytest.raises(RuntimeError, match='ownership'):
        with capture.qualified_handoff(parent, profile, {}):parent.qualified_release = Mock()
    assert parent.qualified_release is original


def test_runtime_arguments_keep_existing_profiles_and_bulk_prepare_is_cpu_only(profile):
    prepared = capture.arguments(profile)
    assert prepared.limit == 5000 and prepared.chunk_size == 16 and prepared.resume is True
    assert prepared.run is False and prepared.prepare_only is True and prepared.approve_bulk_capture is False
    live = capture.arguments(profile, run=True, stop_after_chunks=1)
    assert live.confirmed_environment == profile['environment']
    assert live.approve_bulk_capture is True and live.stop_after_chunks == 1


def test_sealed_payload_is_immutable_anonymous_memfd():
    fd = capture.sealed_payload({'boundary':'fixed'})
    try:
        assert fcntl.fcntl(fd, fcntl.F_GET_SEALS) & capture.SEALS == capture.SEALS
        assert json.loads(os.pread(fd, os.fstat(fd).st_size, 0)) == {'boundary':'fixed'}
        with pytest.raises(PermissionError):os.write(fd, b'changed')
        with pytest.raises(PermissionError):os.ftruncate(fd, 0)
        assert '/memfd:base5-verified-input-boundary' in os.readlink('/proc/self/fd/'+str(fd))
    finally:os.close(fd)


def test_worker_payload_rejects_wrong_parent_identity():
    fd = capture.sealed_payload({'untrusted':'payload'})
    try:
        wrong = capture.identity(os.getppid());wrong['start_ticks'] += 'bad'
        with pytest.raises(RuntimeError, match='live invoking parent'):capture.worker_payload(fd, wrong)
    finally:os.close(fd)


def parent_command(monkeypatch, values):
    original = Path.read_bytes
    command_path = Path('/proc')/str(os.getppid())/'cmdline'
    monkeypatch.setattr(Path, 'read_bytes', lambda self:
        b'\0'.join(os.fsencode(value) for value in values)+b'\0' if self == command_path else original(self))


def test_worker_payload_rejects_unsealed_fd(monkeypatch):
    parent_command(monkeypatch, [capture.sys.executable, '-B', str(Path(capture.__file__).absolute()), '--run'])
    fd = os.memfd_create('base5-verified-input-boundary', os.MFD_ALLOW_SEALING)
    try:
        os.write(fd, b'{"unsealed":true}')
        with pytest.raises(RuntimeError, match='sealed anonymous'):
            capture.worker_payload(fd, capture.identity(os.getppid()))
    finally:os.close(fd)


def test_worker_payload_rejects_sealed_regular_file(tmp_path, monkeypatch):
    parent_command(monkeypatch, [capture.sys.executable, '-B', str(Path(capture.__file__).absolute()), '--run'])
    path = tmp_path/'not-a-memfd';path.write_text('{}')
    with path.open('rb') as stream:
        with pytest.raises((RuntimeError, OSError)):
            capture.worker_payload(stream.fileno(), capture.identity(os.getppid()))


def worker_values(profile):
    return ['--enable-final-direction-reference', '--manifest', profile['manifest'], '--indices', '17',
            '--output', str(Path(profile['output'])/'chunk_0001/recovery/scene17'), '--resume']


def test_bounded_direction_worker_stays_within_campaign(profile):
    values = worker_values(profile)
    target = capture.SCRIPTS/'run_base5_capture_campaign_direction_reference.py'
    assert capture.validate_worker_arguments(target, values, profile) == target


@pytest.mark.parametrize('change', ['unknown_target', 'same_root', 'outside_output', 'source_escape',
                                  'duplicate_manifest', 'duplicate_output', 'completion'])
def test_worker_scope_rejects_foreign_sources_or_outputs(profile, change, tmp_path):
    values = worker_values(profile)
    target = capture.SCRIPTS/'run_base5_capture_campaign_direction_reference.py'
    if change == 'unknown_target':target = capture.SCRIPTS/'run_base5_flexible_completion.py'
    elif change == 'same_root':values[values.index('--output')+1] = profile['output']
    elif change == 'outside_output':values[values.index('--output')+1] = str(tmp_path/'outside')
    elif change == 'source_escape':values[values.index('--manifest')+1] = str(tmp_path/'other_source.json')
    elif change == 'duplicate_manifest':values += ['--manifest',profile['manifest']]
    elif change == 'duplicate_output':values += ['--output',values[values.index('--output')+1]]
    else:values += ['--complete']
    with pytest.raises(RuntimeError):capture.validate_worker_arguments(target, values, profile)


def test_router_passes_only_sealed_boundary_to_child_and_restores(profile, monkeypatch):
    target = capture.SCRIPTS/'run_base5_capture_campaign_direction_reference.py'
    command = [capture.sys.executable, str(target), *worker_values(profile)]
    calls = [];sentinel = object()
    def popen(argv, *args, **kwargs):
        calls.append((argv, kwargs))
        fd = int(argv[argv.index('--session-fd')+1])
        assert fd in kwargs['pass_fds']
        assert fcntl.fcntl(fd, fcntl.F_GET_SEALS) & capture.SEALS == capture.SEALS
        payload = json.loads(os.pread(fd, os.fstat(fd).st_size, 0))
        assert payload['profile'] == profile and payload['boundary'] == {'source':'verified'}
        return sentinel
    monkeypatch.setattr(capture.subprocess, 'Popen', popen)
    boundary = SimpleNamespace(snapshot=lambda:{'source':'verified'})
    with capture.routed_workers(profile, boundary):
        assert capture.subprocess.Popen(command, pass_fds=()) is sentinel
    assert capture.subprocess.Popen is popen
    argv, kwargs = calls[0]
    assert argv[2] == str(Path(capture.__file__).absolute()) and '--worker' in argv
    with pytest.raises(OSError):os.fstat(int(argv[argv.index('--session-fd')+1]))


@pytest.mark.parametrize('change', ['shell', 'foreign_python', 'target_in_other_position'])
def test_router_refuses_unknown_native_invocations(profile, monkeypatch, change):
    target = capture.SCRIPTS/'run_base5_capture_campaign_direction_reference.py'
    command = [capture.sys.executable, str(target), *worker_values(profile)]
    called = Mock();monkeypatch.setattr(capture.subprocess, 'Popen', called)
    if change == 'shell':command = 'python3 '+str(target)
    elif change == 'foreign_python':command[0] = '/unapproved/python'
    else:command = [capture.sys.executable, '-c', 'not-a-script', str(target)]
    with capture.routed_workers(profile, SimpleNamespace(snapshot=lambda:{})):
        with pytest.raises(RuntimeError):capture.subprocess.Popen(command)
    called.assert_not_called()


@pytest.mark.parametrize('stage', ['halted', 'running', 'complete', 'halted_exit_safety'])
def test_resume_never_relabels_halt_or_active_state(tmp_path, stage):
    root = tmp_path/'output';root.mkdir();state = root/'status.json'
    state.write_text(json.dumps({'stage':stage}));before = state.read_bytes()
    with pytest.raises(RuntimeError, match='prepared or safely paused'):capture.release_pause(root)
    assert state.read_bytes() == before


def test_resume_stop_is_never_removed(tmp_path):
    root = tmp_path/'output';root.mkdir()
    (root/'status.json').write_text(json.dumps({'stage':'paused'}));(root/'STOP').touch();(root/'PAUSE').touch()
    with pytest.raises(RuntimeError, match='STOP'):capture.release_pause(root)
    assert (root/'STOP').exists() and (root/'PAUSE').exists()


def test_missing_live_qualification_blocks_before_any_native_start(profile, monkeypatch):
    root = Path(profile['output']);root.mkdir();(root/'imported_prefix.json').write_text('{}')
    profile['import_sha256'] = capture.sha(root/'imported_prefix.json')
    mocked = Mock(side_effect=AssertionError('Native modules must not load'))
    monkeypatch.setattr(capture, 'modules', mocked)
    with pytest.raises(RuntimeError, match='live qualification'):capture.run(profile)
    mocked.assert_not_called()


def test_plan_mode_never_starts_or_prepares(profile, monkeypatch, tmp_path, capsys):
    filename = tmp_path/'profile.json';capture.write_new(filename,profile)
    live = Mock(side_effect=AssertionError('No main capture in plan'))
    prepare = Mock(side_effect=AssertionError('No writes in plan'))
    monkeypatch.setattr(capture, 'run', live);monkeypatch.setattr(capture, 'prepare', prepare)
    assert capture.main(['--profile',str(filename)]) == 0
    result = json.loads(capsys.readouterr().out)
    assert result['captures_started'] is False and result['files_written'] is False
    live.assert_not_called();prepare.assert_not_called()


def test_run_requires_matching_ui_confirmation(profile, monkeypatch, tmp_path):
    filename = tmp_path/'profile.json';filename.write_text(json.dumps(profile))
    live = Mock();monkeypatch.setattr(capture, 'run', live)
    with pytest.raises(RuntimeError, match='MORAI UI'):
        capture.main(['--profile',str(filename),'--run','--confirmed-environment','sunny13'])
    live.assert_not_called()


def test_source_authority_uses_original_destination_for_six_gate_release(profile, monkeypatch, tmp_path):
    profile.pop('source_authority')
    release = dict(schema='same-schema-as-support', qualified_runtime={}, capture_output=str(tmp_path/'original'),
                   runtime_hashes={'original':'427or438'})
    Path(profile['release']).write_text(json.dumps(release));profile['release_sha256'] = capture.sha(profile['release'])
    guard = Path(profile['old_output'])/'old_guard.json';guard.write_text(json.dumps({'runtime_hashes':{'old':'runtime'}}))
    validate = Mock(return_value={'six_gates':'passed'})
    parent = SimpleNamespace(GUARD=guard.name, qualified_release=validate)
    result = capture.source_authority(profile, parent)
    validate.assert_called_once_with(Path(profile['release']), profile['environment'], Path(profile['manifest']),
        Path(release['capture_output']), {'old':'runtime'})
    assert result['source_qualification'] == 'original-six-gate-environment-release'


@pytest.mark.parametrize('command', [
    ['python3','-c','standalone-parent'],
    ['python3','/unapproved/unified_capture.py','--run'],
    ['python3',str(Path(capture.__file__).absolute()),'--prepare'],
])
def test_sealed_snapshot_cannot_be_used_by_standalone_or_wrong_parent(monkeypatch, command):
    parent_command(monkeypatch, command)
    fd = capture.sealed_payload({'forged':'standalone'})
    try:
        with pytest.raises(RuntimeError,match='parent'):
            capture.worker_payload(fd,capture.identity(os.getppid()))
    finally:os.close(fd)


def test_authenticated_parent_can_read_only_immutable_snapshot(monkeypatch):
    parent_command(monkeypatch,[capture.sys.executable,'-B',str(Path(capture.__file__).absolute()),'--run'])
    expected = {'profile':'bound','boundary':'verified'};fd = capture.sealed_payload(expected)
    try:assert capture.worker_payload(fd,capture.identity(os.getppid())) == expected
    finally:os.close(fd)


@pytest.mark.parametrize('extra', [
    ['--approve-bulk-capture'], ['--batch-reuse-network'], ['--network-template','/tmp/wrong'],
    ['--sensor-dir','/tmp/wrong'], ['--scenario-dir','/tmp/wrong'],
    ['--test-interrupt-after-raw-move','scene17'], ['--disable-qa'],
    ['--quota-rounds','9'], ['--limit','5000'], ['--indices','18'],
])
def test_worker_cannot_override_qa_network_profile_or_recovery_bounds(profile,extra):
    target = capture.SCRIPTS/'run_base5_capture_campaign_direction_reference.py'
    with pytest.raises(RuntimeError):capture.validate_worker_arguments(target,worker_values(profile)+extra,profile)


@pytest.mark.parametrize('index',['017','17,18','-1','5000'])
def test_recovery_requires_one_canonical_existing_source_index(profile,index):
    values = worker_values(profile);values[values.index('--indices')+1] = index
    with pytest.raises(RuntimeError):
        capture.validate_worker_arguments(capture.SCRIPTS/'run_base5_capture_campaign_direction_reference.py',values,profile)


def prepared_run(profile,tmp_path,monkeypatch):
    root = Path(profile['output']);root.mkdir()
    imported = root/'imported_prefix.json';imported.write_text('{}')
    profile['import_sha256'] = capture.sha(imported)
    pilot = tmp_path/'pilot.json';pilot.write_text('{"actual":"pilot"}')
    proof = tmp_path/'actual_native_proof.json';proof.write_text('{"actual":"native"}')
    qualification = dict(passed=True,policy=capture.POLICY,manifest_sha256=profile['manifest_sha256'],
        helper_pins=profile['helper_pins'],pilot_manifest=str(pilot),pilot_manifest_sha256=capture.sha(pilot),
        actual_pause_resume=True,completed_replay_no_new_capture=True,proof_hashes={str(proof):capture.sha(proof)})
    (root/'unified_live_qualification.json').write_text(json.dumps(qualification))
    (root/'status.json').write_text(json.dumps({'stage':'prepared_waiting_for_resume','accepted_candidate':69,'review':2}))
    def atomic(path,value):Path(path).write_text(json.dumps(value))
    run = Mock(return_value=0)
    parent = SimpleNamespace(run=run,batch=SimpleNamespace(atomic_json=atomic))
    boundary = SimpleNamespace(installed=lambda **kw:nullcontext(),verify=Mock(return_value=True),stats={'fixed_hits':7})
    monkeypatch.setattr(capture,'modules',lambda:(parent,object()))
    monkeypatch.setattr(capture,'boundary_for',lambda doc:boundary)
    monkeypatch.setattr(capture,'source_authority',lambda doc,parent:doc['source_authority'])
    monkeypatch.setattr(capture,'warm_boundary',lambda *a:None)
    monkeypatch.setattr(capture,'route_compatibility',lambda doc:nullcontext())
    monkeypatch.setattr(capture,'routed_workers',lambda *a:nullcontext())
    monkeypatch.setattr(capture,'qualified_handoff',lambda *a:nullcontext())
    monkeypatch.setattr(capture,'release_pause',lambda *a:None)
    return root,proof,parent,boundary


def test_changed_native_qualification_proof_blocks_before_start(profile,tmp_path,monkeypatch):
    _,proof,parent,_ = prepared_run(profile,tmp_path,monkeypatch)
    proof.write_text('changed actual proof')
    with pytest.raises(RuntimeError,match='proof changed'):capture.run(profile)
    parent.run.assert_not_called()


def test_end_boundary_failure_halts_and_never_certifies_selected_rows(profile,tmp_path,monkeypatch):
    root,_,parent,boundary = prepared_run(profile,tmp_path,monkeypatch)
    boundary.verify.side_effect = RuntimeError('fixed scenario changed')
    with pytest.raises(RuntimeError,match='fixed scenario changed'):capture.run(profile)
    state = capture.read(root/'status.json')
    assert state['stage'] == 'halted' and state['failed'] == 1
    assert state['accepted_candidate'] == 69 and state['review'] == 2
    failed = list(root.glob('integrity_failed_*.json'))
    assert len(failed) == 1 and capture.read(failed[0])['selected_not_certified'] is True
    assert list(root.glob('integrity_boundary_*.json')) == []
    parent.run.assert_called_once()


def test_end_boundary_still_runs_after_native_exception(profile,tmp_path,monkeypatch):
    root,_,parent,boundary = prepared_run(profile,tmp_path,monkeypatch)
    parent.run.side_effect = RuntimeError('native transport failed')
    with pytest.raises(RuntimeError,match='native transport failed'):capture.run(profile)
    boundary.verify.assert_called_once()
    receipts = list(root.glob('integrity_boundary_*.json'))
    assert len(receipts) == 1 and capture.read(receipts[0])['result'] == 1
    assert capture.read(receipts[0])['hot_path_native_qa_cached'] is False


def test_successful_end_receipt_is_not_a_native_quality_waiver(profile,tmp_path,monkeypatch):
    root,_,parent,boundary = prepared_run(profile,tmp_path,monkeypatch)
    assert capture.run(profile) == 0
    boundary.verify.assert_called_once()
    receipt = capture.read(next(root.glob('integrity_boundary_*.json')))
    assert receipt['passed'] is True and receipt['hot_path_native_qa_cached'] is False
    assert receipt['boundary_stats'] == boundary.stats


def worker_setup(profile,monkeypatch,target_name='run_base5_capture_campaign_direction_reference.py'):
    target = capture.SCRIPTS/target_name
    values = worker_values(profile) if target.name == 'run_base5_capture_campaign_direction_reference.py' else [
        '--manifest',profile['manifest'],'--output',str(Path(profile['output'])/'preparation/native'), '--limit','1']
    payload = dict(profile=profile,boundary={'trusted':'fixed snapshot'},parent=capture.identity(os.getppid()))
    monkeypatch.setattr(capture,'worker_payload',lambda *a:payload)
    active = [];created = []
    @contextmanager
    def installed(**kwargs):
        active.append(kwargs)
        try:yield
        finally:active.pop()
    boundary = SimpleNamespace(installed=installed)
    def from_snapshot(value,**kwargs):
        created.append((value,kwargs));return boundary
    monkeypatch.setattr(capture.boundary_module.SessionInputBoundary,'from_verified_snapshot',from_snapshot)
    monkeypatch.setattr(capture,'route_compatibility',lambda doc:nullcontext())
    monkeypatch.setattr(capture.recovery_population,'installed',lambda:nullcontext())
    monkeypatch.setattr(capture,'routed_workers',lambda *a:nullcontext())
    direction = SimpleNamespace(main=Mock())
    monkeypatch.setattr(capture,'modules',lambda:(object(),direction))
    args = argparse.Namespace(worker=target,session_fd=42,parent_pid=os.getppid(),
        parent_start_ticks=capture.identity(os.getppid())['start_ticks'],arguments=['--',*values])
    return target,values,direction,args,active,created


def test_direction_worker_uses_canonical_main_inside_delta_scope_not_fresh_runpy(profile,monkeypatch):
    target,values,direction,args,active,created = worker_setup(profile,monkeypatch)
    old_argv = capture.sys.argv
    def main():
        assert capture.sys.argv == [str(target),*values]
        assert len(active) == 1 and active[0]['direction_module'] is direction
        assert active[0]['delta_input_manifest'] == profile['manifest']
        raise SystemExit(0)
    direction.main.side_effect = main
    execute = Mock(side_effect=AssertionError('run_path defines a second unpatched input_pins'))
    monkeypatch.setattr(capture.runpy,'run_path',execute)
    assert capture.run_worker(args) == 0
    direction.main.assert_called_once();execute.assert_not_called()
    assert created == [({'trusted':'fixed snapshot'},{'authority_verified':True})]
    assert capture.sys.argv is old_argv and not active


@pytest.mark.parametrize('code',[3,7])
def test_canonical_direction_worker_propagates_failure_and_restores_argv(profile,monkeypatch,code):
    _,_,direction,args,active,_ = worker_setup(profile,monkeypatch)
    previous = capture.sys.argv;direction.main.side_effect = SystemExit(code)
    assert capture.run_worker(args) == code
    assert capture.sys.argv is previous and not active


def test_other_native_workers_keep_original_runpy_path_inside_boundary_scope(profile,monkeypatch):
    target,_,direction,args,active,_ = worker_setup(profile,monkeypatch,'run_base5_capture.py')
    execute = Mock(side_effect=SystemExit(0));monkeypatch.setattr(capture.runpy,'run_path',execute)
    assert capture.run_worker(args) == 0
    execute.assert_called_once_with(str(target),run_name='__main__')
    direction.main.assert_not_called()
    assert not active


def test_prototype_worker_uses_canonical_source_boundary_and_pilot_chunk_size(profile,monkeypatch):
    profile['canonical_manifest'] = profile['manifest']
    pilot = Path(profile['manifest']).with_name('pilot_manifest.json')
    pilot.write_bytes(Path(profile['manifest']).read_bytes())
    profile['manifest'] = str(pilot);profile['manifest_sha256'] = capture.sha(pilot)
    profile['chunk_size'] = 1
    target,values,direction,args,active,_ = worker_setup(profile,monkeypatch)
    values[values.index('--output')+1] = str(Path(profile['output'])/'chunk_0017/recovery/scene17')
    args.arguments = ['--',*values]
    def main():
        assert active[0]['delta_input_manifest'] == profile['canonical_manifest']
        assert capture.sys.argv[capture.sys.argv.index('--manifest')+1] == str(pilot)
    direction.main.side_effect = main
    assert capture.run_worker(args) == 0
    direction.main.assert_called_once()


@pytest.mark.parametrize('parent_mode',['--run','--worker','--qualify-prototype'])
def test_only_live_unified_parent_modes_can_supply_sealed_snapshot(monkeypatch,parent_mode):
    parent_command(monkeypatch,[capture.sys.executable,'-B','-u',str(Path(capture.__file__).absolute()),parent_mode])
    fd = capture.sealed_payload({'mode':parent_mode})
    try:
        os.lseek(fd,0,os.SEEK_END)
        assert capture.worker_payload(fd,capture.identity(os.getppid())) == {'mode':parent_mode}
    finally:os.close(fd)


@pytest.mark.parametrize('size',[0,100_000_001])
def test_worker_snapshot_size_bounds_reject_empty_or_excessive_data(monkeypatch,size):
    parent_command(monkeypatch,[capture.sys.executable,str(Path(capture.__file__).absolute()),'--run'])
    fd = os.memfd_create('base5-verified-input-boundary',os.MFD_ALLOW_SEALING)
    try:
        os.ftruncate(fd,size);fcntl.fcntl(fd,fcntl.F_ADD_SEALS,capture.SEALS)
        with pytest.raises(RuntimeError,match='Bounded snapshot'):
            capture.worker_payload(fd,capture.identity(os.getppid()))
    finally:os.close(fd)


def test_partial_memfd_seals_are_insufficient(monkeypatch):
    parent_command(monkeypatch,[capture.sys.executable,str(Path(capture.__file__).absolute()),'--run'])
    fd = os.memfd_create('base5-verified-input-boundary',os.MFD_ALLOW_SEALING)
    try:
        os.write(fd,b'{}')
        fcntl.fcntl(fd,fcntl.F_ADD_SEALS,fcntl.F_SEAL_GROW|fcntl.F_SEAL_SHRINK)
        with pytest.raises(RuntimeError,match='sealed anonymous'):
            capture.worker_payload(fd,capture.identity(os.getppid()))
    finally:os.close(fd)


def refresh_fixture(profile,tmp_path,monkeypatch):
    root = Path(profile['output']);root.mkdir()
    (root/'status.json').write_text(json.dumps(dict(stage='prepared_waiting_for_resume',current=None,
        failed=0,finished=560,accepted_candidate=557,review=3)))
    imported = root/'imported_prefix.json';imported.write_text('{"selected":"preserved"}')
    profile['import_sha256'] = capture.sha(imported)
    helper = tmp_path/'helper.py';helper.write_text('old helper\n')
    old_helpers = {str(helper):capture.sha(helper)}
    profile['helper_pins'] = old_helpers;profile['fixed_pins'] = dict(old_helpers)
    filename = tmp_path/'profile.json';capture.write_new(filename,profile)
    before = filename.read_bytes();before_sha = capture.sha(filename)
    helper.write_text('updated helper before first capture\n')
    current_helpers = {str(helper):capture.sha(helper)}
    monkeypatch.setattr(capture,'helpers',lambda:current_helpers)
    def atomic(path,value):Path(path).write_text(json.dumps(value))
    parent = SimpleNamespace(run=Mock(return_value=0),batch=SimpleNamespace(atomic_json=atomic))
    boundary = SimpleNamespace(installed=lambda **kw:nullcontext(),verify=Mock(return_value=True))
    seen = []
    monkeypatch.setattr(capture,'modules',lambda:(parent,object()))
    monkeypatch.setattr(capture,'boundary_for',lambda doc:seen.append(doc) or boundary)
    monkeypatch.setattr(capture,'source_authority',lambda doc,parent:doc['source_authority'])
    monkeypatch.setattr(capture,'qualified_handoff',lambda *a:nullcontext())
    monkeypatch.setattr(capture,'route_compatibility',lambda doc:nullcontext())
    return root,filename,before,before_sha,current_helpers,parent,boundary,seen


def test_native_zero_refresh_archives_profile_preserves_imported_approved_and_deficits(profile,tmp_path,monkeypatch):
    root,path,before,old_sha,helpers,parent,boundary,seen = refresh_fixture(profile,tmp_path,monkeypatch)
    state_before = (root/'status.json').read_bytes();import_before = (root/'imported_prefix.json').read_bytes()
    result = capture.refresh_prepared_profile(path)
    revised = capture.read(path)
    assert revised['helper_pins'] == helpers and revised['fixed_pins'] == helpers
    expected = dict(profile,helper_pins=helpers,fixed_pins=helpers)
    assert revised == expected
    assert capture.read(result['previous_profile']) == json.loads(before)
    assert capture.sha(result['previous_profile']) == old_sha
    assert result['main_native_captures'] == 0 and result['capture_started'] is False
    assert (root/'status.json').read_bytes() == state_before
    assert (root/'imported_prefix.json').read_bytes() == import_before
    receipt = capture.read(next(root.glob('prepared_profile_revision_*.json')))
    assert receipt['original_source_and_import_unchanged'] is True
    boundary.verify.assert_called_once();parent.run.assert_called_once()
    args = parent.run.call_args.args[0]
    assert args.prepare_only is True and args.run is False and args.approve_bulk_capture is False
    assert seen[0]['fixed_pins'] == helpers


@pytest.mark.parametrize('change',['native_directory','session_contract','qualification','stop','pause','halt','current','failed'])
def test_refresh_never_retrofits_native_or_qualified_or_halted_output(profile,tmp_path,monkeypatch,change):
    root,path,before,_,_,parent,boundary,_ = refresh_fixture(profile,tmp_path,monkeypatch)
    if change == 'native_directory':(root/'chunk_0000').mkdir()
    elif change == 'session_contract':(root/'session_contract.json').write_text('{}')
    elif change == 'qualification':(root/'unified_live_qualification.json').write_text('{}')
    elif change == 'stop':(root/'STOP').touch()
    elif change == 'pause':(root/'PAUSE').touch()
    else:
        value = capture.read(root/'status.json')
        if change == 'halt':value['stage'] = 'halted'
        elif change == 'current':value['current'] = 'scene-active'
        else:value['failed'] = 1
        (root/'status.json').write_text(json.dumps(value))
    with pytest.raises(RuntimeError,match='forbidden'):capture.refresh_prepared_profile(path)
    assert path.read_bytes() == before
    parent.run.assert_not_called();boundary.verify.assert_not_called()


def test_refresh_rejects_changed_import_before_parent_prepare(profile,tmp_path,monkeypatch):
    root,path,before,_,_,parent,_,_ = refresh_fixture(profile,tmp_path,monkeypatch)
    (root/'imported_prefix.json').write_text('{"tampered":true}')
    with pytest.raises(RuntimeError,match='import changed'):capture.refresh_prepared_profile(path)
    assert path.read_bytes() == before;parent.run.assert_not_called()


def test_refresh_cannot_expand_helper_scope(profile,tmp_path,monkeypatch):
    _,path,before,_,helpers,parent,_,_ = refresh_fixture(profile,tmp_path,monkeypatch)
    helpers[str(tmp_path/'new_unqualified_helper.py')] = 'a'*64
    with pytest.raises(RuntimeError,match='Helper scope changed'):capture.refresh_prepared_profile(path)
    assert path.read_bytes() == before;parent.run.assert_not_called()


def test_refresh_parent_failure_keeps_profile_and_runs_end_boundary(profile,tmp_path,monkeypatch):
    _,path,before,_,_,parent,boundary,_ = refresh_fixture(profile,tmp_path,monkeypatch)
    parent.run.side_effect = RuntimeError('Original native source verifier failed')
    with pytest.raises(RuntimeError,match='Original native source verifier failed'):
        capture.refresh_prepared_profile(path)
    assert path.read_bytes() == before;boundary.verify.assert_called_once()


def test_refresh_end_boundary_failure_never_publishes_revised_profile(profile,tmp_path,monkeypatch):
    _,path,before,_,_,_,boundary,_ = refresh_fixture(profile,tmp_path,monkeypatch)
    boundary.verify.side_effect = RuntimeError('fixed input changed during refresh')
    with pytest.raises(RuntimeError,match='fixed input changed'):capture.refresh_prepared_profile(path)
    assert path.read_bytes() == before
    assert list(path.parent.glob('profile.prequalification_*.json')) == []


def test_bounded_successful_stop_creates_pause_marker_only_after_safe_exit(profile,tmp_path,monkeypatch):
    root,_,parent,boundary = prepared_run(profile,tmp_path,monkeypatch)
    def stopped(args):
        value = capture.read(root/'status.json');value.update(stage='paused',current=None)
        (root/'status.json').write_text(json.dumps(value));return 0
    parent.run.side_effect = stopped
    verify = Mock();monkeypatch.setattr(capture.handoff,'verify_exit',verify)
    assert capture.run(profile,stop_after_chunks=1) == 0
    verify.assert_called_once_with(root,require_marker=False)
    assert capture.read(root/'PAUSE')['reason'] == 'bounded_chunk_stop'
    boundary.verify.assert_called_once()


def test_unsafe_bounded_stop_cannot_gain_normal_pause_marker(profile,tmp_path,monkeypatch):
    root,_,parent,boundary = prepared_run(profile,tmp_path,monkeypatch)
    def stopped(args):
        value = capture.read(root/'status.json');value.update(stage='paused',current=None)
        (root/'status.json').write_text(json.dumps(value));return 0
    parent.run.side_effect = stopped
    monkeypatch.setattr(capture.handoff,'verify_exit',Mock(side_effect=RuntimeError('Park missing')))
    with pytest.raises(RuntimeError,match='Park missing'):capture.run(profile,stop_after_chunks=1)
    assert not (root/'PAUSE').exists();boundary.verify.assert_called_once()


def test_completed_replay_selects_original_readonly_verification_and_restores(tmp_path):
    output = tmp_path/'pilot';output.mkdir()
    (output/'status.json').write_text(json.dumps({'stage':'complete'}))
    observed_calls = []
    def execute(*args,**kwargs):
        observed_calls.append((args,kwargs))
        kwargs['_verification'].update(selected={'scene0':'actual proof'},deficits={})
        return 0
    parent = SimpleNamespace(batch=SimpleNamespace(execute=execute))
    with capture.completed_readonly_replay(parent,output) as verified:
        assert parent.batch.execute(tmp_path/'source.json',[0],output,1,True,prepare_only=False) == 0
    assert parent.batch.execute is execute
    assert verified == {'selected':{'scene0':'actual proof'},'deficits':{}}
    args,kwargs = observed_calls[0]
    assert args == (tmp_path/'source.json',[0],output,1,True)
    assert kwargs['_verification'] is verified and kwargs['prepare_only'] is False


@pytest.mark.parametrize('change',['nonterminal','different_output','injected_verification','too_few_args'])
def test_completed_replay_refuses_wrong_output_state_or_verification_override(tmp_path,change):
    output = tmp_path/'pilot';output.mkdir()
    (output/'status.json').write_text(json.dumps({'stage':'running' if change == 'nonterminal' else 'complete'}))
    original = Mock();parent = SimpleNamespace(batch=SimpleNamespace(execute=original))
    values = [tmp_path/'manifest.json',[0],tmp_path/'other' if change == 'different_output' else output]
    kwargs = {'_verification':{}} if change == 'injected_verification' else {}
    if change == 'too_few_args':values = values[:2]
    with capture.completed_readonly_replay(parent,output):
        with pytest.raises(RuntimeError,match='exact terminal output'):parent.batch.execute(*values,**kwargs)
    original.assert_not_called()
    assert parent.batch.execute is original


def test_completed_replay_restores_after_verifier_failure_or_ownership_change(tmp_path):
    output = tmp_path/'pilot';output.mkdir();(output/'status.json').write_text('{"stage":"complete_with_deficit"}')
    original = Mock(side_effect=RuntimeError('Actual native proof failed'))
    parent = SimpleNamespace(batch=SimpleNamespace(execute=original))
    with pytest.raises(RuntimeError,match='Actual native proof failed'):
        with capture.completed_readonly_replay(parent,output):parent.batch.execute(tmp_path/'source',[0],output)
    assert parent.batch.execute is original
    with pytest.raises(RuntimeError,match='ownership changed'):
        with capture.completed_readonly_replay(parent,output):parent.batch.execute = Mock()
    assert parent.batch.execute is original


def cached_authority_fixture(profile,tmp_path):
    source_input = tmp_path/'source_reference.json';source_input.write_text('{"actual":"source"}')
    runtime = tmp_path/'old_runtime.py';runtime.write_text('ORIGINAL = True\n')
    gate = tmp_path/'native_gate.json';gate.write_text('{"actual":"six gate proof"}')
    values = dict(qualified_runtime={},input_hashes={str(source_input):capture.sha(source_input)},
        runtime_hashes={str(runtime):capture.sha(runtime)},
        gate_evidence={'native_exact':{'path':str(gate),'sha256':capture.sha(gate)}})
    Path(profile['release']).write_text(json.dumps(values));profile['release_sha256'] = capture.sha(profile['release'])
    profile['fixed_pins'] = capture.merge(values['input_hashes'],values['runtime_hashes'],{str(gate):capture.sha(gate)})
    old_profile = {'native':'unchanged'}
    profile['source_authority'] = dict(path=profile['release'],sha256=profile['release_sha256'],
        original_runtime_count=1,source_qualification='original-six-gate-environment-release',
        original_authority=dict(input_hashes=values['input_hashes'],gate_evidence=values['gate_evidence'],
            runtime_profile=old_profile))
    parent = SimpleNamespace(profiles=lambda:old_profile,qualified_release=Mock(side_effect=AssertionError('No native/bag re-decoding')))
    return values,parent


def test_cached_source_authority_reuses_only_same_original_result_and_all_byte_pins(profile,tmp_path):
    _,parent = cached_authority_fixture(profile,tmp_path)
    result = capture.source_authority(profile,parent)
    assert result == profile['source_authority'] and result is not profile['source_authority']
    parent.qualified_release.assert_not_called()


@pytest.mark.parametrize('group',['input_hashes','runtime_hashes','gate_evidence'])
def test_cached_authority_cannot_omit_any_original_source_runtime_or_gate_pin(profile,tmp_path,group):
    values,parent = cached_authority_fixture(profile,tmp_path)
    key = next(iter(values[group]))
    path = values[group][key]['path'] if group == 'gate_evidence' else key
    profile['fixed_pins'].pop(path)
    with pytest.raises(RuntimeError,match='complete boundary evidence pins'):capture.source_authority(profile,parent)
    parent.qualified_release.assert_not_called()


@pytest.mark.parametrize('change',['release_path','release_sha','runtime_count','source_type','inputs','gates','profile'])
def test_cached_authority_rejects_changed_original_verifier_result(profile,tmp_path,change):
    _,parent = cached_authority_fixture(profile,tmp_path)
    saved = profile['source_authority']
    if change == 'release_path':saved['path'] += '.different'
    elif change == 'release_sha':saved['sha256'] = 'a'*64
    elif change == 'runtime_count':saved['original_runtime_count'] += 1
    elif change == 'source_type':saved['source_qualification'] = 'original-support-release'
    elif change == 'inputs':saved['original_authority']['input_hashes'] = {}
    elif change == 'gates':saved['original_authority']['gate_evidence'] = {}
    else:saved['original_authority']['runtime_profile'] = {'other':'profile'}
    with pytest.raises(RuntimeError,match='qualification lacks|source result differs'):
        capture.source_authority(profile,parent)
    parent.qualified_release.assert_not_called()


def test_boundary_rechecks_native_bytes_before_cached_authority_can_be_used(profile,tmp_path):
    values,_ = cached_authority_fixture(profile,tmp_path)
    boundary = capture.boundary_module.SessionInputBoundary(profile['fixed_pins'])
    boundary.verify()
    filename = next(iter(values['gate_evidence'].values()))['path']
    Path(filename).write_text('changed actual native qualification proof')
    with pytest.raises(capture.boundary_module.BoundaryInputError,match='changed at session boundary'):
        boundary.verify()
