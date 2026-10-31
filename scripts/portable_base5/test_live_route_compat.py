"""CPU fixtures만 사용한다. ROS/MORAI·원격·실제 collector를 실행하지 않는다."""
from contextlib import redirect_stdout
import fcntl
import importlib.util
import io
import json
import os
from pathlib import Path
import shutil
import sys
import time
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

SOURCE = Path(__file__).parent


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True,mode=0o700)
    path.write_text(json.dumps(value)+'\n');path.chmod(0o600)
    return path


@pytest.fixture
def fixture(tmp_path,monkeypatch):
    workspace = tmp_path/'home/ajounice_base5_capture';checks = workspace/'checks'
    checks.mkdir(parents=True,mode=0o700)
    for name in ('live_route_compat','control','status','finish_collection','finish','route_compat'):
        destination = checks/(name+'.py');shutil.copyfile(SOURCE/(name+'.py'),destination);destination.chmod(0o600)
    shutil.copyfile(SOURCE/'live.py',checks/'live_v2.py');(checks/'live_v2.py').chmod(0o600)
    (checks/'route_dispatch.py').write_text('# CPU fixture, never execute\n');(checks/'route_dispatch.py').chmod(0o600)
    spec = importlib.util.spec_from_file_location('route_live_fixture',checks/'live_route_compat.py')
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    facade = module.Facade();old = facade.old
    mr = workspace.parent/'MoraiLauncher_Lin';mr.mkdir(mode=0o700)
    bundle = workspace/'capture_bundle';bundle.mkdir(mode=0o700)
    graph = dict(files={},bindings=[],release=dict(source='/home/luv/AjouNice2026/release.json'),
        manifest=dict(source='/home/luv/AjouNice2026/manifest.json'))
    ctx = dict(workspace=workspace,bundle=bundle,mr=mr,graph=graph,
        session=workspace/'work/ros-session-v2',home=workspace/'work/ros-home-v2')
    old.prepare(ctx)
    smoke = workspace/'collections/smoke_v1'
    write(smoke/'status.json',dict(stage='complete',accepted_candidate=3,review=0,failed=0))
    write(smoke/'ledger.jsonl',dict(id='historical-smoke'))
    write(smoke/'shutdown_safety.json',dict(park_verified=True))
    proof = dict(host_contract=old.contract(ctx),native_frames=3,native_files={})
    for field,name in (('ledger_sha256','ledger.jsonl'),('status_sha256','status.json'),('shutdown_sha256','shutdown_safety.json')):
        proof[field] = old.digest(smoke/name);proof['native_files'][name] = old.digest(smoke/name)
    qualification = write(ctx['session']/'qualification.json',proof)
    witness = write(checks/'canonical_route_v1.json',dict(cpu_fixture_only=True))
    files = {str(path):old.digest(path) for path in checks.iterdir() if path.is_file()}
    for path in (ctx['session']/'host_contract.json',qualification):files[str(path)] = old.digest(path)
    intent = dict(schema=module.SCHEMA,workspace=str(workspace),scripts=str(module.LOGICAL_SCRIPTS),
        release=graph['release']['source'],manifest=graph['manifest']['source'],
        output=str(workspace/'collections/sunny13_5k'),environment='sunny13',witness=str(witness),
        witness_sha256=old.digest(witness),files=files,source_runtime_count=427,new_live_smoke_executed=False,
        host_contract_sha256=old.digest(ctx['session']/'host_contract.json'),qualification_sha256=old.digest(qualification))
    write(facade.intent_path(ctx),intent)
    monkeypatch.setattr(module,'Facade',lambda:facade)
    monkeypatch.setattr(facade,'load',lambda *args,**kwargs:ctx)
    return module,facade,ctx


def cpu_qualified(facade,ctx,**changes):
    receipt = dict(passed=True,captures_requested=0,native_capture_executed=False,
        replacement_count=1,manifest_sha256=facade.old.DECK_SHA)
    receipt.update(changes)
    path = write(ctx['workspace']/'work/route-compat-validation/v1/qualification.json',receipt)
    intent = facade.old.read(facade.intent_path(ctx))
    intent.update(cpu_qualification=str(path),cpu_qualification_sha256=facade.old.digest(path))
    write(facade.intent_path(ctx),intent)
    return path


def snapshot(root):
    return {str(p):dict(bytes=p.read_bytes(),inode=p.stat().st_ino,mtime=p.stat().st_mtime_ns)
        for p in root.rglob('*') if p.is_file() and not p.is_symlink()}


def invoke(module,ctx,arguments):
    output = io.StringIO()
    with redirect_stdout(output):result = module.main(['--workspace',str(ctx['workspace']),*arguments])
    return result,output.getvalue()


def test_readonly_plans_preserve_original_identity_contracts_and_qualification(fixture,monkeypatch):
    module,facade,ctx = fixture;before = snapshot(ctx['workspace']);old_path = facade.old.__file__
    for action in ('capture','pause','resume-paused'):
        with monkeypatch.context() as patch:
            process = Mock();probe = Mock()
            patch.setattr(module.subprocess,'Popen',process);patch.setattr(facade.old,'port_open',probe)
            result,text = invoke(module,ctx,[action]);report = json.loads(text)
            assert result == 0 and not report['files_written'] and not report['capture_started']
            process.assert_not_called();probe.assert_not_called()
    assert facade.old.__file__ == old_path
    assert snapshot(ctx['workspace']) == before
    assert 'historical_host_contract' in facade.contract(ctx)
    assert facade.old.read(ctx['session']/'host_contract.json') == facade.old.contract(ctx)


def test_capture_command_routes_only_entrypoint_and_keeps_original_arguments(fixture):
    _,facade,ctx = fixture
    original = facade.old.action_command(ctx,'capture');command = facade.capture_command(ctx)
    assert command[:3] == ['/usr/bin/python3','-B',str(facade.directory/'route_dispatch.py')]
    assert command[command.index('--role')+1] == 'launcher'
    assert command[command.index('--')+1:] == original[3:]
    assert '--complete' not in command


@pytest.mark.parametrize('kind',['wrapper','dispatcher','qualification','host','output','scope','source','witness'])
def test_changed_helper_evidence_or_scope_is_refused(fixture,kind):
    _,facade,ctx = fixture;path = facade.intent_path(ctx);doc = facade.old.read(path)
    if kind == 'wrapper':Path(facade.__file__).write_text('# changed')
    if kind == 'dispatcher':(facade.directory/'route_dispatch.py').write_text('# changed')
    if kind == 'qualification':write(ctx['session']/'qualification.json',{})
    if kind == 'host':write(ctx['session']/'host_contract.json',{})
    if kind == 'output':doc['output'] = str(ctx['workspace']/'foreign')
    if kind == 'scope':doc['source_runtime_count'] = 438
    if kind == 'source':doc['manifest'] = '/foreign/source'
    if kind == 'witness':doc['witness_sha256'] = '0'*64
    write(path,doc)
    with pytest.raises(ValueError):facade.intent(ctx)


def test_legacy_service_identity_is_delegated_without_rewriting_module_file(fixture,monkeypatch):
    _,facade,ctx = fixture;identity = facade.old.__file__
    delegate = Mock(return_value=True);monkeypatch.setattr(facade.old,'service_owned',delegate)
    assert facade.service_owned(ctx,'ros') and facade.service_owned(ctx,'bridge')
    assert delegate.call_args_list[0].args == (ctx,'ros')
    assert delegate.call_args_list[1].args == (ctx,'bridge')
    assert facade.old.__file__ == identity


def test_capture_refuses_missing_cpu_receipt_before_port_or_process(fixture,monkeypatch):
    module,facade,ctx = fixture;process = Mock();port = Mock()
    monkeypatch.setattr(module.subprocess,'Popen',process);monkeypatch.setattr(facade.old,'port_open',port)
    with pytest.raises(ValueError,match='CPU route qualification'):
        invoke(module,ctx,['capture','--run','--confirmed-environment','sunny13'])
    process.assert_not_called();port.assert_not_called()


@pytest.mark.parametrize('changes',[dict(passed=False),dict(captures_requested=1),dict(native_capture_executed=True),
    dict(replacement_count=0),dict(replacement_count=True),dict(manifest_sha256='foreign')])
def test_actual_zero_capture_cpu_route_receipt_is_required(fixture,changes):
    _,facade,ctx = fixture;cpu_qualified(facade,ctx,**changes)
    with pytest.raises(ValueError,match='zero-capture'):
        facade.verify_cpu_qualification(ctx)


def test_capture_runs_only_qualified_new_command_using_existing_ros_bridge(fixture,monkeypatch):
    module,facade,ctx = fixture;cpu_qualified(facade,ctx)
    monkeypatch.setattr(facade.old,'port_open',lambda port:True)
    services = Mock(return_value=True);monkeypatch.setattr(facade.old,'service_owned',services)
    run = Mock(return_value=0);monkeypatch.setattr(facade,'supervise_capture',run)
    result,_ = invoke(module,ctx,['capture','--run','--confirmed-environment','sunny13'])
    assert result == 0 and run.call_args.args == (ctx,facade.capture_command(ctx))
    assert {call.args[1] for call in services.call_args_list} == {'ros','bridge'}


def test_pause_dependency_is_new_facade_without_changing_frozen_control(fixture,monkeypatch):
    module,facade,ctx = fixture;before = snapshot(ctx['workspace'])
    monkeypatch.setattr(facade,'service_owned',lambda ctx,role:role=='capture')
    root = ctx['workspace']/'collections/sunny13_5k';root.mkdir(mode=0o700)
    _,text = invoke(module,ctx,['pause','--run'])
    assert json.loads(text)['created'] and (root/'PAUSE').exists()
    for path,record in before.items():
        assert Path(path).read_bytes() == record['bytes']
    assert facade.helper('control').live is facade


def test_status_is_readonly_and_does_not_start_any_service(fixture,monkeypatch):
    module,facade,ctx = fixture;before = snapshot(ctx['workspace']);process = Mock()
    monkeypatch.setattr(module.subprocess,'Popen',process)
    result,text = invoke(module,ctx,['status'])
    assert result == 0 and '준비 대기' in text
    assert snapshot(ctx['workspace']) == before;process.assert_not_called()


def test_supervisor_identity_cannot_be_forged_by_c_argument(fixture):
    module,facade,_ = fixture
    assert module.supervisor_argv(['/usr/bin/python3','-B',facade.__file__,'capture','--run'],facade.__file__)
    assert not module.supervisor_argv(['/usr/bin/python3','-c','pass',facade.__file__,'capture','--run'],facade.__file__)
    assert not module.supervisor_argv(['/usr/bin/python3','-B','/foreign.py',facade.__file__,'capture','--run'],facade.__file__)


def test_capture_ownership_needs_actual_held_lock_and_exact_current_process_identity(fixture,monkeypatch):
    module,facade,ctx = fixture;lock = ctx['session']/'capture.lock';lock.write_text('');lock.chmod(0o600)
    assert not facade.service_owned(ctx,'capture')
    argv = ['/usr/bin/python3','-B',facade.__file__,'--workspace',str(ctx['workspace']),'capture','--run']
    proc = Path('/proc')/str(os.getpid())
    record = dict(schema=module.SERVICE_SCHEMA,action='capture',supervisor_pid=os.getpid(),
        pid_start_ticks=(proc/'stat').read_text().rsplit(')',1)[1].split()[19],supervisor_argv=argv,
        executor_sha256=facade.old.digest(facade.__file__),executor_path=facade.__file__,
        intent_sha256=facade.old.digest(facade.intent_path(ctx)))
    record_path = write(ctx['session']/'route_capture_1.json',record)
    original = Path.read_bytes
    def read_bytes(path):
        if path == proc/'cmdline':return b'\0'.join(os.fsencode(value)for value in argv)+b'\0'
        return original(path)
    monkeypatch.setattr(Path,'read_bytes',read_bytes)
    with lock.open() as handle:
        fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
        assert facade.service_owned(ctx,'capture')
        record['pid_start_ticks'] = 'wrong';write(record_path,record)
        assert not facade.service_owned(ctx,'capture')


def test_actual_supervisor_identity_is_required_before_starting_child(fixture,monkeypatch):
    module,facade,ctx = fixture;cpu_qualified(facade,ctx);process = Mock()
    monkeypatch.setattr(module.subprocess,'Popen',process)
    with pytest.raises(ValueError,match='before process start'):
        facade.supervise_capture(ctx,facade.capture_command(ctx))
    process.assert_not_called()


def test_finish_uses_same_capture_lock_and_dispatcher_self_reentry(fixture,monkeypatch):
    module,facade,ctx = fixture;helper = facade.helper('finish_collection');old_file = helper.__file__
    # 자체 승인/원본 재구성은 frozen helper가 수행한다. 여기서는 외부→내부 argv만 검사한다.
    monkeypatch.setattr(helper,'intent',lambda ctx:dict(cpu_fixture_only=True))
    monkeypatch.setattr(facade,'helper',lambda name:helper)
    process = Mock(return_value=SimpleNamespace(returncode=0));monkeypatch.setattr(module.subprocess,'run',process)
    args = SimpleNamespace(run=True,inner_intent=None)
    assert facade.finish(args,ctx) == 0
    command = process.call_args.args[0]
    assert '--role' in command and command[command.index('--role')+1] == 'finish-support'
    assert 'finish' in command and '--inner-intent' in command
    assert helper.__file__ == old_file and facade.old.__file__ == facade.old_file
    assert Path(process.call_args.kwargs['env']['ROS_LOG_DIR']) == ctx['session']/'log'


def test_missing_or_changed_frozen_live_is_rejected_before_import(tmp_path,monkeypatch):
    import live_route_compat as module
    dangerous = tmp_path/'fake.py';dangerous.write_text('raise RuntimeError("must not execute")\n');dangerous.chmod(0o600)
    with pytest.raises(ValueError,match='frozen helper bytes'):
        module.checked_helper(dangerous,module.PINNED['live_v2'])


def test_resume_preserves_pause_bytes_and_returns_the_new_entrypoint(fixture,monkeypatch):
    module,facade,ctx = fixture
    root = ctx['workspace']/'collections/sunny13_5k';native = root/'chunk_0000/capture'
    write(native/'session_contract.json',{})
    write(native/'status.json',dict(stage='complete'))
    write(native/'ledger.jsonl',dict(status='accepted_candidate'))
    write(native/'shutdown_safety.json',dict(park_verified=True,gear=1,brake_held_during_requests=True,
        request_fields=dict(option=2,gear=1)))
    write(root/'status.json',dict(stage='paused',requested=5000,finished=16,current=None,updated_at=time.time()+1))
    marker = write(root/'PAUSE',dict(reason='operator'));original = marker.read_bytes()
    for directory in (root, root/'chunk_0000', native):
        directory.chmod(0o700)
    lock = ctx['session']/'capture.lock';lock.write_text('');lock.chmod(0o600)
    monkeypatch.setattr(facade,'service_owned',lambda *args:False)
    result,text = invoke(module,ctx,['resume-paused','--run']);report = json.loads(text)
    assert result == 0 and not marker.exists()
    assert Path(report['archived_marker']).read_bytes() == original
    assert report['next_manual_command'][:3] == ['/usr/bin/python3','-B',facade.__file__]
    assert 'capture' in report['next_manual_command'] and not report['capture_started']


def test_capture_signal_requests_pause_and_never_signals_ros_or_bwrap(fixture,monkeypatch):
    module,facade,ctx = fixture;cpu_qualified(facade,ctx)
    argv = ['/usr/bin/python3','-B',facade.__file__,'--workspace',str(ctx['workspace']),'capture','--run']
    original_read = Path.read_bytes
    def read_bytes(path):
        if str(path) == '/proc/self/cmdline':return b'\0'.join(os.fsencode(value)for value in argv)+b'\0'
        return original_read(path)
    monkeypatch.setattr(Path,'read_bytes',read_bytes)
    handlers = {}
    def handle(signum,callback):handlers[signum] = callback;return None
    monkeypatch.setattr(module.signal,'signal',handle)
    pause = Mock(return_value=ctx['workspace']/'collections/sunny13_5k/PAUSE')
    monkeypatch.setattr(facade.old,'request_pause',pause)
    broad_signal = Mock();monkeypatch.setattr(facade.old,'signal_inner_app',broad_signal)
    class Child:
        pid = 123456;done = False
        def poll(self):return 0 if self.done else None
        def wait(self):
            handlers[module.signal.SIGINT](module.signal.SIGINT,None)
            self.done = True;return 0
    process = Mock(return_value=Child());monkeypatch.setattr(module.subprocess,'Popen',process)
    assert facade.supervise_capture(ctx,facade.capture_command(ctx)) == 0
    pause.assert_called_once_with(ctx);broad_signal.assert_not_called()
    record = facade.old.read(next(ctx['session'].glob('route_capture_*.json')))
    assert record['executor_path'] == facade.__file__ and record['supervisor_argv'] == argv
    assert record['executor_sha256'] == facade.old.digest(facade.__file__)


def test_completion_or_hidden_nonfinish_invocation_cannot_start_capture(fixture,monkeypatch):
    module,_,ctx = fixture;process = Mock();monkeypatch.setattr(module.subprocess,'Popen',process)
    with pytest.raises(SystemExit):invoke(module,ctx,['capture','--complete'])
    with pytest.raises(ValueError,match='only permitted'):
        invoke(module,ctx,['capture','--inner-intent','/foreign','--run'])
    process.assert_not_called()
