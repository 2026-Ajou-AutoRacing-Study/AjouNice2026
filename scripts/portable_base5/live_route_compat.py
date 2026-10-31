#!/usr/bin/env python3
"""동결 live_v2 서비스는 보존하고 새 route adapter 실행만 명시적으로 관리한다.

import와 기본 명령은 실행하지 않는다. ROS/bridge는 기존 소유권으로만 검증하고
capture는 이 실제 파일의 PID/argv/SHA와 같은 capture.lock으로 따로 관리한다.
원형 module.__file__, host_contract, qualification, source/runtime는 변경하지 않는다.
"""
import argparse
from contextlib import contextmanager
import fcntl
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import stat
import subprocess
import sys
import time


SCHEMA = 'base5_borrowed_route_compat_intent_v1'
SERVICE_SCHEMA = 'base5_borrowed_route_capture_service_v1'
PINNED = dict(
    live_v2='7850aed3a4a2881393af75ed2da96752e8fe292606c837abd5790281e481fd00',
    control='02c1a71c0e33ca261d167b5fce0f4b02e2fb2f2df330b0a2702e0018795e9c7f',
    status='e431ce8dc3fc03b98b5c5bd0ab2a8c9ebde79c3822536e68ec3dcf4e876081ca',
    finish_collection='773fc0fad451eea7290a2da8d0fce3e9b1e190808099463d9783722057bf4895',
    finish='219e1e04b80182d3574a3d3b5de9f9f207a18e8ba5b21aedbbbc3f1ae60c1fe5')
LOGICAL_SCRIPTS = Path('/home/luv/AjouNice2026/dataset/morai_od/scripts')


def module_at(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def checked_helper(path, expected):
    """원형 live조차 신뢰하기 전에 실제 파일 신원·bytes를 확인한다."""
    path = Path(path).absolute()
    if '..' in path.parts or any(part.is_symlink() for part in (path,*path.parents)):
        raise ValueError('Exact nonsymlink frozen helper required')
    info = path.stat()
    if not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o022:
        raise ValueError('Private owned frozen helper required')
    if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
        raise ValueError('Exact frozen helper bytes required')
    return path


def supervisor_argv(argv, entrypoint):
    """경로 토큰을 -c 인자로 붙인 다른 Python을 실행기로 인정하지 않는다."""
    if not argv or not Path(argv[0]).name.startswith('python3'):return False
    index = 2 if len(argv)>1 and argv[1]=='-B' else 1
    return len(argv)>index and argv[index]==str(entrypoint) and 'capture' in argv[index+1:] and '--run' in argv[index+1:]


class Facade:
    def __init__(self):
        self.__file__ = str(Path(__file__).resolve())
        self.directory = Path(self.__file__).parent
        path = self.directory/'live_v2.py'
        if not path.exists():
            # 로컬 체크아웃의 동일 bytes만 허용한다. 다른 bootstrap은 SHA에서 거부한다.
            path = self.directory/'live.py'
        checked_helper(path,PINNED['live_v2'])
        self.old = module_at('_base5_original_live_for_route_compat', path)
        self.old.owned(path, False)
        self.old.require(self.old.digest(path) == PINNED['live_v2'], 'Exact frozen live_v2 bytes required')
        self.old_file = str(path.resolve())

    def __getattr__(self, name):
        return getattr(self.old, name)

    @contextmanager
    def original_import(self):
        prior = sys.modules.get('live_v2')
        sys.modules['live_v2'] = self.old
        try:
            yield
        finally:
            if prior is None:sys.modules.pop('live_v2', None)
            else:sys.modules['live_v2'] = prior

    def helper(self, name):
        path = self.directory/(name+'.py')
        self.old.owned(path, False)
        self.old.require(self.old.digest(path) == PINNED[name], 'Frozen helper changed: '+name)
        with self.original_import():
            module = module_at('_base5_original_'+name+'_for_route_compat', path)
        module.live = self
        return module

    def intent_path(self, ctx):
        return ctx['workspace']/'checks/route_compat_intent.json'

    def intent(self, ctx):
        """외부 호스트에서 가능한 실제 helper/evidence만 검사한다. logical repo는 실행 전 namespace에서 검사한다."""
        path = self.old.owned(self.intent_path(ctx), False)
        doc = self.old.read(path);workspace = ctx['workspace']
        self.old.require(doc.get('schema') == SCHEMA and doc.get('workspace') == str(workspace)
            and Path(doc.get('scripts', '')) == LOGICAL_SCRIPTS
            and doc.get('output') == str(workspace/'collections/sunny13_5k')
            and doc.get('environment') == 'sunny13' and doc.get('source_runtime_count') == 427
            and doc.get('new_live_smoke_executed') is False,
            'Exact borrowed route compatibility scope required')
        graph = ctx.get('graph')
        if graph is not None:
            self.old.require(doc.get('manifest') == graph['manifest']['source']
                and doc.get('release') == graph['release']['source'], 'Original release/source binding changed')
        host = workspace/'work/ros-session-v2/host_contract.json'
        qualification = workspace/'work/ros-session-v2/qualification.json'
        required = {Path(self.__file__), self.directory/'route_dispatch.py', self.directory/'route_compat.py',
            Path(self.old_file), host, qualification,
            *(self.directory/(name+'.py') for name in ('control','status','finish_collection','finish'))}
        files = doc.get('files', {})
        self.old.require(isinstance(files, dict) and required <= {Path(name) for name in files},
            'Every new executor and frozen helper/evidence pin is required')
        for filename, expected in files.items():
            actual = self.old.owned(filename, False)
            self.old.require(workspace/'checks' in actual.parents or actual in (host,qualification),
                'Compatibility helper/evidence escaped approved scope')
            self.old.require(isinstance(expected, str) and len(expected) == 64
                and self.old.digest(actual) == expected, 'Compatibility helper/evidence changed: '+filename)
        self.old.require(doc.get('host_contract_sha256') == self.old.digest(host)
            and doc.get('qualification_sha256') == self.old.digest(qualification), 'Historical host qualification changed')
        witness = self.old.owned(doc['witness'], False)
        self.old.require(witness == workspace/'checks/canonical_route_v1.json'
            and self.old.digest(witness) == doc.get('witness_sha256'), 'Pinned canonical route witness changed')
        self.old.require(str(Path(self.old.__file__).resolve()) == self.old_file,
            'Original live module identity must remain unchanged')
        self.verify_cpu_qualification(ctx,doc,required=False)
        return doc

    def verify_cpu_qualification(self, ctx, doc=None, required=True):
        doc = self.old.read(self.intent_path(ctx)) if doc is None else doc
        path,expected = doc.get('cpu_qualification'),doc.get('cpu_qualification_sha256')
        if path is None and expected is None and not required:return None
        self.old.require(path is not None and expected is not None,'Actual CPU route qualification required before capture')
        path = self.old.owned(path,False)
        self.old.require(path == ctx['workspace']/'work/route-compat-validation/v1/qualification.json'
            and self.old.digest(path) == expected,'CPU qualification scope/SHA changed')
        receipt = self.old.read(path)
        self.old.require(receipt.get('passed') is True and receipt.get('captures_requested') == 0
            and receipt.get('native_capture_executed') is False
            and type(receipt.get('replacement_count')) is int and receipt['replacement_count'] >= 1
            and receipt.get('manifest_sha256') == self.old.DECK_SHA,
            'Actual zero-capture replacement-route CPU qualification must pass')
        return receipt

    def load(self, workspace, verify=True):
        ctx = self.old.load(workspace, verify=verify)
        self.intent(ctx)
        return ctx

    def ready(self, ctx):
        self.old.ready(ctx)
        self.intent(ctx)

    def verify_qualification(self, ctx):
        self.old.verify_qualification(ctx)
        self.intent(ctx)

    def contract(self, ctx):
        doc = self.intent(ctx)
        return dict(schema='base5_borrowed_route_host_facade_v1', historical_host_contract=self.old.contract(ctx),
            executor_sha256=self.old.digest(self.__file__), intent_sha256=self.old.digest(self.intent_path(ctx)),
            helper_evidence_hashes=doc['files'], witness_sha256=doc['witness_sha256'],
            original_native_qualification_sha256=doc['qualification_sha256'], new_live_smoke_executed=False)

    def capture_command(self, ctx):
        doc = self.intent(ctx)
        original = self.old.action_command(ctx, 'capture')
        self.old.require(original[:2] == ['/usr/bin/python3','-B']
            and original[2] == str(LOGICAL_SCRIPTS/'launch_base5_obstacle_support.py'), 'Unexpected original launcher')
        return ['/usr/bin/python3','-B',str(self.directory/'route_dispatch.py'),
            '--intent',str(self.intent_path(ctx)),'--role','launcher','--',*original[3:]]

    def service_owned(self, ctx, role):
        if role != 'capture':return self.old.service_owned(ctx, role)
        self.intent(ctx)
        path = ctx['session']/'capture.lock'
        if not path.is_file() or path.is_symlink():return False
        self.old.owned(path, False)
        with path.open('r') as lock:
            try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            except BlockingIOError:pass
            else:
                fcntl.flock(lock,fcntl.LOCK_UN);return False
        records = sorted(ctx['session'].glob('route_capture_*.json'))
        if not records:return False
        record = self.old.read(self.old.owned(records[-1], False))
        try:
            proc = Path('/proc')/str(record['supervisor_pid'])
            argv = [os.fsdecode(value) for value in (proc/'cmdline').read_bytes().split(b'\0') if value]
            start = (proc/'stat').read_text().rsplit(')',1)[1].split()[19]
            return (proc.stat().st_uid == os.getuid() and record.get('schema') == SERVICE_SCHEMA
                and record.get('action') == 'capture' and record.get('executor_sha256') == self.old.digest(self.__file__)
                and record.get('executor_path') == self.__file__ and record.get('pid_start_ticks') == start
                and record.get('intent_sha256') == self.old.digest(self.intent_path(ctx))
                and record.get('supervisor_argv') == argv and supervisor_argv(argv,self.__file__))
        except (OSError,KeyError,TypeError):return False

    def supervise_capture(self, ctx, command):
        lock_path = self.old.no_symlinks(ctx['session']/'capture.lock')
        if lock_path.exists():self.old.owned(lock_path, False)
        fd = os.open(lock_path,os.O_RDWR|os.O_CREAT|os.O_APPEND|os.O_NOFOLLOW,0o600)
        with os.fdopen(fd,'a') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            self.ready(ctx);self.verify_qualification(ctx)
            self.verify_cpu_qualification(ctx)
            argv = [os.fsdecode(value) for value in Path('/proc/self/cmdline').read_bytes().split(b'\0') if value]
            start = Path('/proc/self/stat').read_text().rsplit(')',1)[1].split()[19]
            self.old.require(supervisor_argv(argv,self.__file__),
                'Actual capture supervisor argv identity required before process start')
            child = None;previous = {}
            def pause(signum,frame):
                if child is not None and child.poll() is None:
                    marker = self.old.request_pause(ctx)
                    print(json.dumps(dict(event='graceful_pause_requested',marker=str(marker),
                        note='Wait for the current chunk and Park; keep ROS/bridge running')),flush=True)
            for signum in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):previous[signum] = signal.signal(signum,pause)
            try:
                child = subprocess.Popen(self.old.namespace(ctx,command),env=self.old.environment(ctx),start_new_session=True)
                record = dict(schema=SERVICE_SCHEMA,action='capture',supervisor_pid=os.getpid(),
                    pid_start_ticks=start,supervisor_argv=argv,child_pid=child.pid,time=time.time(),command=command,
                    executor_path=self.__file__,executor_sha256=self.old.digest(self.__file__),
                    intent_sha256=self.old.digest(self.intent_path(ctx)),original_live_identity_preserved=True)
                self.old.write_new(ctx['session']/('route_capture_'+str(time.time_ns())+'.json'),record)
                return child.wait()
            finally:
                if child is not None and child.poll() is None:
                    self.old.request_pause(ctx);child.wait()
                for signum,handler in previous.items():signal.signal(signum,handler)

    def finish(self, args, ctx=None):
        helper = self.helper('finish_collection')
        if args.inner_intent is not None:
            # 실제 진입 파일은 이 wrapper다. frozen helper의 __file__은 유지한다.
            self.old.require(args.run and args.intent_sha256
                and os.environ.get('BASE5_FINISH_INTENT_SHA256') == args.intent_sha256
                and self.old.digest(args.inner_intent) == args.intent_sha256
                and self.old.digest('/run/base5-bundle/portable_bundle.json') == self.old.MANIFEST_SHA,
                'Explicit namespaced CPU finish intent required')
            document = self.old.read(args.inner_intent)
            workspace = self.old.owned(document['workspace'])
            self.old.require(workspace == Path(args.workspace).resolve(), 'Inner workspace differs from explicit intent')
            inner_ctx = dict(workspace=workspace)
            self.intent(inner_ctx)
            print(json.dumps(helper.finish_inside(document,args.inner_intent),ensure_ascii=False,indent=2))
            return 0
        self.ready(ctx);self.verify_qualification(ctx)
        if not args.run:return helper.main(['--workspace',str(ctx['workspace'])])
        finisher = helper.load_finisher()
        capture_lock = self.old.no_symlinks(ctx['session']/'capture.lock')
        if capture_lock.exists():self.old.owned(capture_lock,False)
        with finisher.exclusive(capture_lock):
            expected = helper.intent(ctx);p = helper.paths(ctx)
            self.old.checked_folder(p['session'])
            record = p['session']/'intent.json';helper.save_unchanged(record,expected)
            digest = self.old.digest(record)
            command = ['/usr/bin/python3','-B',str(self.directory/'route_dispatch.py'),
                '--intent',str(self.intent_path(ctx)),'--role','finish-support','--',
                '--workspace',str(ctx['workspace']),'finish','--inner-intent',str(record),
                '--intent-sha256',digest,'--run']
            env = self.old.environment(ctx);env['BASE5_FINISH_INTENT_SHA256'] = digest
            return subprocess.run(self.old.namespace(ctx,command),env=env,check=False).returncode


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace',type=Path,default=Path.home()/'ajounice_base5_capture')
    parser.add_argument('action',choices=['capture','pause','resume-paused','status','finish'])
    parser.add_argument('--run',action='store_true')
    parser.add_argument('--confirmed-environment',choices=['sunny13'])
    parser.add_argument('--watch',action='store_true')
    parser.add_argument('--inner-intent',type=Path,help=argparse.SUPPRESS)
    parser.add_argument('--intent-sha256',help=argparse.SUPPRESS)
    args = parser.parse_args(argv);facade = Facade()
    if args.inner_intent is not None:
        facade.old.require(args.action == 'finish','Inner invocation is only permitted for CPU finish')
        return facade.finish(args)
    facade.old.require(args.intent_sha256 is None,'Inner digest requires an explicit inner finish intent')
    ctx = facade.load(args.workspace,verify=args.action != 'status');facade.ready(ctx)
    if args.action == 'status':
        facade.old.require(not args.run,'Status is read-only; omit --run')
        return facade.helper('status').main(['--workspace',str(ctx['workspace']),*(['--watch'] if args.watch else [])])
    facade.old.require(not args.watch,'Only status supports --watch')
    if args.action == 'finish':return facade.finish(args,ctx)
    if not args.run:
        print(json.dumps(dict(event='route_compat_plan_only',action=args.action,
            command=facade.capture_command(ctx) if args.action == 'capture' else None,
            files_written=False,capture_started=False,signals_sent=False,new_live_smoke_executed=False),ensure_ascii=False,indent=2))
        return 0
    if args.action in ('pause','resume-paused'):
        helper = facade.helper('control')
        result = helper.pause(ctx) if args.action == 'pause' else helper.resume_paused(ctx)
        if args.action == 'resume-paused':
            result['next_manual_command'] = ['/usr/bin/python3','-B',facade.__file__,'--workspace',str(ctx['workspace']),
                'capture','--run','--confirmed-environment','sunny13']
        print(json.dumps(result,ensure_ascii=False,indent=2));return 0
    facade.old.require(args.confirmed_environment == 'sunny13','Confirm actual MR UI Sunny13 before capture')
    facade.verify_cpu_qualification(ctx)
    facade.old.require(facade.old.port_open(11311) and facade.service_owned(ctx,'ros'),
        'Keep the existing approved ROS master running')
    facade.old.require(facade.old.port_open(9090) and facade.service_owned(ctx,'bridge'),
        'Keep the existing approved ROS bridge running')
    facade.verify_qualification(ctx)
    return facade.supervise_capture(ctx,facade.capture_command(ctx))


if __name__ == '__main__':
    try:raise SystemExit(main())
    except KeyboardInterrupt:
        print('관측만 종료했다. ROS·브리지는 종료하지 않았다.');raise SystemExit(0)
    except (OSError,ValueError,RuntimeError,subprocess.SubprocessError) as error:
        print(json.dumps(dict(event='borrowed_route_compat_blocked',error=str(error),
            fallback_allowed=False,capture_not_started_or_requested_pause=True),ensure_ascii=False),file=sys.stderr)
        raise SystemExit(1)
