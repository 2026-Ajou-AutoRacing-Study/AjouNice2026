#!/usr/bin/env python3
"""원형 Sunny15 계약을 보존하는 명시적 입력 검증 캐시 실행기.

기본은 읽기 전용 계획이다. --attach는 안전한 PAUSE에서 별도 계약만 작성한다.
--run --confirmed-environment sunny15 --resume-paused만 실제 재개를 허용한다.
"""
import sys
sys.dont_write_bytecode = True

import argparse
from contextlib import ExitStack, contextmanager
import fcntl
import importlib
import json
import math
import os
from pathlib import Path
import stat
import subprocess
import time
import uuid

import local_sunny15 as original
import verified_input_cache as cache_module


SCHEMA = 'base5_local_sunny15_verified_input_cache_v1'
DIRECTORY = original.FINAL/'local_input_cache_v1'
CONTRACT = DIRECTORY/'contract.json'
ATTACHMENT = DIRECTORY/'attachment.json'
ORIGINAL_SHA = original.sha
LOCK_NAMES = ('.static_attitude_batch.lock',)
CHILD_LOCKS = {'.fresh.lock','.batch.lock','.campaign.lock','.quality.lock',
    '.runner.lock','.spatial.lock','.static_attitude.lock','.direction_reference.lock'}
ENDED_NATIVE = {'complete','halted','interrupted','stopped_for_resume_test'}


def require(value, message):
    original.require(value, message)


def sha(path):
    return ORIGINAL_SHA(path)


def write_new(path, document):
    path = original.path(path)
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd, 'w') as stream:
        json.dump(document, stream, ensure_ascii=False, indent=2)
        stream.write('\n')


def merge_pins(*groups):
    result = {}
    for group in groups:
        require(type(group) is dict and group, 'Nonempty explicit immutable pin groups required')
        for filename, digest in group.items():
            path = original.path(filename)
            require(type(filename) is str and str(path) == filename and type(digest) is str
                and len(digest) == 64 and all(c in '0123456789abcdef' for c in digest),
                'Malformed immutable cache pin')
            require(path != original.OUTPUT and original.OUTPUT not in path.parents,
                'Collection output can never enter the immutable input cache')
            require(filename not in result or result[filename] == digest, 'Conflicting immutable input pin')
            result[filename] = digest
    return dict(sorted(result.items()))


def immutable_pins():
    require(sha(original.RELEASE) == original.RELEASE_SHA
        and sha(original.MANIFEST) == original.MANIFEST_SHA, 'Original frozen source/release changed')
    release = original.read(original.RELEASE)
    explicit = {str(path):sha(path) for path in
        (original.RELEASE, original.MANIFEST, original.QUALIFICATION)}
    return source_only_pins(merge_pins(release['runtime_hashes'], release['input_hashes'],
        original.helper_hashes(), explicit))


def source_only_pins(pins):
    """Never cache frozen or live native data/proofs; unselected pins use SHA fallback."""
    result = {}
    for filename,digest in pins.items():
        path = original.path(filename)
        name = path.name.lower()
        if path.suffix.lower() not in {'.json','.py','.yaml','.yml','.csv','.scn'}:
            continue
        if path.suffix.lower() != '.py' and any('native' in part.lower()
                or 'training_inputs' in part.lower() for part in path.parent.parts):
            continue
        if any(part.lower() in {'capture','captures','raw','scene_parks'} or part.upper().startswith(('LIDAR_','CAMERA_'))
                for part in path.parts):
            continue
        if (name in {'audit.json','status.json','report.json','shutdown_safety.json','session_contract.json',
                'live_report.json','timing.json','quota.json','batch_contract.json'}
                or name.startswith(('audit','ledger','selected','receipt','event_cmd'))
                or 'receipt' in name or 'snapshot' in name or name.endswith(('_audit.json','_proof.json'))):
            continue
        result[filename] = digest
    require(result, 'No explicitly immutable source/config/runtime input pins')
    return result


def helper_pins():
    return {str(original.path(path)):sha(path) for path in
        (Path(__file__).absolute(), Path(cache_module.__file__).resolve())}


def recipe(pins):
    original.validate_contract(original.OUTPUT/original.CONTRACT)
    guards = {str(original.OUTPUT/original.CONTRACT):sha(original.OUTPUT/original.CONTRACT)}
    guards.update(original.original_guards())
    return dict(schema=SCHEMA, environment='sunny15', output=str(original.OUTPUT),
        original_manifest=str(original.MANIFEST), original_release=str(original.RELEASE),
        original_contract_pins=guards, immutable_input_pins=pins, helper_pins=helper_pins(),
        cache_scope='process-local immutable declared inputs only; no output/native/session cache',
        native_qa_unchanged=True, chunk_size=16, original_512_geometry_8_layout_bounds=True,
        attachment_required_at_paused_park=True)


@contextmanager
def installed_cache(pins):
    cache = cache_module.VerifiedInputCache(pins, forbidden_roots=(original.OUTPUT,))
    old_sha = original.sha
    def cached_sha(filename):
        return cache.sha256(filename, fallback=old_sha)
    # Verify executable bytes before importing the frozen canonical hash modules.
    # The original release hash was checked before building this allowlist.
    for filename,digest in original.read(original.RELEASE)['runtime_hashes'].items():
        require(cache.sha256(filename,fallback=old_sha) == digest,
            'Original runtime changed before cache module installation')
    if str(original.SCRIPTS) not in sys.path:
        sys.path.insert(0,str(original.SCRIPTS))
    with cache.installed(scripts_dir=original.SCRIPTS):
        original.sha = cached_sha
        try:
            yield cache
        finally:
            changed = original.sha is not cached_sha
            original.sha = old_sha
            require(not changed, 'Original helper hash adapter ownership changed')


@contextmanager
def idle_lock(path):
    """Local approved directories may be 775; ownership/type/symlinks still fail closed."""
    path = owned(path, False)
    fd = os.open(path,os.O_RDONLY | os.O_NOFOLLOW)
    try:
        actual,expected = os.fstat(fd),path.stat()
        require(stat.S_ISREG(actual.st_mode) and actual.st_uid == os.getuid()
            and (actual.st_dev,actual.st_ino) == (expected.st_dev,expected.st_ino),
            'Local collection lock identity changed')
        fcntl.flock(fd,fcntl.LOCK_EX | fcntl.LOCK_NB)
        yield
    finally:
        os.close(fd)


def owned(path,directory=True):
    path = original.path(path)
    info = path.stat()
    require(info.st_uid == os.getuid() and
        (stat.S_ISDIR(info.st_mode) if directory else stat.S_ISREG(info.st_mode)),
        'Foreign-owned or nonregular local collection path: '+str(path))
    return path


def resume_evidence(root,stack):
    require(not (root/'STOP').exists() and not (root/'STOP').is_symlink(),
        'STOP must never be released as an ordinary pause')
    status = original.read(owned(root/'status.json',False))
    require(status.get('stage') == 'paused' and status.get('current') is None
        and status.get('requested') == 5000 and type(status.get('finished')) is int
        and 0 < status['finished'] <= 5000 and type(status.get('updated_at')) in (int,float)
        and math.isfinite(status['updated_at']),
        'Nonempty finished paused parent checkpoint required')
    sessions,shutdowns,locks = [],[],[]
    for folder,directories,files in os.walk(root,followlinks=False):
        folder = owned(folder)
        for name in directories:
            owned(folder/name)
        for name in files:
            owned(folder/name,False)
        locks.extend(folder/name for name in files if name in CHILD_LOCKS)
        if 'session_contract.json' in files:
            sessions.append(folder)
        if 'shutdown_safety.json' in files:
            shutdowns.append(folder/'shutdown_safety.json')
    for path in sorted(locks):
        stack.enter_context(idle_lock(path))
    require(sessions and set(shutdowns) == {folder/'shutdown_safety.json' for folder in sessions},
        'Native sessions and complete shutdown evidence must match')
    evidence = {}
    for folder in sorted(sessions):
        original.read(folder/'session_contract.json')
        native = original.read(folder/'status.json')
        require(native.get('stage') in ENDED_NATIVE, 'Native child remains incomplete')
        rows = [json.loads(line) for line in (folder/'ledger.jsonl').read_text().splitlines() if line.strip()]
        require(rows and all(type(row) is dict for row in rows), 'Complete native ledger required')
        shutdown_path = folder/'shutdown_safety.json'
        shutdown = original.read(shutdown_path)
        require(shutdown.get('park_verified') is True and type(shutdown.get('gear')) is int
            and shutdown['gear'] == 1 and shutdown.get('brake_held_during_requests') is True
            and shutdown.get('request_fields') == {'option':2,'gear':1}
            and shutdown_path.stat().st_mtime <= status['updated_at']+1,
            'Native final Park/brake evidence missing or newer than paused parent')
        evidence[str(folder.relative_to(root))] = {name:sha(folder/filename) for name,filename in
            (('session_sha256','session_contract.json'),('status_sha256','status.json'),
             ('ledger_sha256','ledger.jsonl'),('shutdown_sha256','shutdown_safety.json'))}
    return dict(parent_status_sha256=sha(root/'status.json'),native_sessions=evidence,
        recorded_park_only=True,simulator_rest_reobserved=False)


@contextmanager
def stopped_evidence():
    """Acquire old ownership locks and validate every actual native shutdown."""
    with ExitStack() as stack:
        stack.enter_context(idle_lock(original.OUTPUT/original.CONTRACT))
        for name in LOCK_NAMES:
            stack.enter_context(idle_lock(original.OUTPUT/name))
        marker = owned(original.OUTPUT/'PAUSE', False)
        identity = (marker.stat().st_dev, marker.stat().st_ino, sha(marker))
        evidence = resume_evidence(original.OUTPUT, stack)
        yield evidence, marker, identity


def raw_snapshot():
    parent = original.parent_module()
    snapshot = parent.completed_snapshot(original.OUTPUT)
    text = (original.OUTPUT/'selected_ledger.jsonl').read_text()
    require(not text or text.endswith('\n'), 'Torn original selected ledger')
    selected = [json.loads(line) for line in text.splitlines()]
    require(len({row['source_cell_id'] for row in selected}) == len(selected),
        'Duplicate original selected quota slot')
    status = original.read(original.OUTPUT/'status.json')
    require(type(status.get('accepted_candidate')) is int
        and len(selected) == status['accepted_candidate'],
        'Paused selected ledger and approved parent count must match')
    return dict(raw=snapshot, selected_rows=selected)


def attach():
    require(not CONTRACT.exists() and not ATTACHMENT.exists(),
        'Acceleration attachment already exists; never rewrite its capture snapshot')
    original.path(DIRECTORY)
    require(original.OUTPUT not in DIRECTORY.parents and DIRECTORY != original.OUTPUT,
        'Continuation evidence must be outside the collection')
    pins = immutable_pins()
    with stopped_evidence() as (safety, marker, identity), installed_cache(pins):
        original.frozen_context(deep=True)
        expected = recipe(pins)
        snapshot = raw_snapshot()
        require((marker.stat().st_dev, marker.stat().st_ino, sha(marker)) == identity,
            'PAUSE changed during attachment verification')
        require(sha(original.OUTPUT/'status.json') == safety['parent_status_sha256'],
            'Paused parent changed during attachment verification')
        require(not (original.OUTPUT/'STOP').exists() and not (original.OUTPUT/'STOP').is_symlink(),
            'STOP appeared during attachment')
        DIRECTORY.mkdir(parents=True, exist_ok=True, mode=0o700)
        receipt = dict(schema=SCHEMA+'_attachment', recorded_at=time.time(),
            output=str(original.OUTPUT), safety=safety, snapshot=snapshot,
            captures_requested=0, native_capture_executed=False, original_files_modified=False)
        write_new(ATTACHMENT, receipt)
        write_new(CONTRACT, dict(recipe=expected, attachment=str(ATTACHMENT),
            attachment_sha256=sha(ATTACHMENT)))
    return dict(event='local_sunny15_input_cache_attached', contract=str(CONTRACT),
        preserved_selected=len(snapshot['selected_rows']), native_capture_executed=False)


def validate_attachment(pins):
    saved = original.read(CONTRACT)
    require(saved.get('recipe') == recipe(pins) and saved.get('attachment') == str(ATTACHMENT)
        and saved.get('attachment_sha256') == sha(ATTACHMENT),
        'Acceleration continuation contract, helper or original input changed')
    # The attachment is a historical raw-preservation baseline, not a recurring
    # scan or cache entry. Actual resumptions separately inspect terminal safety.
    return saved


def parent_record(pid):
    proc = Path('/proc')/str(pid)
    argv = [os.fsdecode(item) for item in (proc/'cmdline').read_bytes().split(b'\0') if item]
    return dict(argv=argv,uid=proc.stat().st_uid,cwd=(proc/'cwd').resolve(),
        ticks=(proc/'stat').read_text().rsplit(')',1)[1].split()[19])


def verify_parent(pid, ticks):
    require(pid == os.getppid(), 'Fast recovery must belong to its actual parent')
    record = parent_record(pid)
    argv = record['argv']
    index = 1
    flags = []
    while len(argv)>index and argv[index].startswith('-'):
        require(argv[index] in ('-B','-u') and argv[index] not in flags,
            'Unknown fast parent interpreter flags')
        flags.append(argv[index]);index += 1
    script = Path(argv[index]) if len(argv)>index else Path('/invalid-fast-parent')
    if not script.is_absolute():
        script = record['cwd']/script
    require(record['uid'] == os.getuid() and record['ticks'] == ticks
        and original.path(script) == original.path(__file__)
        and argv[index+1:] == ['--run','--confirmed-environment','sunny15','--resume-paused'],
        'Actual fast parent uid/argv/start identity required')


@contextmanager
def routed_children(old_contract):
    saved_popen = subprocess.Popen
    pid,ticks = original.parent_identity()
    def routed(command,*args,**kwargs):
        if isinstance(command,(str,bytes)):
            require(original.RECOVERY.name not in os.fsdecode(command), 'Shell recovery is not qualified')
            return saved_popen(command,*args,**kwargs)
        argv = [os.fsdecode(value) for value in command]
        index = 2 if len(argv)>1 and argv[1]=='-B' else 1
        candidate = argv[index] if len(argv)>index else ''
        if any(Path(value).name == original.RECOVERY.name for value in argv):
            require(candidate == str(original.RECOVERY) and argv[0] == sys.executable
                and not kwargs.get('shell'), 'Unknown fast recovery invocation')
            original.child_arguments(argv[index+1:])
            require(original.path(old_contract) == original.OUTPUT/original.CONTRACT,
                'Unexpected original recovery contract')
            command = [sys.executable,'-B',str(Path(__file__).absolute()),
                '--child-contract',str(CONTRACT),'--contract-sha256',sha(CONTRACT),
                '--parent-pid',str(pid),'--parent-start-ticks',ticks,'--',*argv[index+1:]]
        return saved_popen(command,*args,**kwargs)
    subprocess.Popen = routed
    try:
        yield
    finally:
        changed = subprocess.Popen is not routed
        subprocess.Popen = saved_popen
        require(not changed, 'Fast recovery router ownership changed')


@contextmanager
def routed_parent():
    previous = original.routed_children
    original.routed_children = routed_children
    try:
        yield
    finally:
        changed = original.routed_children is not routed_children
        original.routed_children = previous
        require(not changed, 'Original router adapter ownership changed')


def release_pause():
    with stopped_evidence() as (safety, marker, identity):
        require((marker.stat().st_dev, marker.stat().st_ino, sha(marker)) == identity,
            'PAUSE changed during native exit inspection')
        require(sha(original.OUTPUT/'status.json') == safety['parent_status_sha256'],
            'Parent checkpoint changed during native exit inspection')
        require(not (original.OUTPUT/'STOP').exists() and not (original.OUTPUT/'STOP').is_symlink(),
            'STOP appeared before resume')
        archive = marker.with_name('PAUSE.resumed_'+str(time.time_ns())+'_'+uuid.uuid4().hex+'.json')
        require(not archive.exists() and not archive.is_symlink(), 'Pause archive already exists')
        os.rename(marker, archive)
        return str(archive)


def metrics(cache, started):
    stats = cache.stats() if callable(cache.stats) else cache.stats
    return dict(event='local_sunny15_input_cache_metrics', pid=os.getpid(),
        elapsed_sec=time.monotonic()-started, stats=stats, native_qa_unchanged=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--attach',action='store_true');mode.add_argument('--run',action='store_true')
    parser.add_argument('--confirmed-environment',choices=['sunny15'])
    parser.add_argument('--resume-paused',action='store_true')
    parser.add_argument('--child-contract',type=Path,help=argparse.SUPPRESS)
    parser.add_argument('--contract-sha256',help=argparse.SUPPRESS)
    parser.add_argument('--parent-pid',type=int,help=argparse.SUPPRESS)
    parser.add_argument('--parent-start-ticks',help=argparse.SUPPRESS)
    parser.add_argument('arguments',nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    if args.child_contract is not None:
        require(not args.attach and not args.run and not args.resume_paused
            and args.confirmed_environment is None and args.contract_sha256
            and args.parent_pid is not None and args.parent_start_ticks,
            'Explicit routed fast-child identity required')
        require(original.path(args.child_contract) == CONTRACT
            and sha(CONTRACT) == args.contract_sha256, 'Fast child continuation contract changed')
        verify_parent(args.parent_pid,args.parent_start_ticks)
        arguments = args.arguments[1:] if args.arguments[:1] == ['--'] else args.arguments
        original.child_arguments(arguments)
        pins = immutable_pins();started = time.monotonic()
        with installed_cache(pins) as cache:
            original.frozen_context();validate_attachment(pins)
            quality = importlib.import_module('run_base5_quality_loop')
            try:
                with original.population.installed(quality_loop=quality):
                    return original.original_run(original.RECOVERY, arguments)
            finally:
                print(json.dumps(metrics(cache,started)),flush=True)
    require(not args.arguments and all(value is None for value in
        (args.contract_sha256,args.parent_pid,args.parent_start_ticks)), 'Unexpected fast-launch arguments')
    if args.attach:
        require(not args.resume_paused and args.confirmed_environment is None, 'Attach never starts or resumes capture')
        print(json.dumps(attach()),flush=True);return 0
    if not args.run:
        require(not args.resume_paused and args.confirmed_environment is None, 'Resume requires explicit --run')
        print(json.dumps(dict(event='local_sunny15_fast_plan_only', contract=str(CONTRACT),
            contract_present=CONTRACT.is_file(), files_written=False, captures_requested=0)))
        return 0
    require(args.confirmed_environment == 'sunny15' and args.resume_paused,
        'Explicit --run --confirmed-environment sunny15 --resume-paused required')
    started = time.monotonic();pins = immutable_pins()
    with idle_lock(CONTRACT), installed_cache(pins) as cache:
        original.frozen_context();validate_attachment(pins)
        archive = release_pause()
        print(json.dumps(dict(event='local_sunny15_fast_resume', archived_pause=archive,
            continuation_contract_sha256=sha(CONTRACT), original_contract_unchanged=True)),flush=True)
        try:
            with routed_parent():
                return original.main(['--run','--confirmed-environment','sunny15'])
        finally:
            print(json.dumps(metrics(cache,started)),flush=True)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError,ValueError,RuntimeError,subprocess.SubprocessError) as error:
        print(json.dumps(dict(event='local_sunny15_fast_blocked', error=str(error),
            fallback_allowed=False)),file=sys.stderr)
        raise SystemExit(1)
