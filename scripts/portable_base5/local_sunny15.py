#!/usr/bin/env python3
"""명시적 Sunny15 v159 복구 목록 보존. 기본은 읽기 전용 계획이다.

--prepare는 CPU 자격을 확인하고 취득 0장인 기존 준비본에 별도 계약만 추가한다.
--run은 별도 사용자 승인 경로이며 원형 438개 파일·QA·실행 신원을 바꾸지 않는다.
"""
import argparse
from contextlib import contextmanager
import fcntl
import hashlib
import importlib
import json
import os
from pathlib import Path
import runpy
import subprocess
import sys

import recovery_population as population


ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT/'dataset/morai_od/scripts'
FINAL = ROOT/'artifacts/evaluation/base5_sunny15_261017/finalization_v1'
RELEASE = FINAL/'release_v159_sunny15/release.json'
MANIFEST = FINAL/'assembled_source_v2/final_source/manifest.json'
OUTPUT = Path.home()/'morai_cap/base5_sunny15_v159_261017/collection_5k'
PARENT = SCRIPTS/'run_base5_batch_static_attitude.py'
RECOVERY = SCRIPTS/'run_base5_capture_campaign_direction_reference.py'
QUALIFICATION = FINAL/'local_population_recovery_v1/qualification.json'
QUALIFIER = Path(__file__).with_name('local_sunny15_qualification.py').resolve()
RELEASE_SHA = '9098b8959eb1fc25b4f5ce47ba6e2d638a8b68c199cf8c34fdcdc278a83ec28d'
MANIFEST_SHA = 'b320b67716b9df8b649b73635f1da40c633ecde70080dc82f04c720f872d0ebf'
SCHEMA = 'base5_local_sunny15_population_runtime_v1'
QUALIFICATION_SCHEMA = 'base5_local_population_recovery_cpu_qualification_v1'
CONTRACT = 'local_population_recovery_contract.json'
PREPARED_FILES = {'.batch.lock', '.static_attitude_batch.lock', 'batch_contract.json',
    'deficit_manifest.json', 'quota.json', 'selected_ledger.jsonl',
    'static_attitude_batch_contract.json', 'status.json'}


def require(value, message):
    if not value:
        raise ValueError(message)


def path(value):
    result = Path(value).expanduser().absolute()
    require('..' not in result.parts and all(not p.is_symlink() for p in (result,*result.parents)),
        'Exact nonsymlink local Sunny15 path required')
    return result


def sha(value):
    h = hashlib.sha256()
    with path(value).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):
            h.update(block)
    return h.hexdigest()


def read(value):
    return json.loads(path(value).read_text())


def verify_pins(values):
    require(type(values) is dict and values, 'Nonempty immutable file pins required')
    for filename, expected in values.items():
        require(type(filename) is str and Path(filename).is_absolute()
            and type(expected) is str and len(expected) == 64 and sha(filename) == expected,
            'Local Sunny15 immutable input changed: '+str(filename))


def helper_hashes():
    return {str(path(p)):sha(p) for p in (Path(__file__).absolute(),Path(population.__file__).resolve(),QUALIFIER)}


def parent_module():
    if str(SCRIPTS) not in sys.path:
        sys.path.insert(0,str(SCRIPTS))
    parent = importlib.import_module(PARENT.stem)
    require(path(parent.__file__) == PARENT and parent.RECOVERY == RECOVERY,
        'Exact original parent/recovery module identity required')
    return parent


def frozen_context(*,deep=False):
    """고정 bytes/profile을 검사한다. deep native 6-gate는 요청 때만 재생한다.

    --run의 deep 검사는 원형 parent가 한 번 수행한다. child는 그 살아 있는
    parent 신원을 입증하므로 과거 native 6-gate를 매번 다시 분석하지 않는다.
    """
    require(sha(RELEASE) == RELEASE_SHA and sha(MANIFEST) == MANIFEST_SHA,
        'Exact frozen Sunny15 v159 release/source required')
    document = read(RELEASE)
    require(len(document.get('runtime_hashes',{})) == 438, 'Exact original 438-file closure required')
    verify_pins(document['runtime_hashes'])
    parent = parent_module()
    runtime = parent.runtime_hashes()
    require(runtime and all(document['runtime_hashes'].get(name) == digest for name,digest in runtime.items())
        and document.get('bulk_ready') is True
        and document.get('environment') == 'sunny15' and document.get('qualified_environments') == ['sunny15']
        and document.get('capture_output') == str(OUTPUT)
        and document.get('bulk_training_observations') == 0
        and document.get('qualified_runtime',{}).get('runtime_profile') == parent.profiles(),
        'Original frozen qualified environment/runtime profile changed')
    if deep:
        context = parent.qualified_release(RELEASE,'sunny15',MANIFEST,OUTPUT,runtime)
        require(context.get('bulk_ready') is True and context.get('bulk_training_observations') == 0,
            'Original six-gate native/input/resume qualification required')
    return document, parent


def qualification():
    result = read(QUALIFICATION)
    require(result.get('schema') == QUALIFICATION_SCHEMA and result.get('passed') is True
        and result.get('manifest_sha256') == MANIFEST_SHA and result.get('release_sha256') == RELEASE_SHA
        and result.get('source_runtime_count') == 438
        and result.get('population_policy') == population.POLICY
        and result.get('captures_requested') == 0 and result.get('native_capture_executed') is False
        and result.get('source_inputs_unchanged') is True and result.get('native_qa_unchanged') is True
        and result.get('source_inventory',{}).get('passed') is True
        and result.get('source_inventory',{}).get('source_scenes') == 5000
        and result.get('matrix',{}).get('all_branches_selected') is True
        and type(result.get('replacement_count')) is int and 2 <= result['replacement_count'] <= 24
        and result.get('problem_00082_candidate_pass') is True
        and result.get('bounded_exhaustions') == 0 and result.get('unexpected_contract_stops') == 0,
        'Actual zero-capture Sunny15 branch/replacement CPU qualification required')
    hashes = result.get('helper_hashes',{})
    require(all(hashes.get(name) == digest for name,digest in helper_hashes().items()),
        'CPU qualification must pin the actual wrapper, population guard and qualifier')
    verify_pins(hashes)
    return result


def original_guards():
    parent_path = path(OUTPUT/'static_attitude_batch_contract.json')
    batch_path = path(OUTPUT/'batch_contract.json')
    saved, batch = read(parent_path),read(batch_path)
    require(saved.get('manifest') == str(MANIFEST) and saved.get('manifest_sha256') == MANIFEST_SHA
        and saved.get('environment') == 'sunny15' and saved.get('indices') == list(range(5000))
        and saved.get('chunk_size') == 16 and saved.get('no_automatic_completion') is True
        and saved.get('legacy_batch_contract') == batch
        and batch.get('manifest') == MANIFEST_SHA and batch.get('indices') == list(range(5000))
        and batch.get('chunk_size') == 16 and batch.get('bulk') is True,
        'Original prepared 5K parent/batch guards differ')
    release = read(RELEASE)
    authority = saved.get('qualified_release',{})
    runtime = parent_module().runtime_hashes()
    require(saved.get('runtime_hashes') == runtime
        and all(release['runtime_hashes'].get(name) == digest for name,digest in runtime.items())
        and saved.get('runtime_profile') == release['qualified_runtime']['runtime_profile']
        and authority.get('path') == str(RELEASE) and authority.get('sha256') == RELEASE_SHA,
        'Original prepared qualified runtime changed')
    verify_pins(saved['input_hashes'])
    return {str(parent_path):sha(parent_path),str(batch_path):sha(batch_path)}


def expected_contract():
    qualification()
    return dict(schema=SCHEMA,environment='sunny15',manifest=str(MANIFEST),manifest_sha256=MANIFEST_SHA,
        release=str(RELEASE),release_sha256=RELEASE_SHA,output=str(OUTPUT),
        population_policy=population.POLICY,helper_hashes=helper_hashes(),
        cpu_qualification=str(QUALIFICATION),cpu_qualification_sha256=sha(QUALIFICATION),
        original_guard_hashes=original_guards(),source_runtime_count=438,
        original_native_qa_unchanged=True,original_recovery_bounds_512_8=True,
        regular_static_attitude_unchanged=True,strict_direction_reference_unchanged=True,
        attached_before_any_native_capture=True,new_live_smoke_executed=False)


def native_zero():
    allowed = PREPARED_FILES | ({CONTRACT} if (OUTPUT/CONTRACT).exists() else set())
    require(path(OUTPUT).is_dir() and {p.name for p in OUTPUT.iterdir()} == allowed,
        'Only the exact native-zero prepared output may receive the new sidecar')
    require(all(path(p).is_file() for p in OUTPUT.iterdir()), 'Native-zero output must have no child directories')
    status,quota = read(OUTPUT/'status.json'),read(OUTPUT/'quota.json')
    require(status.get('stage') == 'prepared_waiting_for_resume' and status.get('requested') == 5000
        and all(status.get(k) == 0 for k in ('finished','accepted_candidate','review','failed'))
        and quota == dict(target=5000,accepted=0,deficit=0,pending=5000,cells=[])
        and (OUTPUT/'selected_ledger.jsonl').read_bytes() == b'',
        'Cannot retrofit population recovery to captured, reviewed or halted output')


@contextmanager
def preparation_locks():
    handles = []
    try:
        for name in ('.static_attitude_batch.lock','.batch.lock'):
            handle = path(OUTPUT/name).open('rb');handles.append(handle)
            fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
        yield
    finally:
        for handle in reversed(handles):
            handle.close()


def prepare():
    frozen_context(deep=True)
    with preparation_locks():
        filename = path(OUTPUT/CONTRACT)
        expected = expected_contract()
        native_zero()
        if filename.exists():
            require(read(filename) == expected, 'Existing local recovery sidecar differs')
            return filename
        with filename.open('x') as stream:
            json.dump(expected,stream,ensure_ascii=False,indent=2);stream.write('\n')
    return filename


def validate_contract(filename):
    require(path(filename) == OUTPUT/CONTRACT, 'Unexpected local Sunny15 sidecar')
    require(read(filename) == expected_contract(), 'Local recovery sidecar or qualified recipe changed')
    return read(filename)


def child_arguments(arguments):
    parser = argparse.ArgumentParser(add_help=False,allow_abbrev=False)
    parser.add_argument('--manifest',required=True);parser.add_argument('--indices',required=True)
    parser.add_argument('--output',required=True);parser.add_argument('--seed-capture')
    parser.add_argument('--enable-final-direction-reference',action='store_true')
    parser.add_argument('--resume',action='store_true')
    require(all(arguments.count(name) == 1 for name in ('--manifest','--indices','--output',
        '--enable-final-direction-reference','--resume'))
        and arguments.count('--seed-capture') <= 1,'Exact single-cell original recovery arguments required')
    try:
        args = parser.parse_args(arguments)
        index = int(args.indices)
    except (SystemExit,ValueError) as error:
        raise ValueError('Unknown or malformed local recovery arguments') from error
    require(args.indices == str(index) and 0 <= index < 5000
        and path(args.manifest) == MANIFEST and args.enable_final_direction_reference and args.resume,
        'Recovery must retain the exact original full source and bounded cell')
    entry = read(MANIFEST)['scenes'][index]
    expected = OUTPUT/('chunk_%04d' % (index//16))/'recovery'/entry['id']
    require(path(args.output) == expected, 'Recovery output escaped its original quota/chunk')
    if args.seed_capture is not None:
        require(path(args.seed_capture) == expected.parent/(expected.name+'_seed.json'),
            'Recovery seed escaped its original quota')
    return args


def parent_identity():
    return os.getpid(),Path('/proc/self/stat').read_text().rsplit(')',1)[1].split()[19]


def verify_parent(pid,ticks):
    require(pid == os.getppid(), 'Recovery child must belong to the invoking parent')
    proc = Path('/proc')/str(pid)
    argv = [os.fsdecode(v) for v in (proc/'cmdline').read_bytes().split(b'\0') if v]
    index = 1
    flags = []
    while len(argv)>index and argv[index].startswith('-'):
        require(argv[index] in ('-B','-u') and argv[index] not in flags,
            'Unknown local Sunny15 parent Python flags')
        flags.append(argv[index]);index += 1
    script = Path(argv[index]) if len(argv)>index else Path('/invalid-parent')
    if not script.is_absolute():
        script = (proc/'cwd').resolve()/script
    require(proc.stat().st_uid == os.getuid()
        and (proc/'stat').read_text().rsplit(')',1)[1].split()[19] == ticks
        and len(argv)>index and path(script) == path(__file__)
        and '--run' in argv[index+1:] and '--child-contract' not in argv,
        'Actual approved local Sunny15 parent argv/start identity required')


@contextmanager
def routed_children(contract):
    original = subprocess.Popen
    pid,ticks = parent_identity()
    def routed(command,*args,**kwargs):
        if isinstance(command,(str,bytes)):
            require(RECOVERY.name not in os.fsdecode(command),'Shell recovery invocation is not qualified')
            return original(command,*args,**kwargs)
        argv = [os.fsdecode(v) for v in command]
        index = 2 if len(argv)>1 and argv[1]=='-B' else 1
        candidate = argv[index] if len(argv)>index else ''
        if any(Path(value).name == RECOVERY.name for value in argv):
            require(candidate == str(RECOVERY) and argv[0] == sys.executable and not kwargs.get('shell'),
                'Unknown recovery Python/script invocation is not qualified')
            child_arguments(argv[index+1:])
            command = [sys.executable,'-B',str(Path(__file__).absolute()),
                '--child-contract',str(contract),'--contract-sha256',sha(contract),
                '--parent-pid',str(pid),'--parent-start-ticks',ticks,'--',*argv[index+1:]]
        return original(command,*args,**kwargs)
    subprocess.Popen = routed
    try:
        yield
    finally:
        changed = subprocess.Popen is not routed
        subprocess.Popen = original
        require(not changed,'Local recovery Popen router ownership changed')


def original_run(target,arguments):
    previous = sys.argv
    try:
        sys.argv = [str(target),*arguments]
        try:
            runpy.run_path(str(target),run_name='__main__')
        except SystemExit as error:
            return error.code if isinstance(error.code,int) else (0 if error.code is None else 1)
        return 0
    finally:
        sys.argv = previous


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--prepare',action='store_true');mode.add_argument('--run',action='store_true')
    parser.add_argument('--confirmed-environment',choices=['sunny15'])
    parser.add_argument('--child-contract',type=Path,help=argparse.SUPPRESS)
    parser.add_argument('--contract-sha256',help=argparse.SUPPRESS)
    parser.add_argument('--parent-pid',type=int,help=argparse.SUPPRESS)
    parser.add_argument('--parent-start-ticks',help=argparse.SUPPRESS)
    parser.add_argument('arguments',nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    if args.child_contract is not None:
        require(not args.prepare and not args.run and args.contract_sha256
            and args.parent_pid is not None and args.parent_start_ticks,
            'Explicit routed-child identity required')
        verify_parent(args.parent_pid,args.parent_start_ticks)
        require(sha(args.child_contract) == args.contract_sha256,'Routed sidecar changed')
        frozen_context();validate_contract(args.child_contract)
        arguments = args.arguments[1:] if args.arguments[:1] == ['--'] else args.arguments
        child_arguments(arguments)
        quality = importlib.import_module('run_base5_quality_loop')
        with population.installed(quality_loop=quality):
            return original_run(RECOVERY,arguments)
    require(not args.arguments and all(value is None for value in
        (args.contract_sha256,args.parent_pid,args.parent_start_ticks)), 'Unexpected original/child arguments')
    if args.prepare:
        print(json.dumps(dict(prepared_contract=str(prepare()),captures_requested=0,native_capture_executed=False)))
        return 0
    frozen_context()
    if not args.run:
        print(json.dumps(dict(event='local_sunny15_plan_only',output=str(OUTPUT),
            contract=str(OUTPUT/CONTRACT),contract_present=(OUTPUT/CONTRACT).is_file(),
            files_written=False,captures_requested=0,native_capture_executed=False)))
        return 0
    require(args.confirmed_environment == 'sunny15','Explicit actual MORAI Sunny15 confirmation required')
    contract = path(OUTPUT/CONTRACT);validate_contract(contract)
    with contract.open('rb') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        validate_contract(contract)
        with routed_children(contract):
            return original_run(PARENT,['--manifest',str(MANIFEST),'--output',str(OUTPUT),
                '--environment','sunny15','--confirmed-environment','sunny15',
                '--qualified-release',str(RELEASE),'--limit','5000','--chunk-size','16',
                '--resume','--run','--approve-bulk-capture'])


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError,ValueError,RuntimeError,subprocess.SubprocessError) as error:
        print(json.dumps(dict(event='local_sunny15_blocked',error=str(error),fallback_allowed=False)),file=sys.stderr)
        raise SystemExit(1)
