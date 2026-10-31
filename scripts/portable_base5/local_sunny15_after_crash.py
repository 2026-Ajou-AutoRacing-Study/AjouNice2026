#!/usr/bin/env python3
"""MORAI 종료 후 안전한 승인분만 새 Sunny15 출력으로 인계한다.

기본은 읽기 전용 계획, --prepare도 CPU 작업뿐이다. 실제 촬영은 사용자가
--run --confirmed-environment sunny15로 시작한다. 과거 Park 실패는 고치지 않는다.
"""
import sys
sys.dont_write_bytecode = True

import argparse
from contextlib import contextmanager
import importlib
import json
import os
from pathlib import Path
import subprocess
import time

import local_sunny15 as original
import local_sunny15_fast as fast
import local_sunny15_crash_import as handoff


SCHEMA = 'base5_sunny15_explicit_crash_continuation_v1'
OUTPUT = handoff.NEW
CONTRACT = OUTPUT/'after_crash_contract.json'


def require(value, message):
    original.require(value, message)


def helper_pins():
    return {str(original.path(path)):fast.sha(path) for path in
        (Path(__file__).absolute(), Path(handoff.__file__).resolve(),
         Path(fast.__file__).resolve(), Path(fast.cache_module.__file__).resolve())}


@contextmanager
def installed_cache():
    pins = fast.immutable_pins()
    cache = fast.cache_module.VerifiedInputCache(pins, forbidden_roots=(original.OUTPUT, OUTPUT))
    if str(original.SCRIPTS) not in sys.path:
        sys.path.insert(0, str(original.SCRIPTS))
    with cache.installed(scripts_dir=original.SCRIPTS):
        yield cache


@contextmanager
def qualified_destination(parent):
    """원래 release를 원래 목적지로 전수 확인하고 새 목적지만 명시적으로 인계한다."""
    saved = parent.qualified_release
    def checked(release, environment, manifest, output, runtime):
        require(original.path(output) == OUTPUT and original.path(release) == original.RELEASE
            and original.path(manifest) == original.MANIFEST and environment == 'sunny15',
            'Only the exact recorded crash continuation destination is authorized')
        # Do not rewrite a release or lie to its verifier about its original path.
        return saved(release, environment, manifest, original.OUTPUT, runtime)
    parent.qualified_release = checked
    try:
        yield
    finally:
        changed = parent.qualified_release is not checked
        parent.qualified_release = saved
        require(not changed, 'Qualified destination adapter ownership changed')


def arguments(*, run=False):
    return argparse.Namespace(manifest=original.MANIFEST, output=OUTPUT, environment='sunny15',
        confirmed_environment='sunny15' if run else None, qualified_release=original.RELEASE,
        limit=5000, chunk_size=16, run=run, prepare_only=not run,
        approve_bulk_capture=run, resume=True, stop_after_chunks=0)


def initialize_profile(parent):
    """새 import 출력에만 원형 run()이 다시 전수 대조할 정확한 초기 계약을 둔다."""
    guard = OUTPUT/parent.GUARD
    require(not guard.exists() and not guard.is_symlink(), 'New continuation profile already exists')
    deck = original.read(original.MANIFEST)
    indices = list(range(5000))
    runtime = parent.runtime_hashes()
    authority = parent.qualified_release(original.RELEASE, 'sunny15', original.MANIFEST, OUTPUT, runtime)
    inputs = parent.source_pins(original.MANIFEST, deck, indices)
    legacy = parent.batch.campaign.contract(original.MANIFEST, indices, 2)
    legacy.update(batch_driver_sha256=parent.sha256(parent.batch.__file__), chunk_size=16, bulk=True,
        replenishment_driver_sha256=parent.sha256(original.SCRIPTS/'base5_campaign_replenishment.py'))
    expected = dict(schema=parent.SCHEMA, policy=parent.POLICY, manifest=str(original.MANIFEST),
        manifest_sha256=original.MANIFEST_SHA, environment='sunny15', indices=indices,
        chunk_size=16, runtime_profile=parent.profiles(), runtime_hashes=runtime,
        input_hashes=inputs, qualified_release=authority, legacy_batch_contract=legacy,
        storage_gate=dict(minimum_gib=parent.storage_budget_gib(5000), scope='5K bulk',
            conservative_prototype_bytes_per_frame=100_000_000, prototype_maximum_frames=64),
        prototype_only=False, no_automatic_completion=True, bulk_training_observations_at_qualification=0)
    fast.write_new(guard, expected)
    # Actual frozen parent must accept the profile/import; no mocked receipt is used.
    require(parent.run(arguments()) == 0, 'Original CPU prepare rejected the continuation')
    require(original.read(guard) == expected, 'Original parent changed its exact profile')


def recipe():
    return dict(schema=SCHEMA, environment='sunny15', old_output=str(original.OUTPUT),
        output=str(OUTPUT), manifest=str(original.MANIFEST), manifest_sha256=original.MANIFEST_SHA,
        release=str(original.RELEASE), release_sha256=original.RELEASE_SHA,
        imported_prefix_sha256=fast.sha(handoff.IMPORT), handoff_sha256=fast.sha(handoff.HANDOFF),
        helper_pins=helper_pins(), parent_guard_sha256=fast.sha(OUTPUT/'static_attitude_batch_contract.json'),
        batch_guard_sha256=fast.sha(OUTPUT/'batch_contract.json'), native_qa_unchanged=True,
        recovery_roster_preserved=True, original_512_geometry_8_layout_bounds=True,
        old_failed_park_reclassified=False, unsafe_chunk_imported=False)


def validate_contract():
    require(original.read(CONTRACT) == recipe(), 'Crash continuation helper, import or profile changed')
    old = original.read(original.OUTPUT/'status.json')
    require(old.get('stage') == 'halted' and old.get('error') == 'Batch final Park unverified',
        'Historical crash status must remain unchanged, never marked paused/safe')


def prepare():
    require(not OUTPUT.exists(), 'Continuation already exists; never re-prepare or overwrite it')
    with installed_cache():
        original.frozen_context(deep=True)
        result = handoff.prepare()
        parent = original.parent_module()
        with qualified_destination(parent):
            initialize_profile(parent)
        fast.write_new(CONTRACT, recipe())
        validate_contract()
        handoff.verify()
    return dict(result, event='sunny15_crash_continuation_prepared', capture_output=str(OUTPUT),
        captures_requested=0, native_capture_executed=False)


def child_arguments(values):
    parser = argparse.ArgumentParser(add_help=False, allow_abbrev=False)
    parser.add_argument('--manifest', required=True);parser.add_argument('--indices', required=True)
    parser.add_argument('--output', required=True);parser.add_argument('--seed-capture')
    parser.add_argument('--enable-final-direction-reference', action='store_true')
    parser.add_argument('--resume', action='store_true')
    required = ('--manifest','--indices','--output','--enable-final-direction-reference','--resume')
    require(all(values.count(flag) == 1 for flag in required) and values.count('--seed-capture') <= 1,
        'Exact bounded single-cell recovery flags required')
    try:
        args = parser.parse_args(values);index = int(args.indices)
    except (SystemExit, ValueError) as error:
        raise ValueError('Malformed bounded crash-continuation recovery') from error
    require(args.indices == str(index) and 0 <= index < 5000 and args.resume
        and args.enable_final_direction_reference and original.path(args.manifest) == original.MANIFEST,
        'Original full source and strict single-cell recovery required')
    entry = original.read(original.MANIFEST)['scenes'][index]
    expected = OUTPUT/('chunk_%04d' % (index//16))/'recovery'/entry['id']
    require(original.path(args.output) == expected, 'Recovery escaped the new quota slot')
    require(args.seed_capture is None or original.path(args.seed_capture)
        == expected.parent/(expected.name+'_seed.json'), 'Recovery seed escaped the new quota slot')
    return args


def verify_parent(pid, ticks):
    require(pid == os.getppid(), 'Recovery must belong to its actual invoking parent')
    record = fast.parent_record(pid)
    argv = record['argv'];index = 1;flags = []
    while len(argv)>index and argv[index].startswith('-'):
        require(argv[index] in ('-B','-u') and argv[index] not in flags, 'Unknown interpreter flags')
        flags.append(argv[index]);index += 1
    script = Path(argv[index]) if len(argv)>index else Path('/invalid-continuation-parent')
    if not script.is_absolute():script = record['cwd']/script
    require(record['uid'] == os.getuid() and record['ticks'] == ticks
        and original.path(script) == original.path(__file__)
        and argv[index+1:] == ['--run','--confirmed-environment','sunny15'],
        'Exact crash-continuation parent uid/argv/start identity required')


@contextmanager
def routed_children():
    saved = subprocess.Popen
    pid,ticks = original.parent_identity()
    def routed(command,*args,**kwargs):
        if isinstance(command,(str,bytes)):
            require(original.RECOVERY.name not in os.fsdecode(command), 'Shell recovery is not qualified')
            return saved(command,*args,**kwargs)
        argv = [os.fsdecode(value) for value in command]
        index = 2 if len(argv)>1 and argv[1]=='-B' else 1
        if any(Path(value).name == original.RECOVERY.name for value in argv):
            require(len(argv)>index and argv[index] == str(original.RECOVERY)
                and argv[0] == sys.executable and not kwargs.get('shell'), 'Unknown strict recovery invocation')
            child_arguments(argv[index+1:])
            command = [sys.executable,'-B',str(Path(__file__).absolute()),
                '--child-contract',str(CONTRACT),'--contract-sha256',fast.sha(CONTRACT),
                '--parent-pid',str(pid),'--parent-start-ticks',ticks,'--',*argv[index+1:]]
        return saved(command,*args,**kwargs)
    subprocess.Popen = routed
    try:yield
    finally:
        changed = subprocess.Popen is not routed
        subprocess.Popen = saved
        require(not changed, 'Crash recovery router ownership changed')


def release_pause_if_present():
    state = original.read(OUTPUT/'status.json')
    require(state.get('stage') in ('prepared_waiting_for_resume','paused'),
        'Only a prepared or safely paused NEW continuation may run; never bypass another halt')
    if state['stage'] == 'paused':
        from contextlib import ExitStack
        with ExitStack() as stack:
            stack.enter_context(fast.idle_lock(OUTPUT/'.static_attitude_batch.lock'))
            stack.enter_context(fast.idle_lock(OUTPUT/'.batch.lock'))
            safety = fast.resume_evidence(OUTPUT, stack)
            marker = fast.owned(OUTPUT/'PAUSE', False)
            require(fast.sha(OUTPUT/'status.json') == safety['parent_status_sha256'], 'Paused parent changed')
            destination = marker.with_name('PAUSE.resumed_'+str(time.time_ns()))
            require(not destination.exists() and not destination.is_symlink(), 'Pause archive already exists')
            os.rename(marker, destination)
    else:
        require(not any((OUTPUT/name).exists() or (OUTPUT/name).is_symlink()
            for name in ('STOP','PAUSE')),
            'Prepared continuation has a stop/pause marker; inspect before running')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--prepare', action='store_true');mode.add_argument('--run', action='store_true')
    parser.add_argument('--confirmed-environment', choices=['sunny15'])
    parser.add_argument('--child-contract', type=Path, help=argparse.SUPPRESS)
    parser.add_argument('--contract-sha256', help=argparse.SUPPRESS)
    parser.add_argument('--parent-pid', type=int, help=argparse.SUPPRESS)
    parser.add_argument('--parent-start-ticks', help=argparse.SUPPRESS)
    parser.add_argument('arguments', nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    if args.child_contract is not None:
        require(not args.prepare and not args.run and args.confirmed_environment is None
            and args.contract_sha256 and args.parent_pid is not None and args.parent_start_ticks,
            'Explicit routed recovery identity required')
        require(original.path(args.child_contract) == CONTRACT
            and fast.sha(CONTRACT) == args.contract_sha256, 'Recovery continuation contract changed')
        verify_parent(args.parent_pid,args.parent_start_ticks)
        values = args.arguments[1:] if args.arguments[:1] == ['--'] else args.arguments
        child_arguments(values)
        with installed_cache():
            original.frozen_context();validate_contract()
            quality = importlib.import_module('run_base5_quality_loop')
            with original.population.installed(quality_loop=quality):
                return original.original_run(original.RECOVERY, values)
    require(not args.arguments and all(value is None for value in
        (args.contract_sha256,args.parent_pid,args.parent_start_ticks)), 'Unexpected continuation arguments')
    if args.prepare:
        require(args.confirmed_environment is None, 'Prepare is CPU-only, not live environment confirmation')
        print(json.dumps(prepare(),ensure_ascii=False),flush=True);return 0
    if not args.run:
        require(args.confirmed_environment is None, 'Environment confirmation requires --run')
        print(json.dumps(dict(event='sunny15_crash_continuation_plan', output=str(OUTPUT),
            prepared=CONTRACT.is_file(), files_written=False, captures_requested=0),ensure_ascii=False));return 0
    require(args.confirmed_environment == 'sunny15', 'Confirm actual Sunny15 UI before capture')
    with fast.idle_lock(CONTRACT), installed_cache():
        original.frozen_context();validate_contract();handoff.verify()
        release_pause_if_present()
        parent = original.parent_module()
        print(json.dumps(dict(event='sunny15_crash_continuation_start', output=str(OUTPUT),
            old_failed_park_preserved=True, live_connections_and_park_rechecked_by_native_collector=True)),flush=True)
        with qualified_destination(parent), routed_children():
            return parent.run(arguments(run=True))


if __name__ == '__main__':
    try:raise SystemExit(main())
    except (OSError,ValueError,RuntimeError,subprocess.SubprocessError) as error:
        print(json.dumps(dict(event='sunny15_crash_continuation_blocked',error=str(error),
            fallback_allowed=False)),file=sys.stderr)
        raise SystemExit(1)
