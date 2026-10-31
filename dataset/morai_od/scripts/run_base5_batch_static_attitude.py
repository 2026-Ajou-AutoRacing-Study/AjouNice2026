#!/usr/bin/env python3
"""독립 chunk의 새 정적 지면 QA와 기존 엄격 recovery를 명시해서 연결한다.

기본은 CPU prepare-only다. 64장 이하 --run은 prototype이며, 5K --run은
별도 환경 release 검증기가 실제 native/input/CPU/resume 근거를 재검증한
경우에만 허용한다. 기존 batch/quality/collector 파일은 수정하지 않는다.
"""
import argparse
import copy
import fcntl
import importlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

import base5_front_release_inputs as pins
import base5_layout_split as layout
from base5_capture_safety import atomic_json, sha256
import run_base5_batch_campaign as batch
import run_base5_capture_static_attitude as attitude
from verify_base5_completed_resume import snapshot as completed_snapshot


SCHEMA = 'base5_static_attitude_batch_runtime_v1'
POLICY = 'independent-static-attitude-chunks-with-strict-spatial-recovery-v1'
SCRIPTS = Path(__file__).resolve().parent
CHILD = SCRIPTS/'run_base5_capture_static_attitude.py'
RECOVERY = SCRIPTS/'run_base5_capture_campaign_direction_reference.py'
SPATIAL_RECOVERY = SCRIPTS/'run_base5_capture_campaign_spatial.py'
GUARD = 'static_attitude_batch_contract.json'
RECOVERY_GUARD = 'static_attitude_parent_recovery_contract.json'
EXECUTION_SOURCE = 'execution_source'
EXECUTION_RECEIPT = 'execution_source_receipt.json'
GATES = ('native_exact', 'independent_shadow', 'training_inputs', 'cpu_regression',
         'partial_resume', 'completed_resume')
ENVIRONMENTS = ('sunny11', 'sunny13', 'sunny15', 'foggy11', 'foggy13', 'foggy15')
BULK_HEADROOM_GIB = 55
PROTOTYPE_HEADROOM_GIB = 8


def read(path):
    return json.loads(Path(path).read_text())


def require(condition, message):
    if not condition:
        raise ValueError(message)


def profiles():
    """Profile identity is independent of each child's subset manifest SHA."""
    return dict(policy=POLICY,
        regular_chunk=dict(entrypoint=str(CHILD), entrypoint_sha256=sha256(CHILD),
            setup_profile='static-ready', enable_static_surface_attitude=True,
            allow_static_pedestrians=True, maximum_shared_rows=16,
            manifest_layout='byte-identical sibling execution_source; original chunk source preserved',
            execution='independent subprocess per chunk; network reload and Park per scene'),
        isolated_recovery=dict(entrypoint=str(RECOVERY), entrypoint_sha256=sha256(RECOVERY),
            spatial_entrypoint=str(SPATIAL_RECOVERY), spatial_entrypoint_sha256=sha256(SPATIAL_RECOVERY),
            native_collector=str(attitude.SOURCE), native_collector_sha256=sha256(attitude.SOURCE),
            setup_profile='baseline', enable_static_surface_attitude=False,
            allow_static_pedestrians=False, spatial_geometry_adapter=True,
            enable_final_direction_reference=True, final_yaw_original_pivot_static_reference=True,
            actual_compound_corner_and_arrival_support_envelope=True, original_native_qa_thresholds_unchanged=True,
            execution='independent subprocess per cell; fresh native direction references and stricter unchanged QA'))


def runtime_hashes():
    return pins.collect_runtime(entrypoints=(Path(__file__).name, CHILD.name, RECOVERY.name))


def storage_budget_gib(count):
    require(type(count) is int and 1 <= count <= 5000, 'Bounded selected count required for storage gate')
    # <=64 frames: at most 6.4 GB at the conservative 100 MB/frame estimate,
    # rounded up to 8 GiB with safety headroom. This is not a 5K budget.
    return BULK_HEADROOM_GIB if count > 64 else PROTOTYPE_HEADROOM_GIB


def source_pins(manifest, deck, indices):
    """Exact selected input pins; full-deck authority also supplies full input closure."""
    result = {str(manifest):sha256(manifest)}
    for index in indices:
        entry = deck['scenes'][index]
        require(sha256(entry['scenario']) == entry['scenario_sha256'], 'Selected source payload changed')
        meta = read(entry['metadata'])
        require(meta['split'] == entry['split'], 'Selected capture split differs')
        result[str(Path(entry['scenario']).resolve())] = entry['scenario_sha256']
        result[str(Path(entry['metadata']).resolve())] = sha256(entry['metadata'])
    for value in (deck.get('config', {}), deck.get('split_overlay', {})):
        for path, expected in pins._references(value):
            resolved = Path(path).resolve(strict=True)
            require(sha256(resolved) == expected, 'Declared source reference changed')
            result[str(resolved)] = expected
    project = SCRIPTS.parents[2]
    for path in (attitude.candidate.gen.geo.DEFAULT_MGEO/'link_set.json',
                 attitude.candidate.gen.geo.DEFAULT_MGEO/'lane_boundary_set.json',
                 project/'configs/sensors/Sensor_competition_local.json',
                 project/'configs/network/ego_network_mr_base5_savesensordata_collision_local.json'):
        result[str(path.resolve())] = sha256(path)
    return result


def qualified_release(release, environment, manifest, output, runtime):
    """Future finalizer must deeply verify actual evidence; no JSON-only approval."""
    require(release is not None, '5K requires a separately qualified environment release')
    try:
        module = importlib.import_module('finalize_base5_environment_release')
        verifier = getattr(module, 'qualified_release_context', None)
    except ImportError as error:
        raise ValueError('Environment release verifier is unavailable; bulk remains forbidden') from error
    require(callable(verifier), 'Deep environment release verifier is unavailable; bulk remains forbidden')
    context = verifier(Path(release).resolve(strict=True), environment, manifest)
    require(context.get('passed') is True and context.get('bulk_ready') is True
            and context.get('environment') == environment
            and context.get('manifest_sha256') == sha256(manifest)
            and Path(context['capture_output']).resolve() == output,
            'Qualified release source/environment/output binding differs')
    require(context.get('runtime_profile') == profiles(), 'Qualified chunk/recovery runtime profiles differ')
    require(type(context.get('native_exact_observations')) is int
            and 32 <= context['native_exact_observations'] <= 48
            and context.get('bulk_training_observations') == 0,
            '32..48 exact pilot observations are not 5000 collected training observations')
    require(all(context.get('runtime_hashes', {}).get(path) == value for path, value in runtime.items()),
            'Qualified release does not pin the complete new driver/runtime closure')
    pins.verify_hashes(context['runtime_hashes'], 'Qualified environment runtime')
    pins.verify_hashes(context['input_hashes'], 'Qualified environment inputs')
    gates = context.get('gate_evidence', {})
    require(set(gates) == set(GATES), 'Every native/shadow/input/CPU/resume evidence gate is required')
    for name in GATES:
        gate = gates[name]
        require(gate.get('passed') is True and Path(gate['path']).is_file()
                and sha256(gate['path']) == gate['sha256'], 'Qualified evidence changed: '+name)
    return dict(path=str(Path(release).resolve()), sha256=sha256(release),
        verifier_path=str(Path(module.__file__).resolve()), verifier_sha256=sha256(module.__file__),
        runtime_profile=profiles(), input_hashes=context['input_hashes'],
        gate_evidence=copy.deepcopy(gates), native_exact_observations=context['native_exact_observations'],
        bulk_training_observations=0, bulk_ready=True)


def regular_command(manifest, root, count):
    require(type(count) is int and 1 <= count <= 16, 'Independent native chunk must contain 1..16 scenes')
    return [sys.executable, str(CHILD), '--enable-static-surface-attitude', '--allow-static-pedestrians',
        '--setup-profile', 'static-ready', '--manifest', str(manifest), '--output', str(root),
        '--shared-ros-batch', '--limit', str(count)]


def strict_recovery_command(manifest, index, root, seed=None):
    command = [sys.executable, str(RECOVERY), '--enable-final-direction-reference',
        '--manifest', str(manifest), '--indices', str(index),
        '--output', str(root), '--resume']
    if seed is not None:
        command.extend(['--seed-capture', str(seed)])
    return command


def execution_source_contract(manifest, root):
    """원형 child의 source/output 분리를 만족하는 exact-byte 실행본의 계약."""
    manifest, root = Path(manifest).resolve(strict=True), Path(root)
    require(not root.is_symlink(), 'Native child output cannot be a symlink')
    root = root.resolve()
    directory = root.parent/EXECUTION_SOURCE
    execution = directory/'manifest.json'
    receipt = root.parent/EXECUTION_RECEIPT
    require(not directory.is_symlink() and not execution.is_symlink() and not receipt.is_symlink(),
            'Execution source or receipt cannot be a symlink')
    require(manifest != execution and root != directory and directory not in root.parents,
            'Original, execution source and native output must remain separate')
    expected = dict(schema='base5_static_attitude_execution_source_v1', policy=POLICY,
        driver_sha256=sha256(__file__), original_manifest=str(manifest),
        original_manifest_sha256=sha256(manifest), execution_manifest=str(execution),
        execution_manifest_sha256=sha256(manifest), capture_output=str(root),
        exact_bytes_required=True, original_source_preserved=True, scene_payloads_modified=False)
    return execution, receipt, expected


def verify_execution_source(manifest, root):
    """재개·release 대조는 누락된 실행본이나 receipt를 재생성하지 않는다."""
    execution, receipt, expected = execution_source_contract(manifest, root)
    require(execution.is_file() and receipt.is_file(), 'Complete original execution source and receipt required')
    require(sorted(execution.parent.iterdir()) == [execution], 'Execution source directory inventory changed')
    require(read(receipt) == expected and sha256(execution) == expected['execution_manifest_sha256']
            and execution.read_bytes() == Path(manifest).read_bytes(),
            'Execution source bytes, original source or receipt changed')
    return execution


def prepare_execution_source(manifest, root):
    """새 chunk만 복사한다. source 내용/경로와 기존 raw는 절대 수정하지 않는다."""
    execution, receipt, expected = execution_source_contract(manifest, root)
    if execution.parent.exists() or receipt.exists():
        return verify_execution_source(manifest, root)
    payload = Path(manifest).read_bytes()
    require(sha256(manifest) == expected['original_manifest_sha256'], 'Original chunk changed before execution copy')
    execution.parent.mkdir(parents=True, exist_ok=False)
    with execution.open('xb') as handle:
        handle.write(payload)
    require(sha256(manifest) == sha256(execution) == expected['original_manifest_sha256'],
            'Original chunk changed while creating exact execution copy')
    atomic_json(receipt, expected)
    return verify_execution_source(manifest, root)


def verify_regular(root, manifest):
    verify_execution_source(manifest, root)
    contract = attitude.configuration(manifest, True, True, 'static-ready')
    require(read(root/'static_surface_attitude_runtime_contract.json') == contract,
            'Regular child did not use the exact registered attitude profile')
    session = read(root/'session_contract.json')
    require(session.get('runner_options', {}).get('static_surface_attitude_adapter') == contract,
            'Regular child session omitted or changed its attitude profile')
    for line in (root/'ledger.jsonl').read_text().splitlines():
        row = json.loads(line)
        if row.get('status') in ('accepted_candidate', 'review'):
            attitude.verify_capture(row['capture_root'], contract)


def parent_recovery_guard(root):
    root = Path(root)
    return root.parent/(root.name+'_'+RECOVERY_GUARD)


def verify_strict_recovery(root, parent_contract):
    require(read(parent_recovery_guard(root)) == parent_contract, 'Strict recovery parent profile contract changed')
    for report in root.rglob('report.json'):
        data = read(report)
        if 'attempts' not in data:
            continue
        for attempt in data['attempts']:
            if not attempt.get('capture_root') or attempt.get('kind') == 'static_rigid_raw_reaudit':
                continue
            # A reused initial chunk review legitimately has the regular profile.
            # New recovery/calibration subprocesses must keep the strict profile.
            if attempt.get('kind', '').startswith('seeded_batch_'):
                seed = root.parent/(root.name+'_seed.json')
                require(seed.is_file() and attempt.get('reused_raw') is True
                        and Path(attempt.get('seed_source', '')).resolve() == seed.resolve(),
                        'Only the actual original seed may retain the regular profile')
                original = read(seed)
                require(Path(attempt['capture_root']).resolve() == Path(original['capture_root']).resolve(),
                        'Seeded recovery names another raw session')
                verify_regular(Path(original['capture_root']), Path(original['manifest']))
                continue
            session = read(Path(attempt['capture_root'])/'session_contract.json')
            options = session.get('runner_options', {})
            require('static_surface_attitude_adapter' not in options and 'optimized_adapter' not in options,
                    'Strict recovery silently changed native QA/setup profile')
    direction = importlib.import_module(RECOVERY.stem)
    proof = direction.verify_campaign(root, Path(parent_contract['source_manifest']), [parent_contract['index']])
    require(proof.get('passed') is True and proof.get('native_qa_unchanged') is True,
            'Fresh direction-reference/native-support proof failed')


def run(args):
    manifest = Path(args.manifest).resolve(strict=True)
    raw_output = Path(args.output).expanduser()
    require(not raw_output.is_symlink(), 'Output cannot be a symlink')
    output = raw_output.resolve()
    deck = read(manifest)
    require(deck.get('count') == len(deck.get('scenes', [])) and deck['scenes'], 'Explicit nonempty source deck required')
    require(type(args.limit) is int and 1 <= args.limit <= 5000, 'Selected limit must be 1..5000')
    require(type(args.chunk_size) is int and 1 <= args.chunk_size <= 16, 'Chunk size must be 1..16')
    require(not (args.run and getattr(args, 'prepare_only', False)), 'Choose --run or CPU prepare-only')
    environment = deck['config'].get('environment_tag')
    require(environment in ENVIRONMENTS and args.environment == environment, 'Explicit source environment mismatch')
    require(output != manifest.parent and manifest.parent not in output.parents, 'Output must be outside frozen source')
    require(not output.exists() or args.resume, 'Existing output requires explicit --resume')
    require(not output.exists() or (output/GUARD).is_file(), 'Cannot attach new profile to any old output')
    layout.validate_manifest_overlay(deck, allow_subset=True)
    indices = list(range(min(args.limit, len(deck['scenes']))))
    bulk = len(indices) > 64
    runtime = runtime_hashes()
    authority = None
    if args.qualified_release is not None or (bulk and args.run):
        require(len(deck['scenes']) == len(indices) == 5000, 'Qualified bulk path requires the entire frozen 5K')
        authority = qualified_release(args.qualified_release, environment, manifest, output, runtime)
    if args.run:
        require(args.confirmed_environment == environment, 'Set MORAI UI and explicitly confirm the matching environment')
        if bulk:
            require(args.approve_bulk_capture and authority is not None, '5K --run requires explicit bulk approval and verified release')
        ancestor = output
        while not ancestor.exists():
            ancestor = ancestor.parent
        minimum = storage_budget_gib(len(indices))
        require(shutil.disk_usage(ancestor).free >= minimum*2**30,
                f'At least {minimum} GiB disk headroom required before '+('5K bulk --run' if bulk else 'bounded prototype --run'))
    inputs = source_pins(manifest, deck, indices)
    legacy = batch.campaign.contract(manifest, indices, 2)
    legacy.update(batch_driver_sha256=sha256(batch.__file__), chunk_size=args.chunk_size, bulk=bulk,
        replenishment_driver_sha256=sha256(SCRIPTS/'base5_campaign_replenishment.py'))
    contract = dict(schema=SCHEMA, policy=POLICY, manifest=str(manifest), manifest_sha256=sha256(manifest),
        environment=environment, indices=indices, chunk_size=args.chunk_size, runtime_profile=profiles(),
        runtime_hashes=runtime, input_hashes=inputs, qualified_release=authority,
        legacy_batch_contract=legacy,
        storage_gate=dict(minimum_gib=storage_budget_gib(len(indices)),
            scope='5K bulk' if bulk else '<=64-frame bounded prototype',
            conservative_prototype_bytes_per_frame=100_000_000, prototype_maximum_frames=64),
        prototype_only=authority is None, no_automatic_completion=True,
        bulk_training_observations_at_qualification=0)
    output.mkdir(parents=True, exist_ok=True)
    with (output/'.static_attitude_batch.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX|fcntl.LOCK_NB)
        guard = output/GUARD
        if guard.exists():
            require(read(guard) == contract, 'Static attitude parent/batch/recovery contract changed')
        else:
            atomic_json(guard, contract)
        old_child, old_recover = batch.run_child, batch.recover
        old_inspect, old_verify = batch.inspect_batch, batch.verify_recovery

        def inspect(root, child_manifest):
            result = old_inspect(root, child_manifest)
            verify_regular(root, child_manifest)
            return result

        def child(child_manifest, root, stop, heartbeat):
            pins.verify_hashes(runtime);pins.verify_hashes(inputs)
            execution_manifest = prepare_execution_source(child_manifest, root)
            root.mkdir(parents=True, exist_ok=True)
            count = len(read(child_manifest)['scenes'])
            with (root.parent/'batch.log').open('a') as log:
                worker = subprocess.Popen(regular_command(execution_manifest, root, count),
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                return batch.wait_child(worker, root, stop, heartbeat, 100*count)

        def recover(source, index, root, stop, heartbeat=lambda root:None):
            pins.verify_hashes(runtime);pins.verify_hashes(inputs)
            root.parent.mkdir(parents=True, exist_ok=True)
            expected = dict(parent_contract_sha256=sha256(guard), runtime_profile=profiles()['isolated_recovery'],
                source_manifest=str(Path(source).resolve()), source_manifest_sha256=sha256(source),
                index=index, runtime_hashes=runtime)
            saved = parent_recovery_guard(root)
            if saved.exists():
                require(read(saved) == expected, 'Recovery scope/profile changed')
            else:
                require(not root.exists(), 'Cannot attach recovery profile to a legacy child')
                atomic_json(saved, expected)
            seed = root.parent/(root.name+'_seed.json')
            if seed.exists():
                original = read(seed)
                verify_regular(Path(original['capture_root']), Path(original['manifest']))
            with (root.parent/(root.name+'.log')).open('a') as log:
                worker = subprocess.Popen(strict_recovery_command(source, index, root, seed if seed.exists() else None),
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                code = batch.wait_child(worker, root, stop, heartbeat, 800)
            if not code:
                old_verify(source, index, root)
                verify_strict_recovery(root, expected)
            return code

        def verify(source, index, root):
            result = old_verify(source, index, root)
            expected = dict(parent_contract_sha256=sha256(guard), runtime_profile=profiles()['isolated_recovery'],
                source_manifest=str(Path(source).resolve()), source_manifest_sha256=sha256(source),
                index=index, runtime_hashes=runtime)
            verify_strict_recovery(root, expected)
            return result

        batch.run_child, batch.recover = child, recover
        batch.inspect_batch, batch.verify_recovery = inspect, verify
        try:
            return batch.execute(manifest, indices, output, args.chunk_size, True,
                args.stop_after_chunks, bulk, prepare_only=not args.run)
        finally:
            batch.run_child, batch.recover = old_child, old_recover
            batch.inspect_batch, batch.verify_recovery = old_inspect, old_verify


def verify_completed_resume(manifest, capture, output):
    """실제 terminal resume 경로를 검증하되 모든 새 subprocess를 금지한다."""
    manifest, capture = Path(manifest).resolve(strict=True), Path(capture).resolve(strict=True)
    output = Path(output).resolve()
    require(not output.exists() and output != capture and capture not in output.parents,
            'Fresh completed-resume receipt outside native output required')
    status, contract = read(capture/'status.json'), read(capture/GUARD)
    count = len(read(manifest)['scenes'])
    require(1 <= count <= 64 and status.get('stage') in ('complete', 'complete_with_deficit')
            and contract.get('manifest_sha256') == sha256(manifest)
            and contract.get('indices') == list(range(count)), 'Only a complete exact pilot may enter terminal replay')
    before = completed_snapshot(capture)
    require(before['raw_files'] and before['event_logs'], 'Actual native and service-call evidence required')
    args = argparse.Namespace(manifest=manifest, output=capture, environment=contract['environment'],
        confirmed_environment=contract['environment'], qualified_release=None, limit=count,
        chunk_size=contract['chunk_size'], run=True, prepare_only=False, approve_bulk_capture=False,
        resume=True, stop_after_chunks=0)
    def forbidden(*args, **kwargs):
        raise RuntimeError('Terminal replay forbids every new subprocess; no ROS/capture start is permitted')
    previous = subprocess.Popen
    subprocess.Popen = forbidden
    try:
        result = run(args)
    finally:
        subprocess.Popen = previous
    after = completed_snapshot(capture)
    require(result == 0 and before == after, 'Terminal replay changed raw/events/selected/elapsed evidence')
    receipt = dict(schema='base5_static_attitude_batch_terminal_resume_v1', passed=True,
        driver_sha256=sha256(__file__), manifest=str(manifest), manifest_sha256=sha256(manifest),
        capture_root=str(capture), requested=count, chunk_size=contract['chunk_size'],
        parent_contract_sha256=sha256(capture/GUARD), runtime_profile=profiles(),
        additional_native_captures=0, new_subprocesses_permitted=False,
        raw_unchanged=True, selected_unchanged=True, elapsed_unchanged=True, event_logs_unchanged=True,
        before=before, after=after, bulk_ready=False,
        limitation='Terminal pilot replay only; not an interrupted-scene or 5000-frame completion claim.')
    atomic_json(output, receipt)
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--environment', choices=ENVIRONMENTS, required=True)
    parser.add_argument('--confirmed-environment', choices=ENVIRONMENTS)
    parser.add_argument('--qualified-release', type=Path)
    parser.add_argument('--limit', type=int, default=64)
    parser.add_argument('--chunk-size', type=int, default=16)
    parser.add_argument('--run', action='store_true')
    parser.add_argument('--prepare-only', action='store_true')
    parser.add_argument('--approve-bulk-capture', action='store_true')
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--stop-after-chunks', type=int, default=0)
    parser.add_argument('--verify-completed-resume', type=Path,
        help='CPU-only terminal replay of this capture root; --output names a new receipt')
    args = parser.parse_args()
    if args.verify_completed_resume is not None:
        print(verify_completed_resume(args.manifest, args.verify_completed_resume, args.output))
    else:
        raise SystemExit(run(args))


if __name__ == '__main__':
    main()
