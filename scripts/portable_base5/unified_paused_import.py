#!/usr/bin/env python3
"""안전 PAUSE의 승인·부족·복구 이력을 새 출력으로 인계한다.

기본 명령은 읽기 전용이다. --run도 CPU 검증과 새 인계 자료 작성만 하며,
기존 원본·상태·PAUSE를 변경하거나 ROS·MORAI·캡처를 시작하지 않는다.
호출자는 추가 프로필의 원형 검증기를 callback으로 제공할 수 있다.
"""
import sys
sys.dont_write_bytecode = True

import argparse
from contextlib import ExitStack, contextmanager
import copy
import fcntl
import importlib
import json
import math
import os
from pathlib import Path
import stat

import recovery_import as historical


SCHEMA = 'base5_unified_paused_handoff_v1'
LOCKS = {'.fresh.lock', '.batch.lock', '.campaign.lock', '.quality.lock',
         '.runner.lock', '.spatial.lock', '.optimized.lock', '.static_attitude_batch.lock'}
ENDED_NATIVE = {'complete', 'halted', 'interrupted', 'stopped_for_resume_test'}
read = historical.read
sha = historical.digest
physical = historical.physical
strict_rows = historical.strict_rows
require = historical.require


def _state(source, old, new, *, fresh=True):
    source, old, new = map(physical, (source, old, new))
    require(source.is_file() and old.is_dir() and new.parent.is_dir(),
            'Existing source, historical output and new parent directory are required')
    require(old != new and old not in new.parents and new not in old.parents
            and (not fresh or not new.exists()), 'A fresh separate handoff output is required')
    require(not (old/'STOP').exists() and (old/'PAUSE').is_file(),
            'Normal PAUSE marker required; STOP is never bypassed')
    require(not (old/'replenishment/state.json').exists(),
            'Active replenishment needs a separate cumulative-attempt migration')
    deck, status, quota = read(source), read(old/'status.json'), read(old/'quota.json')
    scenes = deck.get('scenes', [])
    require(scenes and deck.get('count') == len(scenes), 'Complete nonempty source required')
    ids = [entry['id'] for entry in scenes]
    require(len(ids) == len(set(ids)), 'Source cell IDs must be unique')
    selected, deficits = strict_rows(old/'selected_ledger.jsonl'), quota.get('cells')
    require(type(deficits) is list, 'Authoritative deficit cells required')
    finished = len(selected)+len(deficits)
    require(status.get('stage') == 'paused' and status.get('current') is None
            and status.get('error') is None and status.get('failed') == 0
            and status.get('requested') == len(ids) and status.get('finished') == finished
            and status.get('checkpoint_finished') == finished
            and status.get('accepted_candidate') == len(selected)
            and status.get('review') == len(deficits), 'Safe authoritative paused checkpoint required')
    require(quota.get('target') == len(ids) and quota.get('accepted') == len(selected)
            and quota.get('deficit') == len(deficits) and quota.get('pending') == len(ids)-finished,
            'Paused quota and ledger disagree')
    elapsed = status.get('elapsed_sec')
    require(type(elapsed) in (int, float) and math.isfinite(elapsed) and elapsed >= 0,
            'Elapsed time must be finite and nonnegative')
    chosen = [row.get('source_cell_id') for row in selected]+[row.get('id') for row in deficits]
    require(0 < finished <= len(ids) and len(chosen) == len(set(chosen))
            and set(chosen) <= set(ids), 'Imported cells must be unique and in the source')
    lookup = {ident: index for index, ident in enumerate(ids)}
    require(all(row.get('index') == lookup[row['id']] for row in deficits),
            'Deficit source index changed')
    require(all(row.get('status') == 'accepted_candidate'
                and row.get('audit', {}).get('review_reasons') == [] for row in selected),
            'Reviewed or failed raw cannot become accepted')
    return source, old, new, deck, status, quota, selected, deficits


def plan(source, old_output, new_output):
    source, old, new, deck, status, _, selected, deficits = _state(source, old_output, new_output)
    return dict(schema=SCHEMA+'_plan', manifest=str(source), original_output=str(old),
                output=str(new), selected_count=len(selected), deficit_count=len(deficits),
                finished=status['finished'], pending=len(deck['scenes'])-status['finished'],
                elapsed_sec=status['elapsed_sec'], captures_requested=0,
                native_capture_executed=False, files_written=False)


@contextmanager
def idle_locks(old):
    """기존 lock만 RO로 열어 소유한다. 생성·truncate·mtime 변경은 없다."""
    with ExitStack() as stack:
        for path in sorted(p for p in old.rglob('*') if p.name in LOCKS):
            physical(path)
            fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
            stack.callback(os.close, fd)
            info = os.fstat(fd)
            require(stat.S_ISREG(info.st_mode) and info.st_uid == os.getuid(),
                    'Owned regular lock required')
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as error:
                raise RuntimeError('Historical capture or recovery is still active') from error
        yield


def _modules():
    return importlib.import_module('run_base5_batch_campaign'), importlib.import_module('base5_campaign_import')


def _write_new(path, value):
    with open(path, 'x', opener=lambda name, flags: os.open(name, flags | os.O_NOFOLLOW, 0o600)) as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.write('\n'); stream.flush(); os.fsync(stream.fileno())


def _lineage(old, source, indices, importer, selected, deficits, evidence, hashes, sensor_stats):
    """중첩 import를 원형 verifier로 재검증하고 모든 이전 증거 pin을 계승한다."""
    current, seen, records = old, set(), []
    elapsed_ceiling = read(old/'status.json')['elapsed_sec']
    selected_by_id = {row['source_cell_id']: row for row in selected}
    deficits_by_id = {row['id']: row for row in deficits}
    while (current/'imported_prefix.json').is_file():
        require(current not in seen and len(seen) < 16, 'Cyclic or excessive import ancestry')
        seen.add(current)
        receipt = current/'imported_prefix.json'
        previous, misses, elapsed = importer.verify(receipt, source, indices)
        document = read(receipt)
        require(type(elapsed) in (int, float) and math.isfinite(elapsed)
                and 0 <= elapsed <= elapsed_ceiling, 'Imported elapsed time moved backwards or is invalid')
        elapsed_ceiling = elapsed
        require(all(selected_by_id.get(row['source_cell_id']) == row for row in previous)
                and all(deficits_by_id.get(row['id']) == row for row in misses),
                'Inherited selected/deficit evidence changed or disappeared')
        ancestor = physical(document['original_output'])
        require(ancestor.is_dir() and ancestor != current and current not in ancestor.parents
                and ancestor not in current.parents, 'Import ancestry must use separate preserved outputs')
        evidence(receipt)
        for filename, digest in document['hashes'].items():
            physical(filename)
            require(filename not in hashes or hashes[filename] == digest, 'Conflicting ancestor evidence pin')
            hashes[filename] = digest
        for filename, metadata in document['sensor_stats'].items():
            physical(filename)
            require(filename not in sensor_stats or sensor_stats[filename] == metadata,
                    'Conflicting ancestor raw integrity pin')
            sensor_stats[filename] = metadata
        records.append(dict(output=str(current), receipt=str(receipt), sha256=sha(receipt),
                            original_output=str(ancestor), selected=len(previous), deficits=len(misses), elapsed_sec=elapsed))
        current = ancestor
    return records, {old, current, *(Path(record['original_output']) for record in records)}


def _native_safety(old, evidence):
    sessions = []
    for contract in sorted(old.rglob('session_contract.json')):
        root = contract.parent
        session = read(contract)
        status, shutdown = read(root/'status.json'), read(root/'shutdown_safety.json')
        require(status.get('stage') in ENDED_NATIVE, 'Native session is incomplete: '+str(root))
        require(shutdown.get('park_verified') is True and shutdown.get('gear') == 1
                and type(shutdown.get('gear')) is int
                and shutdown.get('brake_held_during_requests') is True
                and shutdown.get('request_fields') == {'option': 2, 'gear': 1},
                'Native final Park/brake proof is incomplete: '+str(root))
        rows = strict_rows(root/'ledger.jsonl')
        require(rows, 'Native ledger is empty: '+str(root))
        # 경계 P는 shared batch에만 존재한다. 독립 단일 장면은 위의
        # final Park와 원형 recovery verifier가 종료 안전성을 확인한다.
        if session.get('runner_options', {}).get('shared_ros_batch') is True:
            require(all(row.get('between_scene_park', {}).get('park_verified') is True for row in rows),
                    'Native boundary Park proof is missing: '+str(root))
        for filename in (contract, root/'status.json', root/'shutdown_safety.json', root/'ledger.jsonl'):
            evidence(filename)
        sessions.append(str(root))
    require(sessions, 'At least one current native session with safe Park is required')
    return sessions


def verify_exit(root, *, require_marker=True):
    """이미 종료한 출력의 lock·모든 native 종료 증거만 읽는다.

    기본은 실제 PAUSE marker를 요구한다. 명시적인 소량 시험의
    stop_after_chunks 종료만 require_marker=False로 검사할 수 있다.
    """
    root = physical(root)
    with idle_locks(root), historical.readonly_cpu(root/'.never_created_exit_probe'):
        require(type(require_marker) is bool, 'Explicit boolean pause marker policy required')
        require(root.is_dir() and (not require_marker or (root/'PAUSE').is_file())
                and not (root/'STOP').exists(),
                'Normal PAUSE without STOP required')
        status = read(root/'status.json')
        require(status.get('stage') == 'paused' and status.get('current') is None
                and status.get('failed') == 0 and status.get('error') is None,
                'Safe finished parent paused checkpoint required')
        before = historical.tree_snapshot(root)
        hashes = {}
        def evidence(path):hashes[str(physical(path))] = sha(path)
        sessions = _native_safety(root, evidence)
        evidence(root/'status.json')
        if (root/'PAUSE').is_file():evidence(root/'PAUSE')
        require(historical.tree_snapshot(root) == before, 'Exit proof changed during verification')
    return dict(paused=True, output=str(root), native_sessions_verified=len(sessions),
                hashes=hashes, historical_inventory=before, files_written=False,
                pause_marker_required=require_marker, pause_marker_present=(root/'PAUSE').is_file(),
                fresh_native_park_required=True)


def export(source, old_output, new_output, *, batch=None, importer=None,
           verify_context=None, verify_regular=None, verify_recovery_extra=None):
    """검증된 safe checkpoint만 schema1로 전달한다. 기존 부족분은 pending이 아니다.

    추가 프로필 검증 callback은 모두 read-only scope 안에서 실행한다.
    verify_context(source, old), verify_regular(root, chunk_manifest),
    verify_recovery_extra(source, index, recovery)가 원형 부가 계약을 검증한다.
    """
    result = plan(source, old_output, new_output)
    source, old, new = map(physical, (source, old_output, new_output))
    if batch is None or importer is None:
        original_batch, original_importer = _modules()
        batch = original_batch if batch is None else batch
        importer = original_importer if importer is None else importer
    with idle_locks(old), historical.readonly_cpu(new):
        source, old, new, deck, status, quota, selected, deficits = _state(source, old, new)
        before = historical.tree_snapshot(old)
        for filename in old.rglob('*.jsonl'):
            strict_rows(filename)  # 원형 loader의 repair보다 먼저 거부한다.
        hashes, sensor_stats = {}, {}
        def evidence(filename):
            filename = physical(filename)
            require(filename.is_file(), 'Historical proof missing: '+str(filename))
            hashes[str(filename)] = sha(filename)
        if verify_context is not None:
            verify_context(source, old)
        if (old/'static_attitude_batch_contract.json').is_file():
            require(verify_context is not None and verify_regular is not None
                    and verify_recovery_extra is not None,
                    'Static-attitude historical output requires its original extra profile verifiers')
        contract = read(old/'batch_contract.json')
        indices = contract.get('indices')
        require(indices == list(range(len(deck['scenes']))) and type(contract.get('chunk_size')) is int
                and 1 <= contract['chunk_size'] <= 16, 'Full source and valid historical chunk contract required')
        expected = copy.deepcopy(batch.campaign.contract(source, indices, 2))
        scripts = Path(batch.__file__).parent
        expected.update(batch_driver_sha256=sha(batch.__file__), chunk_size=contract['chunk_size'],
                        bulk=contract['bulk'], replenishment_driver_sha256=sha(scripts/'base5_campaign_replenishment.py'))
        if (old/'imported_prefix.json').is_file():
            expected.update(imported_prefix_sha256=sha(old/'imported_prefix.json'),
                            import_driver_sha256=sha(scripts/'base5_campaign_import.py'))
        require(contract == expected, 'Historical source/runtime batch contract changed')
        for name, digest in contract.get('code', {}).items():
            filename = Path(name) if Path(name).is_absolute() else scripts/name
            evidence(filename)
            require(hashes[str(physical(filename))] == digest, 'Historical native runtime changed')
        for filename in (Path(batch.__file__), scripts/'base5_campaign_replenishment.py',
                         scripts/'base5_campaign_import.py'):
            evidence(filename)
        for filename in (source, Path(__file__).absolute(), Path(historical.__file__).absolute()):
            evidence(filename)
        for filename in old.rglob('*'):
            physical(filename)
            if filename.is_file():
                info = filename.stat()
                sensor_stats[str(filename)] = [info.st_size, info.st_mtime_ns]
                if filename.suffix in ('.json', '.jsonl') or filename.name == 'PAUSE':
                    evidence(filename)
        lineage, allowed_roots = _lineage(old, source, indices, importer, selected, deficits,
                                         evidence, hashes, sensor_stats)
        sessions = _native_safety(old, evidence)
        inherited_selected, inherited_deficits = {}, {}
        if (old/'imported_prefix.json').is_file():
            inherited, misses, _ = importer.verify(old/'imported_prefix.json', source, indices)
            inherited_selected = {row['source_cell_id']: row for row in inherited}
            inherited_deficits = {row['id']: row for row in misses}
        native_by_id, recoveries = {}, {}
        lookup = {entry['id']: index for index, entry in enumerate(deck['scenes'])}
        for folder in sorted(old.glob('chunk_*')):
            root, manifest = folder/'capture', folder/'manifest.json'
            if (root/'session_contract.json').is_file():
                rows = batch.inspect_batch(root, manifest)
                if verify_regular is not None:
                    verify_regular(root, manifest)
                for row in rows:
                    require(row['id'] not in native_by_id, 'Duplicate native source cell in chunks')
                    native_by_id[row['id']] = row
            for recovery in sorted((folder/'recovery').glob('*')) if (folder/'recovery').is_dir() else []:
                if not recovery.is_dir() or not (recovery/'status.json').is_file():
                    continue
                ident = recovery.name
                require(ident in lookup and read(recovery/'status.json').get('stage')
                        in ('complete', 'complete_with_deficit'), 'Incomplete or unknown recovery cannot be handed off')
                index = lookup[ident]
                require(batch.verify_recovery(source, index, recovery) == 0, 'Original recovery verifier failed')
                if verify_recovery_extra is not None:
                    verify_recovery_extra(source, index, recovery)
                recoveries[ident] = dict(output=str(recovery), index=index,
                    selected=strict_rows(recovery/'selected_ledger.jsonl'), quota=read(recovery/'quota.json'),
                    reports=[dict(path=str(path), sha256=sha(path)) for path in sorted(recovery.rglob('report.json'))])
        for row in selected:
            ident = row['source_cell_id']
            proven = inherited_selected.get(ident)
            native = native_by_id.get(ident)
            if native and native.get('status') == 'accepted_candidate':
                proven = dict(copy.deepcopy(native), source_cell_id=ident)
            if ident in recoveries:
                recovered = recoveries[ident]['selected']
                require(len(recovered) == 1, 'Selected cell has no unique completed recovery')
                proven = recovered[0]
            require(proven == row, 'Selected row differs from original authoritative evidence: '+ident)
            capture = physical(row['capture_root'])
            require(any(root in capture.parents for root in allowed_roots), 'Selected raw escaped preserved lineage')
            require(read(capture/'audit.json') == row['audit']
                    and read(capture/'metadata.json').get('calibration_only') is not True
                    and sha(capture/'scenario.json') == row['scenario_sha256'],
                    'Selected audit/scenario changed or calibration raw selected')
            for filename in capture.rglob('*'):
                if filename.is_file(): evidence(filename)
        for row in deficits:
            ident = row['id']
            if inherited_deficits.get(ident) == row:
                continue
            recovery = recoveries.get(ident)
            if recovery is not None:
                require(recovery['selected'] == [] and recovery['quota'].get('accepted') == 0
                        and recovery['quota'].get('deficit') == 1
                        and row.get('reasons') == recovery['quota'].get('cells'),
                        'Deficit reasons or exhausted recovery bounds changed: '+ident)
            else:
                native = native_by_id.get(ident)
                require(native is not None and row.get('evidence') == native
                        and (batch.is_collision_failure(native) or batch.is_stationary_failure(native)),
                        'Deficit lacks completed recovery or validated safety-failure evidence: '+ident)
        for entry in deck['scenes']:
            evidence(entry['scenario']); evidence(entry['metadata'])
            require(sha(entry['scenario']) == entry['scenario_sha256'], 'Frozen scenario changed')
        require(historical.tree_snapshot(old) == before, 'Historical output changed during read-only export')
        receipt = dict(schema=1, manifest_sha256=sha(source), original_output=str(old),
            selected=copy.deepcopy(selected), deficits=copy.deepcopy(deficits), elapsed_sec=status['elapsed_sec'],
            hashes=hashes, sensor_stats=sensor_stats, handoff_policy=SCHEMA,
            recovery_history=copy.deepcopy(list(recoveries.values())), import_lineage=lineage,
            native_capture_executed=False, captures_requested=0)
        new.mkdir(mode=0o700, exist_ok=False)
        destination = new/'imported_prefix.json'
        _write_new(destination, receipt)
        imported = importer.verify(destination, source, indices)
        require(imported == (selected, deficits, status['elapsed_sec']), 'Original importer rejected exact paused handoff')
        handoff = dict(result, schema=SCHEMA, files_written=True, original_paused_status_retained=True,
            original_pause_marker_retained=True, authoritative_rows_unchanged=True,
            deficit_cells_skipped_not_reset_to_pending=True, recovery_round_bounds_preserved=True,
            imported_prefix=str(destination), imported_prefix_sha256=sha(destination),
            historical_inventory=before, native_sessions_verified=len(sessions),
            completed_recoveries_verified=len(recoveries), import_lineage=lineage,
            original_import_verifier_passed=True, fresh_native_park_required=True)
        _write_new(new/'unified_paused_handoff.json', handoff)
        require(historical.tree_snapshot(old) == before, 'Historical output changed while writing handoff')
    return handoff


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--old-output', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--run', action='store_true', help='CPU-only safe paused handoff, never native capture')
    args = parser.parse_args(argv)
    if args.run:
        scripts = Path(__file__).resolve().parents[2]/'dataset/morai_od/scripts'
        sys.path.insert(0, str(scripts))
    result = export(args.manifest, args.old_output, args.output) if args.run else plan(args.manifest, args.old_output, args.output)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
