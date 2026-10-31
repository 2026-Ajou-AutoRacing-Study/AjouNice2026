#!/usr/bin/env python3
"""통신/Park 실패 직전의 확정 prefix만 새 출력으로 CPU 인계한다.

과거 halted·Park false와 미확정 마지막 chunk는 그대로 보존한다. 임의의
unsafe 세션을 건너뛰지 않고, 정확히 checkpoint 다음 shared chunk 하나만
격리한다. 기존 PAUSE 실행기·수집기·native QA 파일은 변경하지 않는다.
"""
import sys
sys.dont_write_bytecode = True

import argparse
import copy
import math
from pathlib import Path
import re

import unified_paused_import as paused


SCHEMA = 'base5_unified_checkpoint_crash_handoff_v1'
ERRORS = {'Batch final Park unverified', 'Batch boundary Park unverified'}
STALE = 'Ego/Object/CollisionData connection stale'
CHUNK = re.compile(r'^chunk_([0-9]{4})$')
read, sha, physical = paused.read, paused.sha, paused.physical
strict_rows, require = paused.strict_rows, paused.require


def _state(source, old, new):
    source, old, new = map(physical, (source, old, new))
    require(source.is_file() and old.is_dir() and new.parent.is_dir()
        and not new.exists() and old.parent == new.parent and old != new,
        'Existing source and a fresh separate sibling crash output required')
    require(old not in new.parents and new not in old.parents and new not in source.parents
        and old not in source.parents,
        'Historical/source/new outputs must not overlap')
    require(not (old/'STOP').exists() and not (old/'replenishment/state.json').exists(),
        'STOP or active replenishment requires a separate incident review')
    deck, status, quota, contract = (read(source), read(old/'status.json'),
                                    read(old/'quota.json'), read(old/'batch_contract.json'))
    entries = deck.get('scenes', [])
    require(entries and type(deck.get('count')) is int and deck['count'] == len(entries),
        'Complete original source manifest required')
    ids = [row['id'] for row in entries]
    require(len(ids) == len(set(ids)) and contract.get('indices') == list(range(len(ids)))
        and type(contract.get('chunk_size')) is int and 1 <= contract['chunk_size'] <= 16
        and contract.get('bulk') is True, 'Full ordered source and bounded original bulk contract required')
    selected, deficits = strict_rows(old/'selected_ledger.jsonl'), quota.get('cells')
    require(type(deficits) is list, 'Authoritative deficit cells required')
    require(all(type(status.get(key)) is int and status[key] >= 0 for key in
        ('requested','finished','checkpoint_finished','accepted_candidate','review','failed'))
        and all(type(quota.get(key)) is int and quota[key] >= 0 for key in
        ('target','accepted','deficit','pending')), 'Strict integer checkpoint/quota counts required')
    finished = len(selected)+len(deficits)
    require(status.get('stage') == 'halted' and status.get('error') in ERRORS
        and status.get('failed') == 1 and status.get('current') is None
        and status.get('requested') == len(ids) and status.get('finished') == finished
        and status.get('checkpoint_finished') == finished
        and status.get('accepted_candidate') == len(selected) and status.get('review') == len(deficits),
        'Exact halted Park/communication checkpoint and authoritative counts required')
    require(0 < finished < len(ids) and finished % contract['chunk_size'] == 0,
        'Crash checkpoint must end at a complete source chunk boundary')
    require(quota.get('target') == len(ids) and quota.get('accepted') == len(selected)
        and quota.get('deficit') == len(deficits) and quota.get('pending') == len(ids)-finished,
        'Crash quota differs from authoritative selected/deficit rows')
    chosen = [row.get('source_cell_id') for row in selected]+[row.get('id') for row in deficits]
    require(len(chosen) == len(set(chosen)) and set(chosen) == set(ids[:finished]),
        'Only an exact hole-free authoritative source prefix may be handed off')
    require(all(row.get('status') == 'accepted_candidate'
        and row.get('audit', {}).get('review_reasons') == [] for row in selected),
        'Reviewed or failed raw cannot become selected')
    lookup = {ident:index for index, ident in enumerate(ids)}
    require(all(type(row.get('index')) is int and row['index'] == lookup[row['id']] for row in deficits),
        'Deficit source indices changed')
    elapsed = status.get('elapsed_sec')
    require(type(elapsed) in (int, float) and math.isfinite(elapsed) and elapsed >= 0,
        'Finite nonnegative historical elapsed time required')
    ordinal = finished//contract['chunk_size']
    suffix = old/('chunk_%04d'%ordinal)
    folders = sorted(old.glob('chunk_*'))
    require(suffix.is_dir() and all(folder.is_dir() and CHUNK.fullmatch(folder.name)
        and int(CHUNK.fullmatch(folder.name).group(1)) <= ordinal for folder in folders),
        'Only the one exact checkpoint-next unsafe suffix chunk may exist')
    safe = [folder for folder in folders if folder != suffix]
    return source, old, new, deck, status, quota, contract, selected, deficits, safe, suffix


def _unsafe_suffix(deck, status, contract, suffix):
    root = suffix/'capture'
    require((root/'session_contract.json').is_file(),
        'Unsafe suffix must contain its original native session')
    require(not (suffix/'recovery').exists() or (suffix/'recovery').is_dir()
        and not any((suffix/'recovery').glob('*')),
        'Unsafe suffix must be the original shared native session without recovery work')
    session = read(root/'session_contract.json')
    native, shutdown = read(root/'status.json'), read(root/'shutdown_safety.json')
    rows, source = strict_rows(root/'ledger.jsonl'), read(suffix/'manifest.json')
    start = status['checkpoint_finished']; count = contract['chunk_size']
    expected = deck['scenes'][start:start+count]
    require(source.get('count') == len(expected) and source.get('scenes') == expected,
        'Unsafe suffix manifest is not the exact next original source chunk')
    execution = suffix/'execution_source/manifest.json'
    require(execution.is_file() and execution.read_bytes() == (suffix/'manifest.json').read_bytes()
        and session.get('manifest_sha256') == sha(execution), 'Unsafe execution source/session binding changed')
    options = session.get('runner_options', {})
    require(options.get('shared_ros_batch') is True
        and options.get('selected_scene_ids') == [entry['id'] for entry in expected],
        'Unsafe suffix must retain its original exact shared-batch source IDs')
    require(native.get('stage') == 'halted_exit_safety' and native.get('current') is None
        and type(native.get('error')) is str and native['error'].startswith('Park not confirmed:')
        and STALE in native['error'] and shutdown.get('park_verified') is False
        and shutdown.get('request_fields') == {'option':2, 'gear':1}
        and shutdown.get('brake_held_during_requests') is True
        and shutdown.get('errors') and all(error == STALE for error in shutdown['errors']),
        'Unsafe suffix lacks the exact preserved stale-connection/Park-false evidence')
    require(rows and len(rows) <= len(expected)
        and [row.get('id') for row in rows] == [entry['id'] for entry in expected[:len(rows)]]
        and all(row.get('scenario_sha256') == entry['scenario_sha256'] for row,entry in zip(rows,expected)),
        'Unsafe native ledger must be a unique ordered prefix of only its next chunk')
    require(all(row.get('status') in ('accepted_candidate','review')
        and row.get('between_scene_park', {}).get('park_verified') is True for row in rows[:-1])
        and rows[-1].get('status') == 'failed' and rows[-1].get('error') == STALE
        and rows[-1].get('between_scene_park', {}).get('park_verified') is False,
        'Only the final stale native failure may be unsafe; all suffix raw remains unselected')
    require(all(type(native.get(key)) is int and native[key] >= 0 for key in
        ('requested','finished','accepted_candidate','review','failed'))
        and native.get('requested') == len(expected) and native.get('finished') == len(rows)
        and native.get('accepted_candidate') == sum(row['status']=='accepted_candidate' for row in rows)
        and native.get('review') == sum(row['status']=='review' for row in rows)
        and native.get('failed') == 1, 'Unsafe suffix native counts changed')
    return dict(chunk=str(suffix), native_session=str(root), source_indices=list(range(start,start+len(expected))),
        observed_rows=len(rows), ignored_accepted_rows=sum(row['status']=='accepted_candidate' for row in rows),
        ignored_review_rows=sum(row['status']=='review' for row in rows), ignored_failed_rows=1,
        parent_error=status['error'], native_error=native['error'], old_park_verified=False,
        imported_rows=0, action='preserve unchanged; no suffix raw selected or resumed')


def plan(source, old_output, new_output):
    source, old, new, deck, status, _, contract, selected, deficits, _, suffix = _state(source,old_output,new_output)
    isolated = _unsafe_suffix(deck,status,contract,suffix)
    return dict(schema=SCHEMA+'_plan', manifest=str(source), original_output=str(old), output=str(new),
        selected_count=len(selected), deficit_count=len(deficits), finished=status['finished'],
        pending=len(deck['scenes'])-status['finished'], elapsed_sec=status['elapsed_sec'],
        unsafe_suffix=isolated, captures_requested=0, native_capture_executed=False, files_written=False)


def export(source, old_output, new_output, *, batch=None, importer=None,
           verify_context=None, verify_regular=None, verify_recovery_extra=None):
    """동결 verifier callbacks를 유지한 CPU-only crash-prefix 인계 API."""
    result = plan(source,old_output,new_output)
    source, old, new = map(physical,(source,old_output,new_output))
    if batch is None or importer is None:
        original_batch, original_importer = paused._modules()
        batch = original_batch if batch is None else batch
        importer = original_importer if importer is None else importer
    with paused.idle_locks(old), paused.historical.readonly_cpu(new):
        state = _state(source,old,new)
        _,_,_,deck,status,quota,contract,selected,deficits,folders,suffix = state
        before = paused.historical.tree_snapshot(old)
        for filename in old.rglob('*.jsonl'):strict_rows(filename)
        isolated = _unsafe_suffix(deck,status,contract,suffix)
        hashes, sensor_stats = {}, {}
        def evidence(filename):
            filename = physical(filename)
            require(filename.is_file(), 'Historical crash proof missing: '+str(filename))
            hashes[str(filename)] = sha(filename)
        require(verify_context is not None, 'Crash handoff requires the original frozen runtime/profile verifier')
        verify_context(source,old)
        if (old/'static_attitude_batch_contract.json').is_file():
            require(verify_regular is not None and verify_recovery_extra is not None,
                'Static-attitude crash handoff requires all original extra profile verifiers')
        indices = contract['indices']; scripts = Path(batch.__file__).parent
        expected = copy.deepcopy(batch.campaign.contract(source,indices,2))
        expected.update(batch_driver_sha256=sha(batch.__file__),chunk_size=contract['chunk_size'],bulk=True,
            replenishment_driver_sha256=sha(scripts/'base5_campaign_replenishment.py'))
        if (old/'imported_prefix.json').is_file():
            expected.update(imported_prefix_sha256=sha(old/'imported_prefix.json'),
                import_driver_sha256=sha(scripts/'base5_campaign_import.py'))
        require(contract == expected, 'Historical crash source/runtime batch contract changed')
        for filename in (source,Path(__file__).absolute(),Path(paused.__file__).absolute(),Path(paused.historical.__file__).absolute()):
            evidence(filename)
        for filename in old.rglob('*'):
            physical(filename)
            if filename.is_file():
                info = filename.stat();sensor_stats[str(filename)] = [info.st_size,info.st_mtime_ns]
                if filename.suffix in ('.json','.jsonl') or filename.name == 'PAUSE':evidence(filename)
        lineage, allowed_roots = paused._lineage(old,source,indices,importer,selected,deficits,evidence,hashes,sensor_stats)
        inherited_selected, inherited_deficits = {}, {}
        if (old/'imported_prefix.json').is_file():
            previous,misses,_ = importer.verify(old/'imported_prefix.json',source,indices)
            inherited_selected = {row['source_cell_id']:row for row in previous}
            inherited_deficits = {row['id']:row for row in misses}
        # Every current native session must belong to a verified prefix chunk
        # or to exactly the one explicitly isolated suffix, never elsewhere.
        allowed_chunks = set(folders+[suffix])
        for filename in old.rglob('session_contract.json'):
            require(any(chunk in filename.parents for chunk in allowed_chunks),
                'Unexpected native session outside the prefix and exact unsafe suffix')
            if suffix in filename.parents:
                require(filename == suffix/'capture/session_contract.json',
                    'Additional unsafe suffix sessions cannot be silently skipped')
        sessions, native_by_id, recoveries = [], {}, {}
        lookup = {entry['id']:index for index,entry in enumerate(deck['scenes'])}
        if verify_regular is not None:
            verify_regular(suffix/'capture',suffix/'manifest.json')
        for folder in folders:
            if list(folder.rglob('session_contract.json')):
                sessions.extend(paused._native_safety(folder,evidence))
            root, manifest = folder/'capture',folder/'manifest.json'
            if (root/'session_contract.json').is_file():
                rows = batch.inspect_batch(root,manifest)
                if verify_regular is not None:verify_regular(root,manifest)
                for row in rows:
                    require(row['id'] not in native_by_id and lookup.get(row['id'],len(indices)) < status['finished'],
                        'Duplicate or nonprefix native source row')
                    native_by_id[row['id']] = row
            for recovery in sorted((folder/'recovery').glob('*')) if (folder/'recovery').is_dir() else []:
                if not recovery.is_dir() or not (recovery/'status.json').is_file():continue
                ident = recovery.name
                require(ident in lookup and lookup[ident] < status['finished']
                    and read(recovery/'status.json').get('stage') in ('complete','complete_with_deficit'),
                    'Every prefix recovery must be completed and inside the authoritative prefix')
                index = lookup[ident]
                require(batch.verify_recovery(source,index,recovery) == 0, 'Original completed recovery verifier failed')
                if verify_recovery_extra is not None:verify_recovery_extra(source,index,recovery)
                recoveries[ident] = dict(output=str(recovery),index=index,
                    selected=strict_rows(recovery/'selected_ledger.jsonl'),quota=read(recovery/'quota.json'),
                    reports=[dict(path=str(path),sha256=sha(path)) for path in sorted(recovery.rglob('report.json'))])
        for row in selected:
            ident = row['source_cell_id'];proven = inherited_selected.get(ident);native = native_by_id.get(ident)
            if native and native.get('status') == 'accepted_candidate':proven = dict(copy.deepcopy(native),source_cell_id=ident)
            if ident in recoveries:
                recovered = recoveries[ident]['selected']
                require(len(recovered) == 1,'Selected prefix recovery must have exactly one accepted row')
                proven = recovered[0]
            require(proven == row,'Authoritative prefix selection differs from original evidence: '+ident)
            capture = physical(row['capture_root'])
            require(suffix not in capture.parents and any(root in capture.parents for root in allowed_roots),
                'Selected raw escaped preserved prefix lineage or entered unsafe suffix')
            require(read(capture/'audit.json') == row['audit']
                and read(capture/'metadata.json').get('calibration_only') is not True
                and sha(capture/'scenario.json') == row['scenario_sha256'],'Selected prefix raw/audit changed')
            for filename in capture.rglob('*'):
                if filename.is_file():evidence(filename)
        for row in deficits:
            ident = row['id']
            if inherited_deficits.get(ident) == row:continue
            recovery = recoveries.get(ident)
            if recovery is not None:
                require(recovery['selected'] == [] and recovery['quota'].get('accepted') == 0
                    and recovery['quota'].get('deficit') == 1 and row.get('reasons') == recovery['quota'].get('cells'),
                    'Deficit reasons or exhausted recovery bounds changed: '+ident)
            else:
                native = native_by_id.get(ident)
                require(native is not None and row.get('evidence') == native
                    and (batch.is_collision_failure(native) or batch.is_stationary_failure(native)),
                    'Prefix deficit lacks original completed recovery/safety evidence: '+ident)
        for entry in deck['scenes']:
            evidence(entry['scenario']);evidence(entry['metadata'])
            require(sha(entry['scenario']) == entry['scenario_sha256'],'Original source payload changed')
        require(paused.historical.tree_snapshot(old) == before,'Crash tree changed during read-only verification')
        receipt = dict(schema=1,manifest_sha256=sha(source),original_output=str(old),
            selected=copy.deepcopy(selected),deficits=copy.deepcopy(deficits),elapsed_sec=status['elapsed_sec'],
            hashes=hashes,sensor_stats=sensor_stats,handoff_policy=SCHEMA,
            recovery_history=copy.deepcopy(list(recoveries.values())),import_lineage=lineage,
            unsafe_suffix=isolated,native_capture_executed=False,captures_requested=0)
        new.mkdir(mode=0o700,exist_ok=False)
        destination = new/'imported_prefix.json';paused._write_new(destination,receipt)
        require(importer.verify(destination,source,indices) == (selected,deficits,status['elapsed_sec']),
            'Original schema1 verifier rejected the exact crash prefix')
        final = dict(result,schema=SCHEMA,files_written=True,original_halted_status_retained=True,
            original_false_park_retained=True,unsafe_suffix=isolated,authoritative_rows_unchanged=True,
            deficit_cells_skipped_not_reset_to_pending=True,recovery_round_bounds_preserved=True,
            imported_prefix=str(destination),imported_prefix_sha256=sha(destination),historical_inventory=before,
            native_sessions_verified=len(sessions),completed_recoveries_verified=len(recoveries),import_lineage=lineage,
            original_import_verifier_passed=True,fresh_native_park_required=True)
        paused._write_new(new/'unified_crash_handoff.json',final)
        require(paused.historical.tree_snapshot(old) == before,'Crash tree changed while writing new handoff')
    return final


def main(argv=None):
    import json
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,required=True)
    parser.add_argument('--old-output',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args(argv)
    print(json.dumps(plan(args.manifest,args.old_output,args.output),ensure_ascii=False,indent=2))
    return 0


if __name__ == '__main__':raise SystemExit(main())
