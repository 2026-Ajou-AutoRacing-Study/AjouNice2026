#!/usr/bin/env python3
"""MORAI 종료 사고의 승인 414장을 원본 변경 없이 새 output으로 인계한다.

기본은 읽기 전용 계획이다. prepare()도 CPU 검증과 인계 파일 작성만 하며
ROS 연결, Park 확인, 시나리오 로드, 센서 캡처를 실행하지 않는다.
"""
import sys
sys.dont_write_bytecode = True

import argparse
from contextlib import ExitStack, contextmanager
import copy
import json
import math
import os
from pathlib import Path

import local_sunny15 as original
import local_sunny15_fast as fast
import recovery_import as historical


OLD = original.OUTPUT
NEW = OLD.with_name('collection_5k_after_crash_v1')
IMPORT = NEW/'imported_prefix.json'
HANDOFF = NEW/'crash_handoff.json'
SCHEMA = 'base5_local_sunny15_crash_handoff_v1'
EXPECTED_SELECTED = 414
EXPECTED_DEFICITS = 2
EXPECTED_FINISHED = 416
SOURCE_COUNT = 5000
CHUNK_SIZE = 16
ERROR = 'Batch final Park unverified'
HEADERS = ('status.json','quota.json','selected_ledger.jsonl','deficit_manifest.json',
    'batch_contract.json','static_attitude_batch_contract.json',original.CONTRACT)


def require(value, message):
    if not value:
        raise ValueError(message)


def read(filename):
    return original.read(filename)


def sha(filename):
    return original.sha(filename)


def helper_hashes():
    return {str(original.path(filename)):sha(filename) for filename in
        (Path(__file__).absolute(),Path(historical.__file__).resolve(),
         Path(fast.__file__).resolve(),Path(fast.cache_module.__file__).resolve())}


def _modules():
    parent = original.parent_module()
    import base5_campaign_import
    return parent, parent.batch, base5_campaign_import


def _headers():
    return {str(original.path(OLD/name)):sha(OLD/name) for name in HEADERS}


def _inventory():
    """Metadata includes every file, including the excluded crash session."""
    return historical.tree_snapshot(original.path(OLD))


@contextmanager
def historical_locks():
    with ExitStack() as stack:
        paths = [OLD/name for name in ('.batch.lock','.static_attitude_batch.lock',original.CONTRACT)]
        paths.extend(path for path in OLD.rglob('*') if path.name in fast.CHILD_LOCKS)
        for path in sorted(set(paths)):
            stack.enter_context(fast.idle_lock(path))
        yield


def _state(*, fresh):
    old, new = original.path(OLD), original.path(NEW)
    require(old.is_dir() and new.parent.is_dir() and old != new
        and old not in new.parents and new not in old.parents,
        'Historical and fresh continuation paths must be separate siblings')
    require(not fresh or not new.exists(), 'Fresh crash continuation output already exists')
    require(sha(original.MANIFEST) == original.MANIFEST_SHA,
        'Exact frozen Sunny15 manifest required')
    deck, status, quota = read(original.MANIFEST),read(old/'status.json'),read(old/'quota.json')
    require(deck.get('count') == len(deck.get('scenes',[])) == SOURCE_COUNT,
        'Original complete source required')
    ids = [entry['id'] for entry in deck['scenes']]
    require(len(ids) == len(set(ids)), 'Source IDs must be unique')
    require(status.get('stage') == 'halted' and status.get('error') == ERROR
        and status.get('current') is None and status.get('failed') == 1
        and status.get('requested') == SOURCE_COUNT and status.get('finished') == EXPECTED_FINISHED
        and status.get('checkpoint_finished') == EXPECTED_FINISHED
        and status.get('accepted_candidate') == EXPECTED_SELECTED
        and status.get('review') == EXPECTED_DEFICITS,
        'Only the explicitly inspected final-Park crash checkpoint may be inherited')
    elapsed = status.get('elapsed_sec')
    require(type(elapsed) in (int,float) and math.isfinite(elapsed) and elapsed >= 0,
        'Historical elapsed time must be finite and nonnegative')
    selected = historical.strict_rows(old/'selected_ledger.jsonl')
    deficits = quota.get('cells')
    require(type(deficits) is list and len(selected) == EXPECTED_SELECTED
        and len(deficits) == EXPECTED_DEFICITS
        and quota.get('target') == SOURCE_COUNT and quota.get('accepted') == EXPECTED_SELECTED
        and quota.get('deficit') == EXPECTED_DEFICITS
        and quota.get('pending') == SOURCE_COUNT-EXPECTED_FINISHED,
        'Historical authoritative ledger/quota counts differ')
    imported_ids = [row.get('source_cell_id') for row in selected]+[row.get('id') for row in deficits]
    require(len(imported_ids) == len(set(imported_ids)) == EXPECTED_FINISHED
        and set(imported_ids) == set(ids[:EXPECTED_FINISHED])
        and EXPECTED_FINISHED % CHUNK_SIZE == 0,
        'Only the exact completed prefix may be imported')
    for row in selected:
        require(row.get('status') == 'accepted_candidate'
            and row.get('audit',{}).get('review_reasons') == [],
            'Reviewed or failed raw cannot be imported as accepted')
        capture = original.path(row['capture_root'])
        require(capture.is_dir() and old in capture.parents,
            'Historical selected raw escaped its original tree')
    lookup = {ident:index for index,ident in enumerate(ids)}
    for row in deficits:
        require(row.get('index') == lookup[row['id']], 'Historical deficit index changed')
    return deck,status,quota,selected,deficits


def plan():
    deck,status,quota,selected,deficits = _state(fresh=True)
    return dict(schema=SCHEMA+'_plan', original_output=str(OLD),output=str(NEW),
        selected_count=len(selected),deficit_count=len(deficits),
        finished=EXPECTED_FINISHED,pending=SOURCE_COUNT-EXPECTED_FINISHED,
        captures_requested=0,native_capture_executed=False,files_written=False,
        old_halted_status_retained=True,unsafe_chunk_excluded=True)


def _original_context():
    original.frozen_context()
    original.validate_contract(OLD/original.CONTRACT)
    fast.validate_attachment(fast.immutable_pins())


def _expected_recovery(parent,index):
    return dict(parent_contract_sha256=sha(OLD/parent.GUARD),
        runtime_profile=parent.profiles()['isolated_recovery'],
        source_manifest=str(original.MANIFEST.resolve()),source_manifest_sha256=sha(original.MANIFEST),
        index=index,runtime_hashes=parent.runtime_hashes())


def _write_new(filename,value):
    # Keep the filename visible to the read-only export audit. fdopen would
    # produce an anonymous integer-fd write that the audit correctly rejects.
    with open(filename,'x',opener=lambda name,flags:os.open(name,flags|os.O_NOFOLLOW,0o600)) as stream:
        json.dump(value,stream,ensure_ascii=False,indent=2)
        stream.write('\n');stream.flush();os.fsync(stream.fileno())


def prepare():
    """Create only the CPU-verified import and handoff; never capture or repair."""
    plan()
    _original_context()
    parent,batch,importer = _modules()
    with historical_locks():
        deck,status,quota,selected,deficits = _state(fresh=True)
        before = _inventory()
        headers = _headers()
        helpers = helper_hashes()
        with historical.readonly_cpu(NEW):
            # prepare_session/load_ledger may repair a torn ledger. Reject it
            # first so the original validators remain strictly read-only.
            for filename in OLD.rglob('*.jsonl'):
                historical.strict_rows(filename)
            hashes,sensor_stats = dict(headers),{}
            def evidence(filename):
                filename = original.path(filename)
                require(filename.is_file(),'Historical proof missing: '+str(filename))
                hashes[str(filename)] = sha(filename)
            for filename in (original.MANIFEST,original.RELEASE,original.QUALIFICATION,
                    fast.CONTRACT,fast.ATTACHMENT):
                evidence(filename)
            native_by_chunk = {}
            for ordinal in range(EXPECTED_FINISHED//CHUNK_SIZE):
                folder = OLD/('chunk_%04d' % ordinal)
                manifest,root = folder/'manifest.json',folder/'capture'
                # Never inspect the final unsafe chunk as if it had safe Park.
                rows = batch.inspect_batch(root,manifest)
                parent.verify_regular(root,manifest)
                require(len(rows) == CHUNK_SIZE,'Completed historical chunk lost rows')
                native_by_chunk[ordinal] = {row['id']:row for row in rows}
                for filename in folder.rglob('*'):
                    if filename.is_file() and filename.suffix in ('.json','.jsonl'):
                        evidence(filename)
            lookup = {entry['id']:index for index,entry in enumerate(deck['scenes'])}
            verified_recoveries = set()
            def verify_recovery(ident,index):
                recovery = OLD/('chunk_%04d' % (index//CHUNK_SIZE))/'recovery'/ident
                require((recovery/'status.json').is_file(),
                    'Authoritative cell lacks original completed recovery proof: '+ident)
                require(batch.verify_recovery(original.MANIFEST,index,recovery) == 0,
                    'Original completed recovery proof failed')
                parent.verify_strict_recovery(recovery,_expected_recovery(parent,index))
                verified_recoveries.add(ident)
                return historical.strict_rows(recovery/'selected_ledger.jsonl'),read(recovery/'quota.json')
            for row in selected:
                ident,index = row['source_cell_id'],lookup[row['source_cell_id']]
                native = native_by_chunk[index//CHUNK_SIZE].get(ident)
                proven = None
                if native and native.get('status') == 'accepted_candidate':
                    proven = copy.deepcopy(native);proven['source_cell_id'] = ident
                if proven != row:
                    recovered,_ = verify_recovery(ident,index)
                    require(len(recovered) == 1,'Selected recovery must contain exactly one accepted row')
                    proven = recovered[0]
                require(proven == row,'Selected row differs from original native proof: '+ident)
                root = original.path(row['capture_root'])
                require(read(root/'metadata.json').get('calibration_only') is not True
                    and read(root/'audit.json') == row['audit']
                    and sha(root/'scenario.json') == row['scenario_sha256'],
                    'Selected raw audit/scenario differs or is calibration-only')
                for filename in root.rglob('*'):
                    if filename.is_file():
                        evidence(filename)
                        result = filename.stat()
                        sensor_stats[str(filename)] = [result.st_size,result.st_mtime_ns]
            for deficit in deficits:
                ident,index = deficit['id'],deficit['index']
                recovered,recovery_quota = verify_recovery(ident,index)
                require(recovered == [] and recovery_quota.get('accepted') == 0
                    and recovery_quota.get('deficit') == 1
                    and deficit.get('reasons') == recovery_quota.get('cells'),
                    'Existing deficit or exhausted recovery bounds changed')
            require(_inventory() == before and _headers() == headers,
                'Historical tree changed during CPU export')
            receipt = dict(schema=1,manifest_sha256=original.MANIFEST_SHA,
                original_output=str(OLD),selected=copy.deepcopy(selected),deficits=copy.deepcopy(deficits),
                elapsed_sec=status['elapsed_sec'],hashes=hashes,sensor_stats=sensor_stats,
                handoff_policy='Authoritative safe prefix only; final unsafe chunk excluded',
                native_capture_executed=False,captures_requested=0)
            NEW.mkdir(mode=0o700,exist_ok=False)
            _write_new(IMPORT,receipt)
            inherited,misses,elapsed = importer.verify(IMPORT,original.MANIFEST,list(range(SOURCE_COUNT)))
            require((inherited,misses,elapsed) == (selected,deficits,status['elapsed_sec']),
                'Original import verifier rejected the exact historical prefix')
            handoff = dict(schema=SCHEMA,original_output=str(OLD),output=str(NEW),
                manifest=str(original.MANIFEST),manifest_sha256=original.MANIFEST_SHA,
                release=str(original.RELEASE),release_sha256=original.RELEASE_SHA,
                selected_count=len(selected),deficit_count=len(deficits),finished=EXPECTED_FINISHED,
                unsafe_chunk=str(OLD/('chunk_%04d' % (EXPECTED_FINISHED//CHUNK_SIZE))),
                unsafe_chunk_imported=False,old_halted_status_retained=True,
                old_final_park_relabelled=False,fresh_native_park_required=True,
                import_receipt=str(IMPORT),import_receipt_sha256=sha(IMPORT),
                header_hashes=headers,helper_hashes=helpers,
                historical_inventory=before,completed_recoveries_verified=len(verified_recoveries),
                native_qa_unchanged=True,recovery_round_bounds_preserved=True,
                captures_requested=0,native_capture_executed=False)
            _write_new(HANDOFF,handoff)
            require(_inventory() == before,'Historical tree changed during handoff write')
    return verify()


def verify():
    """Read-only verification also remains valid after the new parent advances."""
    require(NEW.is_dir() and HANDOFF.is_file() and IMPORT.is_file(),
        'Complete explicit crash handoff required')
    document = read(HANDOFF)
    require(document.get('schema') == SCHEMA and document.get('original_output') == str(OLD)
        and document.get('output') == str(NEW)
        and document.get('manifest') == str(original.MANIFEST)
        and document.get('manifest_sha256') == original.MANIFEST_SHA
        and document.get('release') == str(original.RELEASE)
        and document.get('release_sha256') == original.RELEASE_SHA
        and document.get('selected_count') == EXPECTED_SELECTED
        and document.get('deficit_count') == EXPECTED_DEFICITS
        and document.get('finished') == EXPECTED_FINISHED
        and document.get('unsafe_chunk_imported') is False
        and document.get('old_halted_status_retained') is True
        and document.get('old_final_park_relabelled') is False
        and document.get('fresh_native_park_required') is True
        and document.get('native_qa_unchanged') is True
        and document.get('recovery_round_bounds_preserved') is True
        and document.get('native_capture_executed') is False and document.get('captures_requested') == 0,
        'Crash handoff policy or exact incident identity changed')
    require(document.get('import_receipt') == str(IMPORT)
        and document.get('import_receipt_sha256') == sha(IMPORT)
        and document.get('helper_hashes') == helper_hashes()
        and document.get('header_hashes') == _headers()
        and document.get('historical_inventory') == _inventory(),
        'Historical tree, import, helpers or checkpoint changed')
    _,status,_,selected,deficits = _state(fresh=False)
    _,_,importer = _modules()
    inherited,misses,elapsed = importer.verify(IMPORT,original.MANIFEST,list(range(SOURCE_COUNT)))
    require((inherited,misses,elapsed) == (selected,deficits,status['elapsed_sec']),
        'Inherited authoritative rows or deficit budgets differ')
    return document


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument('--prepare',action='store_true')
    arguments = parser.parse_args(argv)
    print(json.dumps(prepare() if arguments.prepare else plan(),ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
