#!/usr/bin/env python3
"""CPU-only handoff of the 69 authoritative accepted Sunny13 captures.

The halted tree is never repaired, resumed or relabelled. The original batch
import verifier consumes the resulting schema-1 receipt in a fresh output.
"""
import argparse
from contextlib import contextmanager
import copy
import hashlib
import importlib
import importlib.abc
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import sys

import route_compat

EXPECTED_SELECTED = 69
EXPECTED_SCENES = 5000
CANONICAL_MANIFEST_SHA256 = route_compat.CANONICAL_MANIFEST_SHA256
_WRITE_SCOPES = []
_AUDIT_INSTALLED = False


def require(value, message):
    if not value:
        raise RuntimeError(message)


def read(path):
    return json.loads(Path(path).read_text())


def digest(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def within(path, root):
    try:
        Path(path).relative_to(root)
        return True
    except ValueError:
        return False


def physical(path):
    path = Path(path).absolute()
    require('..' not in path.parts and not any(p.is_symlink() for p in (path, *path.parents)),
            'Symlink/relative-parent paths are not allowed: ' + str(path))
    return path


def strict_rows(path):
    """Refuse a repairable ledger before the original loader can touch it."""
    raw = Path(path).read_bytes()
    require(not raw or raw.endswith(b'\n'), 'Torn/non-newline ledger cannot be imported: ' + str(path))
    rows = [json.loads(line) for line in raw.splitlines()]
    require(all(isinstance(row, dict) for row in rows), 'Ledger rows must be objects')
    return rows


def tree_snapshot(root):
    rows = []
    for path in sorted(root.rglob('*')):
        require(not path.is_symlink(), 'Historical capture symlink is not allowed: ' + str(path))
        result = path.stat()
        rows.append([str(path.relative_to(root)), result.st_mode, result.st_dev, result.st_ino,
                     result.st_size, result.st_mtime_ns, result.st_nlink])
    return dict(entries=len(rows), sha256=hashlib.sha256(
        json.dumps(rows, separators=(',', ':')).encode()).hexdigest())


def _audit(event, arguments):
    if not _WRITE_SCOPES:
        return
    output = _WRITE_SCOPES[-1]
    def allowed(path):
        return isinstance(path, (str, bytes, os.PathLike)) and within(
            Path(os.fsdecode(path)).absolute(), output)
    if event == 'open':
        path, mode, flags = arguments
        writing = flags & (os.O_WRONLY | os.O_RDWR | os.O_CREAT | os.O_APPEND | os.O_TRUNC)
        require(not writing or allowed(path), 'Read-only historical/export scope forbids file writes: ' + str(path))
    elif event == 'os.mkdir':
        path = Path(os.fsdecode(arguments[0])).absolute()
        # prepare_session uses mkdir(exist_ok=True), even for completed sessions.
        require(allowed(path) or path.is_dir(), 'Export cannot create an external directory: ' + str(path))
    elif event in ('os.rename', 'os.link', 'os.symlink'):
        require(False, 'Export never renames or links original files')
    elif event in ('os.remove', 'os.rmdir', 'os.chmod', 'os.chown', 'os.utime', 'os.truncate'):
        require(False, 'Export never mutates/removes original filesystem objects')
    elif event.startswith('subprocess.') or event in ('os.system', 'os.exec', 'os.posix_spawn', 'socket.connect'):
        require(False, 'Export is CPU-only; no process/network/simulator action')


@contextmanager
def readonly_cpu(output):
    """Audit filesystem writes and forbid ROS/network/process execution."""
    global _AUDIT_INSTALLED
    if not _AUDIT_INSTALLED:
        sys.addaudithook(_audit)
        _AUDIT_INSTALLED = True
    blocked_roots = {'rospy', 'roslib', 'roslaunch', 'rosbag', 'rosnode', 'rosservice',
                     'rosgraph', 'rostopic', 'rosbridge_library', 'rosbridge_server'}
    require(not any(name.split('.')[0] in blocked_roots for name in sys.modules),
            'Exporter cannot run inside a ROS process')
    class NoROS(importlib.abc.MetaPathFinder):
        def find_spec(self, fullname, path=None, target=None):
            require(fullname.split('.')[0] not in blocked_roots, 'Exporter forbids ROS imports: ' + fullname)
            return None
    def denied(*args, **kwargs):
        raise RuntimeError('Exporter forbids network/subprocess/simulator execution')
    finder = NoROS()
    functions = [(socket, 'socket'), (socket, 'create_connection'), (subprocess, 'Popen'), (os, 'system')]
    originals = [(owner, name, getattr(owner, name)) for owner, name in functions]
    previous_bytecode = sys.dont_write_bytecode
    sys.dont_write_bytecode = True
    sys.meta_path.insert(0, finder)
    _WRITE_SCOPES.append(physical(output))
    for owner, name, _ in originals:
        setattr(owner, name, denied)
    try:
        yield
    finally:
        for owner, name, function in originals:
            setattr(owner, name, function)
        _WRITE_SCOPES.pop()
        sys.meta_path.remove(finder)
        sys.dont_write_bytecode = previous_bytecode


def plan(source, old_output, new_output):
    source, old, new = map(physical, (source, old_output, new_output))
    require(old.is_dir() and new.parent.is_dir() and not new.exists(), 'Existing historical root and fresh sibling output required')
    require(not within(new, old) and not within(old, new) and not within(source, new),
            'Historical/source and new output must not overlap')
    require(digest(source) == CANONICAL_MANIFEST_SHA256, 'Frozen Sunny13 source SHA changed')
    deck, status, contract = read(source), read(old / 'status.json'), read(old / 'batch_contract.json')
    require(deck.get('count') == len(deck.get('scenes', [])) == EXPECTED_SCENES,
            'Original full 5000-cell manifest required')
    ids = [entry['id'] for entry in deck['scenes']]
    require(len(ids) == len(set(ids)) and contract.get('indices') == list(range(EXPECTED_SCENES)),
            'Original full source order must be unique and unchanged')
    require(status.get('stage') == 'halted', 'Only the preserved halted parent can be handed off')
    selected = strict_rows(old / 'selected_ledger.jsonl')
    chosen_ids = [row.get('source_cell_id') for row in selected]
    require(len(selected) == status.get('accepted_candidate') == EXPECTED_SELECTED,
            'Exactly the authoritative 69 accepted rows are required')
    quota = read(old / 'quota.json')
    require(quota.get('target') == EXPECTED_SCENES and quota.get('accepted') == len(selected)
            and all(type(quota.get(key)) is int and quota[key] >= 0 for key in ('accepted','deficit','pending'))
            and sum(quota[key] for key in ('accepted','deficit','pending')) == EXPECTED_SCENES,
            'Historical quota does not agree with the authoritative accepted rows')
    require(len(chosen_ids) == len(set(chosen_ids)) and set(chosen_ids) <= set(ids),
            'Imported IDs must be unique source cells')
    elapsed = status.get('elapsed_sec')
    require(type(elapsed) in (int, float) and math.isfinite(elapsed) and elapsed >= 0,
            'Historical elapsed time is invalid')
    require(not (old / 'imported_prefix.json').exists(), 'Nested historical imports are outside this recovery scope')
    for row in selected:
        require(row.get('status') == 'accepted_candidate' and not row.get('audit', {}).get('review_reasons'),
                'Reviewed or failed row cannot be imported')
        capture = physical(row['capture_root'])
        require(capture.is_dir() and within(capture, old), 'Selected raw must stay in the preserved old tree')
    return dict(schema='base5_borrowed_recovery_import_plan_v1', source=str(source),
                old_output=str(old), new_output=str(new), selected_count=len(selected),
                source_count=len(ids), pending_count=len(ids)-len(selected), elapsed_sec=elapsed,
                manifest_sha256=digest(source), captures_requested=0, native_capture_executed=False,
                files_written=False, original_halted_output_modified=False)


def _modules():
    return (importlib.import_module('run_base5_batch_campaign'),
            importlib.import_module('base5_campaign_import'))


def verify_runtime_guards(old, source, scripts, legacy, contract, evidence):
    fresh = read(old / 'fresh_runtime_contract.json')
    fresh_files = dict(driver_sha256='run_base5_fresh_batch.py',
        spatial_sha256='base5_spatial_diversity.py',
        recovery_sha256='run_base5_capture_campaign_spatial.py',
        overlap_driver_sha256='run_base5_batch_overlap.py',
        overlap_worker_sha256='run_base5_capture_overlap.py',
        optimized_driver_sha256='run_base5_batch_optimized.py')
    expected = {key:digest(scripts/name) for key,name in fresh_files.items()}
    expected.update(runtime_overlap=False, manifest_sha256=digest(source))
    require(fresh == expected, 'Historical fresh runtime profile changed')
    optimized = read(old / 'optimized_contract.json')
    require(optimized == dict(schema=1,manifest_sha256=digest(source),
        driver_sha256=digest(scripts/'run_base5_batch_optimized.py'),
        adapter_sha256=digest(scripts/'run_base5_capture_optimized.py'),
        setup_profile='static-ready',chunk_size=contract['chunk_size'],legacy=legacy),
        'Historical optimized static-ready profile changed')
    route = read(old / 'route_compat_contract.json')
    require(route.get('schema') == 'base5_borrowed_route_compat_intent_v1'
            and route.get('manifest_sha256') == digest(source)
            and route.get('release_sha256') == '0a3dd5c96b4522bc9d0547bed845c97ebb02d228b30ff398e1d7ed688b726180'
            and route.get('original_native_qa') is True
            and route.get('original_bounded_recovery') is True
            and route.get('original_shutdown_safety') is True,
            'Historical route compatibility safety/source profile changed')
    files = route.get('compatibility_files')
    require(isinstance(files,dict) and files and len(route.get('intent_sha256','')) == 64
            and len(route.get('witness_sha256','')) == 64,
            'Historical route compatibility executable/witness pins missing')
    for name, expected_hash in files.items():
        require(digest(physical(name)) == expected_hash, 'Historical compatibility helper/evidence changed: ' + name)
        evidence(name)
    for name in set(fresh_files.values()) | {'run_base5_capture_optimized.py'}:
        evidence(scripts/name)


def export(source, old_output, new_output):
    preparation = plan(source, old_output, new_output)
    source, old, new = map(physical, (source, old_output, new_output))
    before = tree_snapshot(old)
    with readonly_cpu(new):
        batch, importer = _modules()
        deck, status, contract = read(source), read(old / 'status.json'), read(old / 'batch_contract.json')
        selected = strict_rows(old / 'selected_ledger.jsonl')
        indices = list(range(EXPECTED_SCENES))
        legacy = batch.campaign.contract(source, indices, 2)
        expected = copy.deepcopy(legacy)
        expected.update(batch_driver_sha256=digest(batch.__file__), chunk_size=contract['chunk_size'],
            bulk=contract['bulk'], replenishment_driver_sha256=digest(
                Path(batch.__file__).with_name('base5_campaign_replenishment.py')))
        require(expected == contract, 'Original frozen batch contract changed')
        require(contract['bulk'] is True and 1 <= contract['chunk_size'] <= 16, 'Original bulk/chunk contract changed')
        hashes, sensor_stats = {}, {}
        def evidence(path):
            path = physical(path)
            require(path.is_file(), 'Historical evidence is missing: ' + str(path))
            hashes[str(path)] = digest(path)
        for path in (source, old / 'batch_contract.json', old / 'status.json',
                     old / 'selected_ledger.jsonl', old / 'quota.json', Path(__file__)):
            evidence(path)
        for name in ('fresh_runtime_contract.json', 'optimized_contract.json', 'route_compat_contract.json'):
            require((old / name).is_file(), 'Historical guarded runtime evidence missing: ' + name)
            evidence(old / name)
        verify_runtime_guards(old, source, Path(batch.__file__).parent, legacy, contract, evidence)
        # Refuse all torn ledgers before prepare_session/load_ledger could repair them.
        for path in old.rglob('*.jsonl'):
            strict_rows(path)
        lookup = {entry['id']:index for index, entry in enumerate(deck['scenes'])}
        checked_batches, checked_recoveries = {}, {}
        for row in selected:
            ident, index = row['source_cell_id'], lookup[row['source_cell_id']]
            folder = old / ('chunk_%04d' % (index // contract['chunk_size']))
            manifest, capture_root = folder / 'manifest.json', folder / 'capture'
            require(manifest.is_file(), 'Selected source chunk manifest missing')
            proven = None
            if (capture_root / 'session_contract.json').exists():
                if folder not in checked_batches:
                    native = batch.inspect_batch(capture_root, manifest)
                    checked_batches[folder] = {entry['id']:entry for entry in native}
                    for path in (manifest, capture_root / 'session_contract.json',
                                 capture_root / 'ledger.jsonl', capture_root / 'shutdown_safety.json'):
                        evidence(path)
                native = checked_batches[folder].get(ident)
                if native and native.get('status') == 'accepted_candidate':
                    proven = copy.deepcopy(native)
                    proven['source_cell_id'] = ident
            if proven != row:
                recovery = folder / 'recovery' / ident
                require((recovery / 'status.json').exists() and read(recovery / 'status.json').get('stage')
                        in ('complete', 'complete_with_deficit'),
                        'Selected row lacks completed original recovery proof: ' + ident)
                require(batch.verify_recovery(source, index, recovery) == 0,
                        'Selected completed recovery failed original native validation')
                recovered = strict_rows(recovery / 'selected_ledger.jsonl')
                require(len(recovered) == 1, 'Selected recovery must provide one original accepted row')
                proven = recovered[0]
                checked_recoveries[ident] = str(recovery)
                seed = recovery.parent / (recovery.name + '_seed.json')
                if seed.exists():
                    evidence(seed)
                for path in recovery.rglob('*'):
                    if path.is_file() and path.suffix in ('.json', '.jsonl'):
                        evidence(path)
            require(proven == row, 'Authoritative selected row differs from original native evidence: ' + ident)
            root = physical(row['capture_root'])
            metadata = read(root / 'metadata.json')
            require(metadata.get('calibration_only') is not True and read(root / 'audit.json') == row['audit'],
                    'Calibration/audit mismatch in selected raw: ' + ident)
            require(digest(root / 'scenario.json') == row['scenario_sha256'], 'Selected raw scenario changed')
            for path in root.rglob('*'):
                if path.is_file():
                    evidence(path)
                    result = path.stat()
                    sensor_stats[str(path)] = [result.st_size, result.st_mtime_ns]
        # Include every original source payload/metadata and every frozen code/config
        # expected by the original batch contract, not just the 69 selected slots.
        scripts = Path(batch.__file__).parent
        for name, expected_hash in contract['code'].items():
            evidence(scripts / name)
            require(hashes[str(physical(scripts / name))] == expected_hash, 'Historical core runtime changed')
        for entry in deck['scenes']:
            evidence(entry['scenario'])
            require(digest(entry['scenario']) == entry['scenario_sha256'], 'Frozen scenario changed')
            evidence(entry['metadata'])
        after = tree_snapshot(old)
        require(before == after, 'Historical tree changed during read-only import verification')
        receipt = dict(schema=1, manifest_sha256=digest(source), original_output=str(old),
            selected=copy.deepcopy(selected), deficits=[], elapsed_sec=status['elapsed_sec'],
            hashes=hashes, sensor_stats=sensor_stats,
            handoff_policy='69 authoritative accepted rows only; unfinished/review/orphan rows never imported',
            exporter_sha256=digest(__file__), native_capture_executed=False, captures_requested=0,
            historical_snapshot_before=before, historical_snapshot_after=after,
            validation=dict(original_batch_contract=True, original_native_inspect_batch=True,
                            completed_recovery_count=len(checked_recoveries),
                            selected_exactly_equal_to_original_authoritative_ledger=True,
                            raw_checksum_scope='All files of the 69 selected captures; all original source scenario/metadata',
                            old_halted_status_retained=True, qa_waived=False))
        new.mkdir(exist_ok=False)
        destination = new / 'imported_prefix.json'
        with destination.open('x') as stream:
            json.dump(receipt, stream, ensure_ascii=False, indent=2)
            stream.write('\n')
        verified, deficits, elapsed = importer.verify(destination, source, indices)
        require(verified == selected and deficits == [] and elapsed == status['elapsed_sec'],
                'Original schema-1 import verifier rejected exact handoff')
        require([path.name for path in new.iterdir()] == ['imported_prefix.json'] and tree_snapshot(old) == before,
                'Exporter must not create a new batch contract or change the old tree')
    return dict(preparation, files_written=True, imported_prefix=str(destination),
                imported_prefix_sha256=digest(destination), original_import_verifier_passed=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--old-output', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--run', action='store_true', help='Allow CPU-only creation of the fresh imported_prefix receipt')
    args = parser.parse_args(argv)
    result = export(args.manifest, args.old_output, args.output) if args.run else plan(
        args.manifest, args.old_output, args.output)
    print(json.dumps(result, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
