#!/usr/bin/env python3
"""One-off, explicitly approved deletion of four obsolete capture directories.

This tool never follows capture symlinks and never accepts arbitrary targets.
The approval scope is fixed in TARGETS, not inferred from directory names.
"""

import argparse
import collections
import datetime
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import stat
import subprocess
import tarfile
import time


REPO = Path(__file__).resolve().parents[2]
CAP = Path('/home/luv/morai_cap')
TARGETS = (
    CAP / 'base5_v2_261006',
    CAP / 'base5_v2_261007',
    CAP / 'base5_postcollection_261010/runtime_benchmark',
    CAP / '260829_161242_base3_dynamic_ped_addon_v1_final75',
)
OUTPUT = REPO / 'artifacts/cleanup/approved_storage_261018/legacy_captures'
RELEASES = (
    REPO / 'artifacts/evaluation/base5_unit_release_261017/release_v158a/release.json',
    REPO / 'artifacts/evaluation/base5_sunny15_261017/finalization_v1/release_v159_sunny15/release.json',
)
CANONICAL = CAP / ('base5_v2_261008/final_appearance_5k_v132_release/'
                   'collection_5k_sunny13_flexible_v140/selected_ledger.jsonl')
MAX_SMALL_FILE = 1024 * 1024
MAX_SMALL_TOTAL_PER_ROOT = 16 * 1024 * 1024
SMALL_NAME = re.compile(r'(manifest|report|summary|status|contract|audit|quota|ledger|receipt)', re.I)


def stamp():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def dump(path, value):
    with path.open('x') as out:
        json.dump(value, out, ensure_ascii=False, indent=2, sort_keys=True)
        out.write('\n')


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def inside(path, root):
    return os.path.commonpath((str(path), str(root))) == str(root)


def conflict(path, root):
    return inside(path, root) or inside(root, path)


def strings(value):
    if isinstance(value, dict):
        for key, item in value.items():
            yield str(key)
            yield from strings(item)
    elif isinstance(value, (list, tuple)):
        for item in value:
            yield from strings(item)
    elif isinstance(value, str):
        yield value


def current_protection():
    pins = {}
    release_digests = {}
    references = set()
    for path in RELEASES:
        release_digests[str(path)] = sha(path)
        document = json.loads(path.read_text())
        for key in ('runtime_hashes', 'input_hashes'):
            for name, expected in document[key].items():
                if name in pins and pins[name] != expected:
                    raise RuntimeError('Conflicting release hashes: ' + name)
                pins[name] = expected
        for value in strings(document):
            if value.startswith('/'):
                references.add(value)
    captures = []
    for line in CANONICAL.read_text().splitlines():
        entry = json.loads(line)
        root = Path(entry['capture_root'])
        if not root.is_dir():
            raise RuntimeError('Canonical capture is missing: ' + str(root))
        captures.append(str(root))
    if len(captures) != 5000:
        raise RuntimeError('Expected exactly 5000 canonical captures')
    references.update(captures)
    references.add(str(CANONICAL))
    for reference in references:
        path = Path(reference)
        # References which are broad parent roots do not identify data to delete.
        # Actual pin keys and capture roots are additionally checked below.
        if any(inside(path, target) for target in TARGETS):
            raise RuntimeError('Deletion target contains a release/canonical reference: ' + reference)
        resolved = path.resolve()
        if any(inside(resolved, target.resolve()) for target in TARGETS):
            raise RuntimeError('Resolved release/canonical reference conflicts: ' + reference)
    return pins, {
        'release_sha256': release_digests,
        'canonical_ledger': str(CANONICAL),
        'canonical_ledger_sha256': sha(CANONICAL),
        'canonical_selected_count': len(captures),
        'canonical_capture_roots_sha256': hashlib.sha256(
            json.dumps(captures, separators=(',', ':')).encode()).hexdigest(),
        'pinned_files': len(pins),
        'absolute_reference_count': len(references),
        'conflicting_references': 0,
    }


def verify_pins(pins):
    logical_bytes = 0
    for name, expected in sorted(pins.items()):
        path = Path(name)
        if not path.is_file() or sha(path) != expected:
            raise RuntimeError('Current release pin is missing/changed: ' + name)
        logical_bytes += path.stat().st_size
    return {'files_verified': len(pins), 'logical_bytes_hashed': logical_bytes}


def checked_root(root):
    if root not in TARGETS or not inside(root, CAP) or root == CAP:
        raise RuntimeError('Unapproved path: ' + str(root))
    for ancestor in [root] + list(root.parents):
        if ancestor.is_symlink():
            raise RuntimeError('Symlink target/ancestor is forbidden: ' + str(ancestor))
    result = root.lstat()
    if not stat.S_ISDIR(result.st_mode):
        raise RuntimeError('Target is not a physical directory: ' + str(root))
    for line in Path('/proc/self/mountinfo').read_text().splitlines():
        mount = line.split()[4]
        mount = re.sub(r'\\([0-7]{3})', lambda match: chr(int(match.group(1), 8)), mount)
        if inside(Path(mount), root):
            raise RuntimeError('Nested mount prevents deletion: ' + mount)
    return (result.st_dev, result.st_ino)


def entry_record(path, root, result):
    record = {
        'path': str(path.relative_to(root)), 'mode': result.st_mode,
        'device': result.st_dev, 'inode': result.st_ino,
        'nlink': result.st_nlink, 'logical_bytes': result.st_size,
        'allocated_bytes': result.st_blocks * 512,
        'mtime_ns': result.st_mtime_ns,
    }
    if stat.S_ISLNK(result.st_mode):
        record['symlink_target'] = os.readlink(str(path))
    return record


def inventory(root, output=None):
    identity = checked_root(root)
    counts = collections.Counter()
    inodes = {}
    digest = hashlib.sha256()
    candidates = []
    handle = gzip.open(str(output), 'xt', encoding='utf-8') if output else None
    try:
        for current, dirs, files in os.walk(str(root), followlinks=False):
            dirs.sort()
            files.sort()
            for name in ['.'] + dirs + files:
                path = Path(current) if name == '.' else Path(current) / name
                # Each directory gets one record via its own visit, not its parent.
                if name != '.' and path.is_dir() and not path.is_symlink():
                    continue
                result = path.lstat()
                if result.st_dev != identity[0]:
                    raise RuntimeError('Device boundary inside target: ' + str(path))
                if not (stat.S_ISREG(result.st_mode) or stat.S_ISDIR(result.st_mode)
                        or stat.S_ISLNK(result.st_mode)):
                    raise RuntimeError('Unexpected special file: ' + str(path))
                kind = ('file' if stat.S_ISREG(result.st_mode) else
                        'directory' if stat.S_ISDIR(result.st_mode) else 'symlink')
                counts[kind] += 1
                record = entry_record(path, root, result)
                line = json.dumps(record, sort_keys=True, separators=(',', ':')) + '\n'
                digest.update(line.encode())
                if handle:
                    handle.write(line)
                if kind == 'file':
                    key = (result.st_dev, result.st_ino)
                    inode = inodes.setdefault(key, {
                        'allocated': result.st_blocks * 512,
                        'nlink': result.st_nlink, 'occurrences': 0,
                    })
                    if inode['nlink'] != result.st_nlink:
                        raise RuntimeError('Hardlink count changed during scan')
                    inode['occurrences'] += 1
                    if (SMALL_NAME.search(name) and path.suffix.lower() in ('.json', '.jsonl', '.txt', '.md')
                            and result.st_size <= MAX_SMALL_FILE):
                        candidates.append((len(path.relative_to(root).parts), str(path), result.st_size))
    finally:
        if handle:
            handle.close()
    external = [value for value in inodes.values() if value['nlink'] > value['occurrences']]
    summary = {
        'path': str(root), 'root_device_inode': list(identity),
        'counts': dict(counts), 'unique_regular_inodes': len(inodes),
        'allocated_unique_file_bytes': sum(value['allocated'] for value in inodes.values()),
        'allocated_single_link_bytes': sum(value['allocated'] for value in inodes.values() if value['nlink'] == 1),
        'allocated_all_links_inside_bytes': sum(value['allocated'] for value in inodes.values()
                                                if value['nlink'] == value['occurrences']),
        'allocated_external_shared_bytes': sum(value['allocated'] for value in external),
        'external_shared_inode_count': len(external),
        'inventory_sha256_uncompressed': digest.hexdigest(),
    }
    if any(value['occurrences'] > value['nlink'] for value in inodes.values()):
        raise RuntimeError('Invalid hardlink inventory')
    return summary, set(inodes), sorted(candidates)


def open_file_conflicts(inodes):
    conflicts = []
    unreadable = []
    for process in Path('/proc').iterdir():
        if not process.name.isdigit():
            continue
        try:
            descriptors = list((process / 'fd').iterdir())
        except FileNotFoundError:
            continue
        except PermissionError:
            try:
                owner = process.stat().st_uid
                command = (process / 'comm').read_text().strip()
            except FileNotFoundError:
                continue
            unreadable.append({'pid': int(process.name), 'uid': owner, 'comm': command})
            continue
        for descriptor in descriptors + [process / 'cwd', process / 'exe', process / 'root']:
            try:
                link = os.readlink(str(descriptor))
                result = descriptor.stat()
            except (FileNotFoundError, ProcessLookupError, PermissionError, OSError):
                continue
            lexical_match = link.startswith('/') and any(inside(Path(link.removesuffix(' (deleted)')
                    if hasattr(str, 'removesuffix') else link.replace(' (deleted)', '')), root) for root in TARGETS)
            if lexical_match or (result.st_dev, result.st_ino) in inodes:
                conflicts.append({'pid': int(process.name), 'reference': descriptor.name, 'path': link})
        try:
            maps = (process / 'maps').read_text()
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        for line in maps.splitlines():
            fields = line.split(None, 5)
            if len(fields) == 6 and fields[5].startswith('/') and any(
                    inside(Path(fields[5]), root) for root in TARGETS):
                conflicts.append({'pid': int(process.name), 'reference': 'maps', 'path': fields[5]})
    # User-approved limitation: no privilege bypass to inspect unrelated OS/security
    # services. An inaccessible capture/development/runtime process remains a blocker.
    own_service_names = {'(sd-pam)', 'gnome-keyring-d', 'ssh-agent', 'sunshine'}
    app_name = re.compile(r'(python|ros|morai|unity|train|collect|capture|node|code|cmake|make|docker-)', re.I)
    blocked = [item for item in unreadable if
               (item['uid'] == os.getuid() and item['comm'] not in own_service_names)
               or app_name.search(item['comm'])]
    if blocked:
        raise RuntimeError('Cannot inspect a possible capture/development process: ' + json.dumps(blocked))
    if conflicts:
        raise RuntimeError('Capture files are in use: ' + json.dumps(conflicts))
    check = subprocess.run(['fuser', '-v'] + [str(root) for root in TARGETS],
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                           encoding='utf-8', timeout=30, check=False)
    if check.returncode == 0 or check.stdout.strip():
        raise RuntimeError('fuser found target users: ' + check.stdout + check.stderr)
    if check.returncode != 1:
        raise RuntimeError('fuser inspection failed: ' + check.stderr)
    return {'open_target_files': 0, 'unreadable_unrelated_processes': unreadable,
            'inspection_scope': 'All readable fd/cwd/exe/root/maps; unreadable unrelated OS/security services explicitly excluded.',
            'fuser_exit_code': check.returncode, 'fuser_stderr': check.stderr,
            'privilege_bypass': False}


def preserve_small(root, candidates, archive):
    selected = []
    skipped = []
    total = 0
    with tarfile.open(str(archive), 'x:gz', dereference=False) as out:
        for _, name, size in candidates:
            path = Path(name)
            relative = str(path.relative_to(root))
            if total + size > MAX_SMALL_TOTAL_PER_ROOT:
                skipped.append(relative)
                continue
            if path.is_symlink() or not path.is_file() or path.stat().st_size != size:
                raise RuntimeError('Archival source changed: ' + name)
            selected.append({'path': relative, 'bytes': size, 'sha256': sha(path)})
            out.add(str(path), arcname=relative, recursive=False)
            total += size
    return {'files': selected, 'logical_bytes': total, 'skipped_over_budget': skipped,
            'max_file_bytes': MAX_SMALL_FILE, 'max_total_bytes': MAX_SMALL_TOTAL_PER_ROOT,
            'archive_sha256': sha(archive)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute-approved-four-directories', action='store_true', required=True)
    parser.parse_args()
    if OUTPUT.exists():
        raise RuntimeError('Audit output already exists; refusing to overwrite')
    OUTPUT.mkdir(parents=True)
    started = time.monotonic()
    pins, protected_before = current_protection()
    dump(OUTPUT / 'protected_before.json', protected_before)
    print(json.dumps({'event': 'verifying_current_release_pins', 'files': len(pins)}), flush=True)
    hashes_before = verify_pins(pins)
    dump(OUTPUT / 'verified_pins_before.json', hashes_before)
    inventories = []
    all_inodes = set()
    for index, root in enumerate(TARGETS):
        label = '{:02d}_{}'.format(index + 1, root.name)
        print(json.dumps({'event': 'inventory', 'path': str(root)}), flush=True)
        summary, inodes, candidates = inventory(root, OUTPUT / (label + '.inventory.jsonl.gz'))
        summary['small_evidence'] = preserve_small(root, candidates, OUTPUT / (label + '.small_evidence.tar.gz'))
        dump(OUTPUT / (label + '.summary.json'), summary)
        inventories.append(summary)
        all_inodes.update(inodes)
    process_scan = open_file_conflicts(all_inodes)
    # Stat-digest every original entry again, detecting modifications during archival.
    for summary, root in zip(inventories, TARGETS):
        current, _, _ = inventory(root)
        if (current['root_device_inode'] != summary['root_device_inode'] or
                current['inventory_sha256_uncompressed'] != summary['inventory_sha256_uncompressed']):
            raise RuntimeError('Capture tree changed during preflight: ' + str(root))
    _, protected_recheck = current_protection()
    if protected_recheck != protected_before:
        raise RuntimeError('Current protection inputs changed before deletion')
    process_scan = open_file_conflicts(all_inodes)
    plan = {
        'schema': 'approved_legacy_capture_cleanup_261018_v1',
        'approved_exact_targets': [str(path) for path in TARGETS],
        'started_utc': stamp(), 'protection': protected_before,
        'process_scan': process_scan, 'inventories': inventories,
        'symlinks_followed': False, 'arbitrary_target_arguments_supported': False,
        'external_hardlinks_are_not_reclaimed': True,
    }
    dump(OUTPUT / 'deletion_plan.json', plan)
    deleted = []
    for summary, root in zip(inventories, TARGETS):
        if list(checked_root(root)) != summary['root_device_inode']:
            raise RuntimeError('Target identity changed before deletion')
        print(json.dumps({'event': 'deleting_approved_root', 'path': str(root)}), flush=True)
        shutil.rmtree(str(root))
        if root.exists() or root.is_symlink():
            raise RuntimeError('Target still exists after deletion: ' + str(root))
        deleted.append(str(root))
        dump(OUTPUT / ('deleted_{:02d}.json'.format(len(deleted))), {'path': str(root), 'deleted_utc': stamp()})
    pins_after, protected_after = current_protection()
    if pins_after != pins or protected_after != protected_before:
        raise RuntimeError('Release or canonical protection changed after deletion')
    print(json.dumps({'event': 'verifying_unchanged_release_pins', 'files': len(pins)}), flush=True)
    hashes_after = verify_pins(pins_after)
    final = {
        'schema': 'approved_legacy_capture_cleanup_result_261018_v1',
        'passed': True, 'finished_utc': stamp(), 'elapsed_seconds': time.monotonic() - started,
        'deleted_exact_paths': deleted, 'deleted_paths_present': sum(path.exists() for path in TARGETS),
        'protected_before': protected_before, 'protected_after': protected_after,
        'pins_before': hashes_before, 'pins_after': hashes_after,
        'allocated_all_links_inside_bytes_before': sum(value['allocated_all_links_inside_bytes'] for value in inventories),
        'allocated_external_shared_bytes_not_claimed': sum(value['allocated_external_shared_bytes'] for value in inventories),
        'small_evidence_note': 'Selected small reports only; this is not a backup of raw captures.',
        'helper_sha256': sha(Path(__file__)),
    }
    dump(OUTPUT / 'cleanup_result.json', final)
    print(json.dumps(final, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
