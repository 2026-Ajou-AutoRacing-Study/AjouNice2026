#!/usr/bin/env python3
"""사용자가 승인한 2026-10-18 Group 1 파일만 정리하는 일회성 실행기.

기본은 계획만 출력한다. --run도 아래 고정 allowlist 외의 파일은 삭제하지 않는다.
ROS/MORAI/원격/서비스/패키지 관리자는 실행하거나 중단하지 않는다.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import time

HOME = Path('/home/luv')
REPO = HOME/'AjouNice2026'
OUTPUT = REPO/'artifacts/cleanup/approved_storage_261018/archives_caches'
BACKUP = REPO/'artifacts/backups/base5_sunny13_5k_v140_261010'
PARTS = BACKUP/'parts'
PIP = HOME/'.cache/pip'
PKGS = HOME/'miniconda3/pkgs'
OLD_SESSIONS = tuple(HOME/'.ros/log'/name for name in (
    'ae98da16-b876-11f1-bc67-573cfa8d2ddd',
    '6685f550-b877-11f1-bc67-573cfa8d2ddd',
    '7e9b166a-b8c3-11f1-bc67-573cfa8d2ddd'))
CURRENT_ROS = HOME/'.ros/log/130333b8-c9e6-11f1-a14d-ad5dd0b3acdc'
RELEASES = (
    REPO/'artifacts/evaluation/base5_unit_release_261017/release_v158a/release.json',
    REPO/'artifacts/evaluation/base5_sunny15_261017/finalization_v1/release_v159_sunny15/release.json')
PRESERVED_ROOTS = (
    REPO/'artifacts/handoff/base5_sunny13_portable_261010',
    REPO/'artifacts/handoff/base5_capture_portable_261017/capture_bundle')
ARCHIVES = {
    REPO/'artifacts/handoff/base5_remote_code_261010/base5_sunny13_portable.tar.zst':
        '5e2a85c2b40ffdccc7a08ddaa235cfa965227af64ff3b6033761d36e141e5efd',
    REPO/'artifacts/handoff/base5_capture_portable_261017/base5_capture_native_v1_261017.tar.zst':
        'e56cb9493a18579c636c06b34171e0c34ab30c5a6424fc35703adbb6fc2282d1'}
BACKUP_SHA = 'ab24307ee17007c4170dacabb00e6c97130e090ddbdb45f0e99c38b244a8374f'
FD_SCAN_LIMITATIONS = {}
PROTECTED_SECURITY_DAEMONS = ('(sd-pam)', 'gnome-keyring-d', 'ssh-agent', 'gpg-agent', 'sunshine')


def require(condition, message):
    if not condition:
        raise ValueError(message)


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(8*1024*1024), b''):
            value.update(block)
    return value.hexdigest()


def info(path):
    result = path.lstat()
    return dict(device=result.st_dev, inode=result.st_ino, bytes=result.st_size,
                allocated_bytes=result.st_blocks*512, mtime_ns=result.st_mtime_ns,
                uid=result.st_uid, mode=result.st_mode, hard_links=result.st_nlink)


def safe(path, directory=False):
    require(path.is_absolute() and '..' not in path.parts, 'Absolute normalized path required')
    for parent in (path, *path.parents):
        require(not parent.is_symlink(), 'Symlink forbidden: '+str(parent))
        if parent != Path('/'):
            require(not os.path.ismount(parent), 'Mount boundary forbidden: '+str(parent))
    result = path.lstat()
    require(result.st_uid == os.getuid(), 'Foreign owner: '+str(path))
    require(stat.S_ISDIR(result.st_mode) if directory else stat.S_ISREG(result.st_mode),
            'Unexpected file type: '+str(path))
    require(result.st_dev == REPO.stat().st_dev, 'Different device: '+str(path))
    return result


def references(value):
    if isinstance(value, dict):
        for key, item in value.items():
            if isinstance(key, str) and key.startswith('/'):
                yield Path(key)
            yield from references(item)
    elif isinstance(value, list):
        for item in value:
            yield from references(item)
    elif isinstance(value, str) and value.startswith('/'):
        yield Path(value)


def active_installers():
    matches = []
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            if proc.stat().st_uid != os.getuid():
                continue
            command = (proc/'cmdline').read_bytes().split(b'\0')[:5]
            names = [Path(os.fsdecode(value)).name for value in command if value]
            if any(name in ('pip', 'pip3', 'conda', 'mamba', 'micromamba', 'apt', 'apt-get', 'dpkg')
                   for name in names):
                matches.append(int(proc.name))
        except PermissionError:
            name = (proc/'comm').read_text().strip()
            require(name in PROTECTED_SECURITY_DAEMONS, 'Cannot inspect owned process descriptors: '+proc.name+' '+name)
            FD_SCAN_LIMITATIONS[proc.name] = name+': OS /proc access denied; not bypassed or stopped'
        except (FileNotFoundError, ProcessLookupError):
            pass
    return matches


def open_candidates(paths):
    candidates = {(path.stat().st_dev, path.stat().st_ino): str(path) for path in paths}
    opened = []
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            if proc.stat().st_uid != os.getuid():
                continue
            for descriptor in (proc/'fd').iterdir():
                try:
                    item = descriptor.stat()
                    if (item.st_dev, item.st_ino) in candidates:
                        opened.append(dict(pid=int(proc.name), path=candidates[(item.st_dev,item.st_ino)]))
                except (FileNotFoundError, ProcessLookupError):
                    pass
        except PermissionError:
            name = (proc/'comm').read_text().strip()
            require(name in PROTECTED_SECURITY_DAEMONS, 'Cannot inspect owned process descriptors: '+proc.name+' '+name)
            FD_SCAN_LIMITATIONS[proc.name] = name+': OS /proc access denied; not bypassed or stopped'
        except (FileNotFoundError, ProcessLookupError):
            pass
    return opened


def frozen_context():
    runtime, protected, releases = {}, set(PRESERVED_ROOTS), {}
    for path in RELEASES:
        safe(path)
        doc = json.loads(path.read_text())
        releases[str(path)] = digest(path)
        protected.update(references(doc))
        for filename, expected in doc['runtime_hashes'].items():
            candidate = Path(filename)
            require(digest(candidate) == expected, 'Frozen runtime mismatch: '+filename)
            runtime[filename] = expected
    return dict(release_sha256=releases, runtime_hashes=runtime,
                protected_paths=sorted(map(str, protected)))


def plan():
    expected, targets = dict(ARCHIVES), {}
    for path in ARCHIVES:
        targets[path] = 'duplicate_archive'
    part_paths = sorted(PARTS.glob('*.tar.zst.part[0-9][0-9][0-9]'))
    require(len(part_paths) == 19, 'Exactly 19 approved split archives required')
    require([path.name.rsplit('part',1)[1] for path in part_paths] == [f'{n:03}' for n in range(19)],
            'Exact part sequence required')
    part_hashes = {name: sha for sha,name in (line.split() for line in (PARTS/'SHA256SUMS').read_text().splitlines())}
    for path in part_paths:
        require(path.name in part_hashes, 'Part lacks preserved SHA')
        targets[path] = 'duplicate_backup_part'; expected[path] = part_hashes[path.name]
    safe(PIP, True)
    for root, directories, files in os.walk(PIP, followlinks=False):
        for name in directories:
            safe(Path(root)/name, True)
        for name in files:
            targets[Path(root)/name] = 'pip_download_cache'
    package_paths = sorted([*PKGS.glob('*.conda'), *PKGS.glob('*.tar.bz2')])
    require(len(package_paths) == 153, 'Exactly 153 approved Conda archives required')
    for path in package_paths:
        targets[path] = 'conda_archive_only'
    for root in OLD_SESSIONS:
        safe(root, True)
        for path in root.iterdir():
            if re.fullmatch(r'rosout\.log\.[0-9]+', path.name):
                targets[path] = 'old_rotated_rosout_only'
    require(sum(group == 'old_rotated_rosout_only' for group in targets.values()) == 30,
            'Only 30 explicitly old-session rotated logs expected')
    for path in targets:
        safe(path)
        require(CURRENT_ROS not in path.parents, 'Current ROS session must be preserved')
    require(not active_installers(), 'Active installer detected; caches must not be touched')
    opened = open_candidates(targets)
    require(not opened, 'Approved candidate currently open: '+json.dumps(opened))
    for root in PRESERVED_ROOTS:
        safe(root, True)
    preserved = [*PRESERVED_ROOTS, CURRENT_ROS, BACKUP/'base5_sunny13_5k_v140_261010.tar.zst',
                 *(path for path in PARTS.iterdir() if path not in targets),
                 *(path for root in OLD_SESSIONS for path in root.iterdir() if path not in targets)]
    snapshots = {str(path): info(path) for path in preserved}
    frozen = frozen_context()
    protected_paths = {Path(reference) for reference in frozen['protected_paths']}
    protected_ancestors = {parent for path in protected_paths for parent in path.parents}
    collision = [str(path) for path in targets
                 if path in protected_paths or path in protected_ancestors
                 or any(parent in protected_paths for parent in path.parents)]
    require(not collision, 'Frozen source/runtime/input/gate collision: '+json.dumps(collision))
    return targets, expected, snapshots, frozen


def save(report):
    path = OUTPUT/'report.json'
    temporary = OUTPUT/'report.json.tmp'
    with temporary.open('x') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write('\n');stream.flush();os.fsync(stream.fileno())
    temporary.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', action='store_true')
    args = parser.parse_args()
    targets, expected, preserved, frozen = plan()
    report = dict(schema='approved_storage_archives_caches_261018_v1', approved_scope='Group 1 exact five scopes',
                  started_at=time.time(), run=args.run, files=[], protected=frozen, preserved_before=preserved,
                  frozen_path_collisions=0, services_changed=False, remote_contacted=False,
                  retained_backup_root_sha256=BACKUP_SHA,
                  retained_backup_verification='Parent independently verified full archive + 19-part concatenation SHA immediately before this run; no backup source is deleted.',
                  cache_hash_policy='SHA256 computed for every deleted regular file, including caches',
                  fd_scan_limitations=FD_SCAN_LIMITATIONS,
                  planned_files=len(targets), planned_bytes=sum(info(path)['bytes'] for path in targets))
    if not args.run:
        print(json.dumps({key:value for key,value in report.items() if key not in ('protected','preserved_before')}, indent=2))
        return
    OUTPUT.mkdir(parents=True, exist_ok=False, mode=0o700)
    save(report)
    try:
        ordered = list(targets.items())
        for index,(path, group) in enumerate(ordered):
            if index % 50 == 0 or group.startswith('duplicate'):
                require(not open_candidates([row[0] for row in ordered[index:index+50]]),
                        'Approved candidate opened before batch unlink')
                require(not active_installers(), 'Installer appeared during cleanup')
            before = info(path); sha = digest(path)
            require(path not in expected or sha == expected[path], 'Archive SHA mismatch: '+str(path))
            safe(path)
            require(info(path) == before, 'File changed during checksum: '+str(path))
            path.unlink()
            report['files'].append(dict(path=str(path), group=group, before=before, sha256=sha,
                                        deleted=not path.exists(), deleted_at=time.time()))
            if index % 50 == 0 or group.startswith('duplicate'):
                save(report)
                print(json.dumps(dict(event='approved_cleanup_progress', completed=index+1, total=len(targets), group=group)), flush=True)
        for root, directories, files in os.walk(PIP, topdown=False):
            path = Path(root)
            if path != PIP and not any(path.iterdir()):
                path.rmdir()
        report['preserved_after'] = {path:info(Path(path)) for path in preserved}
        require(report['preserved_after'] == preserved, 'Preserved path changed')
        report['protected_after'] = frozen_context()
        require(report['protected_after'] == frozen, 'Frozen source/runtime references changed')
        report.update(passed=True, completed_at=time.time(), deleted_files=len(report['files']),
                      deleted_bytes=sum(row['before']['bytes'] for row in report['files']),
                      allocated_bytes_unlinked=sum(row['before']['allocated_bytes'] for row in report['files']),
                      note='Unlinked bytes can differ from actual free-space change because of hardlinks and concurrent approved cleanup agents.')
    except BaseException as error:
        report.update(passed=False, error=str(error), error_type=type(error).__name__)
        save(report)
        raise
    save(report)
    print(json.dumps(dict(event='approved_cleanup_complete', passed=True, report=str(OUTPUT/'report.json'),
                          deleted_files=report['deleted_files'], deleted_GiB=report['deleted_bytes']/2**30)), flush=True)


if __name__ == '__main__':
    main()
