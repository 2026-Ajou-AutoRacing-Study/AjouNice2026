#!/usr/bin/env python3
"""One specifically authorized deletion: halted 63-frame borrowed Sunny13 root.

Default is read-only. Never stop a process, edit the immutable bundle, delete
the native smoke, change a protected TXT or traverse an arbitrary symlink.
"""
import argparse
from contextlib import ExitStack
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import stat
import time


def require(value, message):
    if not value:
        raise ValueError(message)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def owned(path, directory):
    path = Path(path).absolute()
    for item in (path, *path.parents):
        require(not item.is_symlink(), 'Symlink in deletion scope')
    info = path.stat()
    require(info.st_uid == os.getuid()
            and (stat.S_ISDIR(info.st_mode) if directory else stat.S_ISREG(info.st_mode)),
            'Unexpected deletion path owner/type')
    return path


def reset(workspace, expected_sha=None, run=False):
    workspace = owned(workspace, True)
    require(workspace == Path.home() / 'ajounice_base5_capture', 'Only approved real HOME workspace allowed')
    root = owned(workspace / 'collections/sunny13_5k', True)
    require(not os.path.ismount(root) and root.stat().st_dev == workspace.stat().st_dev,
            'Collection root cannot be a mounted or foreign-device tree')
    work = owned(workspace / 'work', True)
    root_device = root.stat().st_dev
    status_path = owned(root / 'status.json', False)
    raw_status = status_path.read_bytes()
    status = json.loads(raw_status)
    require(status.get('stage') == 'halted' and status.get('requested') == 5000
            and status.get('accepted_candidate') == status.get('finished') == 63
            and status.get('error') == 'Single-cell recovery halted; no next load',
            'Only the explicitly authorized halted 63-frame collection may be deleted')
    observed_sha = sha(status_path)
    if run:
        require(expected_sha == observed_sha, 'Explicit unchanged status SHA required before deletion')
    files, allocated, logical, locks = 0, 0, 0, []
    for folder, directories, names in os.walk(root, followlinks=False):
        for name in directories:
            child = owned(Path(folder) / name, True)
            require(child.stat().st_dev == root_device and not os.path.ismount(child),
                    'Mountpoint in deletion scope')
        for name in names:
            path = owned(Path(folder) / name, False)
            info = path.stat(); files += 1
            require(info.st_dev == root_device, 'Foreign device in deletion scope')
            allocated += info.st_blocks * 512; logical += info.st_size
            if name.endswith('.lock') or name.endswith('_lock'):
                locks.append(path)
    capture_lock = workspace / 'work/ros-session-v2/capture.lock'
    if capture_lock.exists():
        locks.append(owned(capture_lock, False))
    with ExitStack() as stack:
        # Use existing files read-only: a diagnostic must not create locks.
        for path in locks:
            fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
            stack.callback(os.close, fd)
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        # Independently reject an active application using this collection,
        # even if it is a legacy process without one of the expected locks.
        for proc in Path('/proc').iterdir():
            if not proc.name.isdigit() or int(proc.name) == os.getpid():
                continue
            try:
                if proc.stat().st_uid != os.getuid():
                    continue
                argv = (proc / 'cmdline').read_bytes().split(b'\0')
                prefix = str(root).encode()
                require(not any(arg == prefix or arg.startswith(prefix+b'/')
                        or (b'=' in arg and (arg.split(b'=',1)[1] == prefix
                            or arg.split(b'=',1)[1].startswith(prefix+b'/'))) for arg in argv),
                        'A process still uses the collection output')
                require(not any(b'live_route_compat.py' in arg for arg in argv)
                        or b'capture' not in argv or b'--run' not in argv,
                        'New collector is active')
            except (FileNotFoundError, ProcessLookupError):
                continue
        require(status_path.read_bytes() == raw_status, 'Collection changed during deletion checks')
        diagnostics = {}
        for relative in ('status.json', 'quota.json', 'fresh_runtime_contract.json',
                         'batch_contract.json', 'optimized_contract.json'):
            path = root / relative
            if path.is_file():
                diagnostics[relative] = dict(sha256=sha(path), document=json.loads(path.read_text()))
        for path in root.glob('chunk_0003/recovery/**/report.json'):
            require(path.stat().st_size < 2 * 1024**2, 'Unexpected diagnostic report size')
            diagnostics[str(path.relative_to(root))] = dict(sha256=sha(path), document=json.loads(path.read_text()))
        before = shutil.disk_usage(workspace).free
        record = dict(schema='base5_authorized_halted63_reset_v1', root=str(root),
            status_sha256=observed_sha, accepted_deleted=63 if run else 0,
            files=files, allocated_bytes=allocated, logical_bytes=logical,
            preserved=['capture_bundle', 'collections/smoke_v1', 'work/ros-session-v2',
                       'MoraiLauncher_Lin', 'MORAI_START.txt'],
            diagnostics=diagnostics, executed=run, process_signals_sent=0,
            new_captures_requested=0)
        if run:
            report = work / ('halted63_reset_' + str(time.time_ns()) + '.json')
            # Save the small audit before deleting raw; never delete the audit.
            with report.open('x') as stream:
                json.dump(record, stream, ensure_ascii=False, indent=2);stream.write('\n')
            shutil.rmtree(root)
            require(not root.exists(), 'Collection root still present after deletion')
            record.update(report=str(report), free_before_bytes=before,
                free_after_bytes=shutil.disk_usage(workspace).free, collection_removed=True)
        return record


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--workspace', type=Path, default=Path.home() / 'ajounice_base5_capture')
    p.add_argument('--expected-status-sha256')
    p.add_argument('--run', action='store_true')
    a = p.parse_args(argv)
    print(json.dumps(reset(a.workspace, a.expected_status_sha256, a.run), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
