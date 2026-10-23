#!/usr/bin/env python3
"""Request a chunk-boundary pause; explicitly release a proven safe pause.

No signals, process starts, marker deletion, system writes, or TXT edits.
The default is a read-only plan. Resuming here only archives PAUSE; the
operator must separately start the already qualified collection command.
"""
import argparse
from contextlib import ExitStack, contextmanager
import fcntl
import json
import math
import os
from pathlib import Path
import stat
import sys
import time
import uuid

try:
    # checks/live.py can still be the historical bootstrap executor. Prefer
    # the separately approved v2 host contract rather than silently using it.
    import live_v2 as live
except ModuleNotFoundError as error:
    if error.name != 'live_v2':
        raise
    # Local source/test checkout keeps the current executor as live.py.
    import live


CHILD_LOCKS = {'.fresh.lock', '.batch.lock', '.campaign.lock', '.quality.lock',
               '.runner.lock', '.spatial.lock'}
ENDED_NATIVE = {'complete', 'halted', 'interrupted', 'stopped_for_resume_test'}


def output_root(ctx):
    return live.owned(ctx['workspace'] / 'collections/sunny13_5k')


@contextmanager
def idle_lock(path):
    """Acquire an existing lock without creating/truncating any runtime file."""
    path = live.owned(path, False)
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
    try:
        info = os.fstat(fd)
        expected = path.stat()
        live.require(stat.S_ISREG(info.st_mode) and info.st_uid == os.getuid()
                     and not info.st_mode & 0o022
                     and (info.st_dev, info.st_ino) == (expected.st_dev, expected.st_ino),
                     'Runtime lock changed while opening it')
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise ValueError('Collection/child is still active; keep ROS and bridge running') from error
        yield
    finally:
        os.close(fd)


def read_owned(path):
    value = live.read(live.owned(path, False))
    live.require(isinstance(value, dict), 'Expected an object-shaped runtime record: ' + str(path))
    return value


def resume_evidence(root, stack):
    """Inspect every native session, including failed attempts later recovered.

    Checking only a selected capture or one latest Park file could hide an
    unsafe previous child. A parent paused checkpoint is required in addition
    to all child exit records. This is recorded Park evidence, not a claim
    that the current simulator position has been physically re-observed.
    """
    live.require(not (root / 'STOP').exists() and not (root / 'STOP').is_symlink(),
                 'STOP is present; a pause release must not bypass a safety stop')
    status_path = root / 'status.json'
    status = read_owned(status_path)
    live.require(status.get('stage') == 'paused' and status.get('current') is None,
                 'A finished parent paused checkpoint is required')
    updated = status.get('updated_at')
    live.require(isinstance(updated, (int, float)) and not isinstance(updated, bool)
                 and math.isfinite(updated), 'Parent checkpoint timestamp is missing')
    live.require(status.get('requested') == 5000
                 and type(status.get('finished')) is int and 0 < status['finished'] <= 5000,
                 'Expected the nonempty approved 5K campaign checkpoint')
    sessions, locks, shutdowns = [], [], []
    for folder, directories, files in os.walk(root, followlinks=False):
        folder = Path(folder)
        # Do not let a symlink hide an unexamined session or redirect an audit.
        for name in directories + files:
            live.no_symlinks(folder / name)
        live.owned(folder)
        locks.extend(folder / name for name in files if name in CHILD_LOCKS)
        if 'session_contract.json' in files:
            sessions.append(folder)
        if 'shutdown_safety.json' in files:
            shutdowns.append(folder / 'shutdown_safety.json')
    for path in sorted(locks):
        stack.enter_context(idle_lock(path))
    live.require(sessions, 'No native session exit evidence; preserve PAUSE for manual inspection')
    expected_shutdowns = {folder / 'shutdown_safety.json' for folder in sessions}
    live.require(set(shutdowns) == expected_shutdowns,
                 'Native shutdown records and session contracts do not match')
    evidence = {}
    for folder in sorted(sessions):
        read_owned(folder / 'session_contract.json')
        native = read_owned(folder / 'status.json')
        live.require(native.get('stage') in ENDED_NATIVE,
                     'A native child is incomplete or its exit safety failed: ' + str(folder))
        ledger = live.owned(folder / 'ledger.jsonl', False)
        rows = [json.loads(line) for line in ledger.read_text().splitlines() if line.strip()]
        live.require(rows and all(isinstance(row, dict) for row in rows),
                     'Native session ledger is missing or empty: ' + str(folder))
        shutdown_path = folder / 'shutdown_safety.json'
        shutdown = read_owned(shutdown_path)
        live.require(shutdown.get('park_verified') is True and type(shutdown.get('gear')) is int
                     and shutdown.get('gear') == 1
                     and shutdown.get('brake_held_during_requests') is True
                     and shutdown.get('request_fields') == {'option': 2, 'gear': 1},
                     'Native final Park/brake evidence is incomplete: ' + str(folder))
        live.require(shutdown_path.stat().st_mtime <= updated + 1,
                     'Native exit is newer than parent paused checkpoint')
        evidence[str(folder.relative_to(root))] = dict(
            session_sha256=live.digest(folder / 'session_contract.json'),
            status_sha256=live.digest(folder / 'status.json'),
            ledger_sha256=live.digest(ledger), shutdown_sha256=live.digest(shutdown_path))
    return dict(parent_status_sha256=live.digest(status_path), native_sessions=evidence,
                recorded_park_only=True, simulator_rest_reobserved=False)


def pause(ctx):
    live.require(live.service_owned(ctx, 'capture'),
                 'No active capture owned by the approved executor; no PAUSE file was created')
    root = output_root(ctx)
    marker = live.no_symlinks(root / 'PAUSE')
    created = False
    if not marker.exists():
        try:
            live.write_new(marker, dict(reason='operator_graceful_pause', requested_at=time.time(),
                semantics='finish current bounded chunk and native Park cleanup'))
            created = True
        except FileExistsError:
            pass
    live.owned(marker, False)
    return dict(event='borrowed_capture_pause_requested', marker=str(marker), created=created,
                capture_started=False, signals_sent=False,
                note='Keep MORAI, ROS and bridge running until the collector exits with stage paused')


def resume_paused(ctx):
    root = output_root(ctx)
    marker = live.owned(root / 'PAUSE', False)
    expected = (marker.stat().st_dev, marker.stat().st_ino, live.digest(marker))
    with ExitStack() as stack:
        stack.enter_context(idle_lock(ctx['session'] / 'capture.lock'))
        live.require(not live.service_owned(ctx, 'capture'), 'The approved capture is still active')
        evidence = resume_evidence(root, stack)
        # Release only an unchanged pause request, and preserve its original
        # bytes under a unique audit name. Never delete STOP, statuses or raw.
        audit = live.no_symlinks(root / ('PAUSE.resumed_' + str(time.time_ns()) + '_' + uuid.uuid4().hex + '.json'))
        live.require(not audit.exists(), 'Pause audit path already exists')
        actual = (marker.stat().st_dev, marker.stat().st_ino, live.digest(marker))
        live.require(actual == expected, 'PAUSE changed while checking exit evidence')
        live.require(live.digest(root / 'status.json') == evidence['parent_status_sha256'],
                     'Parent checkpoint changed during exit verification')
        live.require(not (root / 'STOP').exists() and not (root / 'STOP').is_symlink(),
                     'STOP appeared during exit verification')
        os.rename(marker, audit)
    return dict(event='borrowed_capture_pause_released', archived_marker=str(audit),
                evidence=evidence, capture_started=False, signals_sent=False,
                next_manual_command=['bash', str(ctx['workspace'] / 'start_base5_collection.sh'),
                                     '--run', '--confirmed-environment', 'sunny13'],
                note='Confirm actual Sunny13 UI, then manually run the collection command')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, default=Path.home() / live.WORKSPACE)
    parser.add_argument('action', choices=['pause', 'resume-paused'])
    parser.add_argument('--run', action='store_true')
    args = parser.parse_args(argv)
    ctx = live.load(args.workspace)
    live.ready(ctx)
    if not args.run:
        print(json.dumps(dict(event='borrowed_capture_control_plan_only', action=args.action,
            output=str(ctx['workspace'] / 'collections/sunny13_5k'), files_written=False,
            capture_started=False, signals_sent=False), ensure_ascii=False, indent=2))
        return 0
    result = pause(ctx) if args.action == 'pause' else resume_paused(ctx)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError) as error:
        print(json.dumps(dict(event='borrowed_capture_control_blocked', error=str(error),
                              capture_started=False, fallback_allowed=False), ensure_ascii=False),
              file=sys.stderr)
        raise SystemExit(1)
