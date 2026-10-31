#!/usr/bin/env python3
"""Upload one verified training tar.zst to Drive with a bounded 8 MB/s gate.

The default only reads local inputs and prints a plan. --run permits one local
rclone upload, followed by a scoped Drive size/MD5/file-ID check. It never sends
credentials to another host, changes source files, removes remote objects,
starts training, or retries a failed upload automatically. An interrupted Drive
upload may leave a partial object; inspect it manually before another attempt.
"""
import argparse
from collections import deque
import hashlib
import json
import math
import os
from pathlib import Path
import re
import selectors
import signal
import subprocess
import sys
import time

SCHEMA = 'base5_bounded_training_archive_upload_v1'
MIN_RATE = 8_000_000  # Decimal MB/s, not MiB/s.
STARTUP_SECONDS = 20.0
WINDOW_SECONDS = 10.0
MAX_SECONDS = 900.0


def require(condition, message):
    if not condition:
        raise ValueError(message)


def safe_path(value, directory=False, missing=False):
    raw = Path(value)
    require('..' not in raw.parts, 'Normalized local paths required')
    path = raw.absolute()
    require(not any(p.is_symlink() for p in (path, *path.parents)), 'Symlink input/output forbidden')
    if not missing:
        require(path.is_dir() if directory else path.is_file(), 'Required local input missing or wrong type')
    return path


def inside(path, parent):
    return path == parent or parent in path.parents


def read_json(path):
    return json.loads(path.read_text())


def write_new(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2, sort_keys=True)
        stream.write('\n'); stream.flush(); os.fsync(stream.fileno())


def hashes(path):
    sha, md5 = hashlib.sha256(), hashlib.md5()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            sha.update(block); md5.update(block)
    return dict(sha256=sha.hexdigest(), md5=md5.hexdigest())


def pin(path):
    stat = path.stat()
    return dict(bytes=stat.st_size, device=stat.st_dev, inode=stat.st_ino,
                mtime_ns=stat.st_mtime_ns, sha256=hashes(path)['sha256'])


def destination_folder(value):
    require(re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.-]*:[A-Za-z0-9_./-]+/', value or '')
            is not None, 'Explicit normalized remote:dedicated/folder/ required')
    remote, path = value.split(':', 1)
    parts = path[:-1].split('/')
    require(all(part and part not in ('.', '..') for part in parts),
            'Remote root, traversal and empty path components forbidden')
    return remote + ':' + '/'.join(parts) + '/'


def prepare(args):
    archive = safe_path(args.archive)
    pack = safe_path(args.pack_receipt, directory=True)
    receipt = safe_path(args.receipt_dir, missing=True)
    require(archive.name.endswith('.tar.zst') and re.fullmatch(r'[A-Za-z0-9_.-]+', archive.name),
            'One plain-name .tar.zst archive required')
    require(not receipt.exists() and not receipt.is_symlink(), 'Fresh upload receipt directory required')
    require(not inside(receipt, pack) and not inside(pack, receipt)
            and not inside(archive, receipt), 'Receipt overlaps protected archive/pack inputs')
    controls = [safe_path(pack / name) for name in ('COMPLETE.json', 'intent.json',
                                                  'archive_manifest.json', 'SHA256SUMS')]
    require(not (pack / 'INCOMPLETE.json').exists(), 'Incomplete pack receipt cannot be uploaded')
    complete, intent, manifest = [read_json(p) for p in controls[:3]]
    require(intent.get('schema') == 'base5_qualified_training_archive_v1'
            and complete.get('event') == 'qualified_training_archive_complete'
            and complete.get('complete') is True
            and complete.get('verified_decompressed_members') is True
            and complete.get('source_package_unchanged') is True,
            'Verified complete pack_training receipt required')
    require(intent.get('expected_scenes', 0) > 0
            and complete.get('frames') == intent['expected_scenes'], 'Pack scene counts differ')
    require(Path(intent.get('output', '')).absolute() == archive
            and Path(complete.get('archive', '')).absolute() == archive
            and Path(manifest.get('archive', '')).absolute() == archive,
            'Pack receipt binds a different archive')
    before = archive.stat()
    digest = hashes(archive)
    after = archive.stat()
    require((before.st_ino, before.st_size, before.st_mtime_ns) ==
            (after.st_ino, after.st_size, after.st_mtime_ns), 'Archive changed during hashing')
    require(complete.get('bytes') == manifest.get('bytes') == after.st_size > 0
            and complete.get('sha256') == manifest.get('sha256') == digest['sha256'],
            'Archive size/SHA256 differs from verified pack receipt')
    require(controls[3].read_text() == digest['sha256'] + '  ' + archive.name + '\n',
            'Pack SHA256SUMS differs from archive')
    require(isinstance(manifest.get('members'), dict) and manifest['members'],
            'Pack decompressed member inventory missing')
    folder = destination_folder(args.destination)
    deadline = min(MAX_SECONDS, after.st_size / MIN_RATE + 60.0)
    pins = {str(path): pin(path) for path in controls}
    pins[str(archive)] = dict(bytes=after.st_size, device=after.st_dev, inode=after.st_ino,
                             mtime_ns=after.st_mtime_ns, sha256=digest['sha256'])
    return dict(schema=SCHEMA, archive=str(archive), pack_receipt=str(pack),
        receipt_dir=str(receipt), destination=folder, remote_object=folder + archive.name,
        bytes=after.st_size, **digest, frames=complete['frames'], source_pins=pins,
        minimum_bytes_per_second=MIN_RATE, rolling_window_seconds=WINDOW_SECONDS,
        startup_allowance_seconds=STARTUP_SECONDS, upload_deadline_seconds=deadline,
        rclone_bin=str(args.rclone_bin), automatic_retries=False, source_deletion=False,
        remote_deletion=False, training_started=False)


class RateGuard:
    """Use byte-counter deltas, not rclone's cumulative/averaged speed field."""
    def __init__(self, start, deadline, expected_bytes=None):
        self.start, self.deadline = start, deadline
        self.expected_bytes = expected_bytes
        self.samples = deque()
        self.previous_time, self.previous_bytes = start, 0
        self.payload_start, self.payload_end = None, None
        self.last_bytes, self.minimum_window_rate = 0, None

    def tick(self, now):
        require(now - self.start <= self.deadline, 'Upload hard deadline exceeded; no automatic retry')
        if self.payload_start is None:
            require(now - self.start <= STARTUP_SECONDS,
                    'No payload progress within startup allowance; upload stopped')
        elif self.payload_end is None:
            require(now - self.previous_time <= WINDOW_SECONDS,
                    'Structured progress stats stopped; cannot enforce sustained upload rate')

    def observe(self, now, stats):
        self.tick(now)
        count = stats.get('bytes')
        require(isinstance(count, (int, float)) and not isinstance(count, bool)
                and math.isfinite(count) and count >= self.last_bytes,
                'Missing/nonmonotonic structured byte counter; cannot enforce upload rate')
        count = int(count)
        require(self.expected_bytes is None or count <= self.expected_bytes,
                'Upload byte counter exceeds the single expected payload')
        if count > 0 and self.payload_start is None:
            # The prior zero-byte sample bounds startup. For the first sample,
            # assume at most one configured (2 s) stats interval of payload.
            self.payload_start = max(self.previous_time, now - 2.0)
            self.samples.append((self.payload_start, 0))
        self.last_bytes = count
        if self.payload_start is not None and self.payload_end is None:
            self.samples.append((now, count))
            cutoff = now - WINDOW_SECONDS
            while len(self.samples) > 1 and self.samples[1][0] <= cutoff:
                self.samples.popleft()
            first_time, first_bytes = self.samples[0]
            if now - first_time >= WINDOW_SECONDS:
                rate = (count - first_bytes) / (now - first_time)
                self.minimum_window_rate = rate if self.minimum_window_rate is None else min(
                    self.minimum_window_rate, rate)
                require(rate >= MIN_RATE, 'Sustained upload below 8 MB/s; stopped without retry')
            if count == self.expected_bytes:
                self.payload_end = now
        self.previous_time, self.previous_bytes = now, count

    def summary(self, now):
        return dict(transferred_bytes=self.last_bytes, elapsed_seconds=now - self.start,
                    minimum_observed_window_bytes_per_second=self.minimum_window_rate,
                    payload_seconds=None if self.payload_start is None else
                    (self.payload_end if self.payload_end is not None else now) - self.payload_start)


def upload_command(contract):
    return [contract['rclone_bin'], 'copyto', contract['archive'], contract['remote_object'],
            '--immutable', '--ignore-existing', '--checksum', '--no-traverse',
            '--transfers', '1', '--checkers', '1', '--drive-chunk-size', '32M',
            '--retries', '1', '--low-level-retries', '1', '--retries-sleep', '0s',
            '--contimeout', '10s', '--timeout', '15s', '--stats', '2s',
            '--stats-log-level', 'NOTICE', '--use-json-log', '--ask-password=false']


def stop_own_child(process):
    if process.poll() is not None:
        return
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(process.pid, sig)  # Popen creates this process group.
            process.wait(timeout=3)
            return
        except ProcessLookupError:
            return
        except subprocess.TimeoutExpired:
            pass


def monitor_upload(contract, receipt):
    process = subprocess.Popen(upload_command(contract), stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True)
    start = time.monotonic()
    guard = RateGuard(start, contract['upload_deadline_seconds'], contract['bytes'])
    buffer = b''
    with selectors.DefaultSelector() as selector, (receipt / 'progress.jsonl').open('x') as log:
        selector.register(process.stdout, selectors.EVENT_READ)
        try:
            while selector.get_map():
                now = time.monotonic(); guard.tick(now)
                for key, _ in selector.select(timeout=.5):
                    block = os.read(key.fileobj.fileno(), 65536)
                    if not block:
                        selector.unregister(key.fileobj)
                        continue
                    buffer += block
                    require(len(buffer) <= 1024 * 1024, 'Unbounded rclone log line; upload stopped')
                    while b'\n' in buffer:
                        line, buffer = buffer.split(b'\n', 1)
                        try:
                            record = json.loads(line)
                        except (ValueError, UnicodeDecodeError):
                            continue  # Do not print or save arbitrary auth/error text.
                        if isinstance(record, dict) and isinstance(record.get('stats'), dict):
                            guard.observe(time.monotonic(), record['stats'])
                            safe = dict(event='training_archive_upload_progress',
                                        **guard.summary(time.monotonic()))
                            log.write(json.dumps(safe) + '\n'); log.flush()
                            print(json.dumps(safe), flush=True)
            code = process.wait(timeout=3)
            require(code == 0, 'rclone upload failed (exit %d); inspect credentials/network separately' % code)
            require(guard.last_bytes == contract['bytes'], 'Upload byte counter does not prove full payload')
            summary = guard.summary(time.monotonic())
            require(summary['payload_seconds'] and
                    contract['bytes'] / summary['payload_seconds'] >= MIN_RATE,
                    'Completed payload average below 8 MB/s; not accepted as fast upload')
            return summary
        except BaseException:
            stop_own_child(process)
            raise
        finally:
            process.stdout.close()


def listing(contract, allow_missing=False):
    command = [contract['rclone_bin'], 'lsjson', contract['destination'],
               '--files-only', '--hash', '--no-modtime', '--retries', '1',
               '--low-level-retries', '1', '--contimeout', '10s', '--timeout', '10s',
               '--ask-password=false']
    result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=20)
    if result.returncode == 3 and allow_missing:
        return []  # rclone's directory-not-found exit status; auth failures stay errors.
    require(result.returncode == 0, 'Scoped Drive listing failed (exit %d)' % result.returncode)
    require(len(result.stdout) <= 2 * 1024 * 1024, 'Dedicated Drive folder listing unexpectedly large')
    rows = json.loads(result.stdout)
    require(isinstance(rows, list) and all(isinstance(row, dict) for row in rows),
            'Invalid scoped Drive object listing')
    return rows


def check_rclone(contract):
    """Old rclone emits JSON text logs without machine-readable stats."""
    result = subprocess.run([contract['rclone_bin'], 'version'], stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, timeout=5)
    first_line = result.stdout.splitlines()[0] if result.stdout else b''
    match = re.fullmatch(rb'rclone v(\d+)\.(\d+)\.(\d+)(?:[-.][A-Za-z0-9.-]+)?', first_line)
    require(result.returncode == 0 and match is not None,
            'Cannot qualify the local rclone binary')
    version = tuple(int(value) for value in match.groups())
    require(version >= (1, 70, 0),
            'rclone >=1.70 required for structured stats; old binary must not start bulk upload')
    return first_line.decode('ascii')


def remote_matches(contract, rows):
    name = Path(contract['archive']).name
    return [row for row in rows if row.get('Name') == name or row.get('Path') == name]


def verify_remote(contract, rows):
    matches = remote_matches(contract, rows)
    require(len(matches) == 1, 'Uploaded Drive object absent or duplicated; no completion receipt')
    row = matches[0]
    md5 = {str(k).lower(): str(v).lower() for k, v in row.get('Hashes', {}).items()}.get('md5')
    require(row.get('IsDir') is False and row.get('Size') == contract['bytes']
            and md5 == contract['md5'], 'Drive object size/MD5 differs from verified local archive')
    file_id = row.get('ID')
    require(isinstance(file_id, str) and re.fullmatch(r'[A-Za-z0-9_-]+', file_id),
            'Drive file ID missing/unsafe; cannot hand off a verified download')
    return dict(file_id=file_id, name=Path(contract['archive']).name,
                bytes=row['Size'], md5=md5, sha256=contract['sha256'])


def verify_pins(contract):
    for path, expected in contract['source_pins'].items():
        require(pin(safe_path(path)) == expected, 'Archive or pack receipt changed during upload')


def upload(args):
    contract = prepare(args)
    plan = dict(event='training_archive_upload_plan', execute=bool(args.run), **contract)
    if not args.run:
        return plan
    receipt = Path(contract['receipt_dir'])
    receipt.mkdir(parents=True, exist_ok=False)
    safe_path(receipt, directory=True)
    write_new(receipt / 'intent.json', dict(contract, started_at=time.time()))
    try:
        version = check_rclone(contract)
        require(not remote_matches(contract, listing(contract, allow_missing=True)),
                'Destination archive already exists; never overwrite or silently replace')
        summary = monitor_upload(contract, receipt)
        verify_pins(contract)
        verified = verify_remote(contract, listing(contract))
        complete = dict(event='training_archive_upload_complete', complete=True,
            schema=SCHEMA, remote_object=contract['remote_object'], remote=verified,
            upload=summary, rclone_version=version, source_unchanged=True, automatic_retries=False,
            training_started=False, completed_at=time.time())
        write_new(receipt / 'COMPLETE.json', complete)
        return complete
    except BaseException as error:
        # Never write subprocess output or arbitrary OAuth responses to receipts.
        text = str(error) if isinstance(error, ValueError) else type(error).__name__
        write_new(receipt / 'INCOMPLETE.json', dict(schema=SCHEMA, complete=False,
            error=text, error_type=type(error).__name__, automatic_retries=False,
            remote_partial_may_exist=True, source_deletion=False, training_started=False))
        raise


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('archive', 'pack-receipt', 'receipt-dir'):
        p.add_argument('--' + name, type=Path, required=True)
    p.add_argument('--destination', required=True)
    p.add_argument('--rclone-bin', default='rclone')
    p.add_argument('--run', action='store_true')
    return p


if __name__ == '__main__':
    try:
        print(json.dumps(upload(parser().parse_args()), indent=2))
    except (ValueError, OSError, subprocess.SubprocessError) as error:
        safe = str(error) if isinstance(error, ValueError) else type(error).__name__
        print(json.dumps(dict(event='training_archive_upload_blocked', error=safe)), file=sys.stderr)
        raise SystemExit(2)
