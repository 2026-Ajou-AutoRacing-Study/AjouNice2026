#!/usr/bin/env python3
"""Bounded Drive archive delivery to the existing Vast instance; never extracts.

Default is a local plan only. --run refreshes OAuth locally and sends only a
short-lived access token through SSH stdin. No renewable credential is remote.
"""
import argparse
import collections
import configparser
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request

HOST = 'root@175.155.64.164'
PORT = 19463
MIN_BPS = 8000000
RANGE_BYTES = 32 * 1024 * 1024


def emit(event):
    print(json.dumps(event, sort_keys=True), flush=True)


def validate_spec(p):
    if not re.fullmatch(r'[A-Za-z0-9_-]{10,200}', p['file_id']):
        raise ValueError('Invalid Drive file ID')
    if not re.fullmatch(r'[a-f0-9]{64}', p['sha256']):
        raise ValueError('Invalid archive SHA256')
    if type(p['bytes']) is not int or not 0 < p['bytes'] <= 64 * 1024**3:
        raise ValueError('Invalid archive byte count')
    dest = Path(p['destination'])
    if not dest.is_absolute() or '..' in dest.parts or dest.parts[:2] != ('/', 'workspace') or len(dest.parts) < 3:
        raise ValueError('Destination must be a fresh file below /workspace')
    if dest.suffixes[-2:] != ['.tar', '.zst']:
        raise ValueError('Expected .tar.zst archive destination')
    return min(900, int(math.ceil(p['bytes'] / MIN_BPS + 60)))


def validate_destination(destination):
    dest = Path(destination)
    if dest.parent.resolve(strict=True) != dest.parent or not dest.parent.is_dir():
        raise ValueError('Destination parent must exist without symlink aliases')
    paths = (dest, Path(str(dest) + '.partial'), Path(str(dest) + '.transfer.json'))
    if any(os.path.lexists(str(path)) for path in paths):
        raise FileExistsError('Archive, partial, or receipt already exists; use a fresh destination')
    return paths


def validate_range(status, headers, start, end, total):
    if status != 206:
        raise ValueError('Drive did not honor byte range')
    if headers.get('Content-Range') != 'bytes %d-%d/%d' % (start, end, total):
        raise ValueError('Content-Range differs')
    if headers.get('Content-Length') != str(end - start + 1):
        raise ValueError('Range length differs')


class SpeedGate:
    """Require sustained low speed, not a single slow read or HTTP startup."""
    def __init__(self, minimum=MIN_BPS, warmup=10, window=10, sustain=10):
        self.minimum, self.warmup, self.window, self.sustain = minimum, warmup, window, sustain
        self.first = None
        self.low_since = None
        self.samples = collections.deque()

    def update(self, now, total):
        if self.first is None:
            if not total:
                return None, False
            self.first = now
        self.samples.append((now, total))
        while len(self.samples) > 2 and self.samples[1][0] <= now - self.window:
            self.samples.popleft()
        elapsed = now - self.samples[0][0]
        rate = (total - self.samples[0][1]) / elapsed if elapsed > 0 else None
        if now - self.first < self.warmup or elapsed < self.window * 0.8:
            return rate, False
        if rate is not None and rate < self.minimum:
            if self.low_since is None:
                self.low_since = now
        else:
            self.low_since = None
        return rate, self.low_since is not None and now - self.low_since >= self.sustain


def save_receipt(path, data):
    temporary = Path(str(path) + '.tmp')
    fd = os.open(str(temporary), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump(data, stream, indent=2)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(str(temporary), str(path))
    finally:
        if temporary.exists():
            temporary.unlink()


def publish_no_overwrite(partial, destination):
    # Same-filesystem hardlink is atomic and refuses overwrite, unlike rename.
    os.link(str(partial), str(destination), follow_symlinks=False)
    partial.unlink()


def fetch_archive(p, opener=urllib.request.urlopen):
    budget = validate_spec(p)
    destination, partial, receipt = validate_destination(p['destination'])
    fd = os.open(str(partial), os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
    start = time.monotonic()
    state = dict(schema='vast_drive_archive_fetch_v1', state='receiving', file_id=p['file_id'],
                 destination=str(destination), expected_bytes=p['bytes'], expected_sha256=p['sha256'],
                 received_bytes=0, verified_bytes=0, complete=False, credentials_persisted=False,
                 minimum_bytes_per_second=MIN_BPS, total_deadline_seconds=budget)
    save_receipt(receipt, state)
    stop, lock = threading.Event(), threading.Lock()
    errors = []
    next_offset = [0]
    first_byte = [None]
    last_byte = [None]
    workers = []

    def worker():
        while not stop.is_set():
            with lock:
                offset = next_offset[0]
                next_offset[0] += RANGE_BYTES
            if offset >= p['bytes']:
                return
            end = min(offset + RANGE_BYTES, p['bytes']) - 1
            try:
                request = urllib.request.Request(
                    'https://www.googleapis.com/drive/v3/files/' + p['file_id'] + '?alt=media',
                    headers={'Authorization': 'Bearer ' + p['access_token'],
                             'Range': 'bytes=%d-%d' % (offset, end)})
                with opener(request, timeout=8) as response:
                    validate_range(response.status, response.headers, offset, end, p['bytes'])
                    received = 0
                    while received <= end - offset and not stop.is_set():
                        chunk = response.read(min(512 * 1024, end - offset + 1 - received))
                        if not chunk:
                            raise ValueError('Truncated Drive range')
                        written = 0
                        while written < len(chunk):
                            count = os.pwrite(fd, chunk[written:], offset + received + written)
                            if count <= 0:
                                raise OSError('Archive write made no progress')
                            written += count
                        received += len(chunk)
                        with lock:
                            if first_byte[0] is None:
                                first_byte[0] = time.monotonic()
                            last_byte[0] = time.monotonic()
                            state['received_bytes'] += len(chunk)
                    if not stop.is_set() and response.read(1):
                        raise ValueError('Oversized Drive range')
            except Exception as error:
                with lock:
                    errors.append(dict(error_type=type(error).__name__, http_status=getattr(error, 'code', None)))
                stop.set()
                return

    gate = SpeedGate()
    failure = None
    last_progress = start - 2
    try:
        for _ in range(4):
            thread = threading.Thread(target=worker, daemon=True)
            workers.append(thread)
            thread.start()
        while any(thread.is_alive() for thread in workers):
            now = time.monotonic()
            with lock:
                received = state['received_bytes']
            if now - start >= max(1, budget - 9):
                failure = 'total_deadline'
                stop.set()
                break
            if now - last_progress >= 2:
                rate, too_slow = gate.update(now, received)
                emit(dict(event='drive_archive_progress', received_bytes=received, expected_bytes=p['bytes'],
                          elapsed_seconds=now-start, rolling_bytes_per_second=rate))
                last_progress = now
                if too_slow:
                    failure = 'sustained_below_8MBps'
                    stop.set()
                    break
            if errors:
                failure = 'range_failed'
                break
            time.sleep(0.1)
        stop.set()
        for thread in workers:
            thread.join(timeout=max(0, min(8.5, start + budget - time.monotonic())))
        if any(thread.is_alive() for thread in workers):
            raise TimeoutError('Worker did not stop within deadline')
        if failure or errors or state['received_bytes'] != p['bytes']:
            raise RuntimeError(failure or 'range_failed')
        os.fsync(fd)
        os.close(fd)
        fd = None
        digest = hashlib.sha256()
        with partial.open('rb') as stream:
            for chunk in iter(lambda: stream.read(4*1024*1024), b''):
                digest.update(chunk)
        if partial.stat().st_size != p['bytes'] or digest.hexdigest() != p['sha256']:
            failure = 'archive_size_or_sha256_mismatch'
            raise ValueError('Archive verification failed')
        publish_no_overwrite(partial, destination)
        state.update(state='complete', complete=True, verified_bytes=p['bytes'], sha256=digest.hexdigest())
    except Exception as error:
        stop.set()
        state.update(state='failed', error_type=type(error).__name__, reason=failure or 'transfer_or_verification_failed', range_errors=errors)
    finally:
        if fd is not None:
            os.close(fd)
        state.update(elapsed_seconds=time.monotonic()-start,
                     request_to_first_chunk_seconds=None if first_byte[0] is None else first_byte[0]-start,
                     payload_seconds=None if first_byte[0] is None else last_byte[0]-first_byte[0])
        state['average_payload_bytes_per_second'] = (state['received_bytes']/state['payload_seconds']
            if state['payload_seconds'] else None)
        save_receipt(receipt, state)
        emit(state)
    return state


def refresh_access_token(remote):
    cfg = configparser.ConfigParser(interpolation=None)
    cfg.read(Path.home()/'.config/rclone/rclone.conf')
    account = cfg[remote]
    token = json.loads(account['token'])
    body = urllib.parse.urlencode(dict(client_id=account['client_id'], client_secret=account['client_secret'],
        refresh_token=token['refresh_token'], grant_type='refresh_token')).encode()
    with urllib.request.urlopen('https://oauth2.googleapis.com/token', data=body, timeout=15) as response:
        return json.load(response)['access_token']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--file-id')
    parser.add_argument('--sha256')
    parser.add_argument('--bytes', type=int)
    parser.add_argument('--destination')
    parser.add_argument('--rclone-remote', default='ajou_drive_private')
    parser.add_argument('--run', action='store_true')
    parser.add_argument('--remote-worker', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()
    try:
        if args.remote_worker:
            p = json.load(sys.stdin)
            budget = validate_spec(p)
            def alarm(*unused):
                # Do not acquire a worker lock in a signal handler. The receipt
                # contains only public contract/progress fields, never the token.
                receipt = Path(p['destination'] + '.transfer.json')
                try:
                    state = json.loads(receipt.read_text())
                    state.update(state='failed', complete=False, error_type='TimeoutError',
                                 reason='hard_total_deadline')
                    save_receipt(receipt, state)
                except Exception:
                    pass  # The original incomplete receipt remains evidence.
                emit(dict(event='drive_archive_hard_deadline', complete=False, error_type='TimeoutError'))
                os._exit(124)
            signal.signal(signal.SIGALRM, alarm)
            signal.alarm(budget)
            result = fetch_archive(p)
            signal.alarm(0)
            return 0 if result['complete'] else 1
        p = dict(file_id=args.file_id or '', sha256=args.sha256 or '', bytes=args.bytes,
                 destination=args.destination or '')
        budget = validate_spec(p)
        emit(dict(event='drive_archive_plan', host=HOST, port=PORT, **p,
                  total_deadline_seconds=budget, connections=4, range_bytes=RANGE_BYTES,
                  minimum_bytes_per_second=MIN_BPS, run=args.run, extracts=False, trains=False))
        if not args.run:
            return 0
        p['access_token'] = refresh_access_token(args.rclone_remote)
        remote_code = Path(__file__).read_text()
        command = ['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8',
                   '-o', 'StrictHostKeyChecking=yes', '-p', str(PORT), HOST,
                   'python -B -u -c ' + shlex.quote(remote_code) + ' --remote-worker']
        result = subprocess.run(command, input=json.dumps(p), text=True, timeout=budget+30)
        return result.returncode
    except Exception as error:
        emit(dict(event='drive_archive_blocked', error_type=type(error).__name__))
        return 1


if __name__ == '__main__':
    sys.exit(main())
