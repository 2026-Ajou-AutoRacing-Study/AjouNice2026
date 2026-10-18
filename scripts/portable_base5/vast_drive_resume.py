#!/usr/bin/env python3
"""Recover only SHA-missing archive ranges into a new file; default is plan.

The failed archive and its receipt remain read-only. --allow-slow-completion is
an explicit exception to the normal 8MB/s gate, never to the 120-second limit.
No extraction, installation, binding, preparation, or training is performed.

Example plan arguments for this delivery:
  --archive artifacts/handoff/base5_sunny15_5k_jpeg90_upright3_261018.tar.zst
  --pack-receipt artifacts/evaluation/base5_sunny15_training_export_261018_v2/archive_receipt/COMPLETE.json
  --file-id 1D1y7ePCTEEaoy2D9bGjqaJwGoCvupPJ2
  --old-destination /workspace/base5_sunny15_5k_jpeg90_upright3_261018.tar.zst
  --destination /workspace/base5_sunny15_5k_jpeg90_upright3_261018.resume_v1.tar.zst
The pack receipt is the COMPLETE.json file, not its containing directory.
Only add --run after review; slow completion needs separate explicit approval.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import threading
import time
import urllib.request

import vast_drive_fetch as fetch

NETWORK_SECONDS = 120


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(4*1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def make_plan(archive, pack_receipt, file_id, old_destination, destination, allow_slow=False):
    archive, pack_receipt = Path(archive).resolve(strict=True), Path(pack_receipt).resolve(strict=True)
    receipt = json.loads(pack_receipt.read_text())
    if (receipt.get('complete') is not True or receipt.get('verified_decompressed_members') is not True
            or receipt.get('source_package_unchanged') is not True
            or Path(receipt['archive']).resolve(strict=True) != archive):
        raise ValueError('Require completed trusted archive packing receipt')
    before = archive.stat()
    pin = (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns)
    p = dict(file_id=file_id, sha256=receipt['sha256'], bytes=receipt['bytes'],
             old_destination=str(old_destination), destination=str(destination),
             allow_slow_completion=bool(allow_slow), network_deadline_seconds=NETWORK_SECONDS,
             range_bytes=fetch.RANGE_BYTES, blocks=[])
    validate_contract(p, require_blocks=False)
    whole = hashlib.sha256()
    with archive.open('rb') as stream:
        offset = 0
        while True:
            block = stream.read(fetch.RANGE_BYTES)
            if not block:
                break
            whole.update(block)
            p['blocks'].append(dict(offset=offset, bytes=len(block), sha256=hashlib.sha256(block).hexdigest()))
            offset += len(block)
    after = archive.stat()
    if (pin != (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns)
            or after.st_size != p['bytes'] or whole.hexdigest() != p['sha256']):
        raise ValueError('Local trusted archive size/SHA changed')
    validate_contract(p)
    return p


def validate_contract(p, require_blocks=True):
    fetch.validate_spec(p)
    fetch.validate_spec(dict(p, destination=p['old_destination']))
    if p['old_destination'] == p['destination'] or p.get('network_deadline_seconds') != NETWORK_SECONDS:
        raise ValueError('Require distinct destination and fixed 120-second deadline')
    if type(p.get('allow_slow_completion')) is not bool or p.get('range_bytes') != fetch.RANGE_BYTES:
        raise ValueError('Explicit policy and fixed range size required')
    if not require_blocks:
        return
    if not isinstance(p['blocks'], list):
        raise ValueError('Range manifest must be a list')
    if len(p['blocks']) != (p['bytes'] + fetch.RANGE_BYTES - 1) // fetch.RANGE_BYTES:
        raise ValueError('Range count differs')
    offset = 0
    for block in p['blocks']:
        if (not isinstance(block, dict) or type(block.get('offset')) is not int
                or type(block.get('bytes')) is not int or not 0 < block['bytes'] <= fetch.RANGE_BYTES
                or block.get('offset') != offset or block.get('bytes') != min(fetch.RANGE_BYTES, p['bytes']-offset)
                or not fetch.re.fullmatch(r'[a-f0-9]{64}', block.get('sha256', ''))):
            raise ValueError('Range manifest differs')
        offset += block['bytes']
    if offset != p['bytes']:
        raise ValueError('Incomplete range manifest')


def pwrite_all(fd, block, offset):
    written = 0
    while written < len(block):
        count = os.pwrite(fd, block[written:], offset + written)
        if count <= 0:
            raise OSError('Archive write made no progress')
        written += count


def recover(p, opener=urllib.request.urlopen, hard_deadline=False):
    validate_contract(p)
    destination, partial, receipt_path = fetch.validate_destination(p['destination'])
    old = Path(p['old_destination'] + '.partial')
    old_receipt = Path(p['old_destination'] + '.transfer.json')
    if (old.parent.resolve(strict=True) != old.parent or old.is_symlink() or not old.is_file()
            or old_receipt.is_symlink() or not old_receipt.is_file()):
        raise ValueError('Old partial/receipt must be regular read-only inputs')
    proof = json.loads(old_receipt.read_text())
    if (proof.get('complete') is not False or proof.get('state') != 'failed'
            or proof.get('destination') != p['old_destination'] or proof.get('file_id') != p['file_id']
            or proof.get('expected_sha256') != p['sha256'] or proof.get('expected_bytes') != p['bytes']):
        raise ValueError('Failed receipt binds a different archive')
    old_pin = old.stat()
    old_identity = (old_pin.st_dev, old_pin.st_ino, old_pin.st_size, old_pin.st_mtime_ns)
    old_receipt_sha = digest(old_receipt)
    state = dict(schema='vast_drive_missing_range_recovery_v1', state='copying_verified_ranges',
                 complete=False, destination=p['destination'], old_partial=str(old),
                 expected_bytes=p['bytes'], expected_sha256=p['sha256'], file_id=p['file_id'],
                 reused_bytes=0, reused_ranges=0, missing_bytes=0, missing_ranges=0,
                 network_received_bytes=0, network_verified_bytes=0,
                 allow_slow_completion=p['allow_slow_completion'], network_deadline_seconds=NETWORK_SECONDS,
                 credentials_persisted=False, extracts=False, trains=False)
    fd = os.open(str(partial), os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
    fetch.save_receipt(receipt_path, state)
    started = time.monotonic()
    missing = []
    stop, lock = threading.Event(), threading.Lock()
    errors, threads = [], []
    cursor = [0]
    first = [None]
    network_started = [None]
    failure = None

    def worker():
        while not stop.is_set():
            with lock:
                index = cursor[0]
                cursor[0] += 1
            if index >= len(missing):
                return
            item = missing[index]
            offset, length = item['offset'], item['bytes']
            try:
                request = urllib.request.Request('https://www.googleapis.com/drive/v3/files/' + p['file_id'] + '?alt=media',
                    headers={'Authorization': 'Bearer ' + p['access_token'],
                             'Range': 'bytes=%d-%d' % (offset, offset+length-1)})
                blocks, received, h = [], 0, hashlib.sha256()
                with opener(request, timeout=8) as response:
                    fetch.validate_range(response.status, response.headers, offset, offset+length-1, p['bytes'])
                    while received < length and not stop.is_set():
                        chunk = response.read(min(512*1024, length-received))
                        if not chunk:
                            raise ValueError('Truncated range')
                        blocks.append(chunk); h.update(chunk); received += len(chunk)
                        with lock:
                            state['network_received_bytes'] += len(chunk)
                            if first[0] is None:
                                first[0] = time.monotonic()
                    if stop.is_set():
                        return
                    if response.read(1) or h.hexdigest() != item['sha256']:
                        raise ValueError('Range SHA/length differs')
                position = offset
                for chunk in blocks:
                    pwrite_all(fd, chunk, position)
                    position += len(chunk)
                with lock:
                    state['network_verified_bytes'] += received
            except Exception as error:
                with lock:
                    errors.append(dict(error_type=type(error).__name__, http_status=getattr(error, 'code', None)))
                stop.set()

    try:
        with os.fdopen(os.open(str(old), os.O_RDONLY | os.O_NOFOLLOW), 'rb') as source:
            stat = os.fstat(source.fileno())
            if old_identity != (stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns):
                raise ValueError('Old partial changed before qualification')
            for item in p['blocks']:
                source.seek(item['offset'])
                block = source.read(item['bytes'])
                if len(block) == item['bytes'] and hashlib.sha256(block).hexdigest() == item['sha256']:
                    pwrite_all(fd, block, item['offset'])
                    state['reused_bytes'] += len(block)
                    state['reused_ranges'] += 1
                else:
                    missing.append(item)
        if old_identity != (old.stat().st_dev, old.stat().st_ino, old.stat().st_size, old.stat().st_mtime_ns):
            raise ValueError('Old partial changed while qualifying ranges')
        state.update(state='receiving_missing_ranges', missing_ranges=len(missing),
                     missing_bytes=sum(item['bytes'] for item in missing))
        fetch.emit(dict(event='drive_resume_ranges_qualified', **state))
        fetch.save_receipt(receipt_path, state)
        network_started[0] = time.monotonic()
        if hard_deadline:
            signal.alarm(NETWORK_SECONDS)
        for _ in range(min(4, len(missing))):
            thread = threading.Thread(target=worker, daemon=True)
            threads.append(thread); thread.start()
        gate, last_report = fetch.SpeedGate(), 0
        while any(thread.is_alive() for thread in threads):
            now = time.monotonic()
            if now-network_started[0] >= NETWORK_SECONDS-9:
                failure = 'network_deadline'; stop.set(); break
            if now-last_report >= 2:
                with lock: total = state['network_received_bytes']
                rate, too_slow = gate.update(now, total)
                fetch.emit(dict(event='drive_resume_progress', reused_bytes=state['reused_bytes'],
                                missing_bytes=state['missing_bytes'], network_received_bytes=total,
                                elapsed_network_seconds=now-network_started[0], rolling_bytes_per_second=rate,
                                allow_slow_completion=p['allow_slow_completion']))
                last_report = now
                if too_slow and not p['allow_slow_completion']:
                    failure = 'sustained_below_8MBps'; stop.set(); break
            if errors:
                failure = 'range_failed'; break
            time.sleep(.1)
        stop.set()
        for thread in threads:
            thread.join(timeout=max(0, min(8.5, network_started[0]+NETWORK_SECONDS-time.monotonic())))
        if any(thread.is_alive() for thread in threads) or failure or errors or state['network_verified_bytes'] != state['missing_bytes']:
            raise RuntimeError(failure or 'range_failed')
        os.fsync(fd); os.close(fd); fd = None
        state['state'] = 'verifying_whole_archive'
        if partial.stat().st_size != p['bytes'] or digest(partial) != p['sha256']:
            failure = 'whole_archive_sha_or_size_mismatch'
            raise ValueError('Whole archive differs')
        if (old_identity != (old.stat().st_dev, old.stat().st_ino, old.stat().st_size, old.stat().st_mtime_ns)
                or digest(old_receipt) != old_receipt_sha):
            raise ValueError('Original failure evidence changed')
        fetch.publish_no_overwrite(partial, destination)
        state.update(state='complete', complete=True, sha256=p['sha256'], verified_bytes=p['bytes'],
                     old_partial_and_receipt_preserved=True)
    except Exception as error:
        stop.set()
        state.update(state='failed', error_type=type(error).__name__, reason=failure or 'qualification_or_verification_failed', range_errors=errors)
    finally:
        if fd is not None: os.close(fd)
        state.update(elapsed_seconds=time.monotonic()-started,
                     elapsed_network_seconds=None if network_started[0] is None else time.monotonic()-network_started[0])
        fetch.save_receipt(receipt_path, state)
        fetch.emit(state)
    return state


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', type=Path)
    parser.add_argument('--pack-receipt', type=Path, help='Archive packer COMPLETE.json file, not its directory')
    parser.add_argument('--file-id')
    parser.add_argument('--old-destination')
    parser.add_argument('--destination')
    parser.add_argument('--allow-slow-completion', action='store_true')
    parser.add_argument('--run', action='store_true')
    parser.add_argument('--remote-worker', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()
    try:
        if args.remote_worker:
            p = json.load(sys.stdin)
            validate_contract(p)
            def alarm(*unused):
                receipt = Path(p['destination'] + '.transfer.json')
                try:
                    state = json.loads(receipt.read_text())
                    state.update(state='failed', complete=False, reason='hard_network_deadline', error_type='TimeoutError')
                    fetch.save_receipt(receipt, state)
                except Exception: pass
                fetch.emit(dict(event='drive_resume_hard_deadline', complete=False))
                os._exit(124)
            signal.signal(signal.SIGALRM, alarm)
            signal.alarm(180)
            result = recover(p, hard_deadline=True)
            signal.alarm(0)
            return 0 if result['complete'] else 1
        p = make_plan(args.archive, args.pack_receipt, args.file_id or '', args.old_destination or '',
                      args.destination or '', args.allow_slow_completion)
        fetch.emit(dict(event='drive_resume_plan', expected_bytes=p['bytes'], expected_sha256=p['sha256'],
                        block_count=len(p['blocks']), destination=p['destination'], old_destination=p['old_destination'],
                        allow_slow_completion=p['allow_slow_completion'], run=args.run, network_deadline_seconds=NETWORK_SECONDS))
        if not args.run: return 0
        p['access_token'] = fetch.refresh_access_token('ajou_drive_private')
        bootstrap = ('import types,sys;module=types.ModuleType("vast_drive_fetch");'
                     'sys.modules["vast_drive_fetch"]=module;exec(' + repr(Path(fetch.__file__).read_text()) + ',module.__dict__);'
                     'exec(' + repr(Path(__file__).read_text()) + ',{"__name__":"__main__"})')
        command = ['ssh','-T','-o','BatchMode=yes','-o','ConnectTimeout=8','-o','StrictHostKeyChecking=yes',
                   '-p',str(fetch.PORT),fetch.HOST,'python -B -u -c '+shlex.quote(bootstrap)+' --remote-worker']
        return subprocess.run(command, input=json.dumps(p), text=True, timeout=210).returncode
    except Exception as error:
        fetch.emit(dict(event='drive_resume_blocked', error_type=type(error).__name__))
        return 1


if __name__ == '__main__':
    sys.exit(main())
