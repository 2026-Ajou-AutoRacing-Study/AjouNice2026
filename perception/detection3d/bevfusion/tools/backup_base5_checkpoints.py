#!/usr/bin/env python3
"""Back up completed trusted Base5 checkpoints; checkpoint hash before marker.

Local destination is useful for qualification. remote:path uses preconfigured
rclone, no embedded credentials. Backup errors are visible and retried; they
never delete checkpoints or silently claim an external copy is complete.
"""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import time

import torch

from prepare_base5_portable_dataset import sha256
from base5_checkpoint_parts import split_checkpoint, verify_parts


def emit(event, **fields):
    print(json.dumps(dict(event=event, **fields)), flush=True)


def remote_copy(source, destination, options, directory=False, phase='upload', transfers=16,
                expected_bytes=None):
    """Drain rclone logs live; expose counters and classified errors, never URLs."""
    command = ['rclone', 'copy' if directory else 'copyto', str(source), str(destination), *options,
               '--tpslimit', '2', '--tpslimit-burst', '1', '--drive-chunk-size', '8M',
               '--contimeout', '10s', '--timeout', '120s', '--retries', '2',
               '--low-level-retries', '2', '--use-json-log', '--stats', '5s',
               '--stats-log-level', 'NOTICE']
    reasons = set()
    if directory: command += ['--transfers', str(transfers), '--checkers', '2']
    label = Path(str(source)).name
    if expected_bytes is None and Path(str(source)).exists():
        path = Path(str(source))
        expected_bytes = sum(p.stat().st_size for p in path.rglob('*') if p.is_file()) if directory else path.stat().st_size
    emit(phase+'_started', file=label, totalBytes=expected_bytes)
    process = subprocess.Popen(command, stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, text=True)
    for line in process.stdout:
        for reason in ('rateLimitExceeded', 'userRateLimitExceeded',
                       'insufficientPermissions', 'storageQuotaExceeded', 'invalid_grant',
                       'directory not found', 'permission denied', 'Illegal seek',
                       'operation not permitted'):
            if reason.lower() in line.lower(): reasons.add(reason)
        if 'lookup' in line and 'timeout' in line: reasons.add('DNS_lookup_timeout')
        try:
            entry = json.loads(line)
        except (ValueError, TypeError):
            continue
        if isinstance(entry.get('stats'), dict):
            stats = entry['stats']
            fields = {key: stats[key] for key in ('bytes', 'totalBytes', 'speed', 'elapsedTime',
                                                  'errors', 'transfers', 'eta') if key in stats}
            if expected_bytes is not None:
                fields['totalBytes'] = expected_bytes
                if stats.get('speed', 0) > 0:
                    fields['eta'] = max(0, expected_bytes-stats.get('bytes', 0))/stats['speed']
            emit(phase+'_progress', file=label, **fields)
        if entry.get('level') == 'error':
            emit(phase+'_error', file=label,
                 reasons=sorted(reasons) or ['unclassified_copy_failure'])
    if process.wait():
        error = RuntimeError('Remote copy failed')
        error.safe_reasons = sorted(reasons) or ['unclassified_copy_failure']
        raise error
    emit(phase+'_finished', file=label)


def checkpoint_complete(path, contract):
    checkpoint = torch.load(path, map_location='cpu', weights_only=False)
    required = ('state_dict', 'optimizer', 'param_schedulers', 'message_hub', 'base5_rng', 'meta')
    if any(key not in checkpoint for key in required):
        raise ValueError('Not a full resumable checkpoint')
    if checkpoint.get('base5_contract_sha256') != contract:
        raise ValueError('Checkpoint/run contract mismatch')
    if 'loss_scaler' not in checkpoint['optimizer']:
        raise ValueError('AMP scaler missing')
    return dict(epoch=checkpoint['meta']['epoch'], iteration=checkpoint['meta']['iter'])


def copy_verified(source, destination, rclone_options=()):
    if ':' in destination:
        remote_copy(source, destination, rclone_options)
        # Read back remote bytes and SHA256; uploading a marker alone would
        # not prove the remote payload survived unchanged.
        emit('readback_started', file=source.name, bytes=source.stat().st_size)
        process = subprocess.Popen(['rclone', 'cat', destination, *rclone_options,
                                   '--contimeout', '10s', '--timeout', '60s',
                                   '--retries', '1', '--low-level-retries', '1'], stdout=subprocess.PIPE,
                                   stderr=subprocess.DEVNULL)
        import hashlib
        h = hashlib.sha256()
        count = 0; started = last = time.monotonic()
        for block in iter(lambda: process.stdout.read(4 * 1024 * 1024), b''):
            h.update(block)
            count += len(block)
            now = time.monotonic()
            if now-last >= 5:
                emit('readback_progress', file=source.name, bytes=count,
                     speed=count/max(now-started, 0.001))
                last = now
        if process.wait() != 0 or h.hexdigest() != sha256(source):
            raise ValueError('Remote read-back checksum failed')
        emit('readback_verified', file=source.name, bytes=count, sha256=h.hexdigest())
    else:
        target = Path(destination)
        target.parent.mkdir(parents=True, exist_ok=True)
        pending = target.with_name(target.name + '.partial')
        shutil.copy2(source, pending)
        if sha256(pending) != sha256(source):
            raise ValueError('Local backup checksum failed')
        pending.replace(target)


def backup_once(work_dir, destination, state, rclone_options=(), part_bytes=0, transfers=16):
    work_dir = Path(work_dir).resolve(strict=True)
    manifest = json.loads((work_dir/'run_manifest.json').read_text())
    contract = manifest['contract_sha256']
    state.setdefault('uploaded', {})
    state['errors'] = []
    # Atomic runner exposes .pth only AFTER save/fsync; leftover partials are
    # deliberately ignored. Snapshot protects against max_keep deletion.
    for checkpoint in sorted(work_dir.glob('*.pth')):
        stat = checkpoint.stat()
        fingerprint = [stat.st_size, stat.st_mtime_ns]
        if state['uploaded'].get(checkpoint.name, {}).get('fingerprint') == fingerprint:
            continue
        try:
            with tempfile.TemporaryDirectory(prefix='.backup_stage_', dir=work_dir) as temporary:
                folder = Path(temporary); snapshot = folder/checkpoint.name
                shutil.copy2(checkpoint, snapshot)
                if checkpoint.stat().st_size != stat.st_size or checkpoint.stat().st_mtime_ns != stat.st_mtime_ns:
                    raise ValueError('Checkpoint changed during snapshot')
                progress = checkpoint_complete(snapshot, contract)
                checksum = sha256(snapshot)
                # Content-addressed paths avoid overwriting any previous good
                # checkpoint before validation of a newly written best file.
                prefix = destination.rstrip('/') + '/' + work_dir.name + '/' + checksum
                record = dict(name=checkpoint.name, sha256=checksum, bytes=snapshot.stat().st_size,
                              contract_sha256=contract, **progress)
                if ':' in destination and part_bytes and snapshot.stat().st_size > part_bytes:
                    # Different layouts must not overwrite an earlier complete
                    # backup while a replacement layout is still uploading.
                    directory = f'parts_{part_bytes}'
                    parts = split_checkpoint(snapshot, folder/directory, part_bytes)
                    remote_copy(folder/directory, prefix+'/'+directory, rclone_options,
                                directory=True, transfers=transfers)
                    readback = folder/'readback'; readback.mkdir()
                    remote_copy(prefix+'/'+directory, readback, rclone_options,
                                directory=True, phase='readback', transfers=transfers,
                                expected_bytes=parts['bytes'])
                    restored = folder/'restored.pth'
                    if verify_parts(readback, parts, restored) != checksum:
                        raise ValueError('Multipart checkpoint differs from snapshot')
                    if checkpoint_complete(restored, contract) != progress:
                        raise ValueError('Restored checkpoint metadata differs from snapshot')
                    emit('readback_verified', file=checkpoint.name, bytes=parts['bytes'], sha256=checksum)
                    copy_verified(folder/'PARTS.json', prefix+'/PARTS.json', rclone_options)
                    copy_verified(Path(__file__).with_name('base5_checkpoint_parts.py'),
                                  prefix+'/RESTORE.py', rclone_options)
                    record['storage'] = 'base5_checkpoint_parts_v1'
                    record['parts'] = len(parts['parts'])
                else:
                    copy_verified(snapshot, prefix + '/' + checkpoint.name, rclone_options)
                    record['storage'] = 'single_file'
                for name in ('run_manifest.json', 'resolved_train.py', 'resolved_resume.py'):
                    source = work_dir/name
                    if source.is_file(): copy_verified(source, prefix + '/' + name, rclone_options)
                marker = folder/'COMPLETE.json'; marker.write_text(json.dumps(record, indent=2)+'\n')
                copy_verified(marker, prefix + '/COMPLETE.json', rclone_options)
                # Commit the latest pointer ONLY after checkpoint, marker and
                # resume metadata are all confirmed at the destination.
                latest = folder/'LATEST.json'
                latest.write_text(json.dumps(dict(record, location=prefix), indent=2)+'\n')
                if checkpoint.name.startswith('epoch_') and record['iteration'] >= state.get('latest_iteration', -1):
                    copy_verified(latest, destination.rstrip('/')+'/'+work_dir.name+'/LATEST.json', rclone_options)
                    state['latest_iteration'] = record['iteration']
                state['uploaded'][checkpoint.name] = dict(fingerprint=fingerprint, **record)
        except Exception as exc:
            # Keep secrets out of error text from command stderr/environment.
            state['errors'].append(dict(checkpoint=checkpoint.name, error=type(exc).__name__,
                                       reasons=getattr(exc,'safe_reasons',[])))
    state['checked_at_unix'] = time.time()
    (work_dir/'backup_status.json').write_text(json.dumps(state, indent=2)+'\n')
    return state


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--work-dir', type=Path, required=True)
    p.add_argument('--destination', required=True, help='Local path or existing rclone remote:path')
    p.add_argument('--interval-sec', type=float, default=120)
    p.add_argument('--once', action='store_true')
    p.add_argument('--part-mib', type=int, default=0, help='0: original single-file backup; positive: lossless parallel parts')
    p.add_argument('--transfers', type=int, default=16)
    p.add_argument('--drive-root-folder-id', help='Optional shared Drive root, never an account credential')
    a = p.parse_args()
    if a.interval_sec < 10: p.error('Polling interval must be at least ten seconds')
    if a.part_mib < 0 or not 1 <= a.transfers <= 16:
        p.error('Require nonnegative part size and 1..16 transfers')
    status = a.work_dir/'backup_status.json'
    state = json.loads(status.read_text()) if status.exists() else {}
    # A status for one destination must not skip uploads to a different one.
    options = ['--drive-root-folder-id', a.drive_root_folder_id] if a.drive_root_folder_id else []
    if state.get('destination') != a.destination or state.get('rclone_options') != options:
        state = dict(destination=a.destination, rclone_options=options)
    while True:
        state = backup_once(a.work_dir, a.destination, state, options, a.part_mib*1024**2, a.transfers)
        print(json.dumps(dict(uploaded=len(state['uploaded']), errors=state['errors'])), flush=True)
        if a.once:
            raise SystemExit(1 if state['errors'] else 0)
        time.sleep(a.interval_sec)


if __name__ == '__main__': main()
