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
        subprocess.run(['rclone', 'copyto', str(source), destination, *rclone_options, '--retries', '3',
                        '--low-level-retries', '5'], check=True, capture_output=True, text=True)
        # Read back remote bytes and SHA256; uploading a marker alone would
        # not prove the remote payload survived unchanged.
        process = subprocess.Popen(['rclone', 'cat', destination, *rclone_options], stdout=subprocess.PIPE,
                                   stderr=subprocess.DEVNULL)
        import hashlib
        h = hashlib.sha256()
        for block in iter(lambda: process.stdout.read(4 * 1024 * 1024), b''):
            h.update(block)
        if process.wait() != 0 or h.hexdigest() != sha256(source):
            raise ValueError('Remote read-back checksum failed')
    else:
        target = Path(destination)
        target.parent.mkdir(parents=True, exist_ok=True)
        pending = target.with_name(target.name + '.partial')
        shutil.copy2(source, pending)
        if sha256(pending) != sha256(source):
            raise ValueError('Local backup checksum failed')
        pending.replace(target)


def backup_once(work_dir, destination, state, rclone_options=()):
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
                copy_verified(snapshot, prefix + '/' + checkpoint.name, rclone_options)
                record = dict(name=checkpoint.name, sha256=checksum, bytes=snapshot.stat().st_size,
                              contract_sha256=contract, **progress)
                marker = folder/'COMPLETE.json'; marker.write_text(json.dumps(record, indent=2)+'\n')
                copy_verified(marker, prefix + '/COMPLETE.json', rclone_options)
                for name in ('run_manifest.json', 'resolved_train.py', 'resolved_resume.py'):
                    source = work_dir/name
                    if source.is_file(): copy_verified(source, prefix + '/' + name, rclone_options)
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
            state['errors'].append(dict(checkpoint=checkpoint.name, error=type(exc).__name__))
    state['checked_at_unix'] = time.time()
    (work_dir/'backup_status.json').write_text(json.dumps(state, indent=2)+'\n')
    return state


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--work-dir', type=Path, required=True)
    p.add_argument('--destination', required=True, help='Local path or existing rclone remote:path')
    p.add_argument('--interval-sec', type=float, default=120)
    p.add_argument('--once', action='store_true')
    p.add_argument('--drive-root-folder-id', help='Optional shared Drive root, never an account credential')
    a = p.parse_args()
    if a.interval_sec < 10: p.error('Polling interval must be at least ten seconds')
    status = a.work_dir/'backup_status.json'
    state = json.loads(status.read_text()) if status.exists() else {}
    # A status for one destination must not skip uploads to a different one.
    options = ['--drive-root-folder-id', a.drive_root_folder_id] if a.drive_root_folder_id else []
    if state.get('destination') != a.destination or state.get('rclone_options') != options:
        state = dict(destination=a.destination, rclone_options=options)
    while True:
        state = backup_once(a.work_dir, a.destination, state, options)
        print(json.dumps(dict(uploaded=len(state['uploaded']), errors=state['errors'])), flush=True)
        if a.once:
            raise SystemExit(1 if state['errors'] else 0)
        time.sleep(a.interval_sec)


if __name__ == '__main__': main()
