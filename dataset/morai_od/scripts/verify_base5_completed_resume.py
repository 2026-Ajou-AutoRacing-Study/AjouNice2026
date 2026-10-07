#!/usr/bin/env python3
"""Verify completed batch resume never recaptures or changes raw/selected evidence."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

from base5_capture_safety import atomic_json, sha256


def snapshot(root):
    raw = {}
    for path in root.rglob('*'):
        if path.is_file() and 'captures' in path.relative_to(root).parts:
            st = path.stat()
            raw[str(path.relative_to(root))] = dict(sha256=sha256(path), inode=st.st_ino,
                                                   mtime_ns=st.st_mtime_ns, size=st.st_size)
    events = {str(p.relative_to(root)):sha256(p) for p in root.rglob('event_cmd_requests.jsonl')}
    return dict(raw_files=raw, event_logs=events, selected_sha256=sha256(root/'selected_ledger.jsonl'),
                cumulative_elapsed_sec=json.loads((root/'status.json').read_text())['elapsed_sec'])


def verify(root, manifest, count, chunk, output):
    status = json.loads((root/'status.json').read_text())
    if status['stage'] not in ('complete', 'complete_with_deficit'):
        raise ValueError('Verification must not start new captures on an incomplete campaign')
    for contract in root.rglob('session_contract.json'):
        if not json.loads((contract.parent/'shutdown_safety.json').read_text()).get('park_verified'):
            raise ValueError('Prior Park not verified')
    before = snapshot(root)
    command = [sys.executable, str(Path(__file__).with_name('run_base5_batch_campaign.py')),
               '--manifest', str(manifest), '--output', str(root), '--limit', str(count),
               '--chunk-size', str(chunk), '--resume']
    if count > 64:
        command.append('--approve-bulk-capture')
    subprocess.run(command, check=True)
    after = snapshot(root)
    if before != after:
        raise ValueError('Completed resume changed raw/capture events/selection/cumulative timing')
    atomic_json(output, dict(status='PASS', manifest_sha256=sha256(manifest), capture_root=str(root.resolve()),
        preserved_raw_files=len(before['raw_files']), preserved_event_logs=len(before['event_logs']),
        raw_sha256_inode_mtime_size_unchanged=True, selected_ledger_unchanged=True,
        cumulative_elapsed_unchanged=True, extra_capture_events=0,
        cumulative_elapsed_sec=before['cumulative_elapsed_sec']))
    return output


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--capture-root', type=Path, required=True);p.add_argument('--manifest', type=Path, required=True)
    p.add_argument('--limit', type=int, required=True);p.add_argument('--chunk-size', type=int, default=4)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args();print(verify(a.capture_root, a.manifest, a.limit, a.chunk_size, a.output))
