#!/usr/bin/env python3
"""Replay a terminal fresh campaign offline and preserve its native inventory."""
import argparse
from argparse import Namespace
import json
from pathlib import Path

from base5_capture_safety import atomic_json, sha256
import run_base5_fresh_batch as fresh
from verify_base5_completed_resume import snapshot


def partial_snapshot(manifest, capture, output):
    manifest, capture, output = Path(manifest).resolve(), Path(capture).resolve(), Path(output).resolve()
    if output.exists() or capture in output.parents:
        raise ValueError('Fresh partial receipt outside capture required')
    state = json.loads((capture/'status.json').read_text())
    rows = [json.loads(line) for line in (capture/'selected_ledger.jsonl').read_text().splitlines()]
    if state.get('stage') != 'paused' or len(rows) != 4:
        raise ValueError('Registered four-row pause required')
    atomic_json(output, dict(manifest_sha256=sha256(manifest), capture_root=str(capture),
        selected_rows=rows, before=snapshot(capture), preserved_selected=4))
    return output


def verify_partial(partial, manifest, capture, output):
    partial, manifest, capture, output = map(Path, (partial, manifest, capture, output))
    doc = json.loads(partial.read_text())
    if (output.exists() or doc['manifest_sha256'] != sha256(manifest)
            or Path(doc['capture_root']).resolve() != capture.resolve()):
        raise ValueError('Partial receipt source/output differs')
    now = snapshot(capture)
    for group in ('raw_files','event_logs'):
        if not doc['before'][group] or any(now[group].get(k) != v for k,v in doc['before'][group].items()):
            raise ValueError('Partial native evidence changed during resume')
    rows = [json.loads(line) for line in (capture/'selected_ledger.jsonl').read_text().splitlines()]
    current = {row['source_cell_id']:row for row in rows}
    if (len(current) != len(rows) or any(current.get(row['source_cell_id']) != row for row in doc['selected_rows'])
            or now['cumulative_elapsed_sec'] < doc['before']['cumulative_elapsed_sec']):
        raise ValueError('Partial approvals/elapsed changed during resume')
    atomic_json(output, dict(passed=True, source_partial_sha256=sha256(partial),
        manifest_sha256=sha256(manifest), preserved_selected=4,
        prior_raw_unchanged=True, before=doc['before'], after=now))
    return output


def verify(manifest, capture, output, chunk_size=4):
    manifest, capture, output = Path(manifest).resolve(), Path(capture).resolve(), Path(output).resolve()
    if output.exists() or output == capture or capture in output.parents:
        raise ValueError('New verification receipt outside the capture is required')
    count = len(json.loads(manifest.read_text())['scenes'])
    state = json.loads((capture/'status.json').read_text())
    if not 1 <= count <= 64 or state.get('stage') not in ('complete', 'complete_with_deficit'):
        raise ValueError('Only a terminal pilot can enter no-capture replay')
    before = snapshot(capture)
    if not before['raw_files'] or not before['event_logs']:
        raise ValueError('Native raw and actual service-call evidence required')
    result = fresh.run(Namespace(manifest=manifest, output=capture, resume=True, limit=count,
        chunk_size=chunk_size, setup_profile='static-ready', runtime_overlap=False,
        prepare_only=False, stop_after_chunks=0, approve_bulk_capture=False))
    after = snapshot(capture)
    if result or before != after:
        raise ValueError('Terminal replay changed native evidence/selection/elapsed')
    receipt = dict(schema='base5_fresh_terminal_resume_v1', passed=True,
        verifier_sha256=sha256(__file__), manifest=str(manifest), manifest_sha256=sha256(manifest),
        capture_root=str(capture), requested=count, chunk_size=chunk_size,
        additional_native_captures=0, raw_unchanged=True, selected_unchanged=True,
        elapsed_unchanged=True, before=before, after=after,
        limitation='Completed replay only, not mid-epoch training or interrupted-scene replay.')
    atomic_json(output, receipt)
    return output


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('manifest','capture','output'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--chunk-size', type=int, default=4)
    parser.add_argument('--partial-snapshot', action='store_true')
    parser.add_argument('--verify-partial', type=Path)
    args = parser.parse_args()
    if args.partial_snapshot:
        print(partial_snapshot(args.manifest, args.capture, args.output))
    elif args.verify_partial:
        print(verify_partial(args.verify_partial, args.manifest, args.capture, args.output))
    else:
        print(verify(args.manifest, args.capture, args.output, args.chunk_size))
