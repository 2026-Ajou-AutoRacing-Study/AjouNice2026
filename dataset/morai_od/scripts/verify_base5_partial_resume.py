#!/usr/bin/env python3
"""Read-only proof that a resumed pilot preserves its prior successful raw."""
import argparse
import json
from pathlib import Path

from base5_capture_safety import atomic_json, sha256


def snapshot(root):
    text = (root / 'selected_ledger.jsonl').read_text()
    if text and not text.endswith('\n'):
        raise ValueError('Torn selected ledger; verification must not repair it')
    rows = [json.loads(line) for line in text.splitlines()]
    if len({row['source_cell_id'] for row in rows}) != len(rows):
        raise ValueError('Duplicate selected quota slot')
    return dict(root=str(root.resolve()), selected_rows=rows,
                ledger_sha256=sha256(root / 'selected_ledger.jsonl'),
                native_intensity_sha256={str(path.resolve()): sha256(path)
                                         for path in root.rglob('*_Intensity.bin')})


def verify(saved, root):
    before = json.loads(saved.read_text()); after = snapshot(root)
    if before['root'] != after['root']:
        raise ValueError('Wrong resumed capture root')
    by_id = {r['source_cell_id']: r for r in after['selected_rows']}
    kept = all(by_id.get(r['source_cell_id']) == r for r in before['selected_rows'])
    raw = all(after['native_intensity_sha256'].get(p) == digest
              for p, digest in before['native_intensity_sha256'].items())
    result = dict(passed=kept and raw, preserved_selected=len(before['selected_rows']),
                  selected_after=len(after['selected_rows']), prior_raw_unchanged=raw,
                  native_before=len(before['native_intensity_sha256']),
                  native_after=len(after['native_intensity_sha256']), snapshot_sha256=sha256(saved))
    if not result['passed']:
        raise ValueError('Resume changed prior selection/raw')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--snapshot', type=Path)
    args = parser.parse_args()
    result = verify(args.snapshot, args.capture) if args.snapshot else snapshot(args.capture)
    atomic_json(args.output, result)
    print(json.dumps({k:v for k,v in result.items() if k not in ('selected_rows','native_intensity_sha256')}, indent=2))
