#!/usr/bin/env python3
"""Summarize bounded campaign cost, including rejected captures and retries."""
import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path
from base5_capture_safety import atomic_json


def report(root, manifest, output):
    status = json.loads((root / 'status.json').read_text())
    quota = json.loads((root / 'quota.json').read_text())
    deck = json.loads(manifest.read_text())
    by_family = defaultdict(lambda: dict(cells=0, attempts=0, capture_service_sec=0.0))
    reasons = Counter()
    kinds = Counter()
    safety_errors = []
    completed = 0
    for row in deck['scenes']:
        paths = sorted((root / row['id']).glob('round_*/report.json'))
        if not paths:
            continue
        family = by_family[row['family']]
        selected = False
        for path in paths:
            data = json.loads(path.read_text())
            selected = selected or bool(data['selected'])
            for attempt in data['attempts']:
                if 'capture_root' not in attempt:
                    continue
                family['attempts'] += 1
                kinds[attempt['kind']] += 1
                reasons.update(attempt.get('reasons', []))
                child_status = Path(attempt['capture_root']) / 'status.json'
                if child_status.exists():
                    family['capture_service_sec'] += json.loads(child_status.read_text()).get('elapsed_sec', 0.0)
                if not attempt.get('shutdown', {}).get('park_verified'):
                    safety_errors.append(row['id'])
        if selected:
            family['cells'] += 1
            completed += 1
    elapsed = status['elapsed_sec']
    service_total = sum(v['capture_service_sec'] for v in by_family.values())
    overhead_factor = elapsed / service_total if service_total else None
    full_counts = deck['config']['families']
    weighted_seconds = None
    if completed and all(v['cells'] for v in by_family.values()) and set(by_family) == set(full_counts) and overhead_factor:
        weighted_seconds = sum(full_counts[f] * v['capture_service_sec'] / v['cells']
                               for f, v in by_family.items()) * overhead_factor
    for v in by_family.values():
        v['seconds_per_selected_cell_excluding_parent_overhead'] = v['capture_service_sec'] / v['cells'] if v['cells'] else None
    total_bytes = sum(p.stat().st_size for p in root.rglob('*') if p.is_file())
    total_attempts = sum(v['attempts'] for v in by_family.values())
    result = dict(stage=status['stage'], quota=quota, elapsed_sec=elapsed,
        selected_cells=completed, capture_attempts=total_attempts, attempt_kinds=dict(kinds),
        review_reasons=dict(reasons), unsafe_children=safety_errors, family_cost=dict(by_family),
        total_bytes_including_rejected_and_logs=total_bytes,
        bytes_per_selected_cell=total_bytes/completed if completed else None,
        extrapolated_10k_hours_family_weighted=weighted_seconds/3600 if weighted_seconds else None,
        extrapolated_10k_gib_unweighted=total_bytes/completed*10000/2**30 if completed else None,
        bulk_start_eligible=status['stage']=='complete' and completed==quota['accepted']==quota['target'] and not safety_errors,
        limitations=['Small coverage-biased sample; estimates are not a guaranteed finish time or confidence interval.',
                     'Raw storage includes rejected attempts, but later difficult assets and long-running degradation may cost more.',
                     'Family service cost is observed child time; parent overhead apportioned globally.'])
    atomic_json(output, result)
    return result


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--capture-root', type=Path, required=True)
    p.add_argument('--manifest', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    print(json.dumps(report(a.capture_root, a.manifest, a.output), indent=2))
