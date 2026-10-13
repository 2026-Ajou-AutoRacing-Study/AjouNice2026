#!/usr/bin/env python3
"""Summarize completed Base5/E25 matched cases; incomplete cases remain explicit."""
import argparse
import json
from collections import defaultdict
from pathlib import Path
from statistics import mean

from summarize_teammate_manual_evaluation import extract, ROOT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=ROOT / 'artifacts/evaluation/bevfusion/base5_nearfield_replay_e2_261005')
    args = parser.parse_args()
    manifest = json.loads((args.root / 'runtime_manifest.json').read_text())
    rows, pending = [], []
    for model in manifest['checkpoints']:
        for contract in manifest['cases']:
            case = args.root / model / contract['label']
            if not (case / 'pipeline/summary_cv_stable_status_size_v2.json').exists():
                pending.append(f'{model}/{contract["label"]}')
                continue
            baseline = Path(contract['source'])
            cohort = contract['label'].split('__')[0]
            for tag, path in [(model, case), ('e25', baseline)]:
                row = extract(path, '_status_size_v2')
                row.update(model=tag, cohort=cohort, matched_case=contract['label'])
                rows.append(row)
    unique = {(r['model'], r['matched_case']): r for r in rows}
    rows = list(unique.values())
    members = defaultdict(list)
    for row in rows:
        members[f'{row["cohort"]}/{row["mission"]}/{row["model"]}'].append(row)
    metrics = ['center_ap', 'tracking_f1', 'speed_mae_mps', 'ave_mps', 'fde1_m', 'fde2_m', 'fde3_m']
    groups = {}
    for key, values in sorted(members.items()):
        groups[key] = dict(bags=len(values), id_switches=sum(v['id_switches'] for v in values),
            **{k: mean([v[k] for v in values if v[k] is not None])
               if any(v[k] is not None for v in values) else None for k in metrics})
    result = dict(status='complete' if not pending else 'partial', pending=pending, rows=rows,
        groups=groups, aggregation='Matched case sets, unweighted bag macro. FDE is matched-object conditional; obstacle near-zero speed reflects static policy.')
    (args.root / 'comparison.json').write_text(json.dumps(result, indent=2) + '\n')
    lines = ['| Cohort/mission/model | Bags | Center AP % | Track F1 % | ID switches | Speed MAE m/s | AVE m/s | FDE@1/2/3s m |',
             '|---|---:|---:|---:|---:|---:|---:|---|']
    def fmt(value):
        return 'N/A' if value is None else f'{value:.3f}'
    for key, g in groups.items():
        lines.append(f'| {key} | {g["bags"]} | {g["center_ap"]*100:.2f} | {g["tracking_f1"]*100:.2f} | {g["id_switches"]} | '
            + fmt(g['speed_mae_mps']) + ' | ' + fmt(g['ave_mps']) + ' | '
            + ' / '.join(fmt(g[f'fde{h}_m']) for h in (1, 2, 3)) + ' |')
    (args.root / 'comparison.md').write_text('\n'.join(lines) + '\n\nStatus: ' + result['status'] + '\n')
    print('\n'.join(lines))
    print('STATUS', result['status'], 'PENDING', len(pending))


if __name__ == '__main__':
    main()
