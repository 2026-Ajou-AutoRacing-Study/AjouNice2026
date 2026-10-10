#!/usr/bin/env python3
"""Fix cost-only representative/stress samples before GPU measurements."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import pickle

import numpy as np


def choose(records, representative=200, stress=100):
    ranked = sorted(records, key=lambda r: hashlib.sha256(('261010:' + r['id']).encode()).digest())
    selected = {r['id']: dict(r, sample_group='representative') for r in ranked[:representative]}
    pools = [sorted(records, key=lambda r: (-r['density_rank_nonzero_roi_returns'], r['id'])),
             sorted(records, key=lambda r: (r['nearest_gt_m'], r['id'])),
             sorted(records, key=lambda r: (-r['farthest_gt_m'], r['id'])),
             [r for r in ranked if r['rear_gt']],
             [r for r in ranked if 3 in r['labels']]]
    pools += [[r for r in ranked if r['family'] == f] for f in sorted({r['family'] for r in records})]
    positions = [0] * len(pools)
    goal = representative + stress
    while len(selected) < goal:
        changed = False
        for i, pool in enumerate(pools):
            while positions[i] < len(pool) and pool[positions[i]]['id'] in selected:
                positions[i] += 1
            if positions[i] < len(pool) and len(selected) < goal:
                row = pool[positions[i]]; positions[i] += 1
                selected[row['id']] = dict(row, sample_group='stress'); changed = True
        if not changed:
            raise ValueError('Insufficient distinct scenes')
    return list(selected.values())


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if a.output.exists():
        raise FileExistsError('Use a fresh selection directory')
    ledger = {r['id']: r for r in (json.loads(x) for x in
              (a.data/'provenance/original_selected_ledger.jsonl').read_text().splitlines())}
    records = []
    for split in ('train', 'val'):
        info = pickle.loads((a.data/'infos/448x896'/('morai_base5_infos_'+split+'.pkl')).read_bytes())
        for r in info['data_list']:
            cloud = np.fromfile(a.data/r['lidar_points']['lidar_path'], dtype='<f4').reshape(-1, 4)
            valid = np.isfinite(cloud).all(axis=1) & (cloud[:, :3] != 0).any(axis=1)
            valid &= ((cloud[:, :3] >= [-100, -30, -3]) & (cloud[:, :3] < [100, 30, 2])).all(axis=1)
            ranges = [float(np.hypot(*i['bbox_3d'][:2])) for i in r['instances']]
            records.append(dict(id=r['token'], split=split, family=ledger[r['token']]['family'],
                lidar_path=r['lidar_points']['lidar_path'],
                camera_paths=[r['images'][k]['img_path'] for k in
                    ('Camera_front', 'Camera_front_left', 'Camera_front_right', 'Camera_back')],
                raw_points=len(cloud), density_rank_nonzero_roi_returns=int(valid.sum()),
                nearest_gt_m=min(ranges, default=999), farthest_gt_m=max(ranges, default=0),
                rear_gt=any(i['bbox_3d'][0] < -1 for i in r['instances']),
                labels=sorted({i['bbox_label_3d'] for i in r['instances']})))
    if len(records) != 5000 or len({r['id'] for r in records}) != 5000:
        raise ValueError('Expected complete, distinct 5K')
    selected = choose(records)
    manifest = dict(portable_data_root=str(a.data.resolve()),
        source_sha256=hashlib.sha256((a.data/'manifest.json').read_bytes()).hexdigest(),
        scope='300 distinct cost-only samples: 200 deterministic hash-random representative + 100 stress; no learned AP',
        samples=selected, population_families=dict(Counter(r['family'] for r in records)),
        sampled_families=dict(Counter(r['family'] for r in selected)),
        sampled_groups=dict(Counter(r['sample_group'] for r in selected)))
    a.output.mkdir(parents=True)
    (a.output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps({k:v for k,v in manifest.items() if k!='samples'}, indent=2), flush=True)


if __name__ == '__main__': main()
