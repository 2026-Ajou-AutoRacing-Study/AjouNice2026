#!/usr/bin/env python3
"""Append aggregate AP at original epoch coordinates; never rewrite history.

Run only while this training work directory is stopped. Missing old per-class
validation data is never estimated. Authentication uses the existing W&B setup.
"""
import argparse
import fcntl
import json
from pathlib import Path
import time

from base5_distance_diagnostics import FAR_BINS, far_map_backfill_rows


def missing_rows(rows, histories):
    pending = []
    for row in rows:
        payload = {'epoch': row['epoch']}
        for lower, upper in FAR_BINS:
            prefix = f'val/range/{lower:g}_{upper:g}m'
            key = prefix + '/mAP'
            if key not in row:
                continue
            existing = histories.get(key, {}).get(row['epoch'])
            if existing is not None:
                if abs(existing - row[key]) > 1e-10:
                    raise ValueError('Existing history differs; refusing overwrite: ' + key)
                continue
            payload.update({k: v for k, v in row.items() if k.startswith(prefix + '/')})
        if len(payload) > 1:
            pending.append(payload)
    return pending


def history_values(api, path):
    remote = api.run(path)
    values = {}
    for lower, upper in FAR_BINS:
        key = f'val/range/{lower:g}_{upper:g}m/mAP'
        values[key] = {row['epoch']: row[key] for row in remote.scan_history(
            keys=['epoch', key], page_size=1000) if key in row}
    return values


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--work', required=True, type=Path)
    p.add_argument('--contract', required=True)
    p.add_argument('--entity', default='AjouNice2026')
    p.add_argument('--project', default='morai-3d-object-detection')
    p.add_argument('--run', action='store_true')
    a = p.parse_args()
    manifest = json.loads((a.work / 'run_manifest.json').read_text())
    if manifest['contract_sha256'] != a.contract:
        raise ValueError('Training contract differs')
    # Same lock as the remote training worker: no competing W&B history writers.
    with (a.work / '.oneclick-training.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        receipts = [json.loads(path.read_text()) for path in
                    (a.work / 'distance_diagnostics').glob('epoch_*.json')]
        if not receipts:
            raise ValueError('No exact validation receipts to backfill')
        rows = far_map_backfill_rows(receipts)
        import wandb
        run_id = a.work.name + '-' + a.contract[:12]
        path = a.entity + '/' + a.project + '/' + run_id
        pending = missing_rows(rows, history_values(wandb.Api(timeout=30), path))
        print(json.dumps(dict(event='distance_map_backfill_prepared', rows=pending,
                              append_only=True, run_id=run_id)), flush=True)
        if not a.run:
            return
        if pending:
            with wandb.init(entity=a.entity, project=a.project, id=run_id,
                            resume='must', mode='online', dir=str(a.work)) as run:
                run.define_metric('epoch', hidden=True)
                for lower, upper in FAR_BINS:
                    run.define_metric(f'val/range/{lower:g}_{upper:g}m/mAP',
                                      step_metric='epoch', hidden=False,
                                      summary='max', overwrite=True)
                for row in pending:
                    run.log({**row, 'eval/distance_map_backfill': 1}, commit=True)
        # Read exact online history, not a sampled chart or the max summary.
        verified = not pending
        for _ in range(6) if pending else ():
            if not missing_rows(rows, history_values(wandb.Api(timeout=30), path)):
                verified = True
                break
            time.sleep(2)
        receipt = dict(run_id=run_id, time=time.time(), appended_epochs=[r['epoch'] for r in pending],
                       online_history_verified=verified, rows=rows, append_only=True,
                       scope='aggregate AP only; original validation history and training unchanged')
        (a.work / 'distance_map_backfill.json').write_text(json.dumps(receipt, indent=2) + '\n')
        print(json.dumps(dict(event='distance_map_backfill_complete', **receipt)), flush=True)
        if not verified:
            raise RuntimeError('Online history not yet confirmed; do not claim completion')


if __name__ == '__main__':
    main()
