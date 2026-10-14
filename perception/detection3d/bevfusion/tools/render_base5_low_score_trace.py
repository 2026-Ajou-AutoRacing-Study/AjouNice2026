#!/usr/bin/env python3
"""Data plots of a selected reconstructed vehicle trajectory, not photo edits."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import pickle

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from analyze_base5_far_vehicle_scores import prepare_cases
from evaluate_base5_low_score_probe import score_case, trajectory_velocities
from replay_base5_compiled_tracking import ego_trajectory, load_sensor4_calibration


def render(args):
    if args.output.exists():
        raise ValueError('Fresh trace directory required')
    records = pickle.loads((args.inputs / 'infos_val.pkl').read_bytes())['data_list']
    manifest = json.loads((args.inputs / 'manifest.json').read_text())
    with gzip.open(args.predictions, 'rt') as f:
        payload = json.load(f)
    prepared, _ = prepare_cases(records, payload)
    with gzip.open(args.replay / 'frames.json.gz', 'rt') as f:
        results = json.load(f)
    report = json.loads((args.replay / 'report.json').read_text())
    rows = prepared[args.case]
    ego = ego_trajectory(Path(manifest['cases'][args.case]['raw_bag']))
    extrinsic = load_sensor4_calibration(args.sensor_config)['base_from_lidar']
    velocities = trajectory_velocities(rows, ego, extrinsic)
    periods = np.diff([r['stamp'] for r in rows]); period = float(np.median(periods[(periods > 0) & (periods <= .25)]))
    observations = {}
    for mode in ('baseline', args.candidate):
        _, observations[mode] = score_case(rows, results[args.case + '/' + mode]['frames'], ego, extrinsic, velocities, period)
    values = []
    for row in rows:
        keys = [str(k) for k in row['gt_track_keys']]
        if args.gt_key not in keys:
            continue
        g = keys.index(args.gt_key)
        p = next((p for p, matched in row['assignments'][0][2.].items() if matched == g), None)
        item = dict(token=row['token'], stamp=row['stamp'], distance=float(np.linalg.norm(row['gt_boxes'][g, :2])),
                    lidar_x=float(row['gt_boxes'][g, 0]), score=float(row['pred_scores'][p]) if p is not None else None)
        for mode in observations:
            obs = observations[mode].get(row['token'] + '/' + args.gt_key)
            item[mode] = obs
        values.append(item)
    if len(values) < 3:
        raise ValueError('At least three selected trajectory observations required')
    start = values[0]['stamp']; ts = np.asarray([v['stamp'] - start for v in values])
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True, constrained_layout=True)
    axes[0].plot(ts, [v['distance'] for v in values], label='Radial distance')
    axes[0].plot(ts, [v['lidar_x'] for v in values], label='Signed LiDAR X', alpha=.7)
    axes[0].set_ylabel('Distance (m)'); axes[0].legend(loc='upper left'); axes[0].axhline(0, color='gray', lw=.8)
    axes[1].plot(ts, [v['score'] if v['score'] is not None else np.nan for v in values], '.-', label='Vehicle candidate matched within 2m')
    axes[1].axhline(.4, color='red', ls='--', label='Current threshold 0.40')
    axes[1].set_ylim(0, 1); axes[1].set_ylabel('Detector score'); axes[1].legend(loc='upper left')
    for axis, metric, label in ((axes[2], 'position', 'Center error (m)'), (axes[3], 'velocity', 'Velocity error (m/s)')):
        for mode, color in (('baseline', 'tab:orange'), (args.candidate, 'tab:blue')):
            axis.plot(ts, [v[mode][metric] if v[mode] and v[mode][metric] is not None else np.nan for v in values], label=mode, color=color)
        axis.set_ylabel(label); axis.legend(loc='upper left')
    for axis in axes:
        axis.grid(alpha=.25)
    axes[-1].set_xlabel('Seconds since first eligible GT observation in this bag segment')
    fig.suptitle(args.case + ' / ' + args.gt_key.split('/')[-1] + '\n'
                 'Offline core; gaps = no matched observation; velocity reference = local center-trajectory fit')
    args.output.mkdir(parents=True)
    for suffix in ('svg', 'png'):
        fig.savefig(args.output / ('trace.' + suffix), dpi=130)
    plt.close(fig)
    (args.output / 'trace.json').write_text(json.dumps(dict(case=args.case, gt_key=args.gt_key,
        candidate=args.candidate, observations=values,
        report_sha256=hashlib.sha256((args.replay / 'report.json').read_bytes()).hexdigest()), indent=2) + '\n')
    print('Saved trace', args.output)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('inputs', 'predictions', 'replay', 'sensor-config', 'output'):
        p.add_argument('--' + name, type=Path, required=True)
    p.add_argument('--case', required=True); p.add_argument('--gt-key', required=True)
    p.add_argument('--candidate', default='weak010_xy')
    render(p.parse_args())
