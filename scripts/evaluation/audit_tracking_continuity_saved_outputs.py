#!/usr/bin/env python3
"""Frozen, previously inferred four-bag regression of offline tracker candidates.

No new inference or live ROS. Numeric UUIDs from separate runs are never
called GT switches without the prepared actor correspondence. Prepared GT
and recorded Ego transforms retain the original benchmark limitations.
"""
import argparse
from collections import defaultdict
import gzip
import json
import math
from pathlib import Path
import pickle
import sys
import numpy as np

from audit_tracking_continuity import ROOT
from audit_tracking_continuity_events import core_summary, replay, sha


def nonped_states_equal(reference, candidate):
    """UUID counters can shift when a pedestrian birth is suppressed."""
    def states(frame):
        return sorted(json.dumps({k: v for k, v in track.items() if k != 'id'}, sort_keys=True)
                      for track in frame['tracks'] if track['core_class'] not in (3, 4))
    return len(reference) == len(candidate) and all(a['stamp'] == b['stamp'] and states(a) == states(b)
                                                  for a, b in zip(reference, candidate))


def prepared_cases(inputs, prediction, sensor):
    sys.path.insert(0, str(ROOT/'perception/detection3d/bevfusion/tools'))
    from analyze_base5_runtime_effects import delivered
    from replay_base5_compiled_tracking import ego_trajectory, world_detections, load_sensor4_calibration
    manifest = json.loads((inputs/'manifest.json').read_text())
    infos = {r['token']: r for r in pickle.loads((inputs/'infos_val.pkl').read_bytes())['data_list']}
    payload = json.load(gzip.open(prediction, 'rt'))
    if payload['frames'] != len(infos) or {r['token'] for r in payload['rows']} != set(infos):
        raise ValueError('Prepared inputs and frozen predictions differ')
    extrinsic = load_sensor4_calibration(sensor)['base_from_lidar']
    result = {}
    for case, details in manifest['cases'].items():
        source = Path(details['raw_bag'])
        ego = ego_trajectory(source)
        frames = []
        gt_series = defaultdict(dict)
        ego_points = []
        rows = sorted([r for r in payload['rows'] if r['token'].startswith(case+'_')], key=lambda r: infos[r['token']]['timestamp'])
        for row in rows:
            info = infos[row['token']]; t = info['timestamp']
            ds = [dict(center=b[:3], size=b[3:6], yaw=b[6], score=s, label=l,
                       class_name=('vehicle', 'pedestrian', 'obstacle')[l])
                  for b, l, s in zip(row['pred_boxes'], row['pred_labels'], row['pred_scores'])]
            world = world_detections(delivered(ds, (.4, .1, .1)), t, ego, extrinsic)
            frames.append(dict(stamp=t, world_objects=world))
            # Use each prepared instance, not inferred boxes or a visibility
            # deletion. Asset/actor keys are explicit original benchmark data.
            for instance in info['instances']:
                if not instance['bbox_3d_isvalid']:
                    continue
                b = instance['bbox_3d']; cls = ('vehicle', 'pedestrian', 'obstacle')[instance['bbox_label_3d']]
                obj = world_detections([dict(center=b[:3], size=b[3:6], yaw=b[6])], t, ego, extrinsic)[0]
                gt_series[instance['track_key']][t] = dict(obj, key=instance['track_key'], class_name=cls, asset=instance['asset'])
            # Transform origin of base frame with the exact same recorded pose
            # interpolation as the detections, not an offset LiDAR origin.
            base_origin = world_detections([dict(center=[0., 0., 0.], size=[1., 1., 1.], yaw=0.)], t, ego,
                                           np.eye(4))[0]
            ego_points.append(dict(center=base_origin['center'], yaw=base_origin['yaw']))
        times = [r['stamp'] for r in frames]
        case_gt = dict(ego=(times, ego_points), gt=[(sorted(values), [values[t] for t in sorted(values)])
                                                   for values in gt_series.values()], unresolved=[])
        result[case] = dict(frames=frames, gt=case_gt, raw_bag=str(source),
                            original_frame_resolution=details.get('frame_resolution'),
                            original_catalog_resolution=details.get('catalog_resolution'))
    return result, {k: v for k, v in payload.items() if k != 'rows'}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--inputs', type=Path, required=True)
    p.add_argument('--predictions', type=Path, required=True)
    p.add_argument('--sensor-config', type=Path, required=True)
    p.add_argument('--candidate-root', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if a.output.exists():
        raise FileExistsError('Fresh regression output required')
    a.output.mkdir(parents=True)
    cases, model = prepared_cases(a.inputs, a.predictions, a.sensor_config)
    binaries = {'baseline': a.candidate_root/'core_build/original_core'}
    binaries.update({path.name.removeprefix('candidate_') if hasattr(path.name, 'removeprefix') else path.name[len('candidate_'):]:
                     path/'original_core' for path in sorted(a.candidate_root.glob('candidate_*'))})
    report = dict(predictions_sha256=sha(a.predictions), inputs_sha256=sha(a.inputs/'infos_val.pkl'),
                  sensor_sha256=sha(a.sensor_config), recorded_model=model, cases={},
                  scope='Frozen prepared four-bag offline core regression; not untouched test, live ROS or closed-loop planner.',
                  production_changed=False)
    for name, case in cases.items():
        report['cases'][name] = dict(raw_bag=case['raw_bag'], input_frames=len(case['frames']),
                                    original_frame_resolution=case['original_frame_resolution'], results={})
        reference = None
        for mode, binary in binaries.items():
            rows = replay(binary, case['frames'])
            report['cases'][name]['results'][mode] = core_summary(rows, case['gt'])
            if reference is None:
                reference = rows
            report['cases'][name]['results'][mode]['nonpedestrian_core_states_identical_ignoring_id_counter'] = nonped_states_equal(reference, rows)
            with gzip.open(a.output/(name+'_'+mode+'.json.gz'), 'wt') as handle:
                json.dump(rows, handle, allow_nan=False)
        print(name, {mode: v['confirmed_unique_ids'] for mode, v in report['cases'][name]['results'].items()}, flush=True)
    (a.output/'regression.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')


if __name__ == '__main__':
    main()
