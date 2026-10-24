#!/usr/bin/env python3
"""Freeze saved E12/E24 detections through the unchanged production EKF core.

No GPU, ROS master, runtime configuration edits, threshold tuning or new GT.
All prepared frames feed the tracker; eligibility only masks evaluation.
This is measurement-time core replay, not the live 30 Hz ROS scheduler.
"""
import argparse
from collections import defaultdict
import gzip
import hashlib
import json
import math
from pathlib import Path
import pickle
import shlex
import subprocess
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
TOOLS = ROOT / 'perception/detection3d/bevfusion/tools'
sys.path.insert(0, str(TOOLS))
sys.path.insert(0, str(ROOT / 'docker/bevfusion'))
from analyze_base5_runtime_effects import delivered
from analyze_base5_continuous_tensorrt_tracking import parse_core
from compare_base5_common30 import validate_payload
from replay_base5_compiled_tracking import ego_trajectory
from app.sensor4_calibration import load_sensor4_calibration

CLASSES = ('vehicle', 'pedestrian', 'obstacle')
OLD = ROOT / 'artifacts/evaluation/bevfusion/e24_e10_additional_bags_261023_v1'
NEW = ROOT / 'artifacts/evaluation/bevfusion/e24_e12_bags_261023_v1'
SPECS = {
    'core': (ROOT / 'artifacts/evaluation/bevfusion/base5_far_bag_retention_261012/inputs_v1',
             ROOT / 'artifacts/evaluation/bevfusion/full30750_e2_rosbag_261021_v1/e24_fresh_v2'),
    'september': (OLD / 'inputs_v1', OLD / 'inference/september_e24'),
    'october': (OLD / 'supplemental_inputs_v1', OLD / 'inference/october_e24'),
    'man1_actual50': (OLD / 'man1_actual50_inputs_v1', OLD / 'inference/man1_actual50_e24'),
}


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as handle:
        for block in iter(lambda: handle.read(8 << 20), b''):
            h.update(block)
    return h.hexdigest()


def pose_at(rows, stamp):
    i = int(np.searchsorted(rows[:, 0], stamp))
    if i < len(rows) and abs(rows[i, 0] - stamp) < 1e-8:
        return rows[i, 1:]
    if i == 0 or i == len(rows) or rows[i, 0] - rows[i-1, 0] > .15:
        raise ValueError('Recorded Ego requires bounded interpolation')
    alpha = (stamp - rows[i-1, 0]) / (rows[i, 0] - rows[i-1, 0])
    return rows[i-1, 1:] + alpha * (rows[i, 1:] - rows[i-1, 1:])


def transform_for_pose(pose, extrinsic):
    x, y, z, yaw = map(float, pose)
    c, s = math.cos(yaw), math.sin(yaw)
    matrix = np.eye(4)
    matrix[:2, :2] = [[c, -s], [s, c]]
    matrix[:3, 3] = [x, y, z]
    return matrix @ extrinsic


def world_box(box, pose, extrinsic, bottom):
    box = list(map(float, box))
    center = np.array([*box[:3], 1.])
    if bottom:
        center[2] += box[5] / 2
    world = transform_for_pose(pose, extrinsic) @ center
    return dict(center=world[:3].tolist(), size=box[3:6],
                yaw=box[6] + float(pose[3]) + math.atan2(extrinsic[1, 0], extrinsic[0, 0]))


def core_frames(binary, frames):
    lines = []
    for frame in frames:
        detections = frame['detections']
        lines.append('%.17g %d' % (frame['stamp'], len(detections)))
        for d in detections:
            lines.append(' '.join('%.17g' % v for v in
                [d['label'], *d['center'], d['yaw'], *d['size'], d['score']]))
    output = subprocess.run([str(binary)], input='\n'.join(lines) + '\n', text=True,
                            capture_output=True, check=True, timeout=240)
    return parse_core(output.stdout, [r['stamp'] for r in frames])


def build_core(output):
    tracker = ROOT / 'base_ws/src/ekf-multi-object-tracking'
    harness = TOOLS / 'replay_base5_continuous_tracker_core.cpp'
    source = tracker / 'src/algorithm/ekf_multi_object_tracking.cpp'
    header = tracker / 'include/algorithm/ekf_multi_object_tracking.hpp'
    flags = shlex.split(subprocess.check_output(['pkg-config', '--cflags', '--libs', 'roscpp'], text=True))
    binary = (output / 'production_core').resolve()
    cmd = ['g++', '-std=c++17', '-O1', '-I/usr/include/eigen3', '-I' + str(tracker/'include'),
           str(harness), str(source), '-o', str(binary), *flags]
    subprocess.run(cmd, check=True)
    return binary, dict(command=cmd, source_sha256={str(p): sha(p) for p in (harness, source, header)},
                        binary_sha256=sha(binary))


def prepare(args):
    if args.output.exists():
        raise FileExistsError('Fresh output required')
    args.output.mkdir(parents=True)
    sensor = ROOT / 'configs/sensors/Sensor_competition_lan.json'
    extrinsic = np.asarray(load_sensor4_calibration(sensor)['base_from_lidar'], float)
    binary, build = build_core(args.output)
    pins = {str(sensor): sha(sensor), str(Path(__file__)): sha(__file__)}
    pins.update(build['source_sha256'])
    pack = dict(schema='e12_postprocessing_core_pack_v1', model=args.model,
        production_changed=False, provenance=dict(core_build=build, source_sha256=pins,
        classes=list(CLASSES), thresholds=[.4, .1, .1], rotated_nms_iou=.5,
        world_frame='recorded MORAI Ego.position, heading -1.347deg once',
        box_center='gravity, converted once from detector bottom; GT already gravity',
        backend='cached eager AMP/parallel/nearest; not newly inferred TRT/head outputs',
        support='prepared finite nonzero LiDAR count in catalog OBB +3cm; not native annotation',
        scope='Unmodified actual production EKF at prepared measurement times, not live ROS 30Hz/TF/planner'), cases={})
    for name in args.sets:
        inputs, e24 = SPECS[name]
        predictions = (NEW / ('inference/' + name + '_e12') if args.model == 'E12' else e24) / 'predictions.json.gz'
        paths = [inputs/'infos_val.pkl', inputs/'manifest.json', predictions, predictions.with_name('receipt.json')]
        pins.update({str(p): sha(p) for p in paths})
        records = pickle.loads(paths[0].read_bytes())['data_list']
        manifest = json.loads(paths[1].read_text())
        with gzip.open(predictions, 'rt') as handle:
            payload = json.load(handle)
        validate_payload(payload)
        if payload['info_sha256'] != sha(paths[0]):
            raise ValueError('Prediction/input SHA differs')
        expected_sha = ('f3ade1bf17375c061ea6c0284d636cfc647c98b745f4534cb365499e1394dba8' if args.model == 'E12'
                        else '84c7a849406a37f05250f872d4d0dfa3ebcffdb66b48e47bdfcb3209efecc746')
        if payload['checkpoint_sha256'] != expected_sha:
            raise ValueError('Unexpected checkpoint identity')
        predicted = {r['token']: r for r in payload['rows']}
        if len(predicted) != len(records) or set(predicted) != {r['token'] for r in records}:
            raise ValueError('Missing or duplicate frozen input tokens')
        grouped = defaultdict(list)
        for r in records:
            grouped[r['scene_id']].append(r)
        for case, source_rows in grouped.items():
            contract = manifest['cases'][case]
            ego = ego_trajectory(Path(contract['raw_bag']))
            frames, trajectories = [], defaultdict(list)
            source_rows.sort(key=lambda r: r['timestamp'])
            for source in source_rows:
                row = predicted[source['token']]
                expected = np.asarray([i['bbox_3d'] for i in source['instances']], float).reshape(-1, 7)
                expected[:, 2] -= expected[:, 5]/2
                if not np.allclose(expected, np.asarray(row['gt_boxes']).reshape(-1, 7), atol=1e-5, rtol=0):
                    raise ValueError('GT geometry differs')
                if row['gt_points'] != [i['num_lidar_pts'] for i in source['instances']]:
                    raise ValueError('GT support differs')
                stamp = float(source['timestamp'])
                pose = pose_at(ego, stamp)
                detections = [dict(center=b[:3], size=b[3:6], yaw=b[6], label=int(l),
                    class_name=CLASSES[l], score=float(s)) for b,l,s in
                    zip(row['pred_boxes'], row['pred_labels'], row['pred_scores'])]
                detections = [dict(d, **world_box([*d['center'], *d['size'], d['yaw']], pose, extrinsic, True))
                              for d in delivered(detections, (.4, .1, .1))]
                truth = []
                for instance in source['instances']:
                    label = int(instance['bbox_label_3d'])
                    obj = dict(world_box(instance['bbox_3d'], pose, extrinsic, False),
                        key=str(instance['track_key']), label=label, class_name=CLASSES[label],
                        lidar_points=int(instance['num_lidar_pts']), asset=instance.get('asset'),
                        local_box=list(map(float, instance['bbox_3d'])))
                    truth.append(obj)
                    trajectories[(obj['key'],label)].append([stamp, obj['center']])
                frames.append(dict(stamp=stamp, token=source['token'], native_index=int(source['raw_lidar_index']),
                    scoring_eligible=bool(source['scoring_eligible']),
                    ego=dict(zip(('x','y','z','yaw'), map(float,pose))), gt=truth, detections=detections))
            results = core_frames(binary, frames)
            repeated = core_frames(binary, frames)
            if results != repeated:
                raise ValueError('Same saved detections failed fresh-core determinism')
            for frame, result in zip(frames, results):
                frame['tracks'] = result['tracks']
            deltas = np.diff([f['stamp'] for f in frames])
            index_gaps = sum(b['native_index'] != a['native_index']+1 for a,b in zip(frames, frames[1:]))
            key = name + '/' + case
            pack['cases'][key] = dict(frames=frames, raw_bag=contract['raw_bag'],
                gt_trajectories=[dict(key=k,label=l,class_name=CLASSES[l],series=series)
                                 for (k,l),series in trajectories.items()],
                provenance=dict(input_infos=str(paths[0]), input_sha256=sha(paths[0]),
                    predictions=str(predictions), predictions_sha256=sha(predictions),
                    recorded_model={k:v for k,v in payload.items() if k != 'rows'},
                    all_input_frames=len(frames), scoring_frames=sum(f['scoring_eligible'] for f in frames),
                    max_interframe_sec=float(deltas.max()) if len(deltas) else None,
                    raw_index_gaps=index_gaps, fresh_core_repeat_exact=True))
            print(key, len(frames), 'scoring', sum(f['scoring_eligible'] for f in frames), flush=True)
    if pins != {p:sha(p) for p in pins}:
        raise ValueError('Frozen source/input changed during analysis')
    pack['complete'] = True
    with gzip.open(args.output/'replay_pack.json.gz', 'xt') as handle:
        json.dump(pack, handle, allow_nan=False)
    receipt = dict(complete=True, schema=pack['schema'], model=args.model,
        pack_sha256=sha(args.output/'replay_pack.json.gz'), build=build,
        cases={k:v['provenance'] for k,v in pack['cases'].items()}, source_sha256=pins)
    (args.output/'PREPARED.json').write_text(json.dumps(receipt, indent=2, allow_nan=False)+'\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', choices=('E12','E24'), default='E12')
    parser.add_argument('--sets', nargs='+', choices=tuple(SPECS), default=list(SPECS))
    parser.add_argument('--output', type=Path, required=True)
    prepare(parser.parse_args())
