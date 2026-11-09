#!/usr/bin/env python3
"""Freeze existing validation inputs for an isolated, non-training GPU comparison."""
import argparse
import hashlib
import json
from pathlib import Path
import pickle
import random
import subprocess
import sys


def audit_calibration(root, info):
    import numpy as np
    sys.path.insert(0,str(root/'docker/bevfusion'))
    from app.sensor4_calibration import load_sensor4_calibration
    sensor=root/'configs/sensors/Sensor_competition_local.json'
    calibration=load_sensor4_calibration(sensor)
    mapping=dict(camera_front='Camera_front',camera_front_left='Camera_front_left',
                 camera_front_right='Camera_front_right',camera_back='Camera_back')
    rows=pickle.loads(info.read_bytes())['data_list']
    maxima=dict(intrinsic=0.,extrinsic=0.)
    for row in rows:
        for camera in calibration['cameras']:
            old=row['images'][mapping[camera['frame_id']]]
            if (old['width'],old['height'])!=(camera['width'],camera['height']):
                raise ValueError('Native/runtime image dimensions differ')
            maxima['intrinsic']=max(maxima['intrinsic'],float(np.max(np.abs(
                np.asarray(old['cam2img'])[:3,:3]-camera['cam2img']))))
            maxima['extrinsic']=max(maxima['extrinsic'],float(np.max(np.abs(
                np.asarray(old['lidar2cam'])-camera['lidar2cam']))))
    if max(maxima.values())>1e-8:raise ValueError('Native/runtime camera calibration differs')
    return dict(complete=True,frames=len(rows),camera_records=len(rows)*4,
                sensor_sha256=sha(sensor),maximum_absolute_error=maxima)


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(2**20), b''):
            h.update(chunk)
    return h.hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--audit-calibration-only',action='store_true')
    args = p.parse_args()
    root, out = args.root.resolve(), args.output.resolve()
    if args.audit_calibration_only:
        info=Path(json.loads((out/'INVENTORY.json').read_text())['info'])
        result=audit_calibration(root,info)
        with (out/'CALIBRATION_AUDIT.json').open('x') as stream:json.dump(result,stream,indent=2)
        print(json.dumps(result));return
    if out.exists():
        raise ValueError('Fresh output required')
    out.mkdir(parents=True)
    info = root/'artifacts/evaluation/bevfusion/e24_e12_bags_261023_v1/native_far_review/infos_full_val_4614.pkl'
    blob = pickle.loads(info.read_bytes())
    rows = blob['data_list']
    if len(rows) != 4614 or len({r['token'] for r in rows}) != 4614:
        raise ValueError('Expected all 4614 frozen validation frames')
    inputs = {}
    for r in rows:
        for file in [r['lidar_points']['lidar_path'], *[v['img_path'] for v in r['images'].values()]]:
            file = Path(file)
            if not file.is_file():
                raise FileNotFoundError(file)
            inputs[str(file)] = sha(file)
    cameras = dict(Camera_front='image/camera_front', Camera_front_left='image/camera_front_left',
                   Camera_front_right='image/camera_front_right', Camera_back='image/camera_back')
    # Select by fixed seed and immutable token, never by predictions or success.
    ordered = sorted(rows, key=lambda r: r['token'])
    selected = random.Random(261109).sample(ordered, 96)
    samples = []
    for r in selected:
        point = r['lidar_points']['lidar_path']
        samples.append(dict(id=r['token'], group='frozen_validation_seed261109',
            points=dict(path=point, sha256=inputs[point]), images={key:dict(
                path=r['images'][camera]['img_path'], sha256=inputs[r['images'][camera]['img_path']])
                for camera, key in cameras.items()}))
    manifest = dict(schema='base5_camera_trt_inputs_v1', image_policy='native_jpeg90',
                    point_intensity_source='native', samples=samples)
    (out/'timing_inputs.json').write_text(json.dumps(manifest, indent=2)+'\n')
    (out/'input_sha256.json').write_text(json.dumps(inputs, indent=2)+'\n')
    pilot = dict(blob, data_list=rows[:8])
    with (out/'pilot_infos.pkl').open('xb') as stream:
        pickle.dump(pilot, stream)
    checkpoints = {
        'e25': root/'artifacts/detection3d/bevfusion/releases/base3_bf0_f40_e25/base3_bf0_f40_e25_inference.pth',
        'e12': root/'artifacts/evaluation/bevfusion/e24_e12_bags_261023_v1/checkpoint/E12_best.pth',
    }
    report = dict(schema='portfolio_5060ti_inventory_v1', info=str(info), info_sha256=sha(info),
        frames=len(rows), sensor_files=len(inputs), checkpoints={k:dict(path=str(v),sha256=sha(v))
        for k,v in checkpoints.items()}, git_head=subprocess.check_output(
            ['git','-C',str(root),'rev-parse','HEAD'],text=True).strip(),
        scope='Existing validation, not a new held-out test. Never train or alter production.',
        speed_samples=len(samples), input_bytes=sum(Path(f).stat().st_size for f in inputs))
    (out/'INVENTORY.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report), flush=True)


if __name__ == '__main__':
    main()
