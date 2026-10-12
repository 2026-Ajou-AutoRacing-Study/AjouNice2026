#!/usr/bin/env python3
"""Frozen detector selection; no ROS, Docker, GPU or credential imports."""
import argparse
import hashlib
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PROFILES = {
    'e24': dict(checkpoint='base5_e24_inference.pth',
        sha256='84c7a849406a37f05250f872d4d0dfa3ebcffdb66b48e47bdfcb3209efecc746',
        model_config='perception/detection3d/bevfusion/configs/bevfusion_base5_fine_512_depth25_pedestrian_merged_runtime.py',
        bridge_config='configs/perception/bevfusion_base5_production.yaml',
        launch='base5_production.launch', base5=True, parallel=True,
        thresholds=[.4,.1,.1], sensor='configs/sensors/Sensor_competition_lan.json'),
    'e25': dict(checkpoint='base3_bf0_f40_e25_inference.pth',
        sha256='36816cf769c87d49483d002111abfad8e1afd043afd74a0b9fc54aac82133011',
        model_config='perception/detection3d/bevfusion/configs/bevfusion_base3_b0_4cam_lidar.py',
        bridge_config='configs/perception/bevfusion_sensor4.yaml',
        launch='base3_online.launch', base5=False, parallel=False,
        thresholds=[.4,.15,.1], sensor='configs/sensors/Sensor.json'),
}


def digest(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(8*1024**2),b''):h.update(block)
    return h.hexdigest()


def profile(model='e24',home=None):
    p=dict(PROFILES[model],model=model,amp=True,compile_parts=[],port=29500)
    p['checkpoint_path']=str((Path(home) if home else Path.home())/'bevfusion_artifacts/checkpoints'/p['checkpoint'])
    return p


def resolve_checkpoint(model,environment,home=None):
    # Historical .env INFERENCE_CHECKPOINT is the E25 compatibility path.
    # Do not let it silently select E25 for the newly promoted default E24.
    override=environment.get('BEVFUSION_CHECKPOINT')
    if not override and model=='e25':override=environment.get('BEVFUSION_INFERENCE_CHECKPOINT')
    return override or profile(model,home)['checkpoint_path']


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model',choices=PROFILES,default='e24')
    parser.add_argument('--fields',action='store_true')
    a=parser.parse_args();p=profile(a.model)
    if a.fields:
        p['checkpoint_path']=resolve_checkpoint(a.model,os.environ)
        print('\n'.join(str(p[k]).lower() if isinstance(p[k],bool) else str(p[k]) for k in
            ('checkpoint_path','sha256','model_config','bridge_config','launch','base5','parallel','sensor')))
    else:print(json.dumps(p,indent=2))
