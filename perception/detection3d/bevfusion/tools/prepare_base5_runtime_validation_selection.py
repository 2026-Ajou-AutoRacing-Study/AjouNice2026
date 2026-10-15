#!/usr/bin/env python3
"""Select the entire frozen 745-frame native validation split, without resampling."""
import argparse
import hashlib
import json
from pathlib import Path
import pickle


def build(data, output):
    if output.exists():
        raise ValueError('Fresh validation selection required')
    path = data/'infos/512x1056/morai_base5_infos_val.pkl'
    blob = path.read_bytes()
    info = pickle.loads(blob)
    if info['metainfo'].get('box_geometry_policy') != 'native_rpy_fixed_yaw_enclosure_v1':
        raise ValueError('Current upright validation GT required')
    rows = sorted(info['data_list'], key=lambda r:r['token'])
    if len(rows) != 745 or len({r['token'] for r in rows}) != 745:
        raise ValueError('Expected the complete distinct 745-frame frozen validation split')
    frames = ('Camera_front','Camera_front_left','Camera_front_right','Camera_back')
    samples = [dict(id=r['token'],split='val',sample_group='validation',
        lidar_path=r['lidar_points']['lidar_path'],
        camera_paths=[r['images'][f]['img_path'] for f in frames]) for r in rows]
    result = dict(scope='Entire frozen 745-frame Sunny13 validation; no training samples or stress oversampling.',
        info_sha256=hashlib.sha256(blob).hexdigest(),samples=samples)
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(frames=len(samples),info_sha256=result['info_sha256'])))
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();build(a.data,a.output)
