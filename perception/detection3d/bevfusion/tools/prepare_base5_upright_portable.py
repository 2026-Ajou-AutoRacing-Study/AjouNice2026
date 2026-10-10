#!/usr/bin/env python3
"""Rebuild Base5 upright 7D labels from immutable native-RPY capture audits.

Preserve every selected component, sensor evidence, split and input path.
Fresh output required. Hardlinks save local disk; never modify shared payload.
The same script works on the existing remote JPEG dataset without retransmitting
images/LiDAR. No model, ROS, simulator or cloud connection is started.
"""
import argparse
from collections import Counter
from copy import deepcopy
import itertools
import json
import os
from pathlib import Path
import pickle
import shutil
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[4]))
from dataset.morai_od.scripts.base5_upright_boxes import BOX_GEOMETRY_POLICY, component_bbox
from prepare_base5_portable_dataset import safe_relative, sha256, validate


def rotation(rpy):
    r, p, y = rpy
    rx = np.array([[1,0,0],[0,np.cos(r),-np.sin(r)],[0,np.sin(r),np.cos(r)]])
    ry = np.array([[np.cos(p),0,np.sin(p)],[0,1,0],[-np.sin(p),0,np.cos(p)]])
    rz = np.array([[np.cos(y),-np.sin(y),0],[np.sin(y),np.cos(y),0],[0,0,1]])
    return rz @ ry @ rx


def native_corners(component):
    signs = np.array(list(itertools.product((-1., 1.), repeat=3)))
    return (signs * component['size_m'] / 2) @ rotation(component['rotation_rpy']).T + component['center_lidar_m']


def check_corners(component, box, tolerance=1e-10):
    box = np.asarray(box)
    yaw_rotation = rotation([0,0,box[6]])
    local = (native_corners(component) - box[:3]) @ yaw_rotation
    if not (np.abs(local) <= box[3:6]/2 + tolerance).all():
        raise ValueError('Derived upright box does not enclose native corners')


def build(data, output, hardlink=False):
    data = Path(data).resolve(strict=True); output = Path(output).absolute()
    if output.exists(): raise FileExistsError('Use a fresh upright portable output')
    source = validate(data, verify_hashes=True)
    if source.get('box_geometry_policy') == BOX_GEOMETRY_POLICY:
        raise ValueError('Source already has the current upright geometry policy')
    source_hash = sha256(data/'manifest.json')
    cache = {}; changes = []; stats = {}; rewritten = {}; source_infos = {}
    classes = ('vehicle','pedestrian','obstacle','animal')
    for profile in source['profiles']:
        for split in ('train','val','test'):
            relative = f'infos/{profile}/morai_base5_infos_{split}.pkl'
            source_infos[relative] = sha256(data/relative)
            info = pickle.loads((data/relative).read_bytes())
            if tuple(info['metainfo']['classes']) != classes:
                raise ValueError('Unexpected class ordering')
            if info['metainfo'].get('box_geometry_policy') == BOX_GEOMETRY_POLICY:
                raise ValueError('Source infos already use current upright geometry')
            counts = Counter()
            for row in info['data_list']:
                token = row['token']
                if token not in cache:
                    audit = json.loads((data/safe_relative('data/'+token+'/audit.json')).read_text())
                    components = {c['component_id']:c for c in audit['components']}
                    if len(components) != len(audit['components']):
                        raise ValueError('Duplicate native component identity')
                    cache[token] = components
                for instance in row['instances']:
                    component = cache[token][instance['component_id']]
                    if component['parent_uid'] != instance['parent_uid'] or component['asset'] != instance['asset']:
                        raise ValueError('Native component identity differs')
                    if classes[instance['bbox_label_3d']] != component['training_class']:
                        raise ValueError('Native component class differs')
                    original = deepcopy(instance['bbox_3d'])
                    native_yaw_box = list(component['center_lidar_m']) + list(component['size_m']) + [component['rotation_rpy'][2]]
                    np.testing.assert_allclose(original, native_yaw_box, rtol=1e-7, atol=1e-6)
                    box = component_bbox(component)
                    check_corners(component, box)
                    if list(original[:3]) != box[:3] or original[6] != box[6]:
                        raise ValueError('Center/yaw changed unexpectedly')
                    instance['bbox_3d'] = box
                    counts['gt'] += 1
                    if not np.allclose(original[3:6],box[3:6],rtol=0,atol=1e-8):
                        counts['size_changed'] += 1
                        changes.append(dict(profile=profile,split=split,token=token,
                            component_id=instance['component_id'],asset=instance['asset'],
                            native_rpy=component['rotation_rpy'],before=original,after=box))
                    tilt = np.degrees(np.arccos(np.clip(abs(rotation(component['rotation_rpy'])[2,2]),0,1)))
                    for threshold in (5,20):
                        if tilt > threshold: counts[f'tilt_above_{threshold}_deg'] += 1
                    if component['lidar_points'] == 0: counts['camera_only'] += 1
                counts['frames'] += 1
            info['metainfo']['box_geometry_policy'] = BOX_GEOMETRY_POLICY
            rewritten[relative] = pickle.dumps(info, protocol=4)
            stats[profile+':'+split] = dict(counts)
    # All semantic choices are frozen; only bbox sizes and policy metadata differ.
    report = dict(valid=True,box_geometry_policy=BOX_GEOMETRY_POLICY,
        source_manifest_sha256=source_hash,source_info_sha256=source_infos,stats=stats,
        geometry_helper_sha256=sha256(Path(__file__).resolve().parents[4]/'dataset/morai_od/scripts/base5_upright_boxes.py'),
        unchanged='tokens, splits, centers, yaw, classes, component IDs, sensor evidence, raw and input paths',
        num_lidar_pts_contract='Native full-RPY box support count, not the enlarged box occupancy; unchanged',
        changes=changes,
        limitations=['Fixed-yaw enclosure, not minimum-volume box or mesh segmentation.',
                     'No new camera/LiDAR evidence inferred from expanded labels.',
                     'Raw simulator labels are not universally certified by this conversion.'])
    output.mkdir(parents=True)
    files = {}
    for relative,item in source['files'].items():
        target = output/safe_relative(relative); target.parent.mkdir(parents=True,exist_ok=True)
        if relative in rewritten:
            target.write_bytes(rewritten[relative])
            files[relative] = dict(bytes=target.stat().st_size,sha256=sha256(target))
        else:
            if hardlink: os.link(data/relative,target)
            else: shutil.copy2(data/relative,target)
            files[relative] = deepcopy(item)
    for relative,payload in (
        ('provenance/pre_upright_manifest.json',(data/'manifest.json').read_bytes()),
        ('provenance/upright_conversion.json',(json.dumps(report,indent=2)+'\n').encode())):
        if relative in files: raise ValueError('Geometry provenance already exists')
        target=output/relative;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(payload)
        files[relative]=dict(bytes=len(payload),sha256=sha256(target))
    manifest=deepcopy(source)
    manifest.update(files=files,bytes=sum(i['bytes'] for i in files.values()),
        box_geometry_policy=BOX_GEOMETRY_POLICY,
        source_pre_upright_manifest_sha256=source_hash,
        copy_mode='hardlink' if hardlink else 'copy')
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    validate(output,verify_hashes=True)
    if source_hash != sha256(data/'manifest.json'):
        raise ValueError('Source manifest changed during conversion')
    print(json.dumps({k:v for k,v in report.items() if k!='changes'},indent=2),flush=True)
    return manifest,report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--hardlink',action='store_true',help='Immutable same-disk output only')
    a=p.parse_args();build(a.data,a.output,a.hardlink)


if __name__=='__main__':main()
