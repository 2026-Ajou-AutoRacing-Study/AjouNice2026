#!/usr/bin/env python3
"""Audit geometric image support, NOT visibility/occlusion or detector accuracy."""
import argparse
import json
import math
import pickle
from pathlib import Path

import numpy as np


def projected_support(box, lidar2cam, intrinsic, width, height):
    center = np.asarray(box[:3], dtype=float)
    size = np.asarray(box[3:6], dtype=float)
    signs = np.asarray([(x, y, z) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])
    corners = signs * size / 2
    c, s = math.cos(box[6]), math.sin(box[6])
    rotation = np.asarray([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    corners = corners @ rotation.T + center
    xyz = (np.asarray(lidar2cam) @ np.c_[corners, np.ones(8)].T).T[:, :3]
    if not np.all(xyz[:, 2] > .2):
        return dict(supported=False, reason='behind_or_crossing_near_plane')
    uv = (np.asarray(intrinsic)[:3, :3] @ xyz.T).T
    uv = uv[:, :2]/uv[:, 2:3]
    lo, hi = uv.min(axis=0), uv.max(axis=0)
    clipped_lo = np.maximum(lo, [0, 0])
    clipped_hi = np.minimum(hi, [width, height])
    extent = np.maximum(0, clipped_hi-clipped_lo)
    def model_extent(final_width, final_height):
        # MoraiImageAug3D: resolution-relative isotropic resize, bottom crop.
        # Do not scale height directly by final_height/native_height.
        scale = final_width/width
        crop_top = int(height*scale)-final_height
        low = lo*scale-[0, crop_top]
        high = hi*scale-[0, crop_top]
        return np.maximum(0, np.minimum(high, [final_width, final_height])-np.maximum(low, [0, 0])).tolist()
    return dict(supported=bool(np.all(extent > 0)), reason='projected_rectangle',
                native_extent_px=extent.tolist(), native_area_px=float(np.prod(extent)),
                resized_256x704_px=model_extent(704, 256),
                resized_320x896_px=model_extent(896, 320))


def audit(info_root, output):
    rows = []
    for split in ('train', 'val', 'test'):
        with (info_root/('morai_base3_infos_'+split+'.pkl')).open('rb') as stream:
            data = pickle.load(stream)
        for sample in data['data_list']:
            for actor in sample['instances']:
                projections = {name: projected_support(actor['bbox_3d'], image['lidar2cam'], image['cam2img'],
                               image['width'], image['height']) for name, image in sample['images'].items()}
                rows.append(dict(sample_id=sample.get('sample_idx'), split=split,
                                 instance_id=actor.get('instance_id'), num_lidar_pts=actor['num_lidar_pts'],
                                 box=actor['bbox_3d'], cameras=projections,
                                 geometric_camera_support=any(x['supported'] for x in projections.values())))
    report = dict(schema_version=1, limitation='projection only; occlusion/body masking not checked',
                  source=str(info_root), instances=len(rows),
                  zero_point_instances=sum(x['num_lidar_pts'] == 0 for x in rows),
                  zero_point_with_camera_projection=sum(x['num_lidar_pts'] == 0 and x['geometric_camera_support'] for x in rows),
                  no_camera_projection=sum(not x['geometric_camera_support'] for x in rows), rows=rows)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k != 'rows'}, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--info-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    audit(args.info_root, args.output)
