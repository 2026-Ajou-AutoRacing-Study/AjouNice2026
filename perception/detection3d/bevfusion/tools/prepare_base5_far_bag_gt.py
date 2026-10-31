#!/usr/bin/env python3
"""Extend GT on frozen sensor tokens without copying sensors or rerunning OD.

Uses audited catalog geometry and bounded Object/Ego interpolation. Unresolved
actors exclude whole scoring frames with a conservative ROI-radius guard.
All original tokens remain in the package for uninterrupted tracker replay.
"""
import argparse
import copy
import gzip
import hashlib
import json
from pathlib import Path
import pickle

import numpy as np

from compare_base5_common30 import gt_identity, validate_payload
from prepare_base5_rosbag_quick import (
    ROOT, CLASSES, local_box, count_support, interpolate_scalar,
    read_morai_trajectories, transform_segments_to_asset_catalog_bbox_center,
    read_ego_points, interpolate_ego, interpolate_point, resolved_frame_mask,
    message_timestamp, nearest, load_sensor4_calibration,
)


def inside_roi(box):
    """Model semantic filter uses BOTTOM Z, not geometric box center Z."""
    bottom = box[2] - box[5] / 2
    return -100 <= box[0] <= 100 and -30 <= box[1] <= 30 and -3 <= bottom <= 2


def build(args):
    import rosbag
    if args.output.exists():
        raise ValueError('Fresh output required; preserve previous GT and predictions')
    source = args.inputs.resolve()
    info = pickle.loads((source / 'infos_val.pkl').read_bytes())
    manifest = json.loads((source / 'manifest.json').read_text())
    calibration = load_sensor4_calibration(args.sensor,
        ['camera_front', 'camera_front_left', 'camera_front_right', 'camera_back'])
    if calibration['config_sha256'] != manifest['sensor_sha256']:
        raise ValueError('Frozen sensor calibration differs')
    if hashlib.sha256(args.catalog.read_bytes()).hexdigest() != manifest['catalog_sha256']:
        raise ValueError('Frozen asset catalog differs')
    extrinsic = calibration['base_from_lidar']
    if not np.allclose(extrinsic[:3, :3], np.eye(3)):
        raise ValueError('Audited zero-rotation lidar mounting required')
    args.output.mkdir(parents=True)
    result = copy.deepcopy(manifest)
    result.update(gt_reference='Bounded interpolated status + audited catalog; not native synchronous annotation',
                  original_inputs=str(source), original_info_sha256=hashlib.sha256((source/'infos_val.pkl').read_bytes()).hexdigest(),
                  scoring_roi=[-100, -30, -3, 100, 30, 2],
                  unresolved_guard_radius_m=105., trajectory_minimum_points=1,
                  limitation='Known-scene development GT. Background assets may be absent from Object_topic. No claim that every unmatched box is a ghost.')
    result['scoring_frames'] = 0
    updated = []
    for case, contract in manifest['cases'].items():
        names = json.loads((args.contract_root / case / 'asset_name_map.json').read_text())['asset_names']
        raw = read_morai_trajectories(contract['raw_bag'], include_classes=CLASSES,
            heading_correction_deg=contract['heading_correction_deg'], asset_name_map=names,
            minimum_points=1)
        segments, resolution = transform_segments_to_asset_catalog_bbox_center(raw, args.catalog)
        ego_points = read_ego_points(contract['raw_bag'], '/Ego_topic', (0,0,0), 0,
                                    contract['heading_correction_deg'])
        rows = [r for r in info['data_list'] if r['scene_id'] == case]
        times = [r['timestamp'] for r in rows]
        mask, policy = resolved_frame_mask(raw, segments, ego_points, times,
                                          maximum_range_m=105.)
        heights = {}; lidar_times = set(); object_times = set()
        with rosbag.Bag(contract['raw_bag']) as bag:
            for topic, message, stamp in bag.read_messages(topics=['/Ego_topic','/Object_topic','/sensor/lidar1']):
                t = message_timestamp(message, stamp)
                if topic == '/Ego_topic': heights[t] = message
                elif topic == '/sensor/lidar1': lidar_times.add(t)
                else: object_times.add(t)
        height_rows = sorted(heights.items())
        object_rows = [(t, None) for t in sorted(object_times)]
        indices = {t:i for i,t in enumerate(sorted(lidar_times))}
        excluded = []; legacy_mismatches = []
        for source_row in rows:
            row = copy.deepcopy(source_row)
            t = row['timestamp']
            ego = interpolate_ego(ego_points, t, .15)
            height = interpolate_scalar(height_rows, t, .15)
            if ego is None or height is None:
                raise ValueError('Frozen eligible sensor token lost Ego interpolation')
            path = source / source_row['lidar_points']['lidar_path']
            points = np.fromfile(path, dtype='<f4').reshape(-1,4)
            points = points[np.isfinite(points).all(axis=1) & (points[:,:3]!=0).any(axis=1)]
            instances = []
            for segment in segments:
                point = interpolate_point(segment.points, t, .15)
                if point is None: continue
                box = local_box(point, ego, height, extrinsic[:3,3])
                # Keep all known nearby actors to diagnose boundary effects.
                if np.linalg.norm(box[:2]) > 115: continue
                instances.append(dict(bbox_3d=box, bbox_label_3d=CLASSES.index(segment.class_name),
                    num_lidar_pts=count_support(points,box), bbox_3d_isvalid=True,
                    instance_id=len(instances), asset=segment.asset_name, track_key=segment.key))
            by_key = {i['track_key']:i for i in instances}
            for old in source_row['instances']:
                new = by_key.get(old['track_key'])
                if new is None or not np.allclose(old['bbox_3d'],new['bbox_3d'],atol=1e-5,rtol=0):
                    legacy_mismatches.append(row['token'])
            row['instances'] = instances
            row['raw_lidar_index'] = indices[t]
            row['scoring_eligible'] = bool(mask[t] and nearest(object_rows,t,.075) is not None)
            if not row['scoring_eligible']: excluded.append(row['token'])
            for image in row['images'].values():
                image['img_path'] = str(source / image['img_path'])
            row['lidar_points']['lidar_path'] = str(path)
            updated.append(row)
        if legacy_mismatches:
            raise ValueError('Old nearby GT geometry changed: '+str(legacy_mismatches[:5]))
        case_rows = updated[-len(rows):]
        result['cases'][case].update(catalog_resolution=resolution,far_frame_resolution=policy,
            scoring_frames=sum(r['scoring_eligible'] for r in case_rows),
            excluded_scoring_tokens=excluded,old_gt_geometry_unchanged=True)
        result['scoring_frames'] += sum(r['scoring_eligible'] for r in case_rows)
        print(case,'scoring',sum(r['scoring_eligible'] for r in case_rows),'/',len(rows),flush=True)
    info['data_list'] = updated
    blob = pickle.dumps(info, protocol=4)
    (args.output/'infos_val.pkl').write_bytes(blob)
    (args.output/'manifest.json').write_text(json.dumps(result,indent=2)+'\n')
    sources = {r['token']:r for r in updated}
    for n, path in enumerate(args.predictions):
        with gzip.open(path,'rt') as f: payload = json.load(f)
        validate_payload(payload)
        if payload['frames'] != len(updated) or set(r['token'] for r in payload['rows']) != set(sources):
            raise ValueError('Frozen prediction tokens differ')
        original_gt = payload['gt_identity']
        for row in payload['rows']:
            instances = sources[row['token']]['instances']
            boxes = np.asarray([i['bbox_3d'] for i in instances],float).reshape(-1,7)
            boxes[:,2] -= boxes[:,5]/2
            row.update(gt_boxes=boxes.tolist(),gt_labels=[i['bbox_label_3d'] for i in instances],
                       gt_points=[i['num_lidar_pts'] for i in instances],gt_assets=[i['asset'] for i in instances])
        payload.update(gt_identity=gt_identity(payload['rows']),info_sha256=hashlib.sha256(blob).hexdigest(),
            gt_extension=dict(original_prediction_file=str(path.resolve()),
                original_prediction_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                original_gt_identity=original_gt,detector_outputs_unchanged=True))
        validate_payload(payload)
        with gzip.open(args.output/f'predictions_{n}.json.gz','wt') as f:
            json.dump(payload,f,allow_nan=False)
    print('TOTAL scoring',result['scoring_frames'],'/',len(updated),flush=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--inputs',type=Path,required=True)
    p.add_argument('--predictions',type=Path,nargs='+',required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--contract-root',type=Path,default=ROOT/'artifacts/evaluation/bevfusion/teammate_manual_261004/e25')
    p.add_argument('--sensor',type=Path,default=ROOT/'configs/sensors/legacy/Sensor_4.json')
    p.add_argument('--catalog',type=Path,default=ROOT/'dataset/morai_od/configs/morai_asset_bbox_offsets_status_size_v2.json')
    build(p.parse_args())
