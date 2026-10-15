#!/usr/bin/env python3
"""Paired E24 grouping diagnostics; no visibility/AP or live-latency claim.

Correspondence uses separately catalog-resolved actor geometry at both the
row's host reference and the shared latest native footer. Footer is a source
batch identifier here, NOT proof of exact ray time or synchronized LAN clocks.
"""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'base_ws/src/perception/bevfusion_ros/src'),
               str(ROOT/'base_ws/src/tracking_prediction_evaluation/src'),
               str(ROOT/'docker/bevfusion')]
from bevfusion_ros.rotated_bev_nms import class_aware_rotated_bev_nms
from audit_tracking_continuity import match, sample_series, stats
from lidar_packet_reassembly import read_source

CLASSES = ('vehicle', 'pedestrian', 'obstacle')
THRESHOLDS = dict(vehicle=.4, pedestrian=.1, obstacle=.1)
POLICIES = ('fixed51', 'fixed50', 'cut0', 'cut180', 'cut0_footer')


def operating(predictions):
    """Production server floor/top100, rotated NMS .50, class floor."""
    eligible = sorted((p for p in predictions if p['score'] >= .1),
                      key=lambda p: -p['score'])[:100]
    kept = class_aware_rotated_bev_nms(eligible, .5)
    return [p for p in kept if p['score'] >= THRESHOLDS[p['class_name']]]


def yaw_axis_delta(a, b):
    return abs((a-b+math.pi/2) % math.pi-math.pi/2)


def to_world(predictions, ego, translation):
    """Explicit declared-header pose, not a guessed acquisition correction."""
    c, s = math.cos(ego['yaw']), math.sin(ego['yaw'])
    result = []
    for point in predictions:
        x, y, z = np.asarray(point['center'])+translation
        result.append({**point, 'center': [c*x-s*y+ego['center'][0],
            s*x+c*y+ego['center'][1], z+ego['center'][2]], 'yaw': point['yaw']+ego['yaw']})
    return result


def paired_metrics(before, after, gate=1.):
    pairs, missing, extra = match(before, after, gate)
    by_class = {}
    for cls in CLASSES:
        selected = [(i, j, d) for i, j, d in pairs if before[i]['class_name'] == cls]
        by_class[cls] = dict(
            before=sum(p['class_name'] == cls for p in before),
            after=sum(p['class_name'] == cls for p in after),
            paired=len(selected),
            baseline_unmatched=sum(before[i]['class_name'] == cls for i in missing),
            candidate_unmatched=sum(after[j]['class_name'] == cls for j in extra),
            center_m=[d for _, _, d in selected],
            score_abs=[abs(before[i]['score']-after[j]['score']) for i, j, _ in selected],
            yaw_axis_deg=[math.degrees(yaw_axis_delta(before[i]['yaw'], after[j]['yaw']))
                          for i, j, _ in selected])
    return by_class


def catalog_series(bag, case):
    from tracking_prediction_evaluation.trajectory import read_morai_trajectories
    from tracking_prediction_evaluation.bbox_reference import transform_segments_to_asset_catalog_bbox_center
    names = {}
    companion = bag.parent/'scenario.json'
    if not companion.exists() and case == 0:
        companion = bag.parents[2]/'static_loaded_scenario.json'
    if companion.exists():
        # Use explicit scene evidence, not the caller's numeric case index.
        # Short moving probes can also be case 0; they are not static scenes.
        sidecar = ROOT/'configs/scenario/perception_benchmark/bench_static_obstacle_base3_assets_raw.audit.json'
        scene = json.loads(companion.read_text())
        by_data = {int(o['data_id']): o for o in json.loads(sidecar.read_text())['objects']}
        for obj in scene['objectList']:
            item = by_data.get(int(obj['DataID']))
            if item:
                names['obstacle:'+str(int(obj['UNIQUEID']))] = item['name']
    raw = read_morai_trajectories(bag, include_classes=CLASSES, minimum_points=2,
                                heading_correction_deg=-1.347, asset_name_map=names)
    resolved, resolution = transform_segments_to_asset_catalog_bbox_center(
        raw, ROOT/'dataset/morai_od/configs/morai_asset_bbox_offsets_status_size_v2.json')
    def convert(segment):
        return ([p.timestamp for p in segment.points],
                [dict(key=segment.key, asset=segment.asset_name, class_name=segment.class_name,
                      center=[p.x, p.y, p.z], size=[p.length, p.width, p.height], yaw=p.yaw)
                 for p in segment.points])
    return [convert(s) for s in resolved], resolution, names


def local_gt(series, ego, stamp, translation):
    current = sample_series(ego, stamp, .15)
    if current is None:
        return None
    c, s = math.cos(current['yaw']), math.sin(current['yaw'])
    result = []
    for item in series:
        point = sample_series(item, stamp, .15)
        if point is None:
            continue
        dx, dy = np.asarray(point['center'][:2])-current['center'][:2]
        center = [c*dx+s*dy-translation[0], -s*dx+c*dy-translation[1],
                  point['center'][2]-current['center'][2]-translation[2]]
        if abs(center[0]) <= 100 and abs(center[1]) <= 30:
            result.append({**point, 'center': center, 'yaw': point['yaw']-current['yaw']})
    return result


def analyze(args):
    from app.sensor4_calibration import load_sensor4_calibration
    if args.output.exists():
        raise ValueError('Fresh output required')
    payload = json.loads(args.predictions.read_text())
    inputs = json.loads(args.inputs.read_text())
    if [r['token'] for r in payload['rows']] != [r['token'] for r in inputs['rows']]:
        raise ValueError('Prediction token order differs from input manifest')
    calibration = load_sensor4_calibration(args.sensor, ['camera_'+n for n in
        ('front', 'front_left', 'front_right', 'back')])
    transform = calibration['base_from_lidar']
    if not np.allclose(transform[:3, :3], np.eye(3)):
        raise ValueError('Audit requires the recorded zero-rotation LiDAR mounting')
    cases = {}
    for case in inputs['cases']:
        index = case['case']; bag = Path(case['source'])
        source = read_source(bag)
        series, resolution, names = catalog_series(bag, index)
        ego = sorted({stamp: dict(center=[m.position.x, m.position.y, m.position.z],
                                 yaw=math.radians(m.heading-1.347))
                      for stamp, _, m in source['messages']['/Ego_topic']}.items())
        ego = ([r[0] for r in ego], [r[1] for r in ego])
        rows = [r for r in payload['rows'] if r['case'] == index]
        groups = defaultdict(dict)
        correspondence, comparison, world_comparison, timing = {}, {}, {}, {}
        for row in rows:
            groups[row['pair']][row['policy']] = row
        excluded_pairs = []
        for pair, group in list(groups.items()):
            missing = [policy for policy, row in group.items()
                       if sample_series(ego, row['stamp'], .15) is None]
            if missing:
                excluded_pairs.append(dict(pair=pair, missing_declared_ego=missing))
                del groups[pair]
        rows = [r for r in rows if r['pair'] in groups]
        if not rows:
            raise ValueError('No paired rows with every declared-reference Ego interpolation')
        for row in rows:
            row['operating'] = operating(row['predictions'])
            pose = sample_series(ego, row['stamp'], .15)
            row['world_operating'] = to_world(row['operating'], pose, transform[:3, 3])
        for policy in POLICIES:
            selected = [r for r in rows if r['policy'] == policy]
            if len(selected) != len(groups):
                raise ValueError('Missing paired policy')
            timing[policy] = dict(
                point_count=stats([r['point_count'] for r in selected]),
                footer_distinct=dict(Counter(r['footer_distinct'] for r in selected)),
                header_to_latest_footer_ms=stats([(r['native_latest_stamp']-r['stamp'])*1000 for r in selected]),
                camera_minus_header_ms=stats([v*1000 for r in selected for v in r['camera_offsets_sec'].values()]),
                camera_minus_latest_footer_ms=stats([(r['stamp']+v-r['native_latest_stamp'])*1000
                    for r in selected for v in r['camera_offsets_sec'].values()]))
            for field, target in [('operating', comparison), ('world_operating', world_comparison)]:
                accum = {cls: dict(before=0, after=0, paired=0, baseline_unmatched=0,
                                   candidate_unmatched=0, center_m=[], score_abs=[], yaw_axis_deg=[])
                         for cls in CLASSES}
                for row in selected:
                    delta = paired_metrics(groups[row['pair']]['fixed51'][field], row[field])
                    for cls, values in delta.items():
                        for key, value in values.items():
                            if isinstance(value, list):
                                accum[cls][key].extend(value)
                            else:
                                accum[cls][key] += value
                target[policy] = {cls: {k: stats(v) if isinstance(v, list) else v for k, v in metrics.items()}
                                 for cls, metrics in accum.items()}
            correspondence[policy] = {}
            for reference in ('host_reference', 'latest_footer_reference'):
                counts = {cls: dict(gt=0, matched=0, frame_unmatched_outputs=0,
                                   center_m=[], per_asset=Counter()) for cls in CLASSES}
                excluded = 0
                for row in selected:
                    stamp = row['stamp'] if reference == 'host_reference' else row['native_latest_stamp']
                    gt = local_gt(series, ego, stamp, transform[:3, 3])
                    if gt is None:
                        excluded += 1; continue
                    pairs, missing, extra = match(gt, row['operating'], 2.)
                    for obj in gt:
                        counts[obj['class_name']]['gt'] += 1
                        counts[obj['class_name']]['per_asset'][obj['asset']] += 1
                    for i, _, distance in pairs:
                        counts[gt[i]['class_name']]['matched'] += 1
                        counts[gt[i]['class_name']]['center_m'].append(distance)
                    for j in extra:
                        counts[row['operating'][j]['class_name']]['frame_unmatched_outputs'] += 1
                correspondence[policy][reference] = dict(
                    excluded_ego=excluded,
                    classes={cls: {**r, 'center_m': stats(r['center_m'])} for cls, r in counts.items()})
        cases[str(index)] = dict(pair_count=len(groups), source=str(bag), catalog_resolution=resolution,
                                 ego_common_pair_exclusions=excluded_pairs,
                                 explicit_names=names, timing=timing,
                                 paired_outputs=comparison, world_paired_outputs=world_comparison,
                                 catalog_correspondence=correspondence)
    result = dict(cases=cases, frozen_camera_and_reference=inputs.get('frozen_camera_and_reference', False),
        control_note=inputs.get('control_note'), operating_contract=dict(server_floor=.1, top_k=100,
        nms_iou=.5, thresholds=THRESHOLDS), checkpoint_sha256=payload['checkpoint_sha256'],
        input_sha256=hashlib.sha256(args.inputs.read_bytes()).hexdigest(),
        limitations=['Paired sampled frames, NOT continuous tracker or real-time queue replay.',
            'Resolved catalog actor correspondence is NOT visibility-qualified native GT/AP.',
            'Unmatched outputs are NOT false positives: unknown catalog geometry and background actors exist.',
            'Host/latest-footer GT sensitivity is not an oracle of true ray acquisition or LAN clock synchronization.',
            'Raw local output differences include different declared reference times. World comparison uses each declared header pose, not an acquisition oracle.',
            'Ego body attitude is planar status interpolation, not full native RPY GT.'])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    for index, case in cases.items():
        print(index, {p: {cls: v['after'] for cls, v in c.items()} for p, c in case['paired_outputs'].items()})


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--predictions', type=Path, required=True)
    parser.add_argument('--inputs', type=Path, required=True)
    parser.add_argument('--sensor', type=Path, default=ROOT/'configs/sensors/Sensor_competition_local.json')
    parser.add_argument('--output', type=Path, required=True)
    analyze(parser.parse_args())
