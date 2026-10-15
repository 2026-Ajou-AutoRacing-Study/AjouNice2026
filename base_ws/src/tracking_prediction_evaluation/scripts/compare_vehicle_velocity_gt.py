#!/usr/bin/env python3
"""Compare saved tracker outputs under explicit vehicle velocity GT contracts.

Uses source paths and mask settings from an existing catalog evaluation summary.
No ROS master, GPU, runtime changes or fresh inference are needed.
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np
from tracking_prediction_evaluation.bbox_reference import transform_segments_to_asset_catalog_bbox_center
from tracking_prediction_evaluation.evaluator import estimate_motion_state
from tracking_prediction_evaluation.pipeline_evaluator import (
    active_gt, interpolate_ego, match_estimates, read_ego_points,
    read_output_frames, statistics,
)
from tracking_prediction_evaluation.static_transform import load_morai_to_map_transform
from tracking_prediction_evaluation.trajectory import (
    interpolate_point, read_morai_trajectories, transform_segments_planar,
)
from tracking_prediction_evaluation.unresolved_frame_policy import resolved_frame_mask
from tracking_prediction_evaluation.velocity_reference import axle_to_bbox_velocity


def compare(summary_path):
    saved = json.loads(Path(summary_path).read_text())
    source, contract = saved['source'], saved['evaluation']
    correction = source['morai_heading_correction_deg']
    names_path = source.get('asset_name_map')
    names = json.loads(Path(names_path).read_text())['asset_names'] if names_path else None
    raw = read_morai_trajectories(source['raw_bag'], include_classes=('vehicle',),
        heading_correction_deg=correction, asset_name_map=names)
    centered, calibration = transform_segments_to_asset_catalog_bbox_center(
        raw, source['asset_bbox_catalog'])
    translation, transform_yaw = load_morai_to_map_transform(Path(source['morai_static_tf']))
    segments = transform_segments_planar(centered, translation, transform_yaw)
    raw_map = transform_segments_planar(raw, translation, transform_yaw)
    raw_lookup = {segment.key: segment for segment in raw_map}
    ego = read_ego_points(source['raw_bag'], '/Ego_topic', translation, transform_yaw, correction)
    tracking, predictions = read_output_frames(source['output_bag'],
        '/perception/object_recognition/tracking/objects', '/perception/object_recognition/objects')
    radius = contract['maximum_range_m']
    gate = contract['maximum_association_distance_m']
    if contract.get('resolved_frame_policy'):
        mask, _ = resolved_frame_mask(raw_map, segments, ego,
            set(tracking) | set(predictions), radius, .15)
        tracking = {t: items for t, items in tracking.items() if mask[t]}
    variants = ('legacy_position_projected', 'position_full_xy',
                'message_axle_projected', 'message_axle_full_xy', 'message_bbox_full_xy')
    groups = {name: {group: [] for group in ('all', 'straight_cruise', 'turning', 'accel_decel')}
              for name in variants}
    diagnostics = []
    reference_consistency = []
    tp = 0
    for timestamp, estimates in sorted(tracking.items()):
        current_ego = interpolate_ego(ego, timestamp, .15)
        gt = active_gt(segments, timestamp, .15, current_ego, radius)
        visible = tuple(item for item in estimates if current_ego is None or
            math.hypot(item.x-current_ego.x, item.y-current_ego.y) <= radius)
        matches, _, _ = match_estimates(gt, visible, gate)
        tp += len(matches)
        for gi, ei, _ in matches:
            segment, point = gt[gi]
            history = [p for p in segment.points if timestamp-.5 <= p.timestamp <= timestamp+1e-9]
            if history and abs(history[-1].timestamp-timestamp) > 1e-9:
                history.append(point)
            state = estimate_motion_state(history)
            if state is None:
                continue
            pivot = interpolate_point(raw_lookup[segment.key].points, timestamp, .15)
            if pivot is None:
                raise RuntimeError('Missing raw pivot for matched corrected point')
            c, s = math.cos(point.yaw), math.sin(point.yaw)
            dx, dy = point.x-pivot.x, point.y-pivot.y
            ox, oy = c*dx+s*dy, -s*dx+c*dy
            axle = axle_to_bbox_velocity(point.longitudinal_velocity,
                point.lateral_velocity, point.yaw, 0., 0., 0.)
            bbox = axle_to_bbox_velocity(point.longitudinal_velocity,
                point.lateral_velocity, point.yaw, state.yaw_rate, ox, oy)
            times = np.array([p.timestamp-timestamp for p in history])
            degree = 2 if len(history) >= 5 else 1
            position = tuple(float(np.polyfit(times, [getattr(p, axis) for p in history], degree)[-2])
                             for axis in ('x', 'y'))
            pivot_history = [interpolate_point(raw_lookup[segment.key].points, h.timestamp, .15)
                             for h in history]
            if all(p is not None for p in pivot_history):
                pv = tuple(float(np.polyfit(times, [getattr(p, axis) for p in pivot_history], degree)[-2])
                           for axis in ('x', 'y'))
                reference_consistency.append({'message_to_pivot':math.dist(axle,pv),
                    'message_to_center':math.dist(axle,position),
                    'corrected_message_to_center':math.dist(bbox,position),
                    'message_lateral':point.lateral_velocity,
                    'position_center_lateral':-s*position[0]+c*position[1],
                    'rotation_lateral':state.yaw_rate*ox,
                    'history_span_sec':timestamp-history[0].timestamp,
                    'yaw_rate_abs_deg':abs(math.degrees(state.yaw_rate))})
            estimate = visible[ei]
            ec, es = math.cos(estimate.yaw), math.sin(estimate.yaw)
            estimated_xy = (ec*estimate.speed-es*estimate.lateral_speed,
                            es*estimate.speed+ec*estimate.lateral_speed)
            refs = {'legacy_position_projected': (state.speed*c, state.speed*s),
                    'position_full_xy': position,
                    'message_axle_projected': (point.longitudinal_velocity*c, point.longitudinal_velocity*s),
                    'message_axle_full_xy': axle, 'message_bbox_full_xy': bbox}
            group = ('turning' if abs(state.yaw_rate)>math.radians(3) else
                     'accel_decel' if abs(state.acceleration)>.5 else 'straight_cruise')
            for name, vector in refs.items():
                speed = c*vector[0]+s*vector[1]
                errors = (abs(estimate.speed-speed), math.dist(estimated_xy, vector))
                groups[name]['all'].append(errors)
                groups[name][group].append(errors)
            diagnostics.append((math.dist(axle, bbox), abs(state.yaw_rate),
                timestamp-history[0].timestamp, math.dist(position, axle)))
    metrics = {name: {group: {'count':len(rows),
        'speed_error_mps':statistics([r[0] for r in rows]),
        'velocity_l2_error_mps':statistics([r[1] for r in rows])}
        for group, rows in by_group.items()} for name, by_group in groups.items()}
    baseline = metrics['legacy_position_projected']['all']
    assert tp == saved['tracking']['tp'], (tp, saved['tracking']['tp'])
    for name, source_name in [('speed_error_mps','absolute_speed_error_mps'),
                              ('velocity_l2_error_mps','velocity_l2_error_mps')]:
        for stat in ('mean','p50'):
            assert abs(baseline[name][stat]-saved['tracking'][source_name][stat]) < 1e-8
    return {'source': source, 'tp':tp, 'baseline_reproduced':True,
        'primary_velocity_references': {
            'bbox_center_position_derivative': 'position_full_xy',
            'message_no_rotation_correction': 'message_axle_full_xy',
            'message_assumed_axle_to_bbox': 'message_bbox_full_xy'},
        'legacy_projection_references': ['legacy_position_projected', 'message_axle_projected'],
        'assumptions':['Audited input message velocity is body-frame m/s; physical reference point is unconfirmed.',
                       'message_axle_full_xy is a historical key: it rotates coordinates only, not an axle-origin assertion.',
                       'Only message_bbox_full_xy hypothesizes rear-axle origin and applies omega cross offset.',
                       'Yaw rate from causal 0.5s heading history, same history validity as legacy.',
                       'Groups are position-derived diagnostics, not annotated driving states.',
                       'All variants share matching, catalog, ROI, mask and velocity sample eligibility.'],
        'metrics':metrics, 'correction_magnitude_mps':statistics([r[0] for r in diagnostics]),
        'message_vs_full_position_mps':statistics([r[3] for r in diagnostics]),
        'short_history_under_0_1_sec':sum(r[2]<.1 for r in diagnostics),
        'catalog_supported_segments':calibration['supported_segment_count'],
        'reference_consistency':{
            group:{key:statistics([row[key] for row in reference_consistency if
                (group=='all' or row['yaw_rate_abs_deg']>3) and row['history_span_sec']>=.4])
                for key in ('message_to_pivot','message_to_center','corrected_message_to_center',
                            'message_lateral','position_center_lateral','rotation_lateral')}
            for group in ('all','turning')}}


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('saved_summary', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args=parser.parse_args()
    report=compare(args.saved_summary)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
    print('Baseline reproduced:',report['baseline_reproduced'],'matched:',report['tp'])
    for name, groups in report['metrics'].items():
        values=groups['all']
        print(name, 'speed p50/MAE',values['speed_error_mps']['p50'],values['speed_error_mps']['mean'],
              'AVE',values['velocity_l2_error_mps']['mean'])
