#!/usr/bin/env python3
"""Re-evaluate saved pedestrian tracks without GPU; keep legacy GT alongside."""
import argparse
import json
from pathlib import Path

from tracking_prediction_evaluation.bbox_reference import transform_segments_to_asset_catalog_bbox_center
from tracking_prediction_evaluation.pipeline_evaluator import evaluate_pipeline, read_ego_points, read_output_frames
from tracking_prediction_evaluation.static_transform import load_morai_to_map_transform
from tracking_prediction_evaluation.trajectory import read_morai_trajectories, transform_segments_planar
from tracking_prediction_evaluation.unresolved_frame_policy import resolved_frame_mask


def compare(summary_path, unit, frame):
    saved = json.loads(Path(summary_path).read_text())
    source, contract = saved['source'], saved['evaluation']
    if contract['gt_reference'] != 'asset-catalog-bbox-center-v1':
        raise ValueError('This comparison requires a catalog-based saved summary')
    names_path = source.get('asset_name_map')
    names = json.loads(Path(names_path).read_text())['asset_names'] if names_path else None
    correction = source['morai_heading_correction_deg']
    raw = read_morai_trajectories(source['raw_bag'], include_classes=('pedestrian',),
                                 heading_correction_deg=correction, asset_name_map=names)
    centered, _ = transform_segments_to_asset_catalog_bbox_center(raw, source['asset_bbox_catalog'])
    translation, yaw = load_morai_to_map_transform(Path(source['morai_static_tf']))
    segments = transform_segments_planar(centered, translation, yaw)
    raw_map = transform_segments_planar(raw, translation, yaw)
    ego = read_ego_points(source['raw_bag'], '/Ego_topic', translation, yaw, correction)
    tracking, predictions = read_output_frames(source['output_bag'],
        '/perception/object_recognition/tracking/objects', '/perception/object_recognition/objects', 'pedestrian')
    if contract.get('resolved_frame_policy'):
        mask, _ = resolved_frame_mask(raw_map, segments, ego, set(tracking) | set(predictions),
                                     contract['maximum_range_m'], .15)
        tracking = {t: items for t, items in tracking.items() if mask[t]}
    results = {}
    for policy in ('position', 'message-preferred'):
        result = evaluate_pipeline(segments, ego, tracking, {},
            maximum_range_m=contract['maximum_range_m'],
            maximum_association_distance_m=contract['maximum_association_distance_m'],
            gt_reference=contract['gt_reference'], pedestrian_velocity_reference=policy,
            pedestrian_message_unit=unit, pedestrian_message_frame=frame,
            pedestrian_message_frame_yaw=yaw)
        results[policy] = result['tracking']
    baseline = results['position']
    assert baseline['tp'] == saved['tracking']['tp']
    for metric in ('absolute_speed_error_mps', 'velocity_l2_error_mps'):
        for stat in ('mean', 'p50'):
            assert abs(baseline[metric][stat] - saved['tracking'][metric][stat]) < 1e-8
    return {'source': source, 'message_unit': unit, 'message_frame': frame,
            'baseline_reproduced': True, 'tracking': results,
            'note': 'Offline segment-level availability; no detector/tracker/predictor runtime changes.'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('saved_summary', type=Path)
    parser.add_argument('--message-unit', required=True, choices=('mps', 'kmh'))
    parser.add_argument('--message-frame', required=True, choices=('body', 'morai-map'))
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    report = compare(args.saved_summary, args.message_unit, args.message_frame)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    for policy, values in report['tracking'].items():
        print(policy, 'tp', values['tp'], 'speed p50/MAE',
              values['absolute_speed_error_mps']['p50'], values['absolute_speed_error_mps']['mean'],
              'AVE', values['velocity_l2_error_mps']['mean'], values['velocity_gt_source_counts'])
