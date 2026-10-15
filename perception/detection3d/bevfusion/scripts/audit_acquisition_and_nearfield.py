#!/usr/bin/env python3
"""Audit saved boxes: acquisition/classification timing and near-field misses.

Uses corrected geometric GT and one-to-one spatial matching. Header-time delays
are NOT sensor-to-planning wall-clock latency. No model or simulator is run.
"""
import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path

from tracking_prediction_evaluation.bbox_reference import transform_segments_to_asset_catalog_bbox_center
from tracking_prediction_evaluation.detection_evaluator import read_detection_frames, build_evaluation_frames
from tracking_prediction_evaluation.pipeline_evaluator import (
    read_ego_points, read_output_frames, active_gt, interpolate_ego, match_estimates)
from tracking_prediction_evaluation.static_transform import load_morai_to_map_transform
from tracking_prediction_evaluation.trajectory import read_morai_trajectories, transform_segments_planar
from tracking_prediction_evaluation.unresolved_frame_policy import resolved_frame_mask

ROOT = Path(__file__).resolve().parents[4]


def process(case, pipeline_override=None):
    contract = json.loads((case / 'case_contract.json').read_text())
    cls = {'highway': 'vehicle', 'roundabout': 'vehicle',
           'pedestrian': 'pedestrian', 'obstacle': 'obstacle'}[contract['mission']]
    names = json.loads((case / 'asset_name_map.json').read_text())['asset_names']
    raw = read_morai_trajectories(Path(contract['raw_bag']), include_classes=(cls,),
                                heading_correction_deg=-1.347, asset_name_map=names)
    segments, calibration = transform_segments_to_asset_catalog_bbox_center(raw,
        ROOT / 'dataset/morai_od/configs/morai_asset_bbox_offsets_status_size_v2.json')
    translation, yaw = load_morai_to_map_transform(ROOT / 'configs/tf/morai_lanelet_static_tf.yaml')
    raw = transform_segments_planar(raw, translation, yaw)
    segments = transform_segments_planar(segments, translation, yaw)
    ego = read_ego_points(Path(contract['raw_bag']), '/Ego_topic', translation, yaw, -1.347)
    frames, _ = build_evaluation_frames(segments, ego, read_detection_frames(
        case / 'detections.bag', '/perception/object_recognition/detection/objects'))
    mask, policy = resolved_frame_mask(raw, segments, ego, [f.timestamp for f in frames])
    events = defaultdict(dict)
    bins = defaultdict(Counter)
    near_assets = defaultdict(Counter)
    rows = []
    def event(key, name, t, distance, **extra):
        events[key].setdefault(name, dict(timestamp=t, range_m=distance, **extra))
    for frame in frames:
        if not mask[frame.timestamp]:
            continue
        matches, _, _ = match_estimates(frame.gt, frame.predictions, 2.0)
        # Preserve one-to-one matching; do not count one box for two GT actors.
        by_gt = {match[0]: frame.predictions[match[1]] for match in matches}
        for index, (segment, point) in enumerate(frame.gt):
            distance = math.hypot(point.x - frame.ego.x, point.y - frame.ego.y)
            event(segment.key, 'first_gt_in_30m', frame.timestamp, distance)
            prediction = by_gt.get(index)
            if prediction is not None:
                event(segment.key, 'first_detection_any_class', frame.timestamp, distance,
                      class_name=prediction.class_name)
                if prediction.class_name == cls:
                    event(segment.key, 'first_detection_correct_class', frame.timestamp, distance)
            typed = [p for p in frame.predictions if p.class_name == cls]
            nearest = min((math.hypot(p.x-point.x, p.y-point.y) for p in typed), default=math.inf)
            if prediction is not None and prediction.class_name == cls:
                error = math.hypot(prediction.x-point.x, prediction.y-point.y)
                status = 'matched_1m' if error <= 1 else 'matched_1to2m'
            elif prediction is not None:
                status = 'matched_wrong_class_2m'
            elif nearest <= 5:
                status = 'unmatched_but_same_class_candidate_within_5m'
            else:
                status = 'no_same_class_candidate_within_5m'
            bucket = '0-5' if distance < 5 else '5-10' if distance < 10 else '10-20' if distance < 20 else '20-30'
            bins[bucket][status] += 1
            if distance < 10:
                near_assets[segment.asset_name][status] += 1
                dx, dy = point.x-frame.ego.x, point.y-frame.ego.y
                angle = math.degrees(math.atan2(-math.sin(frame.ego.yaw)*dx+math.cos(frame.ego.yaw)*dy,
                                               math.cos(frame.ego.yaw)*dx+math.sin(frame.ego.yaw)*dy))
                rows.append(dict(timestamp=frame.timestamp, key=segment.key, asset=segment.asset_name,
                                 range_m=distance, bearing_deg=angle, status=status))
    bag = pipeline_override or case / 'pipeline/pipeline_outputs.bag'
    track_by_class = {}
    for label in (cls, 'obstacle') if cls == 'vehicle' else (cls,):
        track_by_class[label], _ = read_output_frames(bag,
            '/perception/object_recognition/tracking/objects', '/perception/object_recognition/objects',
            class_name=label)
    timestamps = sorted(set(t for frames in track_by_class.values() for t in frames))
    mask, _ = resolved_frame_mask(raw, segments, ego, timestamps)
    streak = {}
    for timestamp in timestamps:
        if not mask[timestamp]:
            streak.clear()
            continue
        ep = interpolate_ego(ego, timestamp, 0.15)
        gt = active_gt(segments, timestamp, 0.15, ep, 30)
        labeled = [(obj, label) for label, frames in track_by_class.items() for obj in frames.get(timestamp, ())]
        matches, _, _ = match_estimates(gt, [o for o, _ in labeled], 2)
        correct_keys = set()
        for index, output_index, _ in matches:
            segment, point = gt[index]
            obj, label = labeled[output_index]
            distance = math.hypot(point.x-ep.x, point.y-ep.y)
            event(segment.key, 'first_track_any_class', timestamp, distance, class_name=label)
            if label == cls:
                correct_keys.add(segment.key)
                event(segment.key, 'first_track_correct_class', timestamp, distance)
                previous = streak.get(segment.key)
                count = previous[2]+1 if previous and previous[0] == obj.object_id and timestamp-previous[1] <= 0.15 else 1
                streak[segment.key] = obj.object_id, timestamp, count
                if count >= 3:
                    event(segment.key, 'first_three_same_id_correct_track', timestamp, distance)
            else:
                streak.pop(segment.key, None)
        for key in set(streak) - correct_keys:
            streak.pop(key, None)
    for key, event_map in events.items():
        start = event_map.get('first_detection_correct_class')
        end = event_map.get('first_track_correct_class')
        if start and end:
            event_map['correct_detection_to_correct_track_header_sec'] = end['timestamp']-start['timestamp']
    return dict(case=case.name, mission=contract['mission'],
        contract='2m class-agnostic one-to-one presence; correct-class subsets; headers not receipt latency',
        pipeline_bag=str(bag), frame_policy=policy, calibration=calibration,
        distance_status_counts={b: dict(v) for b, v in bins.items()},
        near_asset_counts={a: dict(v) for a, v in near_assets.items()},
        acquisition_events=dict(events), near_observations=rows)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, default=ROOT / 'artifacts/evaluation/bevfusion/teammate_manual_261004/e25')
    p.add_argument('--case', action='append', help='case names; default all')
    p.add_argument('--pipeline-bag', type=Path, help='override, requires exactly one case')
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if args.pipeline_bag and (not args.case or len(args.case) != 1):
        p.error('pipeline override requires one case')
    cases = [c.parent for c in sorted(args.root.glob('*/case_contract.json')) if not args.case or c.parent.name in args.case]
    result = []
    for case in cases:
        result.append(process(case, args.pipeline_bag))
        print(case.name, result[-1]['distance_status_counts'], flush=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + '\n')


if __name__ == '__main__':
    main()
