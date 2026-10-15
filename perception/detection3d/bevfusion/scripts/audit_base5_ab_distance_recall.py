#!/usr/bin/env python3
"""Attribute globally matched GT to fine distance bins; do not rematch per bin."""
import argparse
import json
import math
from collections import defaultdict
from pathlib import Path

from tracking_prediction_evaluation.bbox_reference import transform_segments_to_asset_catalog_bbox_center
from tracking_prediction_evaluation.detection_evaluator import build_evaluation_frames, read_detection_frames
from tracking_prediction_evaluation.pipeline_evaluator import read_ego_points
from tracking_prediction_evaluation.static_transform import load_morai_to_map_transform
from tracking_prediction_evaluation.trajectory import read_morai_trajectories, transform_segments_planar
from tracking_prediction_evaluation.unresolved_frame_policy import resolved_frame_mask

ROOT = Path(__file__).resolve().parents[4]
BINS = [(0, 2), (2, 5), (5, 10), (10, 20), (20, 30.000001)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=ROOT / 'artifacts/evaluation/bevfusion/base5_nearfield_replay_e2_261005')
    args = parser.parse_args()
    manifest = json.loads((args.root / 'runtime_manifest.json').read_text())
    catalog = ROOT / 'dataset/morai_od/configs/morai_asset_bbox_offsets_status_size_v2.json'
    translation, yaw = load_morai_to_map_transform(ROOT / 'configs/tf/morai_lanelet_static_tf.yaml')
    rows = []
    for case in manifest['cases']:
        if case['mission'] not in ('pedestrian', 'obstacle'):
            continue
        cls = case['mission']
        raw_bag = Path(case['raw_bag'])
        names = json.loads((Path(case['source']) / 'asset_name_map.json').read_text())['asset_names']
        raw = read_morai_trajectories(raw_bag, include_classes=(cls,),
            heading_correction_deg=case['heading_correction_deg'], asset_name_map=names)
        calibrated, _ = transform_segments_to_asset_catalog_bbox_center(raw, catalog)
        raw_map = transform_segments_planar(raw, translation, yaw)
        segments = transform_segments_planar(calibrated, translation, yaw)
        ego = read_ego_points(raw_bag, '/Ego_topic', translation, yaw, case['heading_correction_deg'])
        for model in ['e25'] + list(manifest['checkpoints']):
            directory = Path(case['source']) if model == 'e25' else args.root / model / case['label']
            if not (directory / 'summary_status_size_v2.json').exists():
                continue
            frames, _ = build_evaluation_frames(segments, ego, read_detection_frames(directory / 'detections.bag',
                '/perception/object_recognition/detection/objects'))
            mask, _ = resolved_frame_mask(raw_map, segments, ego, [f.timestamp for f in frames])
            counts = {f'{lo:g}-{min(hi,30):g}': dict(gt=0, tp=0) for lo, hi in BINS}
            assets = defaultdict(lambda: defaultdict(lambda: dict(gt=0, tp=0)))
            for frame in frames:
                if not mask[frame.timestamp]:
                    continue
                gt = [s for segment, s in frame.gt if segment.class_name == cls]
                taken = set()
                threshold = .15 if cls == 'pedestrian' else .10
                predictions = sorted((p for p in frame.predictions if p.class_name == cls and p.score >= threshold),
                    key=lambda p: p.score, reverse=True)
                for prediction in predictions:
                    candidates = [(math.hypot(prediction.x-s.x, prediction.y-s.y), i)
                        for i, s in enumerate(gt) if i not in taken]
                    if candidates:
                        distance, index = min(candidates)
                        if distance < 1.0:
                            taken.add(index)
                for i, s in enumerate(gt):
                    distance = math.hypot(s.x-frame.ego.x, s.y-frame.ego.y)
                    for lo, hi in BINS:
                        if lo <= distance < hi:
                            count = counts[f'{lo:g}-{min(hi,30):g}']
                            count['gt'] += 1
                            count['tp'] += int(i in taken)
                            asset_count = assets[s.asset_name][f'{lo:g}-{min(hi,30):g}']
                            asset_count['gt'] += 1
                            asset_count['tp'] += int(i in taken)
                            break
            rows.append(dict(model=model, case=case['label'], cohort=case['label'].split('__')[0],
                mission=cls, bins=counts, assets=dict(assets)))
    groups = defaultdict(lambda: defaultdict(lambda: dict(gt=0, tp=0)))
    asset_groups = defaultdict(lambda: defaultdict(lambda: defaultdict(lambda: dict(gt=0, tp=0))))
    for row in rows:
        key = f'{row["cohort"]}/{row["mission"]}/{row["model"]}'
        for distance, counts in row['bins'].items():
            for metric in ('gt', 'tp'):
                groups[key][distance][metric] += counts[metric]
        for asset, bins in row['assets'].items():
            for distance, counts in bins.items():
                for metric in ('gt', 'tp'):
                    asset_groups[key][asset][distance][metric] += counts[metric]
    for bins in groups.values():
        for count in bins.values():
            count['recall'] = count['tp'] / count['gt'] if count['gt'] else None
            count['fn'] = count['gt'] - count['tp']
    result = dict(method='Within-frame confidence-ordered 1m matching over all <=30m GT, then distance attribution; pooled counts, not mean AP. Production class thresholds.',
        rows=rows, groups=dict(groups), asset_groups=dict(asset_groups))
    (args.root / 'distance_recall.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(dict(groups), indent=2))


if __name__ == '__main__':
    main()
