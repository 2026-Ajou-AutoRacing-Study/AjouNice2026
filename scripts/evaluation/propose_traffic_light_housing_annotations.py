#!/usr/bin/env python3
"""Research-only full-frame vehicle-signal housing proposals, never auto GT.

Inputs are exact JPEG references and measured development Ego poses. MGeo
projection is approximate, ignores occlusion and body pitch, and only proposes
front-facing vehicle heads. Every frame needs explicit whole-image visual QC;
unreviewed geometry must not be passed to a detector trainer as complete labels.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

from project_traffic_light_housing import parse_ego_document, project_housing


def propose(signals, ego, *, minimum_width=6., minimum_height=3., maximum_view_angle=65.):
    position, heading, _ = parse_ego_document(ego)
    boxes = []
    for signal in signals:
        if signal.get('type') != 'car' or min(signal.get('width', 0), signal.get('height', 0)) <= 0:
            continue
        dx, dy = position[0]-signal['point'][0], position[1]-signal['point'][1]
        distance = math.hypot(dx, dy)
        theta = math.radians(signal['heading'])
        cosine = (math.cos(theta)*dx+math.sin(theta)*dy)/distance if distance else -1
        if cosine < math.cos(math.radians(maximum_view_angle)):
            continue
        projection = project_housing(signal, position, heading, margin_px=0.)
        box = projection['bbox_xyxy_integer_clipped']
        if not box or box[2]-box[0] < minimum_width or box[3]-box[1] < minimum_height:
            continue
        boxes.append({'xyxy': box, 'class_id': 0, 'native_id': signal['idx'],
                      'head_role': 'unknown', 'box_qc_accepted': False,
                      'view_angle_degrees': math.degrees(math.acos(max(-1., min(1., cosine)))),
                      'proposal_only': True, 'distance_xy_m': distance})
    return boxes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', required=True, type=Path)
    parser.add_argument('--mgeo', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Use a fresh output')
    document = json.loads(args.dataset.read_text())
    signals = json.loads(args.mgeo.read_text())
    records = []
    seen = set()
    for row in document['records']:
        if row['image_sha256'] in seen:
            continue
        seen.add(row['image_sha256'])
        ego = row.get('ego') or row.get('provenance', {}).get('ego')
        if ego is None:
            raise ValueError('Measured Ego pose is required')
        image = args.dataset.resolve().parent / row['image_path']
        if hashlib.sha256(image.read_bytes()).hexdigest() != row['image_sha256']:
            raise ValueError('JPEG hash mismatch')
        records.append({**{key: row[key] for key in ('image_path', 'image_sha256', 'split', 'group_id', 'environment')},
                        'width': 1280, 'height': 720, 'ego': ego,
                        'annotation_complete': False, 'boxes': propose(signals, ego)})
    output = {'scope': 'unapproved_geometric_proposals_not_training_ground_truth',
              'mgeo_sha256': hashlib.sha256(args.mgeo.read_bytes()).hexdigest(),
              'label_definition_proposal': 'front-facing vehicle-signal housing, excludes pedestrian/bus/back faces; all visible instances need visual review including missed/occluded projections',
              'records': records, 'summary': dict(Counter(row['group_id'] for row in records))}
    with args.output.open('x') as stream:
        json.dump(output, stream, ensure_ascii=False, indent=2)
    print(json.dumps({'records': len(records), 'boxes': sum(len(x['boxes']) for x in records)}))


if __name__ == '__main__':
    main()
