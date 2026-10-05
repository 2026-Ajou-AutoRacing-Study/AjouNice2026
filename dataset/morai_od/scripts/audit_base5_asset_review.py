#!/usr/bin/env python3
"""Audit target UID, geometric annotations, points and semantic/transport separation."""
import argparse
import json
from pathlib import Path

import numpy as np
from audit_morai_lidar_gt_visibility import parse_instances, points_in_box


def audit(root, manifest):
    scenes = {s['id']: s for s in json.loads(manifest.read_text())['scenes']}
    results = []
    for line in (root / 'captures.jsonl').read_text().splitlines():
        frame = json.loads(line)
        scene = scenes[frame['scene_id']]
        actor = json.loads(Path(scene['metadata']).read_text())['runtime_actors'][0]
        stem = root / 'LIDAR_5' / frame['capture_id']
        boxes = parse_instances(Path(str(stem) + '_Instance.txt'))
        matches = [b for b in boxes if b['unique_id'] == actor['unique_id']]
        if not matches:
            raise ValueError('Target annotation UID missing: {}'.format(frame['scene_id']))
        if len(matches) > 1:
            results.append(dict(scene_id=frame['scene_id'], asset=actor['name'], uid=actor['unique_id'],
                annotation_uid_matched=True, annotation_count=len(matches),
                raw_annotation_classes=[b['raw_class'] for b in matches],
                training_candidate=False, disposition='review_multiple_components_same_uid',
                components=[dict(raw_class=b['raw_class'], center_lidar_m=b['center'].tolist(),
                                 size_m=b['size'].tolist()) for b in matches]))
            continue
        box = matches[0]
        if not np.isfinite(box['size']).all() or not (box['size'] > 0).all():
            raise ValueError('Invalid bbox')
        points = np.fromfile(Path(str(stem) + '_Intensity.bin'), dtype=np.float32).reshape(-1, 4)
        count = int(points_in_box(points, box).sum())
        results.append(dict(scene_id=frame['scene_id'], asset=actor['name'], uid=actor['unique_id'],
            raw_annotation_class=box['raw_class'], transport_class=actor['class'],
            training_class=actor['training_class'], center_lidar_m=box['center'].tolist(),
            size_m=box['size'].tolist(), bottom_lidar_z_m=float(box['center'][2]-box['size'][2]/2),
            points_in_yaw_box=count, ego_speed_raw=float(frame['ego']['speed_raw']),
            saved_files_verified=frame['verified'], annotation_uid_matched=True,
            training_candidate=actor['training_class'] is not None))
    return dict(capture_root=str(root), manifest=str(manifest), frames=results,
                limitations=['Points use yaw-only OBB; no collision/ground certification.',
                             'Review frames never become training positives automatically.',
                             'Stationary capture is not a motion-policy validation.'])


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--capture-root', type=Path, required=True)
    p.add_argument('--manifest', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    result = audit(a.capture_root, a.manifest)
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print('Audited target frames:',len(result['frames']))
