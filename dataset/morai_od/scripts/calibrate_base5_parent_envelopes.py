#!/usr/bin/env python3
"""Recover missing static parent envelopes from accepted raw, including components."""
import argparse
import json
import math
from collections import Counter, defaultdict
from itertools import product
from pathlib import Path

import numpy as np

from base5_capture_audit import parse_instances, rotation_matrix
from base5_capture_safety import atomic_json, sha256
from base5_placement_geometry import POLICY


def parent_envelope(boxes, transform, ego, pivot, actor_yaw):
    yaw = math.radians(ego['heading'] - 1.347)
    ego_rotation = rotation_matrix((0., 0., yaw))
    actor_rotation = rotation_matrix((0., 0., math.radians(actor_yaw)))
    vertices = []
    for box in boxes:
        corners = np.asarray(list(product((-1., 1.), repeat=3)))*np.asarray(box['size'])/2
        points = corners @ rotation_matrix(box['rotation_rpy']).T + box['center']
        points = points @ transform[:3, :3].T + transform[:3, 3]
        world = points @ ego_rotation.T + np.asarray([ego[k] for k in ('x', 'y', 'z')])
        vertices.extend((world-np.asarray(pivot)) @ actor_rotation)
    vertices = np.asarray(vertices)
    if not np.isfinite(vertices).all() or len(vertices) < 8:
        raise ValueError('Invalid parent component geometry')
    return vertices.min(0), vertices.max(0)


def calibrate(ledgers, missing_names):
    groups = defaultdict(list); skipped = Counter(); seen = set()
    for ledger in ledgers:
        for line in Path(ledger).read_text().splitlines():
            entry = json.loads(line)
            if entry.get('status') != 'accepted_candidate':
                continue
            root = Path(entry['capture_root']).resolve()
            if root in seen:
                continue
            seen.add(root)
            if not all((root/name).is_file() for name in ('metadata.json','audit.json','telemetry.json','scenario.json')):
                skipped['archived_or_incomplete_capture'] += 1; continue
            meta = json.loads((root/'metadata.json').read_text())
            eligible = [a for a in meta['runtime_actors'] if a['name'] in missing_names
                        and a['motion'] in ('static', 'standing')]
            if not eligible or meta.get('calibration_only'):
                continue
            audit = json.loads((root/'audit.json').read_text())
            tele = json.loads((root/'telemetry.json').read_text())
            ego = tele['ego_before']
            delta = (ego['heading']-1.347-meta['ego']['yaw']+180)%360-180
            if (audit['review_reasons'] or abs(delta) > .1 or tele['capture_drift_m'] > .005
                    or not audit.get('capture_safety', {}).get('collision', {}).get('no_collision_verified')):
                skipped['unsafe_or_unstable_capture'] += 1
                continue
            files = list(root.glob('LIDAR_*/*_Instance.txt'))
            if len(files) != 1:
                raise ValueError('Missing native annotation')
            boxes = parse_instances(files[0])
            scene = json.loads((root/'scenario.json').read_text())
            objects = {r['unique_id']:r for k in ('npc_list', 'pedestrian_list', 'obstacle_list')
                       for r in tele['objects_before'][k]}
            serialized = {r['UNIQUEID']:r for k in ('objectList', 'vehicleList', 'pedestrianList', 'aiPedstrianList')
                          for r in scene.get(k, [])}
            for actor in eligible:
                uid = actor['unique_id']; components = [b for b in boxes if b['unique_id'] == uid]
                obj = objects.get(uid)
                if not components or obj is None:
                    skipped['missing_parent'] += 1; continue
                pose = serialized[uid].get('initPosition', serialized[uid])
                ay = float(pose['rot']['yaw'])
                delta_actor = (obj['heading']-1.347-ay+180)%360-180
                pivot = [obj['position'][k] for k in ('x', 'y', 'z')]
                if abs(delta_actor) > .15 or math.dist(pivot[:2], [pose['pos'][k] for k in ('x', 'y')]) > .01:
                    skipped['actor_pose_mismatch'] += 1; continue
                lo, hi = parent_envelope(components, np.asarray(meta['lidar_to_ego']), ego, pivot, ay)
                if np.any(hi-lo <= 0) or np.any(hi-lo > 25) or np.max(np.abs((hi+lo)/2)) > 15:
                    skipped['invalid_envelope'] += 1; continue
                groups[actor['name']].append(dict(lower_m=lo.tolist(), upper_m=hi.tolist(),
                    source=str(root), annotation_sha256=sha256(files[0]), data_id=actor['data_id'],
                    component_count=len(components), asset_yaw_deg=ay))
    assets = {}
    for name, observations in groups.items():
        if len(observations) < 2:
            skipped['single_observation_asset'] += 1; continue
        lo = np.min([r['lower_m'] for r in observations], axis=0)
        hi = np.max([r['upper_m'] for r in observations], axis=0)
        lo[:2] -= .03; hi[:2] += .03
        assets[name] = dict(bbox_size_m=(hi-lo).tolist(),
            pivot_to_bbox_center_local_m=((hi+lo)/2).tolist(), observations=len(observations),
            stationary_only=True, data_id=observations[0]['data_id'], evidence=observations,
            limitations='Static parent placement envelope, union of native components/poses; not a collider or dynamic geometry. Training component GT unchanged.')
    return dict(schema_version=1, policy=POLICY, training_gt_modified=False, assets=assets,
                skipped=dict(skipped), unresolved=sorted(set(missing_names)-set(assets)),
                contract='MR Ego heading -1.347 once; actor scenario yaw; scale1 accepted stationary raw only')


if __name__ == '__main__':
    import generate_base5_independent as gen
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--ledgers', type=Path, nargs='*', default=[])
    p.add_argument('--scan-roots', type=Path, nargs='*', default=[])
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if args.output.exists():
        raise FileExistsError('Fresh placement reference required')
    ctx = gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR, gen.geo.DEFAULT_MGEO)
    missing = {a['name'] for pool in ctx['pools'].values() for a in pool if not gen.geometry(a, ctx)[2]}
    ledgers=sorted(set(args.ledgers+[p for root in args.scan_roots for p in root.rglob('selected_ledger.jsonl')]))
    if not ledgers:p.error('At least one ledger required')
    result = calibrate(ledgers, missing)
    atomic_json(args.output, result)
    print(json.dumps(dict(assets=len(result['assets']), unresolved=result['unresolved'], skipped=result['skipped']), ensure_ascii=False))
