#!/usr/bin/env python3
"""Read-only distance/asset/parent support audit using the actual label infos.

Pilot ledgers use measured native masks at the current 512x1056 input, not
the historical 320 crop. Diagnostic richness never changes capture acceptance.
"""
import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path
import pickle

import numpy as np
from base5_capture_safety import atomic_json, sha256
from build_base5_bevfusion_infos import in_roi


BOUNDS = (0, 5, 10, 20, 30, 50, 80, 110)


def band(distance):
    return next((f'{a}_{b}' for a, b in zip(BOUNDS, BOUNDS[1:]) if a <= distance < b), 'outside')


def sector(center):
    x, y = center[:2]
    return ('front' if x >= 0 else 'rear') if abs(x) >= abs(y) else ('left' if y >= 0 else 'right')


def aggregate(frames, rows):
    groups = defaultdict(Counter)
    assets = defaultdict(Counter)
    for r in rows:
        key = f"{r['class']}:{band(r['range'])}:{r['kind']}"
        g = groups[key]
        g['gt'] += 1
        g['lidar_' + ('0' if r['points'] == 0 else '1_4' if r['points'] < 5 else '5_19' if r['points'] < 20 else '20_plus')] += 1
        g[r['sector']] += 1
        if r['points'] >= 20:
            g['lidar_rich'] += 1
        a = assets[r['asset']]
        a['gt'] += 1
        a[band(r['range'])] += 1
        a['lidar5'] += int(r['points'] >= 5)
        a['lidar20'] += int(r['points'] >= 20)
        a[r['kind']] += 1
    return dict(frames=frames, valid_gt=len(rows), groups={k:dict(v) for k,v in sorted(groups.items())},
                assets={k:dict(v) for k,v in sorted(assets.items())},
                limitations=['Parent ordinal is preserved; component GT is not an independent actor.',
                             '20 LiDAR points is a diagnostic cue bucket, not an accuracy or acceptance threshold.'])


def infos_report(paths):
    reports = {}
    for path in paths:
        doc = pickle.loads(path.read_bytes())
        frames = doc['data_list']
        names = doc['metainfo']['classes']
        rows = []
        for frame in frames:
            for i in frame['instances']:
                center = i['bbox_3d'][:3]
                rows.append(dict(**{'class':names[i['bbox_label_3d']]},
                    asset=i['asset'], points=i['num_lidar_pts'], range=math.hypot(*center[:2]),
                    kind='primary' if i['parent_uid'] == 8500 else 'secondary', sector=sector(center)))
        reports[str(path.resolve())] = dict(source_sha256=sha256(path), **aggregate(len(frames), rows))
    return reports


def pilot_report(ledger):
    from analyze_base5_crop_candidates import geometry, mask_statistics
    import ast
    from PIL import Image
    profile = dict(id='bottom512w1056', h=512, w=1056, side_start=None)
    rows, outcomes, errors = [], Counter(), []
    capture_rows = []
    for line in ledger.read_text().splitlines():
        row = json.loads(line)
        outcomes[row['status']] += 1
        if not row.get('capture_root'):
            continue
        root = Path(row['capture_root'])
        audit = json.loads(Path(row.get('audit_path', root/'audit.json')).read_text())
        m = json.loads((root/'metadata.json').read_text())
        masks, annotation = {}, {}
        for camera in sorted(root.glob('CAMERA_*')):
            masks[camera.name] = np.asarray(Image.open(next(camera.glob('*_Instance.png'))).convert('RGBA'))
            values = {}
            files=list(camera.glob('*_BBox2D.txt'))
            for s in (files[0].read_text().splitlines() if files else []):
                f = s.split(maxsplit=6)
                if len(f) == 7:
                    values[(int(f[5]), f[0], tuple(map(float,f[1:5])))] = ast.literal_eval(f[6])
            annotation[camera.name] = values
        components = []
        for c in audit['components']:
            cameras = []
            if not c.get('camera_component_ambiguous'):
                for e in c['cameras']:
                    key = (e['camera_instance_id'],e['annotation_class'],tuple(e['native_rect']))
                    colour = annotation[e['camera']].get(key)
                    if colour is None:
                        errors.append([row['id'],c['component_id'],'missing camera colour'])
                        continue
                    mask = (masks[e['camera']] == np.asarray(colour)).all(axis=2).astype(np.uint8)
                    prefix = np.r_[0,np.cumsum(mask.sum(axis=1),dtype=np.int64)]
                    if int(prefix[-1])!=e['native_instance_pixels']:
                        raise ValueError('Native mask and saved audit disagree: '+row['id'])
                    if prefix[-1]:
                        stats = mask_statistics(mask,prefix,e['native_rect'],geometry(e['camera'],profile))
                        cameras.append(dict(camera=e['camera'],**stats))
            visible = any(e['candidate_camera_gate'] for e in cameras)
            kept = (c['lidar_points'] >= 1 or visible) and in_roi(c['center_lidar_m'])
            components.append(dict(component_id=c['component_id'],asset=c['asset'],points=c['lidar_points'],
                kept512=kept,camera_gate512=visible,cameras=cameras,range=c['range_xy_m'],
                center=c['center_lidar_m'],size=c['size_m'],kind='primary' if c['parent_uid']==8500 else 'secondary',
                **{'class':c['training_class']},sector=sector(c['center_lidar_m'])))
            if kept:
                rows.append(dict(asset=c['asset'],points=c['lidar_points'],range=c['range_xy_m'],
                    **{'class':c['training_class']},kind='primary' if c['parent_uid']==8500 else 'secondary',
                    sector=sector(c['center_lidar_m'])))
        capture_rows.append(dict(id=row['id'],status=row['status'],review_reasons=audit['review_reasons'],
            capture_root=str(root),family=m['spec']['family'],mission=m['spec']['mission'],
            ego=m['ego'],placed=len(m['runtime_actors']),components=components,
            paired_source=m.get('obstacle_support_pair'),population=m.get('population_budget')))
    status_path=ledger.parent/'status.json'
    status=json.loads(status_path.read_text()) if status_path.exists() else None
    return dict(ledger_sha256=sha256(ledger),outcomes=dict(outcomes),parent_status=status and {
                    k:status.get(k) for k in ('stage','requested','accepted_candidate','review','elapsed_sec')},
                **aggregate(len(capture_rows),rows),
                captures=capture_rows,errors=errors,profile='512x1056 current bottom crop',
                acceptance_unchanged=True)


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--infos',type=Path,nargs='*',default=[])
    p.add_argument('--ledger',type=Path)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if not a.infos and not a.ledger: p.error('Provide infos or ledger')
    result=pilot_report(a.ledger) if a.ledger else infos_report(a.infos)
    atomic_json(a.output,result)
    print(json.dumps({k:v for k,v in result.items() if k not in ('captures','assets')},ensure_ascii=False)[:6000])
