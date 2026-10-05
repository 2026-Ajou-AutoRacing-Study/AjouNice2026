#!/usr/bin/env python3
"""Compare planned near-pass geometry with actual synchronous annotation; no collider claims."""
import argparse
import json
from pathlib import Path

import numpy as np
from shapely.geometry import Polygon
from base5_capture_audit import parse_instances,rotation_matrix


def inspect(root):
    rows=[]
    for ledger in Path(root).joinpath('ledger.jsonl').read_text().splitlines():
        entry=json.loads(ledger)
        if not entry.get('capture_root'):
            rows.append(dict(id=entry['id'],status=entry['status'],error=entry.get('error')));continue
        capture=Path(entry['capture_root']);meta=json.loads((capture/'metadata.json').read_text())
        audit=json.loads((capture/'audit.json').read_text());telemetry=json.loads((capture/'telemetry.json').read_text())
        uid=meta['runtime_actors'][0]['unique_id'];transform=np.asarray(meta['lidar_to_ego'])
        ego=Polygon([(-1.05,-1.),(-1.05,1.),(3.75,1.),(3.75,-1.)]);observations=[]
        for box in parse_instances(next(capture.glob('LIDAR_*/*_Instance.txt'))):
            if box['unique_id']!=uid:continue
            signs=np.asarray([[-1,-1],[1,-1],[1,1],[-1,1]])
            corners=np.c_[signs*np.asarray(box['size'][:2])/2,np.zeros(4)]
            corners=corners@rotation_matrix(box['rotation_rpy']).T+box['center']
            corners=(np.c_[corners,np.ones(4)]@transform.T)[:,:2];poly=Polygon(corners)
            center=transform@np.r_[box['center'],1.]
            evidence=[c for c in audit['components'] if c['parent_uid']==uid and c['raw_class']==box['raw_class']]
            observations.append(dict(raw_class=box['raw_class'],center_ego_m=center[:3].tolist(),
                surface_gap_m=poly.distance(ego),overlap_area_m2=poly.intersection(ego).area,
                actual_side='left' if center[1]>0 else 'right',
                size_m=box['size'].tolist(),lidar_points=[c['lidar_points'] for c in evidence],
                maximum_model_instance_pixels=max((v['crop_320x896_instance_pixels'] for c in evidence for v in c['cameras']),default=0)))
        ref=meta['avoidance_observation']
        rows.append(dict(id=entry['id'],status=entry['status'],family=meta['spec']['family'],
            asset=meta['runtime_actors'][0]['name'],phase=ref['phase'],ego_avoidance_side=ref['side'],
            planned_object_side=meta['spec']['role'],planned_surface_gap_m=ref['actual_surface_gap_m'],
            captured_primary=observations,ego_capture_drift_m=telemetry['capture_drift_m'],
            review_reasons=audit['review_reasons'],physical_collision_verified=False,ground_support_verified=False))
    return dict(rows=rows,complete_captures=sum('captured_primary' in r for r in rows),
        primary_annotation_present=sum(bool(r.get('captured_primary')) for r in rows),
        limitations=['Actual LiDAR Instance geometry, not physical collider or road-surface certification.',
                    'Ground/collider review remains mandatory; no automatic approval.'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--capture-root',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);args=p.parse_args();result=inspect(args.capture_root)
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
