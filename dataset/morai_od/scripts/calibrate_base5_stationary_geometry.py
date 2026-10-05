#!/usr/bin/env python3
"""Derive versioned placement references from stationary MR annotation, not training GT edits."""
import argparse
import json
import math
from pathlib import Path
import numpy as np
from base5_capture_audit import parse_instances
from base5_capture_safety import atomic_json,sha256


def calibrate(roots):
    groups={}
    for root in roots:
        root=Path(root);meta=json.loads((root/'metadata.json').read_text());tele=json.loads((root/'telemetry.json').read_text())
        e=tele['ego_before'];expected=meta['ego']['yaw'];yaw=e['heading']-1.347
        delta=(yaw-expected+180)%360-180
        if abs(delta)>.1 or tele['capture_drift_m']>.005:raise ValueError('Nonstationary/ambiguous MR yaw calibration')
        transform=np.asarray(meta['lidar_to_ego']);a=math.radians(yaw)
        rot=np.asarray([[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1]])
        scene=json.loads((root/'scenario.json').read_text());by_uid={r['UNIQUEID']:r for k in ('objectList','vehicleList') for r in scene.get(k,[])}
        objects={r['unique_id']:r for k in ('npc_list','pedestrian_list','obstacle_list') for r in tele['objects_before'][k]}
        boxes=parse_instances(next(root.glob('LIDAR_*/*_Instance.txt')))
        for actor in meta['runtime_actors']:
            uid=actor['unique_id'];bb=[b for b in boxes if b['unique_id']==uid]
            if len(bb)!=1 or uid not in by_uid or uid not in objects:raise ValueError('Calibration requires one stationary component and runtime actor')
            box=bb[0];row=by_uid[uid];pose=row.get('initPosition',row)
            ay=math.radians(float(pose['rot']['yaw']));r=np.asarray([[math.cos(ay),-math.sin(ay),0],[math.sin(ay),math.cos(ay),0],[0,0,1]])
            ego_center=(transform@np.r_[box['center'],1.])[:3]
            world=rot@ego_center+np.asarray([e[k] for k in ('x','y','z')])
            pivot=np.asarray([objects[uid]['position'][k] for k in ('x','y','z')])
            offset=r.T@(world-pivot)
            groups.setdefault(actor['name'],[]).append(dict(size=box['size'].tolist(),offset=offset.tolist(),
                source=str(root.resolve()),annotation_sha256=sha256(next(root.glob('LIDAR_*/*_Instance.txt'))),yaw_delta_deg=delta))
    assets={}
    for name,observations in groups.items():
        assets[name]=dict(bbox_size_m=np.median([r['size'] for r in observations],axis=0).tolist(),
            pivot_to_bbox_center_local_m=np.median([r['offset'] for r in observations],axis=0).tolist(),
            observations=len(observations),evidence=observations,
            limitations='Placement estimate only; current MR scale1 static actor, no collider shape claim')
    return dict(schema_version=1,contract='MR Ego heading -1.347 once; raw LiDAR annotation unchanged; actor scenario yaw',assets=assets)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--capture-roots',nargs='+',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise FileExistsError('Reference overwrite refused')
    result=calibrate(a.capture_roots);atomic_json(a.output,result);print(json.dumps(result,indent=2))
