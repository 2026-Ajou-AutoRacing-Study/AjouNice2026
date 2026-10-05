#!/usr/bin/env python3
"""Summarize retained Base5 review evidence without ROS or model inference."""
import argparse
import json
import math
from pathlib import Path
from base5_capture_safety import atomic_json,load_ledger


def report(root, probes=()):
    rows=[]
    for row in load_ledger(root/'ledger.jsonl'):
        if row['status']!='review':continue
        capture=Path(row['capture_root'])
        meta=json.loads((capture/'metadata.json').read_text())
        telemetry=json.loads((capture/'telemetry.json').read_text())
        actor=meta['runtime_actors'][0]
        objects=telemetry['objects_before']
        observed=next((a for k in ('npc_list','pedestrian_list','obstacle_list')
            for a in objects.get(k,[]) if a['unique_id']==actor['unique_id']),None)
        travel=math.hypot(observed['position']['x']-actor['position']['x'],
            observed['position']['y']-actor['position']['y']) if observed else None
        rows.append(dict(id=row['id'],asset=actor['name'],motion=actor['motion'],
            requested_distance_bounds_m=row['distance_bounds'],
            primary_pivot_travel_before_capture_m=travel,
            primary_displacement_during_verified_wait_m=telemetry['primary_displacement_during_wait_m'],
            components=[dict(range_ego_xy_m=c.get('range_ego_xy_m'),lidar_points=c['lidar_points'],
                cameras=c['cameras'],positive_lidar1_camera0=c['positive_lidar1_camera0'])
                for c in row['audit']['components'] if c['parent_uid']==actor['unique_id']],
            reasons=row['audit']['review_reasons'],capture_root=str(capture),
            limitation='Object pivot travel is not annotation-center velocity or capture timestamp alignment.'))
    failures=[]
    for path in probes:
        for row in load_ledger(path/'ledger.jsonl'):
            failures.append(dict(probe=str(path),id=row['id'],status=row['status'],error=row.get('error')))
    return dict(status='DYNAMIC_TIMING_NOT_RESOLVED; BULK_CAPTURE_BLOCKED',reviews=rows,
        exploratory_probe_results=failures,
        decision='Do not broaden GT tolerance, relabel invisible boxes, or deploy failed pause/staged-load experiments.')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--capture-root',type=Path,required=True)
    p.add_argument('--probe',type=Path,action='append',default=[])
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();result=report(a.capture_root,a.probe);atomic_json(a.output,result)
    print(json.dumps(result,ensure_ascii=False,indent=2))
