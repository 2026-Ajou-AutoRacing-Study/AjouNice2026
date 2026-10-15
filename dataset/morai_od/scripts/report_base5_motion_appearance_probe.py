#!/usr/bin/env python3
"""Raw crop contact sheet and arrival/motion evidence; not detector accuracy."""
import argparse
import json
import math
from pathlib import Path
from collections import defaultdict
import numpy as np
from PIL import Image,ImageDraw
from base5_capture_safety import atomic_json,load_ledger


def report(root,output):
    output.mkdir(parents=True,exist_ok=False);rows=[];tiles=[]
    for r in load_ledger(root/'ledger.jsonl'):
        if r['status']=='failed':rows.append(dict(id=r['id'],status=r['status'],error=r['error']));continue
        p=Path(r['capture_root']);m=json.loads((p/'metadata.json').read_text());t=json.loads((p/'telemetry.json').read_text())
        h=json.loads((p/'collision_history.json').read_text());probe=m['motion_probe'];c=r['audit']['components'][0]
        uid=m['runtime_actors'][0]['unique_id'];request=t['capture_requested_at_monotonic']
        trace=[x for x in t['object_position_samples'] if x['uid']==uid and request-.3<=x['t']<=request]
        span=trace[-1]['t']-trace[0]['t'] if len(trace)>1 else 0
        observed=math.hypot(trace[-1]['x']-trace[0]['x'],trace[-1]['y']-trace[0]['y'])/span if span>.05 else None
        target=probe['target_position'];before=next(x['position'] for x in t['objects_before']['pedestrian_list'] if x['unique_id']==uid)
        err=math.hypot(before['x']-target['x'],before['y']-target['y'])
        yaw=math.radians(t['ego_before']['heading']-1.347);cx,cy=c['center_ego_m'][:2]
        world_center=[t['ego_before']['x']+math.cos(yaw)*cx-math.sin(yaw)*cy,
                      t['ego_before']['y']+math.sin(yaw)*cx+math.cos(yaw)*cy]
        gt_error=math.hypot(world_center[0]-probe['target_center_xy'][0],world_center[1]-probe['target_center_xy'][1])
        camera=max(c['cameras'],key=lambda x:x['native_instance_pixels'])
        image=Image.open(next((p/camera['camera']).glob('*_Intensity.png'))).convert('RGB')
        x0,y0,x1,y1=camera['native_rect'];crop=image.crop((int(x0)-10,int(y0)-10,math.ceil(x1)+10,math.ceil(y1)+10))
        tile=Image.new('RGB',(240,320),'white')
        factor=min(220/crop.width,250/crop.height)
        crop=crop.resize((round(crop.width*factor),round(crop.height*factor)),resample=Image.NEAREST)
        tile.paste(crop,((240-crop.width)//2,45+(250-crop.height)//2))
        label=f"{probe['commanded_speed_mps']:.2f} m/s requested / r{probe['repeat']}"
        ImageDraw.Draw(tile).text((8,8),label,fill='black')
        ImageDraw.Draw(tile).text((8,25),f"observed {observed:.2f} m/s" if observed is not None else 'observed unavailable',fill='black')
        crop.save(output/(r['id']+'.png'));tiles.append((probe['repeat'],probe['commanded_speed_mps'],tile))
        rows.append(dict(id=r['id'],status=r['status'],commanded_speed_mps=probe['commanded_speed_mps'],
            observed_receive_time_speed_mps=observed,backward_start_m=probe['backward_start_m'],
            load_to_capture_sec=request-h['load_time_monotonic'],arrival=t.get('motion_probe_arrival'),
            target_error_before_capture_m=err,size_m=c['size_m'],lidar_points=c['lidar_points'],
            raw_gt_world_xy=world_center,raw_gt_target_error_m=gt_error,
            camera=camera['camera'],native_rect=camera['native_rect'],collision_clear=r['audit']['capture_safety']['collision']['no_collision_verified']))
    sheet=Image.new('RGB',(960,960),'#cccccc');speeds=[0.,.35,1.1,2.2]
    for repeat,speed,tile in tiles:sheet.paste(tile,(speeds.index(speed)*240,repeat*320))
    sheet.save(output/'appearance_contact_sheet.png')
    grouped=defaultdict(list)
    for r in rows:
        if 'commanded_speed_mps' in r:grouped[str(r['commanded_speed_mps'])].append(r)
    summary={}
    for speed,items in grouped.items():
        summary[speed]=dict(captured=len(items),accepted=sum(x['status']=='accepted_candidate' for x in items),
            observed_speed_median=float(np.median([x['observed_receive_time_speed_mps'] for x in items if x['observed_receive_time_speed_mps'] is not None])),
            load_to_capture_median_sec=float(np.median([x['load_to_capture_sec'] for x in items])),
            maximum_target_error_m=max(x['target_error_before_capture_m'] for x in items),
            maximum_raw_gt_target_error_m=max(x['raw_gt_target_error_m'] for x in items))
    result=dict(rows=rows,summary=summary,shutdown=json.loads((root/'shutdown_safety.json').read_text()),
        limitation='Same template and speed-only API. Random animation phase is not synchronized; does not certify explicit Run animation, stride-speed coupling or learned OD quality.')
    atomic_json(output/'report.json',result);print(json.dumps(summary,indent=2));return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--capture-root',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();report(a.capture_root,a.output)
