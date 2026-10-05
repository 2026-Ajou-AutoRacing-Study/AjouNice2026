#!/usr/bin/env python3
"""Component-preserving paired asset appearance report, RGB and local LiDAR evidence."""
import argparse
import json
import math
from pathlib import Path
from collections import defaultdict
import numpy as np
from PIL import Image,ImageDraw
from base5_capture_safety import atomic_json,load_ledger
from base5_capture_audit import rotation_matrix


def condition_valid(mode,speed,error,tolerance):
    if speed is None or not math.isfinite(speed):return False
    return speed<.02 if mode=='static' else speed>.05 and error<=tolerance


def component_size_changes(rows):
    result={}
    for ident in sorted({c['component_id'] for r in rows for c in r['components']}):
        sizes={mode:[c['size_m'] for r in rows if r['mode']==mode and r['condition_valid']
                     for c in r['components'] if c['component_id']==ident] for mode in ('static','moving')}
        if not all(sizes.values()):continue
        static=np.median(sizes['static'],axis=0);moving=np.median(sizes['moving'],axis=0)
        result[ident]=dict(static_median_size_m=static.tolist(),moving_median_size_m=moving.tolist(),
            maximum_axis_median_difference_m=float(np.max(abs(moving-static))))
    return result


def report(root,output,additional_roots=()):
    output.mkdir(parents=True,exist_ok=False);groups=defaultdict(list);failures=[];rows=[]
    # Explicitly ordered runs, immutable raw provenance; corrected pairs supersede old pairs.
    roots=[root,*additional_roots];attempts=[];superseded=[];latest={}
    for source in roots:
        for r in load_ledger(source/'ledger.jsonl'):
            r=dict(r,source_root=str(source.resolve()));attempts.append(r)
            if r['id'] in latest:superseded.append(latest[r['id']])
            latest[r['id']]=r
    for r in attempts:
        if r['status']=='failed':failures.append(r)
    for ident,r in latest.items():
        if r['status']=='failed':continue
        p=Path(r['capture_root']);m=json.loads((p/'metadata.json').read_text());t=json.loads((p/'telemetry.json').read_text())
        h=json.loads((p/'collision_history.json').read_text());probe=m['motion_probe'];components=r['audit']['components'];request=t['capture_requested_at_monotonic']
        uid=m['runtime_actors'][0]['unique_id'];trace=[x for x in t['object_position_samples'] if x['uid']==uid and request-.3<=x['t']<=request]
        span=trace[-1]['t']-trace[0]['t'] if len(trace)>1 else 0
        speed=math.hypot(trace[-1]['x']-trace[0]['x'],trace[-1]['y']-trace[0]['y'])/span if span>.05 else None
        before=next(x['position'] for key in ('npc_list','pedestrian_list','obstacle_list') for x in t['objects_before'].get(key,[]) if x['unique_id']==uid)
        target=probe['target_position'];error=math.hypot(before['x']-target['x'],before['y']-target['y'])
        height_reference=probe.get('height_calibration')
        height_error=abs(before['z']-height_reference['static_z']) if height_reference else None
        cameras=defaultdict(list)
        for c in components:
            for camera in c['cameras']:cameras[camera['camera']].append(camera)
        camera=max(cameras,key=lambda key:sum(x['native_instance_pixels'] for x in cameras[key])) if cameras else None
        tile=Image.new('RGB',(240,320),'white');draw=ImageDraw.Draw(tile)
        draw.text((5,5),f"{probe['asset']} / {probe['mode']} r{probe['repeat']}",fill='black')
        draw.text((5,20),f"requested {probe['commanded_speed_mps']:.2f}; observed {speed:.2f}" if speed is not None else 'speed unavailable',fill='black')
        if camera:
            rects=[x['native_rect'] for x in cameras[camera]]
            x0=min(x[0] for x in rects);y0=min(x[1] for x in rects);x1=max(x[2] for x in rects);y1=max(x[3] for x in rects)
            raw=Image.open(next((p/camera).glob('*_Intensity.png'))).convert('RGB')
            crop=raw.crop((int(x0)-10,int(y0)-10,math.ceil(x1)+10,math.ceil(y1)+10));scale=min(230/crop.width,270/crop.height)
            crop=crop.resize((max(1,round(crop.width*scale)),max(1,round(crop.height*scale))),Image.NEAREST)
            tile.paste(crop,((240-crop.width)//2,40+(270-crop.height)//2))
        tile.save(output/(ident+'_rgb.png'))
        cloud=np.fromfile(next(p.glob('LIDAR_*/*_Intensity.bin')),dtype=np.float32).reshape(-1,4)[:,:3]
        local_clouds=[]
        for c in components:
            local=(cloud-np.asarray(c['center_lidar_m']))@rotation_matrix(c['rotation_rpy'])
            local=local[(abs(local)<=np.asarray(c['size_m'])/2+1e-5).all(axis=1)]
            local_clouds.append(dict(component_id=c['component_id'],local_points_xyz=local.tolist()))
        row=dict(id=ident,asset=probe['asset'],mode=probe['mode'],repeat=probe['repeat'],status=r['status'],
            condition_valid=condition_valid(probe['mode'],speed,error,probe['target_tolerance_m']) and (height_error is None or height_error<=.03),
            requested_speed_mps=probe['commanded_speed_mps'],observed_receive_time_speed_mps=speed,
            target_error_before_capture_m=error,load_to_capture_sec=request-h['load_time_monotonic'],
            backward_start_m=probe['backward_start_m'],review_reasons=r['audit']['review_reasons'],
            collision_clear=r['audit']['capture_safety']['collision']['no_collision_verified'],
            geometry_reference_known=m['runtime_actors'][0]['geometry_reference_known'],
            source_root=r['source_root'],object_position_before=before,scenario_sha256=r['scenario_sha256'],
            height_calibration=height_reference,height_error_m=height_error,
            components=[{k:c[k] for k in ('component_id','raw_class','training_class','size_m','rotation_rpy','lidar_points','camera_visible_candidate','positive_lidar1_camera0')} for c in components],
            local_clouds=local_clouds,rgb_tile=str((output/(ident+'_rgb.png')).resolve()),capture_root=str(p.resolve()))
        rows.append(row);groups[probe['asset']].append((row,tile))
    summaries={}
    for asset,items in groups.items():
        safe=asset.replace('/','_');sheet=Image.new('RGB',(960,320),'#cccccc')
        for row,tile in items:sheet.paste(tile,((0 if row['mode']=='static' else 2)*240+row['repeat']*240,0))
        sheet.save(output/(safe+'_comparison.png'))
        summary={}
        for mode in ('static','moving'):
            rs=[row for row,_ in items if row['mode']==mode];by_component=defaultdict(list)
            for row in rs:
                for c in row['components']:by_component[c['component_id']].append(c)
            summary[mode]=dict(captured=len(rs),accepted=sum(r['status']=='accepted_candidate' for r in rs),
                condition_valid=sum(r['condition_valid'] for r in rs),
                speeds=[r['observed_receive_time_speed_mps'] for r in rs],
                component_size_min_max={key:dict(raw_class=cs[0]['raw_class'],
                    minimum=np.min([c['size_m'] for c in cs],axis=0).tolist(),maximum=np.max([c['size_m'] for c in cs],axis=0).tolist()) for key,cs in by_component.items()})
        summaries[asset]=summary
        import matplotlib
        matplotlib.use('Agg')
        from matplotlib import pyplot as plt
        components=sorted({c['component_id'] for row,_ in items for c in row['local_clouds']})
        fig,axs=plt.subplots(1,len(components),figsize=(4*len(components),4),squeeze=False)
        for ax,component in zip(axs[0],components):
            for mode,color in [('static','tab:orange'),('moving','tab:blue')]:
                pts=[c['local_points_xyz'] for row,_ in items if row['mode']==mode for c in row['local_clouds'] if c['component_id']==component and c['local_points_xyz']]
                if pts:
                    xyz=np.concatenate(pts);ax.scatter(xyz[:,0],xyz[:,2],s=9,c=color,label=mode,alpha=.7)
            ax.set_title(component);ax.set_xlabel('box-local X (m)');ax.set_ylabel('box-local Z (m)');ax.set_aspect('equal');ax.legend()
        fig.suptitle(asset+' / sparse observed returns, not mesh reconstruction');fig.tight_layout()
        fig.savefig(output/(safe+'_lidar_xz.png'),dpi=140);plt.close(fig)
    comparisons={asset:dict(component_size_changes=component_size_changes([r for r,_ in items]),
        static_object_z_median=float(np.median([r['object_position_before']['z'] for r,_ in items if r['mode']=='static'])),
        moving_object_z_median=float(np.median([r['object_position_before']['z'] for r,_ in items if r['mode']=='moving'])))
        for asset,items in groups.items() if all(any(r['mode']==mode for r,_ in items) for mode in ('static','moving'))}
    result=dict(rows=rows,summary=summaries,comparisons=comparisons,failed_attempts=failures,superseded_attempts=superseded,
        source_roots=[str(p.resolve()) for p in roots],
        shutdown=json.loads((roots[-1]/'shutdown_safety.json').read_text()) if (roots[-1]/'shutdown_safety.json').is_file() else {'park_verified':False,'capture_in_progress':True},
        limitation='Two phases per condition, ego stationary. Component OBB can change with animation but box changes alone are not proof. RGB inspection required; sparse LiDAR does not certify equal meshes. Not an all-catalog validation, velocity GT, detector AP, or training release.')
    atomic_json(output/'report.json',result)
    print(json.dumps(dict(conditions=len(rows),accepted=sum(r['status']=='accepted_candidate' for r in rows),
        failed_attempts=len(failures),assets=len(groups),park_verified=result['shutdown']['park_verified']),indent=2))
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--capture-root',type=Path,required=True)
    p.add_argument('--additional-capture-root',type=Path,action='append',default=[],help='Later runs explicitly supersede matching IDs; old records retained')
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();report(a.capture_root,a.output,a.additional_capture_root)
