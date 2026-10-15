#!/usr/bin/env python3
"""Read-only paired live yaw evidence and CPU RTS augmentation sensitivity."""
import argparse
import json
import math
from pathlib import Path
import numpy as np
from base5_reference_frames import center_in_bev,center_in_roi,yaw_runtime_review
from base5_capture_safety import atomic_json,load_ledger
from base5_capture_audit import rotation_matrix


def augmentation_retention(centers,seed=24,trials=5000):
    """Matches BEVFusionGlobalRotScaleTrans: (R @ center + translation) * scale.

    Measured center distribution, not model AP or a natural-traffic estimate.
    Training ObjectRangeFilter is strict XY; PointRangeFilter is separate XYZ.
    """
    c=np.asarray(centers,dtype=float);base=np.array([center_in_bev(x) for x in c])
    rng=np.random.default_rng(seed);angle=rng.uniform(-.1,.1,trials)
    scale=rng.uniform(.95,1.05,trials);translation=rng.normal(0,.2,(trials,3))
    rotated=np.broadcast_to(c,(trials,len(c),3)).copy()
    rotated[:,:,0]=np.cos(angle[:,None])*c[:,0]-np.sin(angle[:,None])*c[:,1]
    rotated[:,:,1]=np.sin(angle[:,None])*c[:,0]+np.cos(angle[:,None])*c[:,1]
    result={}
    for name,rc,s in [('current_RTS',rotated,scale),('no_scale',rotated,np.ones(trials)),
                      ('no_rotation',np.broadcast_to(c,rotated.shape),scale),
                      ('translation_only',np.broadcast_to(c,rotated.shape),np.ones(trials))]:
        x=(rc+translation[:,None,:])*s[:,None,None]
        kept=(abs(x[:,:,0])<100)&(abs(x[:,:,1])<30)
        result[name]=dict(conditional_inside_retention=float(kept[:,base].mean()) if base.any() else None,
            retention_by_box=kept.mean(axis=0).tolist())
    return dict(seed=seed,trials=trials,boxes=len(c),initial_inside=int(base.sum()),profiles=result)


def report(manifest,ledgers,readiness=None):
    m=json.loads(manifest.read_text());entries={e['id']:e for e in m['scenes']}
    # First valid frozen-world capture wins; preserve every excluded attempt.
    selected={};excluded=[];all_rows=[]
    for ledger in ledgers:
        for r in load_ledger(ledger):
            if r['id'] not in entries:continue
            root=Path(r.get('capture_root',''));tp=root/'telemetry.json'
            if not tp.is_file():excluded.append(dict(id=r['id'],ledger=str(ledger),reasons=['no_telemetry']));continue
            meta=json.loads((root/'metadata.json').read_text());t=json.loads(tp.read_text())
            reasons=yaw_runtime_review(meta,t['objects_before'],t['objects_after'])
            if r['status']!='accepted_candidate':reasons.append(r['status'])
            if reasons:excluded.append(dict(id=r['id'],ledger=str(ledger),reasons=reasons));continue
            if r['id'] not in selected:selected[r['id']]=(r,meta,t)
    for ident,(r,meta,t) in selected.items():
        probe=meta['yaw_probe'];uid=meta['runtime_actors'][0]['unique_id']
        c=next(c for c in r['audit']['components'] if c['parent_uid']==uid)
        corrected=t['ego_before']['heading']+probe.get('ego_heading_correction_deg',-1.347)
        yaw_error=(corrected-meta['ego']['yaw']+180)%360-180
        delta=math.radians(corrected-probe['road_yaw_deg']);x,y=c['center_ego_m'][:2]
        cloud=np.fromfile(next(Path(r['capture_root']).glob('LIDAR_*/*_Intensity.bin')),dtype=np.float32).reshape(-1,4)[:,:3]
        boxmask=(abs((cloud-np.asarray(c['center_lidar_m']))@rotation_matrix(c['rotation_rpy']))<=np.asarray(c['size_m'])/2+1e-5).all(axis=1)
        pointmask=((cloud>[-100,-30,-3])&(cloud<[100,30,2])).all(axis=1)
        all_rows.append(dict(id=ident,group=probe['group'],angle_deg=probe['angle_deg'],
            asset=c['asset'],center_lidar_m=c['center_lidar_m'],center_ego_m=c['center_ego_m'],
            observed_road_center_xy_m=[math.cos(delta)*x-math.sin(delta)*y,math.sin(delta)*x+math.cos(delta)*y],
            train_center_inside=center_in_bev(c['center_lidar_m']),
            inference_center_inside_3d=center_in_roi(c['center_lidar_m']),
            lidar_points=c['lidar_points'],camera_visible=c['camera_visible_candidate'],
            lidar_points_inside_model_roi=int((boxmask&pointmask).sum()),
            positive_evidence=c['positive_lidar1_camera0'],yaw_error_deg=yaw_error,
            collision_clear=r['audit']['capture_safety']['collision']['no_collision_verified'],
            capture_root=r['capture_root']))
    all_rows.sort(key=lambda r:(r['group'],r['angle_deg']))
    angles={}
    for angle in m['yaw_probe_provenance']['angles']:
        rows=[r for r in all_rows if r['angle_deg']==angle]
        angles[str(angle)]=dict(captured=len(rows),expected=6,inside=sum(r['train_center_inside'] for r in rows),
            outside_ids=[r['id'] for r in rows if not r['train_center_inside']],
            lidar_supported=sum(r['lidar_points']>0 for r in rows))
    known_roi={'b5yaw_curve_right_p15','b5yaw_curve_left_m15'}
    audit=json.loads((manifest.parent/'offline_audit.json').read_text())
    expected_errors={(ident,'ROI') for ident in known_roi}
    errors={tuple(e) for e in audit['errors']}
    exits={str(p.parent):json.loads((p.parent/'shutdown_safety.json').read_text())
           for p in ledgers if (p.parent/'shutdown_safety.json').is_file()}
    result=dict(status='COMPLETE' if len(selected)==42 else 'INCOMPLETE',
        session_exits=exits,training_release='NOT APPROVED; diagnostic data only',
        model_roi=[-100,-30,-3,100,30,2],train_contract='strict XY center; not full-box inclusion',
        inference_contract='XYZ center post-filter; full-box margins not tested',
        denominator='six deliberately selected road layouts, not population inclusion probability',
        angles=angles,rows=all_rows,excluded_attempts=excluded,
        missing_ids=sorted(set(entries)-set(selected)),
        offline_diagnostic=dict(ordinary_status=audit['status'],expected_roi_only=errors==expected_errors,
            errors=audit['errors'],note='Intentional outside-ROI probes; ordinary OFFLINE_FAIL retained, not training candidates'),
        paired_augmentation=augmentation_retention([r['center_lidar_m'] for r in all_rows]) if all_rows else None)
    if readiness:
        centers=[c['center_lidar_m'] for r in load_ledger(readiness) for c in r['audit']['components']
                 if c['positive_lidar1_camera0']]
        result['readiness_augmentation']=augmentation_retention(centers)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--ledger',type=Path,action='append',required=True)
    p.add_argument('--readiness-ledger',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--retry-manifest',type=Path,help='Optional NEW subset manifest for missing frozen-world conditions')
    a=p.parse_args();r=report(a.manifest,a.ledger,a.readiness_ledger);atomic_json(a.output,r)
    if a.retry_manifest:
        if a.retry_manifest.exists():raise FileExistsError('Retry manifest overwrite refused')
        m=json.loads(a.manifest.read_text());m['scenes']=[e for e in m['scenes'] if e['id'] in r['missing_ids']]
        m['count']=len(m['scenes']);m['retry_of']=str(a.manifest.resolve());atomic_json(a.retry_manifest,m)
    print(json.dumps({k:r[k] for k in ('status','angles','excluded_attempts','missing_ids')},indent=2))
