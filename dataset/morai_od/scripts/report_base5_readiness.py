#!/usr/bin/env python3
"""Read-only capture/PKL readiness evidence; never starts MR, capture or training."""
import argparse
from collections import Counter
import json
from pathlib import Path
import pickle
import numpy as np
from PIL import Image
from base5_capture_safety import atomic_json,load_ledger,motion_capture_review


def report(capture_root,info_root,ledger=None):
    ledger=ledger or capture_root/'ledger.jsonl'
    rows=list({r['id']:r for r in load_ledger(ledger)}.values())
    cells={};motions=[];raw=0;camera_only=Counter();errors=[];secondary_motion=[]
    for row in rows:
        key=str((row['family'],row.get('distance_bounds'),row.get('role')))
        cells.setdefault(key,Counter())[row['status']]+=1
        if row['status']=='failed':continue
        root=Path(row['capture_root']);meta=json.loads((root/'metadata.json').read_text())
        audit=json.loads((root/'audit.json').read_text());tele=json.loads((root/'telemetry.json').read_text())
        raw+=sum(p.stat().st_size for p in root.rglob('*') if p.is_file())
        motion=tele.get('motion_capture',{})
        if motion.get('required'):
            motions.append(dict(id=row['id'],asset=meta['runtime_actors'][0]['name'],
                nominal_speed_mps=meta['runtime_actors'][0]['motion_path']['nominal_speed_mps'],**motion))
        # Historical v22 primary-only live gate: independently replay secondary
        # checks from preserved receive-time traces without rewriting raw/ledger.
        for actor in meta['runtime_actors'][1:]:
            if actor['motion'] not in ('static','standing'):
                evidence=motion_capture_review(actor,tele['objects_before'],tele['objects_after'],
                    tele.get('object_position_samples',[]),tele.get('capture_requested_at_monotonic',0.))
                secondary_motion.append(dict(id=row['id'],asset=actor['name'],uid=actor['unique_id'],**evidence))
                if evidence['review_reasons']:errors.append('Secondary motion review '+row['id'])
        if row['status']=='accepted_candidate':
            for c in audit['components']:
                if c['positive_lidar1_camera0'] and c['lidar_points']==0:camera_only[c['training_class']]+=1
    samples=0;gt=Counter();empty=0
    for path in info_root.glob('morai_base5_infos_*.pkl'):
        payload=pickle.loads(path.read_bytes())
        classes=payload['metainfo']['classes']
        if payload['metainfo']['box_dim']!=7 or payload['metainfo']['velocity_target']:
            errors.append('Non single-frame/7D contract '+str(path))
        for row in payload['data_list']:
            samples+=1;empty+=int(not row['instances'])
            lidar=Path(row['lidar_points']['lidar_path'])
            if not lidar.is_file() or lidar.stat().st_size%16:errors.append('Missing/misaligned 4-feature LiDAR '+str(lidar))
            elif not np.isfinite(np.fromfile(lidar,dtype=np.float32)).all():errors.append('Nonfinite LiDAR '+str(lidar))
            if len(row['images'])!=4:errors.append('Not four cameras '+row['sample_idx'])
            for image in row['images'].values():
                with Image.open(image['img_path']) as im:
                    if im.size!=(image['width'],image['height']):errors.append('Image dimensions '+row['sample_idx'])
                    im.verify()
                for key in ('cam2img','lidar2cam','lidar2img','cam2lidar'):
                    if not np.isfinite(image[key]).all():errors.append('Nonfinite calibration '+row['sample_idx'])
            for instance in row['instances']:
                box=instance['bbox_3d'];label=instance['bbox_label_3d']
                if len(box)!=7 or not np.isfinite(box).all() or min(box[3:6])<=0:errors.append('Invalid box '+row['sample_idx'])
                if not 0<=label<len(classes):errors.append('Invalid class '+row['sample_idx'])
                else:gt[classes[label]]+=1
    times=[r['duration_sec'] for r in rows];mean=float(np.mean(times)) if times else None
    shutdown=json.loads((capture_root/'shutdown_safety.json').read_text())
    return dict(ledger=str(ledger.resolve()),status_counts=dict(Counter(r['status'] for r in rows)),cells={k:dict(v) for k,v in cells.items()},
        motion_evidence=motions,secondary_motion_replayed_from_raw=secondary_motion,
        camera_only_kept_gt=dict(camera_only),shutdown=shutdown,
        loader_sanity=dict(samples=samples,empty_background_samples=empty,gt=dict(gt),errors=errors,
            limitation='Raw image/point/calibration/PKL CPU sanity, not MMDet3D training-forward validation'),
        duration_mean_sec=mean,duration_p50_sec=float(np.median(times)) if times else None,
        duration_p95_sec=float(np.percentile(times,95)) if times else None,capture_bytes=raw,
        estimate_10k=dict(seconds_at_sample_mean=mean*10000 if mean else None,
            bytes_at_sample_mean=raw/max(1,len(rows))*10000,
            limitation='Pilot mean only; excludes failed-cell replenishment, simulator aging, training and derived caches'),
        limitations=[f'{len(rows)}-cell report is not exhaustive catalog validation or detector evaluation.',
            'Endpoint before/after save is conservative, not exact sensor-capture-time motion certification.',
            'Review raw remains preserved and cannot become empty negative training samples.'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--capture-root',type=Path,required=True)
    p.add_argument('--info-root',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--ledger',type=Path,help='Optional canonical selected ledger; use safety selector provenance for all source exits')
    a=p.parse_args();r=report(a.capture_root,a.info_root,a.ledger);atomic_json(a.output,r)
    print(json.dumps({k:v for k,v in r.items() if k not in ('cells','motion_evidence')},indent=2))
    if r['loader_sanity']['errors'] or not r['shutdown']['park_verified']:raise SystemExit(1)
