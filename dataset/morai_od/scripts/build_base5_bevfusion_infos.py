#!/usr/bin/env python3
"""Convert audited Base5 captures to four-class MMDet3D; raw is never rewritten."""
import argparse
from collections import Counter
import json
from pathlib import Path
import pickle

import numpy as np
from dynamic_capture_metadata import normalized_sensor_calibration

CLASSES=('vehicle','pedestrian','obstacle','animal')
DEFAULT_ROI=(-100.,-30.,-3.,100.,30.,2.)


def in_roi(center,roi=DEFAULT_ROI):
    center=np.asarray(center,float);bounds=np.asarray(roi,float)
    return bool((center>=bounds[:3]).all() and (center<bounds[3:]).all())


def convert(ledger, output, sensor, policy='lidar1_camera0', serialized_root=None,roi=DEFAULT_ROI,environment_tag=None):
    roi=list(map(float,roi))
    if len(roi)!=6 or not np.isfinite(roi).all() or any(a>=b for a,b in zip(roi[:3],roi[3:])):
        raise ValueError('Invalid LiDAR-frame training ROI')
    calibration=normalized_sensor_calibration(sensor)
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    captures={};failed=[]
    for line in Path(ledger).read_text().splitlines():
        entry=json.loads(line)
        if entry['status'] in ('accepted_candidate','review'): captures[entry['id']]=entry
        elif entry['status']=='failed':failed.append(entry['id'])
    splits={name:[] for name in ('train','val','test')}; kept=Counter();excluded=Counter();review=[]
    source_root=Path(ledger).resolve().parent
    def mapped(path):
        path=Path(path).resolve()
        return str(Path(serialized_root)/path.relative_to(source_root)) if serialized_root else str(path)
    for scene_id,entry in captures.items():
        if entry['status']!='accepted_candidate':review.append(scene_id);continue
        root=Path(entry['capture_root']);meta=json.loads((root/'metadata.json').read_text())
        if meta.get('calibration_only'):
            review.append(scene_id);continue # empty terrain probes are not training negatives
        if meta.get('sensor_source_sha256') not in (None,calibration['source_sha256']):
            raise ValueError('Capture/conversion sensor calibration checksum mismatch '+scene_id)
        audit=json.loads(Path(entry.get('audit_path',root/'audit.json')).read_text())
        if audit.get('review_reasons'):
            review.append(scene_id);continue
        if meta['runtime_actors']:
            primary_uid=meta['runtime_actors'][0]['unique_id']
            if not any(c['parent_uid']==primary_uid and c['positive_'+policy] and in_roi(c['center_lidar_m'],roi) for c in audit['components']):
                review.append(scene_id);continue
        lidar=list(root.glob('LIDAR_*/*_Intensity.bin'))
        if len(lidar)!=1: raise ValueError('Missing LiDAR')
        instances=[]
        for component in audit['components']:
            if not component['positive_'+policy]: excluded[component['training_class']]+=1;continue
            if not in_roi(component['center_lidar_m'],roi):excluded['outside_lidar_roi']+=1;continue
            cls=component['training_class'];kept[cls]+=1
            box=component['center_lidar_m']+component['size_m']+[component['rotation_rpy'][2]]
            instances.append(dict(bbox_3d=box,bbox_label_3d=CLASSES.index(cls),
                num_lidar_pts=component['lidar_points'],bbox_3d_isvalid=True,
                instance_id=len(instances),parent_uid=component['parent_uid'],
                component_id=component['component_id'],asset=component['asset']))
        if meta['runtime_actors'] and not instances:
            review.append(scene_id);continue # never turn invisible primary into empty background
        images={}
        lidar2ego=np.asarray(calibration['lidar']['lidar_to_ego'])
        for name,cam in calibration['cameras'].items():
            folder=root/('CAMERA_'+str(cam['sensor_unique_id']))
            paths=list(folder.glob('*_Intensity.png'))
            if len(paths)!=1: raise ValueError('Missing camera '+name)
            intrinsic=np.asarray(cam['intrinsic']);lidar2cam=np.asarray(cam['ego_to_sensor'])@lidar2ego
            images[name]=dict(img_path=mapped(paths[0]),height=cam['resolution_px'][1],width=cam['resolution_px'][0],
                cam2img=intrinsic.tolist(),lidar2cam=lidar2cam.tolist(),
                lidar2img=(intrinsic@lidar2cam[:3]).tolist(),cam2lidar=np.linalg.inv(lidar2cam).tolist())
        group=meta.get('scene_group_id',entry.get('source_cell_id',scene_id))
        token=f'{environment_tag}:{group}' if environment_tag else scene_id
        splits[meta['split']].append(dict(sample_idx=token,token=token,scene_id=group,
            scene_group_id=group,environment_tag=environment_tag,timestamp=0.,
            lidar_points=dict(lidar_path=mapped(lidar[0]),num_pts_feats=4,lidar2ego=lidar2ego.tolist()),
            images=images,instances=instances))
    metainfo=dict(dataset='base5_independent',classes=CLASSES,box_dim=7,positive_policy=policy,
        sensor_sha256=calibration['source_sha256'],single_frame=True,velocity_target=False,point_cloud_range=roi,range_frame='lidar')
    for split,rows in splits.items():
        with (output/f'morai_base5_infos_{split}.pkl').open('wb') as f:
            pickle.dump(dict(metainfo=metainfo,data_list=rows),f)
    report=dict(splits={k:len(v) for k,v in splits.items()},kept_gt=dict(kept),excluded_gt=dict(excluded),
        excluded_review_scenes=review,excluded_failed_scenes=failed,policy=policy,raw_preserved=True,
        limitations=['Scene-disjoint link split is not a held-out real-driving test.',
                    'Unsupported components are excluded by explicit evidence policy; all remain in raw audit.'])
    (output/'conversion_audit.json').write_text(json.dumps(report,indent=2)+'\n')
    (output/'calibration.json').write_text(json.dumps(calibration,indent=2)+'\n')
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--ledger',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--sensor-config',type=Path,required=True)
    p.add_argument('--policy',choices=['lidar5','lidar1','lidar1_camera0'],default='lidar1_camera0')
    p.add_argument('--serialized-root',type=Path)
    p.add_argument('--point-cloud-range',type=float,nargs=6,default=DEFAULT_ROI)
    p.add_argument('--environment-tag',help='Unique sample tokens while preserving scene groups across six environments')
    a=p.parse_args();print(json.dumps(convert(a.ledger,a.output,a.sensor_config,a.policy,a.serialized_root,a.point_cloud_range,a.environment_tag),indent=2))
