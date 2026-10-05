"""Component-preserving synchronous GT audit. No Object_topic box reconstruction."""
import ast
import math
from pathlib import Path

import numpy as np
from PIL import Image
from audit_morai_lidar_gt_visibility import parse_instances


def annotation_class(raw_class):
    if raw_class.lower() in {'vehicle','sedan','suv','truck','bus','mpv','wagon','car'}:
        return 'vehicle'
    return {'Pedestrian':'pedestrian','Obstacle':'obstacle'}.get(raw_class)


def rotation_matrix(rpy):
    r,p,y=map(float,rpy);cr,sr=math.cos(r),math.sin(r);cp,sp=math.cos(p),math.sin(p);cy,sy=math.cos(y),math.sin(y)
    return np.asarray([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
        [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])


def camera_evidence(root, uid, raw_class):
    evidence = []
    for folder in sorted(Path(root).glob('CAMERA_*')):
        images = list(folder.glob('*_Instance.png'))
        if not images:
            continue
        pixels = np.asarray(Image.open(images[0]).convert('RGBA'))
        height, width = pixels.shape[:2]
        for annotation in folder.glob('*_BBox2D.txt'):
            for line in annotation.read_text().splitlines():
                fields = line.split(maxsplit=6)
                if len(fields) != 7:
                    continue
                candidate = int(fields[5])
                # MORAI child IDs encode -(parent*100+component ordinal).
                if candidate != uid and not (candidate < 0 and abs(candidate)//100 == uid):
                    continue
                if fields[0] != raw_class:
                    continue
                colour = ast.literal_eval(fields[6])
                if len(colour) != 4:
                    continue
                xmin, ymin, xmax, ymax = map(float, fields[1:5])
                mask = (pixels == np.asarray(colour)).all(axis=2)
                # Count actual instance pixels in the model's deterministic bottom crop.
                scale = 896./width
                top = max(0, int(height-320./scale))
                count = int(mask.sum())
                native_crop_count = int(mask[top:height].sum())
                resized=np.asarray(Image.fromarray(mask).resize((896,round(height*scale)),resample=Image.NEAREST))
                crop_count=int(resized[-320:].sum())
                evidence.append(dict(camera=folder.name, camera_instance_id=candidate,
                    annotation_class=fields[0], native_rect=[xmin,ymin,xmax,ymax],
                    native_instance_pixels=count, crop_320x896_instance_pixels=crop_count,
                    native_bottom_crop_instance_pixels=native_crop_count,
                    model_extent_px=[max(0,xmax-xmin)*scale,
                        max(0,min(ymax,height)-max(ymin,top))*scale],
                    matching='parent+raw_class; same-class components may be ambiguous'))
    return evidence


def audit_capture(root, metadata):
    root = Path(root)
    lidar_files = list(root.glob('LIDAR_*/*_Intensity.bin'))
    annotation_files = list(root.glob('LIDAR_*/*_Instance.txt'))
    if len(lidar_files) != 1 or len(annotation_files) != 1:
        raise ValueError('Expected exactly one synchronous LiDAR capture')
    raw = np.fromfile(lidar_files[0], dtype=np.float32)
    if raw.size == 0 or raw.size % 4:
        raise ValueError('Incomplete XYZI')
    points = raw.reshape(-1,4)
    if not np.isfinite(points).all():
        raise ValueError('Nonfinite LiDAR')
    actors = {int(a['unique_id']):a for a in metadata['runtime_actors']}
    boxes = parse_instances(annotation_files[0])
    # parse_instances is permissive historically; here malformed records are fatal.
    if len(boxes) != len([s for s in annotation_files[0].read_text().splitlines() if s.strip()]):
        raise ValueError('Malformed GT record')
    results = []
    ordinals = {}
    for box in boxes:
        uid = box['unique_id']
        if uid not in actors:
            raise ValueError('Unmapped captured UID '+str(uid))
        actor = actors[uid]
        center, size = box['center'], box['size']
        if not np.isfinite(np.r_[center,size,box['rotation_rpy']]).all() or (size<=0).any():
            raise ValueError('Invalid GT geometry')
        rot = rotation_matrix(box['rotation_rpy'])
        local = (points[:,:3]-center) @ rot
        count = int((np.abs(local)<=size/2+1e-5).all(axis=1).sum())
        cameras = camera_evidence(root,uid,box['raw_class'])
        ambiguous=sum(b['unique_id']==uid and b['raw_class']==box['raw_class'] for b in boxes)>1
        visible = not ambiguous and any(c['crop_320x896_instance_pixels']>=16 and min(c['model_extent_px'])>=4 for c in cameras)
        key = (uid,box['raw_class']); ordinal = ordinals.get(key,0); ordinals[key]=ordinal+1
        cls = 'animal' if (actor['training_class']=='animal' or
            (actor['name'].startswith('Ped_Pet') and box['raw_class']=='Obstacle')) else {
            'Vehicle':'vehicle','Pedestrian':'pedestrian','Obstacle':'obstacle'}.get(box['raw_class'],annotation_class(box['raw_class']))
        if cls is None:
            raise ValueError('Unmapped GT semantic '+box['raw_class'])
        results.append(dict(parent_uid=uid,component_id=f'{uid}:{box["raw_class"]}:{ordinal}',
            asset=actor['name'],data_id=actor['data_id'],raw_class=box['raw_class'],training_class=cls,
            center_lidar_m=center.tolist(),size_m=size.tolist(),rotation_rpy=box['rotation_rpy'],
            range_xy_m=float(np.linalg.norm(center[:2])),lidar_points=count,cameras=cameras,
            camera_visible_candidate=visible,camera_component_ambiguous=ambiguous,
            positive_lidar5=count>=5,positive_lidar1=count>=1,
            positive_lidar1_camera0=count>=1 or visible,
            ignore_reason=None if count>=1 or visible else 'no_visible_sensor_evidence'))
        if metadata.get('lidar_to_ego') is not None:
            center_ego=np.asarray(metadata['lidar_to_ego'])@np.r_[center,1.]
            results[-1]['center_ego_m']=center_ego[:3].tolist()
            results[-1]['range_ego_xy_m']=float(np.linalg.norm(center_ego[:2]))
    missing = sorted(set(actors)-{b['unique_id'] for b in boxes})
    primary_uid=metadata['runtime_actors'][0]['unique_id'] if metadata['runtime_actors'] else None
    primary = [r for r in results if r['parent_uid']==primary_uid]
    background = metadata['spec']['family']=='background'
    # Raw is retained even when primary is invisible. Such frames are NOT silently accepted as empty.
    supported = background or any(r['positive_lidar1_camera0'] for r in primary)
    reasons=[]
    if metadata.get('avoidance_observation'):
        # A planned positive OBB gap does not certify paved ground or a physical collider.
        # Keep these new layouts in review until an independently recorded live check exists.
        check=metadata.get('avoidance_live_review',{})
        if not (check.get('ground_support_verified') and check.get('no_collision_verified') and check.get('evidence_path')):
            reasons.append('avoidance_ground_collider_review_required')
    if missing: reasons.append('missing_actor_annotation')
    if not supported: reasons.append('primary_no_sensor_evidence')
    if not background and primary:
        lo,hi=metadata['spec']['distance_bounds']
        tolerance=.5 if metadata.get('lidar_to_ego') is not None else 2.5
        if not any(lo-tolerance<=r.get('range_ego_xy_m',r['range_xy_m'])<hi+tolerance for r in primary):
            reasons.append('primary_actual_distance_bin_mismatch')
    # Actual capture footprint gate is independent of the placement size estimate.
    # Lidar-to-Ego comes from the explicit sensor calibration, never raw Object_topic.size.
    if metadata.get('lidar_to_ego') is not None:
        from shapely.geometry import Polygon
        transform=np.asarray(metadata['lidar_to_ego'])
        ego=Polygon([(-1.05,-1.),(-1.05,1.),(3.75,1.),(3.75,-1.)])
        footprints=[]
        for box in boxes:
            signs=np.asarray([[-1,-1],[1,-1],[1,1],[-1,1]])
            local=np.c_[signs*np.asarray(box['size'][:2])/2,np.zeros(4)]
            corners=local@rotation_matrix(box['rotation_rpy']).T+box['center']
            corners=(np.c_[corners,np.ones(4)]@transform.T)[:,:2]
            poly=Polygon(corners)
            if poly.distance(ego)<.15: reasons.append('actual_GT_ego_footprint_clearance')
            if any(uid!=box['unique_id'] and poly.distance(other)<.05 for uid,other in footprints):
                reasons.append('actual_GT_actor_footprint_overlap')
            footprints.append((box['unique_id'],poly))
    return dict(components=results,missing_uids=missing,raw_complete=True,
        accepted_candidate=not reasons,review_reasons=sorted(set(reasons)),
        positive_policy='provisional lidar1 OR actual instance pixels in model crop',
        limitations=['With calibration: Ego-frame distance, 0.5m audit tolerance; legacy without calibration: LiDAR frame, 2.5m tolerance.',
                    'Instance pixels are simulator labels, not learned detector evidence.',
                    'No automated mesh ground-support/collider certification.'])
