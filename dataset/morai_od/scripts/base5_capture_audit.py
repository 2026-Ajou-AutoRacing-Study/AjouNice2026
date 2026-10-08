"""Component-preserving synchronous GT audit. No Object_topic box reconstruction."""
import ast
import copy
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


def camera_evidence(root, uid, raw_class, pixels_cache=None):
    evidence = []
    for folder in sorted(Path(root).glob('CAMERA_*')):
        images = list(folder.glob('*_Instance.png'))
        if not images:
            continue
        if pixels_cache is not None and images[0] in pixels_cache:
            pixels = pixels_cache[images[0]]
        else:
            with Image.open(images[0]) as image:
                pixels = np.asarray(image.convert('RGBA'))
            if pixels_cache is not None:
                pixels_cache[images[0]] = pixels
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


def audit_capture(root, metadata, cache_camera_pixels=True):
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
    zero_returns=int((points[:,:3]==0).all(axis=1).sum())
    points=points[(points[:,:3]!=0).any(axis=1)]
    actors = {int(a['unique_id']):a for a in metadata['runtime_actors']}
    boxes = parse_instances(annotation_files[0])
    # parse_instances is permissive historically; here malformed records are fatal.
    if len(boxes) != len([s for s in annotation_files[0].read_text().splitlines() if s.strip()]):
        raise ValueError('Malformed GT record')
    results = []
    pixels_cache = {} if cache_camera_pixels else None
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
        cameras = camera_evidence(root,uid,box['raw_class'],pixels_cache)
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
    ground=None
    ground_config=metadata.get('ground_support_check',{})
    from base5_static_ground_policy import lightweight_static, static_decision
    light_static=lightweight_static(metadata)
    from base5_intersection_ground import explicit_manual_mode, ground_decision
    manual_ground=explicit_manual_mode(metadata)
    if ground_config.get('enabled'):
        if metadata.get('lidar_to_ego') is None:reasons.append('ground_probe_missing_calibration')
        else:
            from base5_ground_support import ground_review
            ground=ground_review(points[:,:3],boxes,metadata['lidar_to_ego'],
                ground_config.get('max_range_m',20.),ground_config.get('maximum_gap_m',.15),
                references=None if light_static else metadata.get('ground_probe_references'),runtime_ego=metadata.get('capture_runtime_ego'),
                required_uids=ground_config.get('required_uids'))
            if metadata.get('static_ground_references') and not light_static:
                from base5_static_ground_reference import apply_reference,ReferenceQualityError
                try:apply_reference(ground,boxes,root,metadata)
                except ReferenceQualityError as exc:
                    ground['static_reference_quality_error']=str(exc)
                    reasons.append('static_ground_reference_pose_or_view_mismatch')
            if manual_ground:
                ground['manual_policy_decision']=ground_decision(ground)
                reasons.extend(ground['manual_policy_decision']['review_reasons'])
            elif light_static:
                ground['static_policy_decision']=static_decision(ground)
                reasons.extend(ground['static_policy_decision']['review_reasons'])
            else:
                reasons.extend(ground['review_reasons'])
                from base5_ground_support import height_response_review
                ground['height_response']=height_response_review(ground,metadata)
                reasons.extend(ground['height_response']['review_reasons'])
    live=metadata.get('capture_safety')
    if live is not None:
        if not live.get('collision',{}).get('observed'):reasons.append('collision_history_incomplete')
        if live.get('collision',{}).get('non_ego_uids'):reasons.append('physical_collision_during_capture_window')
        if not live.get('support',{}).get('stationary_support_observed'):reasons.append('ego_vertical_support_unstable')
    if metadata.get('avoidance_observation'):
        # A planned positive OBB gap does not certify paved ground or a physical collider.
        # Keep these new layouts in review until an independently recorded live check exists.
        check=metadata.get('avoidance_live_review',{})
        live=metadata.get('capture_safety',{})
        operational=(live.get('collision',{}).get('no_collision_verified') and
            live.get('support',{}).get('stationary_support_observed') and live.get('evidence_path'))
        if not (operational or (check.get('ground_support_verified') and check.get('no_collision_verified') and check.get('evidence_path'))):
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
    clearances=[]
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
            clearances.append(dict(parent_uid=box['unique_id'],annotation_surface_gap_m=poly.distance(ego),
                annotation_overlap_area_m2=poly.intersection(ego).area))
            # Annotation margins can overlap without a physical collision. Keep the
            # conservative geometric gate unless live collision history was observed.
            if poly.distance(ego)<.15 and not metadata.get('capture_safety',{}).get('collision',{}).get('no_collision_verified'):
                reasons.append('actual_GT_ego_footprint_clearance')
            if any(uid!=box['unique_id'] and poly.distance(other)<.05 for uid,other in footprints):
                reasons.append('actual_GT_actor_footprint_overlap')
            footprints.append((box['unique_id'],poly))
    from base5_population_budget import visibility_summary,visibility_review
    population_visibility=visibility_summary(metadata['runtime_actors'],results)
    reasons.extend(visibility_review(metadata,population_visibility))
    from base5_roundabout_mission import visibility_review as roundabout_visibility_review
    pair_reasons,roundabout_visibility=roundabout_visibility_review(metadata,results,clearances)
    reasons.extend(pair_reasons)
    from base5_shadow_observation import visibility_review as shadow_visibility_review
    shadow_reasons, shadow_visibility = shadow_visibility_review(metadata, results)
    reasons.extend(shadow_reasons)
    from base5_static_vehicle import runtime_review as static_vehicle_review
    static_vehicle_reasons,static_vehicle_evidence=static_vehicle_review(metadata)
    reasons.extend(static_vehicle_reasons)
    result=dict(components=results,missing_uids=missing,raw_complete=True,ground_support=ground,
        ground_warnings=(ground or {}).get('static_policy_decision',{}).get('warnings',[]),
        population_visibility=population_visibility,
        zero_xyz_returns_excluded_from_audit=zero_returns,valid_lidar_return_count=len(points),
        actual_annotation_clearances=clearances,
        accepted_candidate=not reasons,review_reasons=sorted(set(reasons)),
        positive_policy='provisional lidar1 OR actual instance pixels in model crop',
        limitations=['With calibration: Ego-frame distance, 0.5m audit tolerance; legacy without calibration: LiDAR frame, 2.5m tolerance.',
                    'Instance pixels are simulator labels, not learned detector evidence.',
                    'No automated mesh ground-support/collider certification.'])
    # Ground-only re-audits call this directly, without the runner's later
    # overlay. Keep the original observed safety proof available to selectors
    # and dashboard summaries; never synthesize a successful collision check.
    if live is not None:result['capture_safety']=copy.deepcopy(live)
    if roundabout_visibility is not None:result['roundabout_visibility']=roundabout_visibility
    if shadow_visibility is not None:result['shadow_observation_visibility']=shadow_visibility
    if static_vehicle_evidence is not None:result['stationary_vehicle_runtime']=static_vehicle_evidence
    return result
