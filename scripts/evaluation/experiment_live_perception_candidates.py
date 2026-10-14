#!/usr/bin/env python3
"""Frozen live-probe QA. Never trains or modifies a production default.

Uses original ROS JPEG/XYZI and explicit scenario UID/path-checked catalog GT.
Simulator catalog geometry is a diagnostic reference, not native annotation
or a visibility oracle. Compound actor GT cannot adjudicate all component FPs.
"""
import argparse
from collections import Counter, defaultdict
import gzip
import hashlib
import json
import math
from pathlib import Path
import pickle
import sys
from types import SimpleNamespace

import numpy as np

from experiment_base5_proposal_peaks import ROOT, CAMERAS, bootstrap_source, digest


def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def prepare(args):
    if not math.isfinite(args.heading_correction_deg):
        raise ValueError('Finite explicit heading correction required')
    import rosbag
    sys.path[:0] = [str(ROOT/'docker/bevfusion'), str(ROOT/'perception/detection3d/bevfusion/tools'),
                   str(ROOT/'base_ws/src/tracking_prediction_evaluation/src')]
    from prepare_base5_rosbag_quick import nearest, xyzi, local_box, count_support, interpolate_scalar
    from app.sensor4_calibration import load_sensor4_calibration
    from tracking_prediction_evaluation.trajectory import read_morai_trajectories, interpolate_point
    from tracking_prediction_evaluation.pipeline_evaluator import read_ego_points, interpolate_ego
    from tracking_prediction_evaluation.bbox_reference import transform_segments_to_asset_catalog_bbox_center
    from audit_tracking_continuity import scenario_bindings
    from lidar_packet_reassembly import read_source, audit
    if args.output.exists():
        raise FileExistsError('Fresh output required')
    args.output.mkdir(parents=True)
    completed=[]
    for probe in args.probe:
        plan = json.loads((probe/'plan.json').read_text())
        progress = json.loads((probe/'progress.json').read_text())
        if len(progress['completed']) != len(plan['cases']):
            raise ValueError('Complete bounded raw probe required')
        completed.extend(progress['completed'])
    calibration = load_sensor4_calibration(args.sensor, ['camera_'+c for c in CAMERAS])
    if not np.allclose(calibration['base_from_lidar'][:3,:3], np.eye(3)):
        raise ValueError('Planar GT diagnostic requires the audited unrotated LiDAR mount')
    translation = calibration['base_from_lidar'][:3,3]
    rows, reports = [], []
    catalog = ROOT/'dataset/morai_od/configs/morai_asset_bbox_offsets_status_size_v2.json'
    for case in completed:
        folder = Path(case['folder']).resolve()
        bag = folder/'raw.bag'
        if digest(bag) != case['bag_sha256']:
            raise ValueError('Recorded source changed')
        source = read_source(bag)
        scene=json.loads((folder/'scenario.json').read_text())
        source_yaw=float(scene['egoVehicle']['initPosition']['rot']['yaw'])
        observed_yaw=float(source['messages']['/Ego_topic'][0][2].heading)
        residual=(observed_yaw+args.heading_correction_deg-source_yaw+180)%360-180
        if abs(residual)>.02:
            raise ValueError('Declared stationary heading reference contradicts saved scene yaw')
        observed = defaultdict(list)
        for _,_,msg in source['messages']['/Object_topic']:
            for cls, statuses in (('pedestrian',msg.pedestrian_list),('obstacle',msg.obstacle_list)):
                for status in statuses:
                    observed[f'{cls}:{status.unique_id}'].append(
                        ([float(getattr(status.position,k)) for k in 'xyz'],str(status.name)))
        bindings, companion = scenario_bindings(bag, observed)
        segments = read_morai_trajectories(bag, include_classes=('vehicle','pedestrian','obstacle'),
            heading_correction_deg=args.heading_correction_deg, asset_name_map={key:row['name'] for key,row in bindings.items()})
        corrected, resolution = transform_segments_to_asset_catalog_bbox_center(segments,catalog)
        unresolved = len(corrected) != len(segments)
        if unresolved and case['case']['scene'] != 'pet':
            raise ValueError('Unresolved catalog actors must not silently become negatives')
        ego = tuple(sorted(read_ego_points(bag,'/Ego_topic',(0,0,0),0,args.heading_correction_deg),key=lambda r:r.timestamp))
        if not ego:
            raise ValueError('Ego pose required')
        eligible, skipped = [], Counter()
        lidar = source['messages']['/sensor/lidar1']
        for raw_index,(stamp,receipt,cloud) in enumerate(lidar):
            if not lidar[0][0]+1 <= stamp <= lidar[-1][0]-1:
                skipped['edge'] += 1; continue
            selected = [nearest([(t,m) for t,_,m in source['messages'][f'/sensor/camera/{c}/compressed']],stamp,.05) for c in CAMERAS]
            state = interpolate_ego(ego,stamp,.15)
            height = interpolate_scalar([(t,m) for t,_,m in source['messages']['/Ego_topic']],stamp,.15)
            if state is None or height is None or any(v is None for v in selected):
                skipped['pose_or_camera_sync'] += 1; continue
            eligible.append((stamp,receipt,cloud,selected,state,height,raw_index))
        # Keep the entire contiguous sequence; no frame subsampling around switches.
        for stamp,receipt,cloud,selected,state,height,raw_index in eligible:
            token = f'{folder.name}_{raw_index:05d}'
            dest = args.output/'data'/token
            dest.mkdir(parents=True)
            points = xyzi(cloud)
            point_path = dest/'points.bin'
            point_path.write_bytes(points.astype('<f4').tobytes())
            finite = points[np.isfinite(points).all(axis=1) & (points[:,:3]!=0).any(axis=1)]
            images, offsets = {}, {}
            for c,(t,msg) in zip(CAMERAS,selected):
                path = dest/(c+'.jpg'); data = bytes(msg.data)
                if not data.startswith(b'\xff\xd8'):
                    raise ValueError('Original native JPEG required')
                path.write_bytes(data); images['image/camera_'+c] = str(path.resolve()); offsets[c] = t-stamp
            boxes,labels,assets,keys,support = [],[],[],[],[]
            for segment in corrected:
                point = interpolate_point(segment.points,stamp,.15)
                if point is None:
                    raise ValueError('Actor trajectory interpolation unavailable inside retained sequence')
                box = local_box(point,state,height,translation)
                if abs(box[0])>100 or abs(box[1])>30:
                    continue
                support.append(count_support(finite,box)); box[2] -= box[5]*.5
                boxes.append(box); labels.append(('vehicle','pedestrian','obstacle').index(segment.class_name))
                assets.append(segment.asset_name); keys.append(segment.key)
            rows.append(dict(token=token,source='dev',case=folder.name,policy=case['case']['policy'],
                scene=case['case']['scene'],stamp=stamp,receipt=receipt,raw_lidar_index=raw_index,
                point_path=str(point_path.resolve()),point_sha256=digest(point_path),images=images,
                image_sha256={k:digest(p) for k,p in images.items()},camera_offsets_sec=offsets,
                gt_boxes=boxes,gt_labels=labels,gt_points=support,gt_assets=assets,parent_uids=keys,
                actor_geometry_resolved=not unresolved,
                ego=dict(x=state.x,y=state.y,z=height,yaw=state.yaw),lidar_translation=translation.tolist(),
                heading_correction_deg=args.heading_correction_deg,
                box_center_convention='Export bottom; catalog geometric center before z -= h/2. Heading matches saved scene yaw; recorded default GT TF is not used as a physical-yaw oracle.'))
        report = audit(bag,source)
        report.update(case=folder.name,bag_sha256=digest(bag),selected=len(eligible),skipped=dict(skipped),
                      scenario_bindings=bindings,scenario_companion=companion,catalog_resolution=resolution,
                      actor_geometry_resolved=not unresolved,
                      unresolved_note='Compound status pivot is not native person/pet component GT; excluded from GT correspondence.' if unresolved else None)
        reports.append(report)
        print(json.dumps(dict(case=folder.name,frames=len(eligible),actors=len(corrected))),flush=True)
    save(args.output/'inputs.json',dict(rows=rows,cases=dict(Counter(r['case'] for r in rows)),
        sensor_sha256=digest(args.sensor),catalog_sha256=digest(catalog),
        scope='Original continuous raw inputs. Parked Ego only. Not AP or moving/closed-loop certification. Compound actor GT is not component annotation.'))
    save(args.output/'packet_audit.json',reports)


def infer(args):
    if args.output.exists():
        raise FileExistsError('Fresh inference output required')
    bootstrap_source()
    import cv2
    import torch
    from app.bevfusion_model import BevFusionModel
    from app.sensor4_calibration import load_sensor4_calibration
    from perception.detection3d.bevfusion.plugin.morai_separated_depth_lss import MoraiSeparatedRectangularDepthLSSTransform
    torch.set_num_threads(2); cv2.setNumThreads(1)
    args.output.mkdir(parents=True)
    inputs = json.loads(args.inputs.read_text())
    calibration = load_sensor4_calibration(args.sensor,['camera_'+c for c in CAMERAS])
    model = BevFusionModel(str(args.config),str(args.checkpoint),calibration,use_amp=True,
        preprocess_workers=4,warmup_iterations=2,base5_runtime=True,parallel_camera=True,point_intensity_source='ros255')
    transform = model.model.view_transform
    nearest = transform.forward
    legacy = MoraiSeparatedRectangularDepthLSSTransform.forward.__get__(transform,type(transform))
    modes = tuple(args.modes)
    buffers = {mode:[] for mode in modes}
    try:
        for index,row in enumerate(inputs['rows']):
            if digest(row['point_path'])!=row['point_sha256']:
                raise ValueError('Frozen points changed')
            images = {k:Path(p).read_bytes() for k,p in row['images'].items()}
            if any(hashlib.sha256(v).hexdigest()!=row['image_sha256'][k] for k,v in images.items()):
                raise ValueError('Frozen JPEG changed')
            points = np.fromfile(row['point_path'],'<f4').reshape(-1,4)
            for mode in modes:
                transform.forward = legacy if mode.startswith('legacy') else nearest
                transform.depth_raster_policy = None if mode.startswith('legacy') else 'nearest_positive'
                transform.depth_raster_backend = 'sorted'
                predictions,prep,latency = model.infer(points,images,index,0.,200,timestamp_sec=row['stamp'])
                buffers[mode].append(dict(row,mode=mode,pred_boxes=[p['center']+p['size']+[p['yaw']] for p in predictions],
                    pred_labels=[p['label'] for p in predictions],pred_scores=[p['score'] for p in predictions],
                    query_scores=[],preprocess_ms=prep,inference_ms=latency))
            if (index+1)%50==0:
                print(json.dumps(dict(frames=index+1,total=len(inputs['rows']),gpu_memory_mib=torch.cuda.max_memory_allocated()/1024**2)),flush=True)
        for mode,rows in buffers.items():
            with gzip.open(args.output/(mode+'.json.gz'),'wt') as stream:
                json.dump(dict(rows=rows,input_sha256=digest(args.inputs),checkpoint_sha256=digest(args.checkpoint)),stream)
        save(args.output/'receipt.json',dict(input_sha256=digest(args.inputs),config_sha256=digest(args.config),
            checkpoint_sha256=digest(args.checkpoint),frames=len(inputs['rows']),compile=False,
            gpu=torch.cuda.get_device_name(),max_memory_mib=torch.cuda.max_memory_allocated()/1024**2,
            scope='Independent complete forwards with exact inputs. MORAI concurrently running: latency is not target4090 benchmark.'))
    finally:
        model.preprocessor.close()


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command',required=True)
    p=commands.add_parser('prepare'); p.add_argument('--probe',type=Path,action='append',required=True)
    p.add_argument('--output',type=Path,required=True); p.add_argument('--sensor',type=Path,required=True)
    p.add_argument('--heading-correction-deg',type=float,default=-1.347)
    p=commands.add_parser('infer')
    for key in ('inputs','output','sensor','config','checkpoint'):
        p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--modes',nargs='+',choices=('legacy_A','legacy_B','nearest_A','nearest_B'),
                   default=('legacy_A','legacy_B','nearest_A','nearest_B'))
    args=parser.parse_args()
    {'prepare':prepare,'infer':infer}[args.command](args)
