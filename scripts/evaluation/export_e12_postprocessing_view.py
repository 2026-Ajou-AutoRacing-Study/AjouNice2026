#!/usr/bin/env python3
"""Export cached E12 detections/core tracks and real 3/5s Map-CV for RViz.

Offline CPU/bag I/O only: no detector, CUDA, ROS master, publisher, TF, planner,
controller or MORAI. Marker headers use original BAG RECEIPT time solely for
visualization lifetime. Geometry uses the original measurement-time LiDAR pose.
"""
import argparse
from collections import defaultdict
import gzip
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys

import numpy as np
import yaml

from analyze_e12_map_horizon import (ROOT, CORE_PATH, NODE_PATH, E12_SHA, prepare_case,
    current_assignment, future_reference, prefix_extension_index, key_tuple, digest)
from audit_vehicle_map_prediction import interpolate
from prepare_e12_postprocessing import transform_for_pose

BASE = ROOT/'artifacts/evaluation/bevfusion/e12_postprocessing_261023_v1'
CASES = dict(H04='september/H04_162616', H03='core/H03_162301', R01='core/R01_163034')
TOPICS = ['/e12_view/'+name for name in ('gt','detections','tracks','velocity',
    'prediction3','prediction5','prefix5','gt_future','hud')]
COLORS = dict(gt=(1.,0.,.8), detections=(.1,1.,.2), tracks=(0.,.9,1.),
    coast=(1.,.45,.05), velocity=(.1,1.,1.), prediction3=(1.,1.,.05),
    prediction5=(1.,.4,.05), prefix5=(.15,.35,1.), gt_future=(1.,1.,1.), hud=(1.,1.,1.))


def json_load(path):
    with (gzip.open if str(path).endswith('.gz') else open)(path,'rt') as stream:
        return json.load(stream)


def sensor_inverse(frame, extrinsic):
    return np.linalg.inv(transform_for_pose([frame['ego'][key] for key in ('x','y','z','yaw')], extrinsic))


def to_sensor(point, inverse):
    return (inverse @ np.asarray([*point,1.],float))[:3].tolist()


def box_corners(obj, inverse):
    # All pack centers are already gravity centers. Never add height/2 here.
    x,y,z = obj['center']; length,width,height = obj['size']
    c,s = math.cos(obj['yaw']),math.sin(obj['yaw'])
    return [to_sensor([x+c*a-s*b,y+s*a+c*b,z+dz],inverse)
            for dz in (-height/2,height/2)
            for a,b in ((-length/2,-width/2),(length/2,-width/2),(length/2,width/2),(-length/2,width/2))]


def align_receipts(frames, source):
    """Exact floating stamp identity as originally saved from to_sec()."""
    result=[]
    for frame in frames:
        matches=source.get(float(frame['stamp']),[])
        if len(matches)!=1:
            raise ValueError('Missing/ambiguous original LiDAR HEADER match for '+frame['token'])
        record=matches[0]
        if record['frame_id'].lstrip('/')!='lidar1':
            raise ValueError('Viewer requires actual raw lidar1 frame')
        result.append(dict(record))
    if any(b['receipt_nsec']<=a['receipt_nsec'] for a,b in zip(result,result[1:])):
        raise ValueError('Matched raw LiDAR receipts are not strictly increasing')
    return result


def read_receipts(raw,frames):
    import rosbag
    wanted={float(frame['stamp'])for frame in frames}; source=defaultdict(list)
    with rosbag.Bag(str(raw),'r') as bag:
        start=float(bag.get_start_time())
        for _,message,receipt in bag.read_messages(topics=['/sensor/lidar1']):
            header=float(message.header.stamp.to_sec())
            if header in wanted:
                source[header].append(dict(receipt_nsec=int(receipt.to_nsec()),
                    source_header_nsec=int(message.header.stamp.to_nsec()),
                    frame_id=message.header.frame_id))
    return align_receipts(frames,source),start


def load_predictor(parameters,map_file,projector):
    """Same actual node/math as the horizon evaluator, preserving XYZ too."""
    sys.path.insert(0,str(ROOT/'base_ws/src/perception_object_pipeline/src'))
    import perception_object_pipeline.map_based_prediction as core
    from perception_object_pipeline.kinematic_prediction import yaw_to_quaternion
    from autoware_perception_msgs.msg import ObjectClassification,TrackedObject,TrackedObjects
    spec=importlib.util.spec_from_file_location('e12_view_actual_map_node',NODE_PATH)
    node=importlib.util.module_from_spec(spec);spec.loader.exec_module(node)
    cfg=core.MapPredictionConfig(**{k:v for k,v in parameters.items()if k in core.MapPredictionConfig.__dataclass_fields__})
    loaded=core.LaneletNetwork.load(map_file,projector,cfg)
    networks={h:core.LaneletNetwork(tuple(loaded.records.values()),cfg,
        map_sha256=loaded.map_sha256,lanelet_layer=loaded._lanelet_layer)for h in (3,5)}

    def predict(frame,horizon):
        message=TrackedObjects();message.header.frame_id='map'
        for obj in frame['objects']:
            tracked=TrackedObject();tracked.classification=[ObjectClassification(label=obj['label'],probability=1.)]
            pose=tracked.kinematics.pose_with_covariance.pose
            pose.position.x,pose.position.y,pose.position.z=obj['center']
            pose.orientation=yaw_to_quaternion(obj['yaw'])
            twist=tracked.kinematics.twist_with_covariance.twist
            twist.linear.x,twist.linear.y=obj['body_velocity'];twist.angular.z=obj['yaw_rate']
            message.objects.append(tracked)
        network=networks[horizon];original=network.predict;routes=[]
        def observed(*a,**kw):
            value=original(*a,**kw);routes.append(value);return value
        network.predict=observed
        try:
            output,_,_,_=node.make_map_based_predicted_objects(message,
                [i/10. for i in range(1,horizon*10+1)],network)
        finally:
            network.predict=original
        result=[]
        if len(output.objects)!=len(frame['objects'])or len(routes)!=len(frame['objects']):
            raise ValueError('Actual predictor changed object count/order')
        for obj,trajectories in zip(output.objects,routes):
            paths=[]
            for i,path in enumerate(obj.kinematics.predicted_paths):
                xyz=[[p.position.x,p.position.y,p.position.z]for p in path.path[1:]]
                if len(xyz)!=horizon*10:raise ValueError('Incomplete actual future path')
                paths.append(dict(xyz=xyz,xy=[p[:2]for p in xyz],confidence=float(path.confidence),
                    lanelet_ids=list(trajectories[i].lanelet_ids)if trajectories else []))
            result.append(dict(mapped=bool(trajectories),paths=paths))
        return result
    return predict,loaded.map_sha256


class Markers:
    def __init__(self,stamp,frame='lidar1'):
        import rospy
        from geometry_msgs.msg import Point
        from visualization_msgs.msg import Marker,MarkerArray
        self.Marker,self.Point=Marker,Point;self.stamp,self.frame=stamp,frame
        self.lifetime=rospy.Duration(.35)
        self.topics={topic:MarkerArray()for topic in TOPICS}
        for topic,array in self.topics.items():
            clear=Marker();clear.header.frame_id=frame;clear.header.stamp=stamp
            clear.action=Marker.DELETEALL;array.markers.append(clear)

    def new(self,kind,marker_type,color,alpha=1.,width=.08):
        topic='/e12_view/'+kind;array=self.topics[topic];marker=self.Marker()
        marker.header.frame_id=self.frame;marker.header.stamp=self.stamp
        marker.ns='e12_'+kind;marker.id=len(array.markers);marker.type=marker_type
        marker.action=self.Marker.ADD;marker.pose.orientation.w=1.;marker.lifetime=self.lifetime
        marker.color.r,marker.color.g,marker.color.b=color;marker.color.a=alpha
        marker.scale.x=width;array.markers.append(marker);return marker

    def text(self,kind,point,text,color,scale=.4):
        marker=self.new(kind,self.Marker.TEXT_VIEW_FACING,color)
        marker.pose.position=self.Point(*point);marker.scale.z=scale;marker.text=text
        return marker

    def box(self,kind,obj,inverse,text,color,width=.05):
        marker=self.new(kind,self.Marker.LINE_LIST,color,width=width)
        corners=box_corners(obj,inverse)
        for a,b in ((0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)):
            marker.points.extend([self.Point(*corners[a]),self.Point(*corners[b])])
        p=to_sensor(obj['center'],inverse);p[2]+=obj['size'][2]/2+.2
        self.text(kind,p,text,color,.32)

    def line(self,kind,points,color,alpha=1.,width=.09):
        marker=self.new(kind,self.Marker.LINE_STRIP,color,alpha,width)
        marker.points=[self.Point(*p)for p in points];return marker

    def arrow(self,origin,endpoint,color):
        marker=self.new('velocity',self.Marker.ARROW,color,width=.10)
        marker.scale.y=.25;marker.scale.z=.32
        marker.points=[self.Point(*origin),self.Point(*endpoint)]


def marker_frame(frame,normalized,predictions,trajectories,map_to_world,extrinsic,receipt_nsec,case,index,total):
    import rospy
    receipt=rospy.Time(receipt_nsec//10**9,receipt_nsec%10**9)
    result=Markers(receipt);inverse=sensor_inverse(frame,extrinsic)
    map_to_sensor=inverse@map_to_world
    for obj in frame['gt']:
        result.box('gt',obj,inverse,f"GT {obj['class_name']} pts={obj['lidar_points']}",COLORS['gt'],.04)
    for obj in frame['detections']:
        result.box('detections',obj,inverse,f"D {obj['class_name']} {obj['score']:.2f}",COLORS['detections'],.035)
    for obj in frame['tracks']:
        if not obj['confirmed']:continue
        age=frame['stamp']-obj['birth_stamp'];coast=frame['stamp']-obj['last_measurement_stamp']
        speed=math.hypot(*obj['velocity']);color=COLORS['tracks'if obj['associated']else 'coast']
        label=f"T{obj['id']} {obj['class_name']} v={speed:.1f} age={age:.1f}s"
        if not obj['associated']:label+=f" COAST {coast:.2f}s"
        result.box('tracks',obj,inverse,label,color,.075)
        if obj['label']!=2 and speed>.05:
            origin=to_sensor(obj['center'],inverse)
            endpoint=to_sensor([obj['center'][0]+.5*obj['velocity'][0],
                obj['center'][1]+.5*obj['velocity'][1],obj['center'][2]],inverse)
            result.arrow(origin,endpoint,color)
    for object_index,obj in enumerate(normalized['objects']):
        for horizon in (3,5):
            output=predictions[horizon][object_index]
            for rank,path in enumerate(output['paths']):
                points=[to_sensor(obj['center'],map_to_sensor)]+[to_sensor(p,map_to_sensor)for p in path['xyz']]
                result.line('prediction'+str(horizon),points,COLORS['prediction'+str(horizon)],
                    1. if rank==0 else .18,.14 if horizon==3 and rank==0 else .075)
            end=to_sensor(output['paths'][0]['xyz'][-1],map_to_sensor)
            result.text('prediction'+str(horizon),end,f"{horizon}s T{obj['id']}",COLORS['prediction'+str(horizon)],.36)
        extended=prefix_extension_index(predictions[3][object_index],predictions[5][object_index])
        if extended is not None:
            path=predictions[5][object_index]['paths'][extended]
            result.line('prefix5',[to_sensor(obj['center'],map_to_sensor)]+
                [to_sensor(p,map_to_sensor)for p in path['xyz']],COLORS['prefix5'],.85,.06)
    # Ground-truth futures are references only, never path-selection inputs.
    for truth in normalized['gt']:
        key=key_tuple(truth['key']);series=trajectories.get(key)
        if series is None:continue
        reference,reason=future_reference(series,frame['stamp'],5)
        if reason is not None:continue
        xyz=[interpolate(series[:2],frame['stamp']+i/10.,.15)for i in range(1,51)]
        points=[to_sensor(truth['center'],map_to_sensor)]+[to_sensor(p,map_to_sensor)for p in xyz]
        result.line('gt_future',points,COLORS['gt_future'],.8,.045)
    status='ELIGIBLE'if frame['scoring_eligible']else 'INELIGIBLE GT epoch; view only'
    receipt_sec=receipt_nsec/1e9
    hud=(f"E12 {case} {index+1}/{total} | {status}\n"
         f"measurement={frame['stamp']:.6f} receipt={receipt_sec:.6f} age={receipt_sec-frame['stamp']:.3f}s\n"
         "GT magenta | detections green | tracks cyan / coast orange | arrows=0.5s velocity\n"
         "future: 3s yellow / 5s orange | optional prefix5 blue / valid GT future white\n"
         "Saved core replay; not live GPU, 30Hz scheduler, full-stack or safety certification")
    result.text('hud',[0.,-16.,4.],hud,COLORS['hud'],.5)
    return result.topics


def frozen_index(path,case):
    rows=json_load(path)
    result={}
    for row in rows:
        if row['case']==case and row['output_horizon_s']in(3,5):
            key=(row['stamp'],str(row['id']),row['output_horizon_s'],row['horizon_s'])
            if key in result:raise ValueError('Duplicate frozen endpoint key')
            result[key]=row['top1_endpoint']
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--case',choices=CASES,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--pack',type=Path,default=BASE/'tracker_v1/replay_pack.json.gz')
    p.add_argument('--frozen-horizon',type=Path,default=BASE/'map_horizon/map_horizon_v4')
    a=p.parse_args()
    if a.output.exists():raise FileExistsError('Fresh export directory required')
    pack=json_load(a.pack);case=pack['cases'][CASES[a.case]]
    if not pack.get('complete')or case['provenance']['recorded_model']['checkpoint_sha256']!=E12_SHA:
        raise ValueError('Complete frozen E12 pack required')
    sensor=ROOT/'configs/sensors/Sensor_competition_lan.json'
    parameters_path=ROOT/'base_ws/src/perception_object_pipeline/config/map_based_prediction.yaml'
    transform_path=ROOT/'configs/tf/morai_lanelet_static_tf.yaml'
    map_file=ROOT/'kcity_map/lanelet_map/lanelet2_map_v85.osm';projector=ROOT/'kcity_map/lanelet_map/map_projector_info_v1.yaml'
    source_paths=[Path(__file__),ROOT/'scripts/evaluation/analyze_e12_map_horizon.py',CORE_PATH,NODE_PATH,
        sensor,parameters_path,transform_path,map_file,projector,a.pack,a.frozen_horizon/'report.json',a.frozen_horizon/'rows.json.gz']
    pins={str(path):digest(path)for path in source_paths}
    frozen_report=json_load(a.frozen_horizon/'report.json')
    # Reports may have been created with a repository-relative pack path.
    if not any(Path(path).resolve()==a.pack.resolve() and value==digest(a.pack)
            for path,value in frozen_report['source_sha256'].items()):
        raise ValueError('Frozen horizon pack differs')
    for path in (CORE_PATH,NODE_PATH,parameters_path,transform_path,map_file,projector):
        expected=next((sha for name,sha in frozen_report['source_sha256'].items()
                       if Path(name).resolve()==path.resolve()),None)
        if expected!=digest(path):raise ValueError('Actual predictor source/config differs from frozen evaluation: '+str(path))
    transform=yaml.safe_load(transform_path.read_text())['map_to_morai_map']
    normalized,trajectories=prepare_case(case,transform)
    parameters=yaml.safe_load(parameters_path.read_text());parameters=parameters.get('map_based_prediction',parameters)
    sys.path.insert(0,str(ROOT/'docker/bevfusion'))
    from app.sensor4_calibration import load_sensor4_calibration
    extrinsic=load_sensor4_calibration(sensor)['base_from_lidar']
    tf=np.eye(4);angle=transform['rpy'][2];c,s=math.cos(angle),math.sin(angle)
    tf[:2,:2]=[[c,-s],[s,c]];tf[:3,3]=transform['xyz'];map_to_world=np.linalg.inv(tf)
    raw=Path(case['raw_bag']);receipts,raw_start=read_receipts(raw,case['frames'])
    pins[str(raw)]=digest(raw)
    predict,map_sha=load_predictor(parameters,map_file,projector)
    expected=frozen_index(a.frozen_horizon/'rows.json.gz',CASES[a.case]);checked=set();max_delta=0.
    import rosbag
    import rospy
    a.output.mkdir(parents=True)
    marker_bag=a.output/'markers.bag'
    with rosbag.Bag(str(marker_bag),'w')as bag:
        for index,(frame,norm,receipt)in enumerate(zip(case['frames'],normalized,receipts)):
            outputs={h:predict(norm,h)for h in(3,5)}
            for object_index,obj in enumerate(norm['objects']):
                for horizon in(3,5):
                    path=outputs[horizon][object_index]['paths'][0]
                    for endpoint in(1,2,3,5):
                        if endpoint>horizon:continue
                        key=(frame['stamp'],str(obj['id']),horizon,endpoint)
                        if key in expected:
                            delta=float(np.linalg.norm(np.asarray(path['xy'][endpoint*10-1])-expected[key]))
                            max_delta=max(max_delta,delta);checked.add(key)
                            if delta>1.e-8:raise ValueError('Regenerated path endpoint differs from frozen map v4')
            topics=marker_frame(frame,norm,outputs,trajectories,map_to_world,extrinsic,
                receipt['receipt_nsec'],a.case,index,len(normalized))
            stamp=rospy.Time(receipt['receipt_nsec']//10**9,receipt['receipt_nsec']%10**9)
            for topic,message in topics.items():bag.write(topic,message,stamp)
    if checked!=set(expected):raise ValueError('Frozen path endpoint comparison cohort is incomplete')
    if pins!={path:digest(path)for path in pins}:raise RuntimeError('Pinned source/input changed during export')
    first,last=receipts[0]['receipt_nsec']/1e9,receipts[-1]['receipt_nsec']/1e9
    scene=dict(schema='e12_postprocessing_rviz_scene_v1',complete=True,case=a.case,pack_case=CASES[a.case],
        raw_bag=str(raw.resolve()),marker_bag=str(marker_bag.resolve()),fixed_frame='lidar1',
        start_offset_sec=first-raw_start,duration_sec=last-first+.1,
        first_receipt=first,last_receipt=last,topics=TOPICS,framecount=len(normalized),
        marker_header='original bag receipt time for visualization lifetime only',marker_lifetime_sec=.35,
        geometry='original measurement-time current lidar1; world gravity centers are not shifted again',
        prediction_horizons_sec=[3,5],prefix5='optional offline 3s-branch-prefix-compatible real 5s path; not production policy',
        frozen_endpoint_checks=len(checked),frozen_endpoint_max_xy_error_m=max_delta,
        source_sha256=pins,marker_bag_sha256=digest(marker_bag),map_sha256=map_sha,
        raw_control_topics_replayed=False,ros_master_started=False,detector_rerun=False,
        input_header_to_receipt_age_sec=dict(minimum=min(r['receipt_nsec']/1e9-f['stamp']for r,f in zip(receipts,case['frames'])),
            maximum=max(r['receipt_nsec']/1e9-f['stamp']for r,f in zip(receipts,case['frames']))),
        limitations=['Measurement-time actual EKF-core replay, not native ROS scheduler/TF/full-stack.',
            'No future GT selects predictions. White future references require eligible, gap/index-continuous GT.',
            'Source LiDAR header to pack stamp matched uniquely; only marker visualization header uses receipt clock.'])
    with(a.output/'scene.json').open('x')as stream:json.dump(scene,stream,indent=2);stream.write('\n')
    print(json.dumps(scene),flush=True)


if __name__=='__main__':
    main()
