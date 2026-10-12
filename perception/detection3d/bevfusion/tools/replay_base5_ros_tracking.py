#!/usr/bin/env python3
"""Isolated actual ROS tracker + UNKNOWN policy replay of saved detections.

No detector GPU, global ROS master, MORAI, planning or control. Compile OFF,
ON and independent OFF repeat share the same clock and recorded Ego TF.
"""
import argparse
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import pickle
import signal
import socket
import subprocess
import sys
import threading
import time

import numpy as np
import yaml

from analyze_base5_runtime_effects import CLASSES, POLICIES, delivered
from compare_base5_common30 import validate_payload
from replay_base5_compiled_tracking import ROOT, ego_trajectory, load_sensor4_calibration

sys.path.insert(0,str(ROOT/'base_ws/src/perception_object_pipeline/src'))


def stamp_key(value):
    return round(float(value),6)


def pose_at(ego,t):
    i=int(np.searchsorted(ego[:,0],t))
    if i<len(ego) and abs(ego[i,0]-t)<1e-9:return ego[i,1:]
    if i==0 or i==len(ego) or ego[i,0]-ego[i-1,0]>.15:
        raise ValueError('TF requires bounded recorded Ego interpolation')
    return ego[i-1,1:]+(ego[i,1:]-ego[i-1,1:])*(t-ego[i-1,0])/(ego[i,0]-ego[i-1,0])


def validate_pair(off,on,candidate_thresholds):
    if not off.get('parallel_camera') or off.get('compile_parts')!=[]:
        raise ValueError('Reference requires parallel inference with compile OFF')
    if candidate_thresholds is not None:
        if len(candidate_thresholds)!=3 or not all(.1<=v<=1. for v in candidate_thresholds):
            raise ValueError('Three thresholds at or above the frozen score floor 0.1 required')
        if off!=on:raise ValueError('Threshold comparison requires identical inference options')
    elif on.get('compile_parts')!=['camera','neck'] or {
            k:v for k,v in off.items() if k!='compile_parts'}!={k:v for k,v in on.items() if k!='compile_parts'}:
        raise ValueError('Compile comparison may differ only in camera/neck compilation')


class Replay:
    def __init__(self,args):
        self.args=args;self.processes=[];self.logs=[];self.lock=threading.Lock()
        self.messages={};self.receipts=[]
        if args.output.exists():raise ValueError('Fresh output directory required')
        args.output.mkdir(parents=True)
        with socket.socket() as s:
            s.bind(('127.0.0.1',0));self.port=s.getsockname()[1]
        self.env=os.environ.copy()
        self.env.update(ROS_MASTER_URI=f'http://127.0.0.1:{self.port}',ROS_IP='127.0.0.1',
                        ROS_HOME=str(args.output.resolve()/'ros_home'),
                        ROS_LOG_DIR=str(args.output.resolve()/'ros_logs'))
        self.env.pop('ROS_HOSTNAME',None)
        self.env['PYTHONPATH']=str(ROOT/'base_ws/src/perception_object_pipeline/src')+os.pathsep+self.env.get('PYTHONPATH','')
        os.environ.update({k:self.env[k] for k in ('ROS_MASTER_URI','ROS_IP','ROS_HOME','ROS_LOG_DIR')})
        os.environ.pop('ROS_HOSTNAME',None)

    def launch(self,name,cmd):
        log=(self.args.output/(name+'.log')).open('w');self.logs.append(log)
        p=subprocess.Popen(cmd,env=self.env,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        self.processes.append((name,p));return p

    def wait(self,predicate,timeout,reason):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            if predicate():return
            for name,p in self.processes:
                if p.poll() is not None:raise RuntimeError(name+' exited; inspect its log')
            time.sleep(.005)
        raise TimeoutError(reason)

    def stop(self,owned):
        for _,p in owned:
            if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        for _,p in owned:
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid,signal.SIGTERM)
                try:p.wait(timeout=3)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=3)
        self.processes=[r for r in self.processes if r not in owned]

    def callback(self,key):
        def receive(msg):
            from tf.transformations import euler_from_quaternion
            from autoware_perception_msgs.msg import ObjectClassification as C
            objects=[]
            for obj in msg.objects:
                label=max(obj.classification,key=lambda c:c.probability).label if obj.classification else C.UNKNOWN
                cls='vehicle' if label in (C.CAR,C.TRUCK,C.BUS,C.TRAILER) else 'pedestrian' if label in (
                    C.PEDESTRIAN,C.BICYCLE,C.MOTORCYCLE) else 'obstacle'
                pose=obj.kinematics.pose_with_covariance.pose;q=pose.orientation
                yaw=euler_from_quaternion([q.x,q.y,q.z,q.w])[2]
                twist=obj.kinematics.twist_with_covariance.twist
                objects.append(dict(id=int.from_bytes(bytes(obj.object_id.uuid)[12:],'big'),
                    label=CLASSES.index(cls),class_name=cls,center=[pose.position.x,pose.position.y,pose.position.z],
                    yaw=yaw,size=[obj.shape.dimensions.x,obj.shape.dimensions.y,obj.shape.dimensions.z],
                    score=float(obj.existence_probability),
                    velocity=[math.cos(yaw)*twist.linear.x-math.sin(yaw)*twist.linear.y,
                              math.sin(yaw)*twist.linear.x+math.cos(yaw)*twist.linear.y],
                    body_velocity=[twist.linear.x,twist.linear.y],yaw_rate=twist.angular.z,
                    stationary=obj.kinematics.is_stationary,
                    orientation_availability=obj.kinematics.orientation_availability))
            frame=dict(stamp=msg.header.stamp.to_sec(),frame_id=msg.header.frame_id,objects=objects)
            with self.lock:self.messages[key].append(frame)
        return receive

    def run(self):
        import rosgraph
        import rospy
        import tf2_ros
        from autoware_perception_msgs.msg import DetectedObjects, TrackedObjects
        from geometry_msgs.msg import TransformStamped
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import Header
        from tf.transformations import quaternion_from_matrix
        from perception_object_pipeline.detected_objects import make_detected_objects

        a=self.args
        infos=pickle.loads((a.inputs/'infos_val.pkl').read_bytes())['data_list']
        times={r['token']:r['timestamp'] for r in infos}
        manifest=json.loads((a.inputs/'manifest.json').read_text())
        payloads={}
        paths={'off':a.off,'on':a.on,'off_repeat':a.off}
        for name,path in paths.items():
            with gzip.open(path,'rt') as f:payloads[name]=json.load(f)
            validate_payload(payloads[name])
        for p in payloads.values():
            if any(p[k]!=payloads['off'][k] for k in ('checkpoint_sha256','gt_identity','info_sha256','inference_precision')):
                raise ValueError('Same weights, inputs and precision required')
            if p['frames']!=len(infos) or set(r['token'] for r in p['rows'])!=set(times):
                raise ValueError('Saved outputs and prepared input frames differ')
        off,on=(payloads[k]['inference_optimization'] for k in ('off','on'))
        validate_pair(off,on,a.candidate_thresholds)
        if a.candidate_thresholds is not None and hashlib.sha256(a.off.read_bytes()).digest()!=hashlib.sha256(a.on.read_bytes()).digest():
            raise ValueError('Threshold replay requires exactly the same prediction file')
        self.launch('roscore',['roscore','-p',str(self.port)])
        self.wait(lambda:rosgraph.is_master_online(),15,'Private ROS master did not start')
        rospy.init_node('base5_saved_detection_replay',anonymous=True,disable_signals=True)
        rospy.set_param('/use_sim_time',True)
        clock=rospy.Publisher('/clock',Clock,queue_size=100,latch=True)
        tf=tf2_ros.TransformBroadcaster();static=tf2_ros.StaticTransformBroadcaster()
        extrinsic=load_sensor4_calibration(a.sensor_config)['base_from_lidar']
        static_msg=TransformStamped();static_msg.header.frame_id='probe_base';static_msg.child_frame_id='probe_lidar'
        static_msg.transform.translation.x,static_msg.transform.translation.y,static_msg.transform.translation.z=extrinsic[:3,3]
        q=quaternion_from_matrix(extrinsic)
        static_msg.transform.rotation.x,static_msg.transform.rotation.y,static_msg.transform.rotation.z,static_msg.transform.rotation.w=q
        config=yaml.safe_load((ROOT/'base_ws/src/ekf-multi-object-tracking/config/config.yaml').read_text())
        rospy.set_param('/topic_name',dict(config['topic_name'],
            output_track_jsk='/base5_replay/unused_visualization_boxes',
            output_track_marker='/base5_replay/unused_visualization_markers'))
        cases=[c for c in manifest['cases'] if not a.cases or c in a.cases]
        if not cases:raise ValueError('No requested cases')
        started=time.monotonic()
        for case in cases:
            rows={name:sorted([r for r in p['rows'] if r['token'].startswith(case+'_')],key=lambda r:times[r['token']])
                  for name,p in payloads.items()}
            tokens=[r['token'] for r in rows['off']]
            if any([r['token'] for r in rs]!=tokens for rs in rows.values()):raise ValueError('Different ordered detections')
            if a.limit:tokens=tokens[:a.limit];rows={k:v[:a.limit] for k,v in rows.items()}
            if not tokens:raise ValueError('Empty case')
            ts=[times[t] for t in tokens]
            ego=ego_trajectory(Path(manifest['cases'][case]['raw_bag']))
            clock.publish(Clock(rospy.Time.from_sec(ts[0]-.1)))
            static_msg.header.stamp=rospy.Time.from_sec(ts[0]-.1);static.sendTransform(static_msg)
            time.sleep(.05)
            pubs={};subs=[];owned=[]
            for name in paths:
                ns='/base5_replay/'+case.lower()+'/'+name
                cfg=json.loads(json.dumps(config))
                cfg['configure'].update(fixed_output_rate_hz=a.fixed_output_rate_hz,
                    maximum_fixed_output_prediction_age_sec=.5,
                    compensate_output_to_current_time=False)
                cfg['topic_name'].update(output_track_jsk=ns+'/visualization_boxes',output_track_marker=ns+'/visualization_markers')
                rospy.set_param(ns,cfg)
                rospy.set_param(ns+'/policy/force_unknown_stationary',True)
                pubs[name]=rospy.Publisher(ns+'/detections',DetectedObjects,queue_size=10)
                for stage in ('raw','policy'):
                    key=case+'/'+name+'/'+stage;self.messages[key]=[]
                    subs.append(rospy.Subscriber(ns+'/'+stage,TrackedObjects,self.callback(key),queue_size=2000))
                self.launch(case+'_'+name+'_tracker',[str(a.binary.resolve()),'__name:=tracker','__ns:='+ns,
                    '~input/objects:='+ns+'/detections','~output/objects:='+ns+'/raw',
                    '_world_frame_id:=map'])
                owned.append(self.processes[-1])
                self.launch(case+'_'+name+'_policy',['/usr/bin/python3',str(ROOT/'base_ws/src/perception_object_pipeline/scripts/tracked_object_motion_policy_node.py'),
                    '__name:=policy','__ns:='+ns,'~input/objects:='+ns+'/raw','~output/objects:='+ns+'/policy'])
                owned.append(self.processes[-1])
            self.wait(lambda:all(p.get_num_connections()==1 for p in pubs.values()),20,'Tracker input subscription missing')
            self.wait(lambda:tf.pub_tf.get_num_connections()>=3,15,'Tracker TF subscribers missing')
            self.wait(lambda:static.pub_tf.get_num_connections()>=3,15,'Tracker static TF subscribers missing')
            # A publisher connection alone is not a populated per-node TF
            # buffer. Repeat the latched extrinsic after all nodes connect.
            warm=TransformStamped();warm.header.frame_id='map';warm.child_frame_id='probe_base'
            warm.header.stamp=rospy.Time.from_sec(ts[0]-.1)
            pose=pose_at(ego,ts[0])
            warm.transform.translation.x,warm.transform.translation.y,warm.transform.translation.z=pose[:3]
            warm.transform.rotation.z=math.sin(pose[3]/2);warm.transform.rotation.w=math.cos(pose[3]/2)
            for _ in range(3):
                static.sendTransform(static_msg);tf.sendTransform(warm);time.sleep(.05)
            count=0;sim_clock=ts[0]-.1
            def advance(t):
                nonlocal sim_clock
                if t<sim_clock-1e-6:raise ValueError('Clock went backwards inside case')
                sim_clock=t;clock.publish(Clock(rospy.Time.from_sec(t)))
                time.sleep(a.clock_wall_step_sec)
            for index,token in enumerate(tokens):
                t=times[token]
                if a.fixed_output_rate_hz:
                    while sim_clock+.01<t-1e-7:advance(sim_clock+.01)
                pose=pose_at(ego,t)
                dynamic=TransformStamped();dynamic.header.frame_id='map';dynamic.child_frame_id='probe_base'
                dynamic.header.stamp=rospy.Time.from_sec(t)
                dynamic.transform.translation.x,dynamic.transform.translation.y,dynamic.transform.translation.z=pose[:3]
                dynamic.transform.rotation.z=math.sin(pose[3]/2);dynamic.transform.rotation.w=math.cos(pose[3]/2)
                tf.sendTransform(dynamic);time.sleep(.005)
                for name in paths:
                    r=rows[name][index]
                    ds=delivered([dict(center=b[:3],size=b[3:6],yaw=b[6],score=s,label=int(l),class_name=CLASSES[l])
                        for b,l,s in zip(r['pred_boxes'],r['pred_labels'],r['pred_scores'])],
                        a.candidate_thresholds if name=='on' and a.candidate_thresholds is not None else POLICIES['current'])
                    if a.maximum_range_m:
                        ds=[d for d in ds if np.linalg.norm(d['center'][:2])<a.maximum_range_m and -3<=d['center'][2]<=2]
                    header=Header(seq=index,stamp=rospy.Time.from_sec(t),frame_id='probe_lidar')
                    pubs[name].publish(make_detected_objects(header,
                        [[*d['center'],*d['size'],d['yaw']] for d in ds],
                        [d['score'] for d in ds],[d['class_name'] for d in ds]))
                time.sleep(.01)
                advance(t)
                if not a.fixed_output_rate_hz or index==0:
                    def ready():
                        with self.lock:
                            return all(any(abs(f['stamp']-t)<1e-6 for f in self.messages[case+'/'+name+'/'+stage])
                                for name in paths for stage in ('raw','policy'))
                    if not a.fixed_output_rate_hz:
                        for delta in (.01,.02,.03):
                            if ready():break
                            advance(t+delta)
                    self.wait(ready,3,'Not all nodes acknowledged measurement '+token)
                count+=1
                if count%100==0:print(case,count,'/',len(tokens),flush=True)
            if a.fixed_output_rate_hz:
                while sim_clock<ts[-1]+.08:advance(sim_clock+.01)
            time.sleep(.1)
            receipt=dict(case=case,input_frames=count,first_stamp=ts[0],last_stamp=ts[-1],
                output_frames={key:len(value) for key,value in self.messages.items() if key.startswith(case+'/')})
            self.receipts.append(receipt);print(json.dumps(receipt),flush=True)
            self.stop(owned)
            for sub in subs:sub.unregister()
            for pub in pubs.values():pub.unregister()
            with gzip.open(a.output/'messages.json.gz','wt') as f:json.dump(self.messages,f,allow_nan=False)
        result=dict(scope='Actual ROS TF + C++ tracker + UNKNOWN policy on saved detections; not GPU/queue/LAN/planning certification.',
            comparison_kind='score_threshold' if a.candidate_thresholds is not None else 'compile',
            reference_thresholds=list(POLICIES['current']),
            candidate_thresholds=a.candidate_thresholds or list(POLICIES['current']),
            fixed_output_rate_hz=a.fixed_output_rate_hz,maximum_range_m=a.maximum_range_m,
            fixed_prediction_age_sec=.5,clock_wall_step_sec=a.clock_wall_step_sec,
            private_master_port=self.port,wall_seconds=time.monotonic()-started,cases=self.receipts,
            checkpoint_sha256=payloads['off']['checkpoint_sha256'],gt_identity=payloads['off']['gt_identity'],
            source_sha256={str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest() for path in [
                ROOT/'base_ws/src/ekf-multi-object-tracking/src/ekf_multi_object_tracking_node.cpp',
                ROOT/'base_ws/src/ekf-multi-object-tracking/src/algorithm/ekf_multi_object_tracking.cpp',
                ROOT/'base_ws/src/ekf-multi-object-tracking/include/ekf_multi_object_tracking_node.hpp',
                ROOT/'base_ws/src/ekf-multi-object-tracking/include/vehicle_orientation_canonicalizer.hpp',
                ROOT/'base_ws/src/ekf-multi-object-tracking/config/config.yaml',
                ROOT/'base_ws/src/perception_object_pipeline/src/perception_object_pipeline/object_motion_policy.py']},
            binary_sha256=hashlib.sha256(a.binary.read_bytes()).hexdigest(),
            input_sha256={key:hashlib.sha256(path.read_bytes()).hexdigest() for key,path in paths.items()})
        (a.output/'receipt.json').write_text(json.dumps(result,indent=2)+'\n')

    def close(self):
        self.stop(list(self.processes))
        for log in self.logs:log.close()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('inputs','off','on','binary','sensor-config','output'):p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--cases',nargs='*');p.add_argument('--limit',type=int,default=0)
    p.add_argument('--maximum-range-m',type=float,choices=(30.,),default=None)
    p.add_argument('--fixed-output-rate-hz',type=float,choices=(0.,30.),default=0.)
    p.add_argument('--candidate-thresholds',type=float,nargs=3,metavar=('VEHICLE','PEDESTRIAN','OBSTACLE'),
                   help='Compare the same OFF predictions with a new score policy, not compilation.')
    p.add_argument('--clock-wall-step-sec',type=float,default=.005)
    args=p.parse_args()
    if args.limit<0 or args.clock_wall_step_sec<=0:p.error('Nonnegative limit and positive clock pacing required')
    replay=Replay(args)
    try:replay.run()
    except Exception as error:
        (args.output/'failure.json').write_text(json.dumps(dict(error_type=type(error).__name__,message=str(error)))+'\n')
        raise
    finally:replay.close()
