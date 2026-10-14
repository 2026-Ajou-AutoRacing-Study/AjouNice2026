#!/usr/bin/env python3
"""Bounded real NN→ROS→installed EKF gate with fresh isolated track topics.

Requires a ready, explicitly tagged trial GPU server. Uses current parked
MORAI scene, never drives or loads scenes, changes no production defaults.
Gravity uses the actual bridge adapter; legacy is a declared inverse shadow
copy of that SAME delivered message, not another independently inferred frame.
"""
import argparse
import copy
import importlib.util
import json
import math
import os
from pathlib import Path
import socket
import sys
import threading
import time

import yaml

ROOT=Path(__file__).resolve().parents[3]
VALIDATOR=ROOT/'base_ws/src/planning_addons/planning_scenario_validator'
sys.path[:0]=[str(VALIDATOR/'src'),str(ROOT/'base_ws/src/perception/bevfusion_ros/src'),
             str(ROOT/'base_ws/src/perception_object_pipeline/src')]


def legacy_shadow(message):
    raw=copy.deepcopy(message)
    for obj in raw.objects:
        height=float(obj.shape.dimensions.z)
        if not math.isfinite(height) or height<=0:
            raise ValueError('Shadow requires finite positive box height')
        obj.kinematics.pose_with_covariance.pose.position.z-=height*.5
    return raw


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--port',type=int,default=29515)
    p.add_argument('--duration',type=float,default=30.)
    p.add_argument('--policy',choices=('fixed51','fixed50','cut0'),default='fixed51')
    a=p.parse_args()
    if not 10<=a.duration<=60 or a.output.exists():raise ValueError('Fresh bounded gate required')
    from bevfusion_ros.protocol import TYPE_PING,TYPE_PONG,recv_message,send_message
    with socket.create_connection(('127.0.0.1',a.port),5) as sock:
        sock.settimeout(10);send_message(sock,TYPE_PING,dict(client='live_origin_gate'))
        kind,health,_=recv_message(sock)
    identity=health.get('runtime_identity') or {}
    if kind!=TYPE_PONG or not health.get('model_loaded') or identity.get('box_center_origin')!='lidar_bottom_v1':
        raise ValueError('Loaded explicitly tagged trial server required')
    a.output.mkdir(parents=True)
    import rospy
    from autoware_perception_msgs.msg import DetectedObjects,TrackedObjects
    from morai_msgs.msg import EgoVehicleStatus
    from planning_scenario_validator.runtime import Processes,BrakeHold,LiveMonitor,check_no_conflicting_publishers
    from run_perception_candidate_probe import NETWORK,POLICIES
    check_no_conflicting_publishers()
    rospy.init_node('live_origin_gate',disable_signals=True)
    if rospy.get_param('/use_sim_time',False):raise ValueError('Live receipt-time contract required')
    configuration=yaml.safe_load((ROOT/'configs/perception/bevfusion_base5_trial.yaml').read_text())
    configuration.update(sensor_config=str(ROOT/'configs/sensors/Sensor_competition_local.json'),
        server_port=a.port,operation='inference',box_center_output_origin='gravity_center_v1',
        lidar_deskew_mode='off',
        vehicle_score_threshold=.4,pedestrian_score_threshold=.1,obstacle_score_threshold=.1,
        output_objects='/origin_live/gravity/detection',publish_frame_timing=True)
    for key,value in configuration.items():rospy.set_param('~'+key,value)
    # Root EKF params are shared by the two research nodes. Restore exact prior
    # presence/value afterwards; never overwrite another running tracker.
    before_params={key:(rospy.has_param('/'+key),rospy.get_param('/'+key,None))
        for key in yaml.safe_load((ROOT/'base_ws/src/ekf-multi-object-tracking/config/config.yaml').read_text())}
    processes=Processes(a.output/'stack_logs',os.environ.copy())
    monitor=LiveMonitor();monitor.arm(a.output/'events.jsonl')
    hold=BrakeHold();hold.start()
    bridge=None;lock=threading.Lock();responses=[];delivered=[];tracks={'gravity':[],'legacy':[]}
    failures=[];ego_latest=[None];start_position=None
    legacy_pub=rospy.Publisher('/origin_live/legacy/detection',DetectedObjects,queue_size=10)
    def shadow(message):
        raw=legacy_shadow(message)
        legacy_pub.publish(raw)
        with lock:
            delivered.append(dict(stamp=message.header.stamp.to_sec(),objects=len(message.objects)))
    def collect(message,mode):
        with lock:
            tracks[mode].append(dict(stamp=message.header.stamp.to_sec(),frame=message.header.frame_id,
                objects=[dict(id=list(o.object_id.uuid),center=[getattr(o.kinematics.pose_with_covariance.pose.position,k) for k in 'xyz'],
                              size=[getattr(o.shape.dimensions,k) for k in 'xyz']) for o in message.objects]))
    subscriptions=[rospy.Subscriber('/origin_live/gravity/detection',DetectedObjects,shadow,queue_size=20)]
    subscriptions += [rospy.Subscriber('/origin_live/'+mode+'/tracking',TrackedObjects,collect,callback_args=mode,queue_size=200)
                      for mode in tracks]
    subscriptions.append(rospy.Subscriber('/Ego_topic',EgoVehicleStatus,lambda m:ego_latest.__setitem__(0,(time.monotonic(),m)),queue_size=1))
    try:
        processes.start('sensor_tf',['roslaunch','tf_pub','sensor_tf.launch',f'sensor_config:={ROOT}/configs/sensors/Sensor_competition_local.json'])
        processes.start('udp',['roslaunch','sensor_interface','morai_udp_interface.launch',
            f'sensor_config:={ROOT}/configs/sensors/Sensor_competition_local.json',f'network_config:={NETWORK}',
            'enable_lidar:=false'])
        processes.start('pose',['roslaunch','bringup','localization_morai_gt.launch',
            'start_lanelet_map:=false','use_heading_correction:=true'])
        deadline=time.monotonic()+15
        while ego_latest[0] is None and time.monotonic()<deadline:time.sleep(.05)
        if ego_latest[0] is None:raise RuntimeError('No live Ego')
        ego=ego_latest[0][1];start_position=(ego.position.x,ego.position.y)
        if math.hypot(ego.velocity.x,ego.velocity.y)>.1:raise RuntimeError('Parked scene required')
        time.sleep(1.)  # Actual header TF history before any NN input.
        params=yaml.safe_load((ROOT/'base_ws/src/ekf-multi-object-tracking/config/config.yaml').read_text())
        params['configure'].update(fixed_output_rate_hz=30.,maximum_fixed_output_prediction_age_sec=.5)
        for key,value in params.items():rospy.set_param('/'+key,value)
        binary=ROOT/'base_ws/install_aw149/lib/ekf_multi_object_tracking/ekf_multi_object_tracking'
        for mode in tracks:
            processes.start('tracker_'+mode,[str(binary),'__name:=origin_'+mode+'_tracker','_world_frame_id:=map',
                '_transform_timeout_sec:=0.1',f'~input/objects:=/origin_live/{mode}/detection',
                f'~output/objects:=/origin_live/{mode}/tracking'])
        script=ROOT/'base_ws/src/perception/bevfusion_ros/scripts/bevfusion_ros_bridge.py'
        spec=importlib.util.spec_from_file_location('actual_live_bridge',script)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        bridge=module.BevFusionRosBridge()
        original=bridge.transport.request
        def measured(metadata,blobs):
            response,payload=original(metadata,blobs)
            if response.get('box_center_origin')!='lidar_bottom_v1':
                failures.append('Untagged/changed model origin');raise ValueError(failures[-1])
            with lock:responses.append(dict(sequence=metadata['sequence'],stamp=metadata['stamp'],response=response))
            return response,payload
        bridge.transport.request=measured
        processes.start('record',['rosbag','record','--lz4','-O',str(a.output/'live.bag'),
            '/Ego_topic','/Object_topic','/CollisionData','/tf','/tf_static','/sensor/lidar1','/sensor/lidar1/packets',
            '/perception/bevfusion/frame_timing','/perception/bevfusion/status','/perception/bevfusion/latency_ms',
            *['/origin_live/'+m+'/'+t for m in tracks for t in ('detection','tracking')]])
        lidar_args=['npackets:=50'] if a.policy=='fixed50' else ['cut_angle:=0'] if a.policy=='cut0' else ['npackets:=51']
        processes.start('lidar',['roslaunch','sensor_interface','lidar.launch',*lidar_args])
        start=time.monotonic()
        while time.monotonic()-start<a.duration:
            sample=ego_latest[0]
            if sample is None or time.monotonic()-sample[0]>1:raise RuntimeError('Ego receipt lost')
            ego=sample[1]
            if math.hypot(ego.velocity.x,ego.velocity.y)>.1 or math.hypot(ego.position.x-start_position[0],ego.position.y-start_position[1])>.3:
                raise RuntimeError('Parked Ego moved; preserving diagnostic evidence')
            if failures:raise RuntimeError(failures[-1])
            if monitor.collision_count or monitor.far_collision:raise RuntimeError('CollisionData contact/contamination')
            time.sleep(.1)
    finally:
        processes.stop('lidar')
        if bridge:bridge.shutdown()
        processes.close();hold.stop()
        for key,(existed,value) in before_params.items():
            if existed:rospy.set_param('/'+key,value)
            elif rospy.has_param('/'+key):rospy.delete_param('/'+key)
        report=dict(server_health=health,policy=a.policy,responses=responses,delivered=delivered,tracks=tracks,failures=failures,
            collision_count=monitor.collision_count,far_collision=monitor.far_collision,
            ego_final_speed_mps=None if ego_latest[0] is None else math.hypot(ego_latest[0][1].velocity.x,ego_latest[0][1].velocity.y),
            scopes=['Actual tagged NN server and source ROS bridge gravity adapter, installed C++ TF/30Hz copies.',
                    'Test oracle explicitly uses -1.347deg heading correction, verified against saved scene yaw; production localization is unchanged.',
                    'Legacy shadow is the inverse representation of the same final detection, not a separate legacy GPU forward.',
                    'Current parked scene only; not native compound GT, target4090/LAN or planner closed-loop.'])
        (a.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    if len(responses)<10 or not all(any(row['objects'] for row in values) for values in tracks.values()):
        raise RuntimeError('Insufficient real detections/tracks; no qualification claim')
    print(json.dumps(dict(responses=len(responses),delivered=len(delivered),outputs={k:len(v) for k,v in tracks.items()})),flush=True)


if __name__=='__main__':main()
