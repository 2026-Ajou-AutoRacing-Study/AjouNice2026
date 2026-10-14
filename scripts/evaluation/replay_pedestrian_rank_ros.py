#!/usr/bin/env python3
"""Replay identical frozen E24 observations into fresh installed ROS trackers.

Private master/clock isolates MORAI and the user's ROS graph. This exercises
the real message adapter, node scheduler and runtime switch, not live GPU or
planner safety. World-frame inputs intentionally avoid another TF variable.
"""
import argparse
import gzip
import json
import os
from pathlib import Path
import socket
import sys
import time
import yaml

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'base_ws/src/planning_addons/planning_scenario_validator/src'),
             str(ROOT/'base_ws/src/perception_object_pipeline/src')]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--frames',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--port',type=int,default=29166)
    p.add_argument('--duration',type=float,default=32.)
    a=p.parse_args()
    if a.output.exists() or not 10<=a.duration<=240:raise ValueError('Fresh bounded replay required')
    with socket.socket() as check:check.bind(('127.0.0.1',a.port))
    frames=json.load(gzip.open(a.frames,'rt'))
    frames=[f for f in frames if f['world_objects'] is not None]
    if len(frames)<10:raise ValueError('Complete input sequence required')
    a.output.mkdir(parents=True)
    os.environ['ROS_MASTER_URI']=f'http://127.0.0.1:{a.port}';os.environ['ROS_IP']='127.0.0.1'
    from planning_scenario_validator.runtime import Processes
    processes=Processes(a.output/'logs',os.environ.copy())
    output={'off':[],'on':[]};subscriptions=[]
    try:
        processes.start('master',['roscore','-p',str(a.port)])
        import rosgraph
        deadline=time.monotonic()+15
        while not rosgraph.is_master_online():
            if time.monotonic()>deadline:raise RuntimeError('Private master unavailable')
            time.sleep(.1)
        import rospy
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import Header
        from autoware_perception_msgs.msg import DetectedObjects,TrackedObjects
        from perception_object_pipeline.detected_objects import make_detected_objects
        from audit_tracking_continuity import object_row,output_identity_summary
        rospy.init_node('pedestrian_rank_ros_replay',disable_signals=True)
        rospy.set_param('/use_sim_time',True)
        clock=rospy.Publisher('/clock',Clock,queue_size=10,latch=True)
        first=frames[0]['stamp'];last=min(first+a.duration,frames[-1]['stamp'])
        clock.publish(Clock(rospy.Time.from_sec(first)))
        configuration=yaml.safe_load((ROOT/'base_ws/src/ekf-multi-object-tracking/config/config.yaml').read_text())
        configuration['configure'].update(fixed_output_rate_hz=30.,maximum_fixed_output_prediction_age_sec=.5)
        publishers={m:rospy.Publisher('/rank_ros/'+m+'/detection',DetectedObjects,queue_size=20) for m in output}
        def receive(message,mode):
            output[mode].append(dict(stamp=message.header.stamp.to_sec(),receipt=time.monotonic(),
                                     objects=[object_row(o) for o in message.objects]))
        subscriptions=[rospy.Subscriber('/rank_ros/'+m+'/tracking',TrackedObjects,receive,
                         callback_args=m,queue_size=200) for m in output]
        binary=ROOT/'base_ws/install_aw149/lib/ekf_multi_object_tracking/ekf_multi_object_tracking'
        for mode in output:
            configuration['configure']['pedestrian_gaussian_association_ranking']=mode=='on'
            for key,value in configuration.items():rospy.set_param('/'+key,value)
            processes.start(mode,[str(binary),'__name:=rank_'+mode,'_world_frame_id:=map',
                '~input/objects:=/rank_ros/'+mode+'/detection','~output/objects:=/rank_ros/'+mode+'/tracking',
                '/output_track_jsk:=/rank_ros/'+mode+'/jsk','/output_track_marker:=/rank_ros/'+mode+'/marker'])
            deadline=time.monotonic()+15
            while publishers[mode].get_num_connections()<1:
                if time.monotonic()>deadline:raise RuntimeError('Tracker did not subscribe')
                time.sleep(.05)
        processes.start('recorder',['rosbag','record','--lz4','-O',str(a.output/'replay.bag'),
            '/clock','/rank_ros/off/detection','/rank_ros/on/detection',
            '/rank_ros/off/tracking','/rank_ros/on/tracking','/rosout'])
        time.sleep(1.)
        index=0;sent=0;tick=0;start=time.monotonic()
        while first+tick/30.<=last+.5:
            current=first+tick/30.
            clock.publish(Clock(rospy.Time.from_sec(current)))
            while index<len(frames) and frames[index]['stamp']<=min(current,last):
                frame=frames[index];objects=frame['world_objects']
                # The frozen diagnostic input was exported as bottom center on
                # a planar world transform. Current ROS uses gravity center.
                boxes=[o['center'][:2]+[o['center'][2]+o['size'][2]*.5]+o['size']+[o['yaw']] for o in objects]
                message=make_detected_objects(Header(seq=sent,stamp=rospy.Time.from_sec(frame['stamp']),frame_id='map'),
                    boxes,[o['score'] for o in objects],[o['class_name'] for o in objects])
                for publisher in publishers.values():publisher.publish(message)
                index+=1;sent+=1
            tick+=1
            time.sleep(max(0.,start+tick/30.-time.monotonic()))
        processes.stop('recorder')
        report=dict(frames_source=str(a.frames),sent_frames=sent,outputs={m:len(v) for m,v in output.items()},
                    identities={m:output_identity_summary(v) for m,v in output.items()},
                    scope='Private ROS identical frozen world-frame inputs, gravity representation, two fresh installed nodes and 30Hz scheduler. No live GPU, TF interpolation or planner certificate.')
        (a.output/'outputs.json').write_text(json.dumps(output,allow_nan=False)+'\n')
        (a.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        for mode in output:
            if len(output[mode])<sent:raise RuntimeError('Insufficient scheduled outputs')
        print(json.dumps(dict(sent=sent,outputs=report['outputs'])),flush=True)
    finally:
        for subscription in subscriptions:subscription.unregister()
        processes.close()


if __name__=='__main__':main()
