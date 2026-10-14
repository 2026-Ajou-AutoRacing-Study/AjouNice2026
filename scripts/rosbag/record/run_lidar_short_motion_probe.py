#!/usr/bin/env python3
"""Bounded low-speed MGeo-following raw-input grouping experiment.

Not an autonomous planner or production controller. Uses a short existing
lane segment, keeps source NPC spawn points, stops on contact/stream loss/
road deviation/object proximity, and confirms Park before cleanup.
"""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import numpy as np

from run_perception_candidate_probe import ROOT, NETWORK, INSTALL, POLICIES, install_stem, save, sha
VALIDATOR=ROOT/'base_ws/src/planning_addons/planning_scenario_validator'


def route_window(points, position, distance):
    points=np.asarray(points,dtype=float)
    if points.ndim!=2 or points.shape[0]<2 or points.shape[1]<3 or not np.isfinite(points).all() or not np.isfinite(position).all():
        raise ValueError('Finite supported MGeo geometry required')
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(points[:,:2],axis=0),axis=1))]
    start=int(np.argmin(np.linalg.norm(points[:,:2]-np.asarray(position)[:2],axis=1)))
    if np.linalg.norm(points[start,:2]-np.asarray(position)[:2])>.75:
        raise ValueError('Declared Ego is not supported by its MGeo link')
    if arc[-1]-arc[start] < distance+3.:
        raise ValueError('Short drive needs a supported stopping/lookahead envelope')
    return points,arc,start,float(arc[start]+distance)


def steering(points,arc,index,x,y,yaw):
    target=np.searchsorted(arc,arc[index]+3.)
    target=min(len(points)-1,int(target))
    dx,dy=points[target,0]-x,points[target,1]-y
    bearing=math.atan2(dy,dx)-yaw
    bearing=(bearing+math.pi)%(2*math.pi)-math.pi
    if abs(bearing)>math.radians(65):raise RuntimeError('Unsupported direction; brake')
    return float(np.clip(math.atan2(2*3.0*math.sin(bearing),max(1.,math.hypot(dx,dy))),-.45,.45))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--mission',action='append',choices=('highway','roundabout'))
    p.add_argument('--distance',type=float,default=8.)
    p.add_argument('--run',action='store_true')
    a=p.parse_args()
    if not 5<=a.distance<=10 or a.output.exists():raise ValueError('Fresh 5..10m QA required')
    sys.path[:0]=[str(VALIDATOR/'src'),str(ROOT/'base_ws/src/control/trajectory_batch_runner/src'),
                 str(ROOT/'dataset/morai_od/scripts')]
    import generate_dynamic_object_capture_set as geo
    links={r['idx']:r for r in geo.load_json(geo.DEFAULT_MGEO/'link_set.json')}
    missions=a.mission or ['highway','roundabout']
    scenes={};windows={}
    for mission in missions:
        filename='bench_highway_merge_raw.json' if mission=='highway' else 'bench_roundabout_raw.json'
        source=ROOT/'configs/scenario/perception_benchmark'/filename
        scene=json.loads(source.read_text());initial=scene['egoVehicle']['initPosition']
        windows[mission]=route_window(links[initial['initLink']]['points'],
                                      [initial['pos'][k] for k in 'xyz'],a.distance)
        scenes[mission]=source
    a.output.mkdir(parents=True)
    save(a.output/'plan.json',dict(source_sha256={str(s):sha(s) for s in scenes.values()},
        mgeo_sha256=sha(geo.DEFAULT_MGEO/'link_set.json'),distance_m=a.distance,speed_target_mps=1.5,
        policies=list(POLICIES),missions=missions,scope='Short raw-input QA, not full mission or learned closed-loop.'))
    if not a.run:return
    import rospy
    import rosbag
    from morai_msgs.msg import CtrlCmd,ObjectStatusList
    from std_msgs.msg import Int8
    from planning_scenario_validator.runtime import Processes,LiveMonitor,BrakeHold,check_no_conflicting_publishers,publish_brake,prepare_scenario
    from planning_scenario_validator.runner import _load_scene
    check_no_conflicting_publishers()
    rospy.init_node('short_lidar_motion_probe',disable_signals=True)
    if rospy.get_param('/use_sim_time',False):raise RuntimeError('Live host-clock contract required')
    monitor=LiveMonitor();hold=BrakeHold();processes=Processes(a.output/'stack_logs',os.environ.copy())
    publisher=rospy.Publisher('/ctrl_cmd',CtrlCmd,queue_size=1)
    objects=[None];gear=[None];completed=[];sender=False
    rospy.Subscriber('/Object_topic',ObjectStatusList,lambda m:objects.__setitem__(0,(time.monotonic(),m)),queue_size=1)
    rospy.Subscriber('/vehicle/status/gear',Int8,lambda m:gear.__setitem__(0,(time.monotonic(),m.data)),queue_size=1)
    def sender_command(value):
        return ['rosrun','sensor_interface','morai_ctrl_cmd_udp_sender',f'_network_config:={NETWORK}',
                f'_gear:={value}','_ctrl_mode:=2','__name:=short_motion_sender']
    cleanup_error=None
    try:
        processes.start('sensor_tf',['roslaunch','tf_pub','sensor_tf.launch',
            f'sensor_config:={ROOT}/configs/sensors/Sensor_competition_local.json'])
        processes.start('pose',['roslaunch','bringup','localization_morai_gt.launch',
            'start_lanelet_map:=false','use_heading_correction:=true'])
        for number,(mission,policy) in enumerate((m,p) for m in missions for p in POLICIES):
            processes.stop('sender');sender=False
            processes.stop('udp')
            processes.start('udp',['roslaunch','sensor_interface','morai_udp_interface.launch',
                f'sensor_config:={ROOT}/configs/sensors/Sensor_competition_local.json',
                f'network_config:={NETWORK}','enable_control:=false',*POLICIES[policy]])
            monitor.wait(lambda:time.monotonic()-monitor.stamps.get('ego',0)<.5,15,'fresh Ego')
            folder=a.output/f'{number:02d}_{mission}_{policy}';folder.mkdir()
            source=scenes[mission]
            installed,xy=prepare_scenario(SimpleNamespace(source=source,source_sha256=sha(source),
                 install_stem=install_stem(a.output,number)),INSTALL,NETWORK)
            save(folder/'scenario.json',json.loads(installed.read_text()))
            _load_scene({'document':dict(scenario_ready_timeout_sec=25,
                morai=dict(scenario_load_host='127.0.0.1',scenario_load_port=9095))},installed,monitor,xy)
            processes.start('sender',sender_command(4));sender=True;hold=BrakeHold();hold.start()
            monitor.wait(lambda:math.hypot(monitor.ego.velocity.x,monitor.ego.velocity.y)<.1,10,'initial stop')
            monitor.arm(folder/'events.jsonl');time.sleep(1.)
            save(folder/'parameters.json',rospy.get_param('/'))
            recorder=processes.start('record',['rosbag','record','--lz4','-O',str(folder/'raw.bag'),
                '/Ego_topic','/Competition_topic','/Object_topic','/CollisionData','/ctrl_cmd',
                '/vehicle/status/gear','/vehicle/status/control_mode','/sensor/lidar1','/sensor/lidar1/packets',
                '/sensor/imu','/sensor/gps','/tf','/tf_static','/rosout',
                *[f'/sensor/camera/{c}/compressed' for c in ('front','front_left','front_right','back')]])
            time.sleep(1.);hold.stop()
            points,arc,start,goal=windows[mission];trace=[];begin=time.monotonic();moved=False
            while time.monotonic()-begin<25:
                if recorder.poll() is not None:raise RuntimeError('Raw recorder exited during motion')
                ego=monitor.ego;speed=math.hypot(ego.velocity.x,ego.velocity.y)
                if not all(math.isfinite(v) for v in (ego.position.x,ego.position.y,ego.heading,speed)):
                    raise RuntimeError('Non-finite Ego; brake')
                if time.monotonic()-monitor.stamps.get('ego',0)>.5:raise RuntimeError('Ego stream lost')
                if objects[0] is None or time.monotonic()-objects[0][0]>.5:raise RuntimeError('Object stream lost')
                if monitor.collision_count or monitor.far_collision:raise RuntimeError('Contact; stop')
                index=int(np.argmin(np.linalg.norm(points[:,:2]-[ego.position.x,ego.position.y],axis=1)))
                lateral=float(np.linalg.norm(points[index,:2]-[ego.position.x,ego.position.y]))
                if lateral>1. or speed>3.:raise RuntimeError('Road/speed envelope violated')
                if any(math.hypot(o.position.x-ego.position.x,o.position.y-ego.position.y)<6.
                       for group in ('npc_list','pedestrian_list','obstacle_list') for o in getattr(objects[0][1],group)):
                    raise RuntimeError('Actor entered guarded short-motion corridor')
                moved=moved or speed>.3
                if arc[index]>=goal:break
                if time.monotonic()-begin>10 and not moved:raise RuntimeError('No motion; do not certify parked input')
                command=CtrlCmd();command.longlCmdType=1
                command.steering=steering(points,arc,index,ego.position.x,ego.position.y,
                                           math.radians(ego.heading-1.347))
                command.accel=max(0.,min(.20,.18*(1.5-speed)))
                command.brake=max(0.,min(.5,.3*(speed-1.65)))
                publisher.publish(command)
                trace.append(dict(time=rospy.Time.now().to_sec(),speed=speed,arc=float(arc[index]),
                                  lateral=lateral,steering=command.steering))
                time.sleep(.05)
            else:raise RuntimeError('Short motion timeout')
            hold=BrakeHold();hold.start()
            monitor.wait(lambda:math.hypot(monitor.ego.velocity.x,monitor.ego.velocity.y)<.1,10,'final stop')
            time.sleep(1.);processes.stop('record')
            if monitor.collision_count or monitor.far_collision:raise RuntimeError('Contact during final braking')
            if not moved:raise RuntimeError('No actual motion')
            with rosbag.Bag(str(folder/'raw.bag')) as bag:
                topics=bag.get_type_and_topic_info().topics
                required=['/Ego_topic','/Object_topic','/sensor/lidar1','/sensor/lidar1/packets']+[
                    f'/sensor/camera/{c}/compressed' for c in ('front','front_left','front_right','back')]
                if any(t not in topics or topics[t].message_count<10 for t in required):raise RuntimeError('Incomplete raw inputs')
                counts=sorted({len(m.packets) for _,m,_ in bag.read_messages(topics=['/sensor/lidar1/packets'])})
            if counts!=([51] if policy=='fixed51' else [50]):raise RuntimeError('Actual driver policy mismatch')
            save(folder/'motion_trace.json',trace)
            completed.append(dict(mission=mission,policy=policy,folder=str(folder.resolve()),
                 distance_m=float(trace[-1]['arc']-arc[start]),max_speed_mps=max(r['speed'] for r in trace),
                 max_lateral_m=max(r['lateral'] for r in trace),packets=counts,bag_sha256=sha(folder/'raw.bag')))
            save(a.output/'progress.json',dict(completed=completed));print(json.dumps(completed[-1]),flush=True)
            hold.stop()
    finally:
        processes.stop('record')
        if sender:
            publish_brake(1.)
            try:
                monitor.wait(lambda:monitor.ego is not None and math.hypot(monitor.ego.velocity.x,monitor.ego.velocity.y)<.1,8,'cleanup stop')
                processes.stop('sender');sender=False
                processes.start('park_sender',sender_command(1));time.sleep(.5);publish_brake(1.)
                monitor.wait(lambda:gear[0] is not None and time.monotonic()-gear[0][0]<.5 and gear[0][1]==1,5,'Park acknowledgement')
            except Exception as error:
                cleanup_error=error
                save(a.output/'cleanup_warning.json',dict(error=str(error)))
        save(a.output/'final_stop.json',dict(gear=gear[0],speed_mps=None if monitor.ego is None else
                 math.hypot(monitor.ego.velocity.x,monitor.ego.velocity.y)))
        hold.stop();publisher.unregister();processes.close()
        if cleanup_error is not None:raise RuntimeError('Stop/Park not confirmed; inspect preserved evidence') from cleanup_error


if __name__=='__main__':main()
