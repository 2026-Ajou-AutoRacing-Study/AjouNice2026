#!/usr/bin/env python3
"""Isolated ROS batch1 playback capacity probe; never starts simulator/control.

Requires an already warmed production E25 TCP server. Uses an isolated master,
wall-time measurement and original sensor headers (no sim-time latency math).
"""
import argparse
import json
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import threading
import time
import numpy as np
import yaml


def stop(child):
    if child.poll() is not None:return
    try:os.killpg(child.pid,signal.SIGINT)
    except ProcessLookupError:return
    try:child.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(child.pid,signal.SIGTERM)
        try:child.wait(timeout=3)
        except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait(timeout=3)


def stats(xs):
    return None if not xs else dict(mean=float(np.mean(xs)),p50=float(np.percentile(xs,50)),p95=float(np.percentile(xs,95)))


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--bag',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--server-port',type=int,default=29501)
    p.add_argument('--rates',nargs='+',type=float,default=[1.,8.6]);p.add_argument('--wall-seconds',type=float,default=20.)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    if a.wall_seconds<=0 or not a.rates or any(r<=0 for r in a.rates):
        raise ValueError('Positive playback rates and measurement duration required')
    repo=Path(__file__).resolve().parents[4]
    with socket.socket() as sock:sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
    os.environ.update(ROS_MASTER_URI=f'http://127.0.0.1:{port}',ROS_IP='127.0.0.1',ROS_HOME=str(a.output/'ros_home'))
    config=yaml.safe_load((repo/'configs/perception/bevfusion_sensor4.yaml').read_text());config['server_port']=a.server_port
    (a.output/'runtime.yaml').write_text(yaml.safe_dump(config))
    logs=[];children=[]
    def launch(cmd,name):
        log=(a.output/(name+'.log')).open('w');logs.append(log)
        child=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);children.append(child);return child
    records=[];lock=threading.Lock();active=None;latest_status={};warm_count=0
    try:
        master=launch(['roscore','-p',str(port)],'master')
        for _ in range(100):
            if subprocess.run(['rosparam','list'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode==0:break
            if master.poll() is not None:raise RuntimeError('Own master failed')
            time.sleep(.1)
        else:raise RuntimeError('Own master readiness timeout')
        import rospy
        from sensor_msgs.msg import PointCloud2
        from std_msgs.msg import String
        from autoware_perception_msgs.msg import DetectedObjects
        rospy.init_node('capacity_observer',disable_signals=True)
        def lidar(msg):
            with lock:
                if active is not None:active['lidar_times'].append(time.monotonic())
        def detection(msg):
            with lock:
                if active is not None:active['detection_times'].append(time.monotonic())
        def status(msg):
            nonlocal latest_status,warm_count
            if not msg.data.startswith('ok '):return
            values={k:float(v) for k,v in re.findall(r'(\w+)=(-?\d+(?:\.\d+)?)',msg.data)}
            with lock:
                latest_status=values
                warm_count+=1
                if active is not None:active['statuses'].append(dict(wall=time.monotonic(),**values))
        subscribers=[rospy.Subscriber('/sensor/lidar1',PointCloud2,lidar,queue_size=100),
            rospy.Subscriber('/perception/object_recognition/detection/objects',DetectedObjects,detection,queue_size=100),
            rospy.Subscriber('/perception/bevfusion/status',String,status,queue_size=100)]
        bridge=launch(['roslaunch','bevfusion_ros','base3_online.launch','start_rviz:=false',
            'config:='+str(a.output/'runtime.yaml'),'sensor_config:='+str(repo/'configs/sensors/Sensor_competition_local.json')],'bridge')
        for _ in range(150):
            if bridge.poll() is not None:raise RuntimeError('Bridge failed')
            if subprocess.run(['rosnode','ping','-c','1','/bevfusion_ros_bridge'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode==0:break
            time.sleep(.1)
        else:raise RuntimeError('Bridge readiness timeout')
        time.sleep(1.)
        # Server dummy warmup is insufficient for spconv/cuDNN choices on real
        # clouds. Exclude startup/connect/autotuning from steady-state rates.
        primer=launch(['rosbag','play','--quiet','--loop','--rate=1',str(a.bag),
            '--topics','/sensor/lidar1','/sensor/camera/front/compressed','/sensor/camera/front_left/compressed',
            '/sensor/camera/front_right/compressed','/sensor/camera/back/compressed'],'primer')
        deadline=time.monotonic()+45.
        while warm_count<20:
            if time.monotonic()>deadline or bridge.poll() is not None:raise RuntimeError('Real-frame warmup failed')
            time.sleep(.1)
        stop(primer);time.sleep(1.)
        for rate in a.rates:
            with lock:
                active=dict(rate=rate,lidar_times=[],detection_times=[],statuses=[],baseline=latest_status.copy())
            play=launch(['rosbag','play','--quiet','--loop','--rate='+str(rate),str(a.bag),
                '--topics','/sensor/lidar1','/sensor/camera/front/compressed','/sensor/camera/front_left/compressed',
                '/sensor/camera/front_right/compressed','/sensor/camera/back/compressed'],f'play_{rate}')
            started=time.monotonic();deadline=started+a.wall_seconds
            while time.monotonic()<deadline:
                if play.poll() is not None or bridge.poll() is not None:raise RuntimeError('Own playback/bridge exited unexpectedly')
                time.sleep(.1)
            stop(play);elapsed=time.monotonic()-started
            with lock:result=active;active=None
            rows=result.pop('statuses');inputs=result.pop('lidar_times');outputs=result.pop('detection_times');baseline=result.pop('baseline')
            result.update(elapsed_sec=elapsed,input_observed=len(inputs),output_observed=len(outputs),
                input_hz=len(inputs)/elapsed,output_hz=len(outputs)/elapsed,output_input_ratio=len(outputs)/max(1,len(inputs)),
                output_interval_ms=stats([1000*(b-a) for a,b in zip(outputs,outputs[1:])]),
                metrics={k:stats([r[k] for r in rows if k in r]) for k in ('latency_ms','host_preprocess_ms','pointcloud_ms','server_preprocess_ms','inference_ms','request_ms')},
                drop_delta={k:int(rows[-1].get(k,0)-baseline.get(k,0)) for k in ('dropped_raw','dropped_prepared')} if rows else {},
                status_records=len(rows))
            (a.output/f'status_{rate}.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
            records.append(result)
            (a.output/'report.json').write_text(json.dumps(dict(contract='E25 production bridge, own ROS master, original headers, wall-time playback loop; 20 successful real-frame primer outputs before measurement; no accuracy test; latest-only drops explicitly counted. latency_ms ends before detection publication.',bag=str(a.bag.resolve()),cases=records),indent=2)+'\n')
            print(json.dumps(result),flush=True);time.sleep(1.)
    finally:
        for child in reversed(children):stop(child)
        for log in logs:log.close()


if __name__=='__main__':main()
