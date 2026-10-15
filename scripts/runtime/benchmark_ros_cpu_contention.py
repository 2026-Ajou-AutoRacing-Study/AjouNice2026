#!/usr/bin/env python3
"""Isolated, bounded ROS open-loop CPU contention qualification. No UDP or training.

Source scripts/workspace/source_aw149.sh first. Fresh output required. An
optional Base5 cost-only inference process is independent of GT-driven planning;
it must never be confused with an end-to-end learned-detector accuracy test.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import threading
import time

import numpy as np
import psutil
import rosbag
import rosgraph
import rospy
import yaml
from autoware_perception_msgs.msg import DetectedObjects, PredictedObjects, TrafficLightGroupArray
from autoware_planning_msgs.msg import Trajectory
from autoware_planning_msgs.srv import SetWaypointRoute, SetWaypointRouteRequest
from morai_msgs.msg import CtrlCmd
from nav_msgs.msg import Odometry


def stats(values):
    if not values:return dict(count=0)
    return dict(count=len(values),mean=float(np.mean(values)),
                p95=float(np.percentile(values,95)),maximum=float(max(values)))


def balanced_subset(source,count):
    representative=[r for r in source['samples'] if r['sample_group']=='representative']
    stress=[r for r in source['samples'] if r['sample_group']=='stress']
    selected=[stress[index//3] if index%3==2 else representative[index-index//3]
              for index in range(count)]
    if len({row['id'] for row in selected})!=len(selected):
        raise ValueError('CPU probe selection must use distinct frames')
    return selected


def raw_topics(info):
    return [topic for topic in info if topic.startswith('/sensor/') or topic in
            ('/Ego_topic','/Competition_topic','/Object_topic')]


def active_outputs(topic_stats,traffic_light=False):
    active=all(topic_stats.get(key,{}).get('count',0)>10 for key in
               ('localization','prediction','trajectory','control'))
    active=active and all(topic_stats.get(key,{}).get('nonempty',0)>10 for key in
                         ('prediction','trajectory'))
    return active and (not traffic_light or topic_stats.get('traffic',{}).get('count',0)>10)


class Probe:
    def __init__(self,args):
        self.args=args;self.root=Path(__file__).resolve().parents[2]
        self.processes=[];self.files=[];self.metrics={};self.resource_rows=[]
        self.lock=threading.Lock();self.stop=threading.Event();self.container_pid=None
        self.env=os.environ.copy()
        with socket.socket() as s:
            s.bind(('127.0.0.1',0));self.port=s.getsockname()[1]
        self.env.update(ROS_MASTER_URI='http://127.0.0.1:'+str(self.port),ROS_IP='127.0.0.1',
                        ROS_HOME=str(args.output/'ros_home'))
        os.environ.update({k:self.env[k] for k in ('ROS_MASTER_URI','ROS_IP','ROS_HOME')})

    def launch(self,name,command):
        log=(self.args.output/(name+'.log')).open('w');self.files.append(log)
        p=subprocess.Popen(command,cwd=self.root,env=self.env,stdout=log,stderr=subprocess.STDOUT,
                           start_new_session=True)
        self.processes.append((name,p));return p

    def monitor(self):
        known={};psutil.cpu_percent(percpu=True)
        while not self.stop.wait(.5):
            now=time.monotonic();groups={}
            for name,parent in self.processes:
                try:children=[psutil.Process(parent.pid),*psutil.Process(parent.pid).children(recursive=True)]
                except psutil.Error:continue
                if name=='inference' and self.container_pid:
                    try:
                        container=psutil.Process(self.container_pid)
                        children.extend([container,*container.children(recursive=True)])
                    except psutil.Error:pass
                children=list({proc.pid:proc for proc in children}.values())
                cpu=0.;threads=0;rss=0
                for proc in children:
                    try:
                        if proc.pid not in known:
                            known[proc.pid]=proc;proc.cpu_percent()
                        cpu+=known[proc.pid].cpu_percent();threads+=proc.num_threads();rss+=proc.memory_info().rss
                    except psutil.Error:pass
                groups[name]=dict(cpu_percent_one_core_100=cpu,threads=threads,rss_mib=rss/2**20)
            self.resource_rows.append(dict(monotonic=now,per_cpu_percent=psutil.cpu_percent(percpu=True),
                groups=groups,loadavg=list(os.getloadavg())))

    def callback(self,key):
        def receive(msg):
            now=time.monotonic()
            n=len(getattr(msg,'objects',getattr(msg,'points',[])))
            with self.lock:self.metrics.setdefault(key,[]).append((now,n))
        return receive

    def wait_for(self,predicate,timeout,reason):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            if predicate():return
            for name,p in self.processes:
                if p.poll() is not None:raise RuntimeError(name+' exited; inspect its log')
            time.sleep(.1)
        raise TimeoutError(reason)

    def route(self,ego):
        rospy.wait_for_service('/planning/set_waypoint_route',timeout=25)
        transform=yaml.safe_load((self.root/'configs/tf/morai_lanelet_static_tf.yaml').read_text())['map_to_morai_map']
        if transform['rpy']!=[0.,0.,0.]:raise ValueError('Probe requires the audited translation-only map TF')
        req=SetWaypointRouteRequest();req.header.frame_id='map';req.header.stamp=rospy.Time.now()
        req.goal_pose.position.x=ego.position.x+transform['xyz'][0]
        req.goal_pose.position.y=ego.position.y+transform['xyz'][1]
        req.goal_pose.position.z=ego.position.z+transform['xyz'][2]
        yaw=math.radians(ego.heading-1.347)
        req.goal_pose.orientation.z=math.sin(yaw/2);req.goal_pose.orientation.w=math.cos(yaw/2)
        req.uuid.uuid=list(range(16));req.allow_modification=False
        result=rospy.ServiceProxy('/planning/set_waypoint_route',SetWaypointRoute)(req)
        (self.args.output/'route_result.json').write_text(json.dumps(dict(success=result.status.success,
            message=result.status.message,goal=[req.goal_pose.position.x,req.goal_pose.position.y,
                req.goal_pose.position.z,yaw],goal_bag_fraction=self.args.goal_fraction,
            scope='Isolated replay-only route to a recorded Ego pose; no planner goal checks relaxed'))+'\n')
        if not result.status.success:raise RuntimeError('Replay route request failed: '+result.status.message)

    def run(self):
        a=self.args
        with rosbag.Bag(str(a.bag)) as b:
            info=b.get_type_and_topic_info().topics;duration=b.get_end_time()-b.get_start_time()
            required=['/Ego_topic','/Competition_topic','/Object_topic','/sensor/gps','/sensor/imu']
            if a.mode=='stack' and not all(t in info for t in required):raise ValueError('Missing raw stack inputs')
            ego=None
            cutoff=b.get_start_time()+duration*a.goal_fraction
            for _,msg,stamp in b.read_messages(topics=['/Ego_topic']):
                if stamp.to_sec()<=cutoff:ego=msg
        # Never replay recorded downstream outputs, TF or control commands.
        topics=raw_topics(info)
        self.launch('roscore',['roscore','-p',str(self.port)])
        self.wait_for(lambda:rosgraph.is_master_online(),15,'Private ROS master did not start')
        rospy.init_node('cpu_contention_monitor',anonymous=True,disable_signals=True)
        rospy.set_param('/use_sim_time',True)
        subscriptions=[]
        for key,topic,typ in [('localization','/localization/kinematic_state',Odometry),
             ('detections','/perception/object_recognition/detection/objects',DetectedObjects),
             ('prediction','/perception/object_recognition/objects',PredictedObjects),
             ('trajectory','/planning/trajectory',Trajectory),('control','/ctrl_cmd',CtrlCmd),
             ('traffic','/perception/traffic_light_recognition/traffic_signals',TrafficLightGroupArray)]:
            subscriptions.append(rospy.Subscriber(topic,typ,self.callback(key),queue_size=100))
        if a.mode=='stack':
            self.launch('localization',['roslaunch','bringup','localization_gnss_imu_gt_ctra.launch',
                'gt_source:=competition','use_sim_time:=true','publish_sensor_tf:=false'])
            self.launch('tracking',['roslaunch','perception_object_pipeline','object_pipeline.launch',
                'start_detector:=false','prediction_model:=map_based','use_sim_time:=true',
                'maximum_tracking_internal_prediction_age_sec:=0.5'])
            self.launch('object_throttle',['rosrun','topic_tools','throttle','messages','/Object_topic','13',
                '/cpu_probe/Object_topic'])
            self.launch('gt',['roslaunch','perception_object_pipeline','morai_gt_detection.launch',
                'input_objects:=/cpu_probe/Object_topic'])
            self.launch('planning',['roslaunch','ajounice_planning_integration','planning.launch',
                'start_requester:=false','enable_traffic_light_planner:='+str(a.traffic_light).lower(),
                'behavior_path_forward_length:=400.0','intersection_attention_area_length:=100.0'])
            self.launch('control',['roslaunch','trajectory_follower','pp_pi_autoware.launch',
                'kinematic_state_topic:=/localization/control_kinematic_state'])
            if a.traffic_light:
                self.launch('traffic_light',['roslaunch','traffic_light_detection','traffic_light_node.launch',
                    'inference_python:='+str(Path.home()/'miniconda3/envs/traffic_light_gpu/bin/python'),
                    'isolate_python:=true','device:=0','publish_debug_image:=false'])
        if a.gpu_probe:
            croot='/workspace/AjouNice2026';out=croot+'/'+str(a.output.relative_to(self.root))
            architectures=subprocess.check_output(['nvidia-smi','--query-gpu=compute_cap',
                '--format=csv,noheader'],text=True).strip().splitlines()
            if len(architectures)!=1:
                raise ValueError('This bounded probe requires one visible GPU')
            original=self.root/'artifacts/evaluation/bevfusion/base5_parallel_261011/selection_300.json'
            source=json.loads(original.read_text())
            selected=balanced_subset(source,a.iterations)
            selection=dict(samples=selected,parent_selection_sha256=hashlib.sha256(original.read_bytes()).hexdigest(),
                scope='Deterministic distinct 2:1 representative/stress CPU contention subset; no AP',
                sampled_groups=dict(Counter(r['sample_group'] for r in selected)))
            (a.output/'selection.json').write_text(json.dumps(selection,indent=2)+'\n')
            cmd=['docker','run','--rm','--name',a.container_name,'--gpus','all','--shm-size','4g',
                '-v',str(self.root)+':'+croot,'-w',croot,
                '-e','TORCH_EXTENSIONS_DIR='+croot+'/artifacts/cache/base5_parallel_torch_extensions_261011/py311_cu128',
                '-e','TORCH_CUDA_ARCH_LIST='+architectures[0].strip(),
                '-e','PYTHONPATH='+':'.join([croot,'/tmp/cpu_probe_mmdetection3d',
                    croot+'/docker/bevfusion',croot+'/perception/detection3d/bevfusion/scripts']),
                'ajounice/bevfusion:team-cu128-v1','bash','-c',
                'set -e; cp -a '+croot+'/third_party/mmdetection3d /tmp/cpu_probe_mmdetection3d; '
                'for op in bev_pool voxel; do cp /opt/mmdetection3d/projects/BEVFusion/bevfusion/ops/$op/*.so '
                '/tmp/cpu_probe_mmdetection3d/projects/BEVFusion/bevfusion/ops/$op/; done; exec "$@"',
                'cpu-probe','python',
                'perception/detection3d/bevfusion/scripts/benchmark_base5_inference_runtime.py',
                '--data',croot+'/artifacts/handoff/base5_sunny13_jpeg90_upright_v1_261011_release',
                '--selection',out+'/selection.json',
                '--config',croot+'/perception/detection3d/bevfusion/configs/bevfusion_base5_dense_512_depth25_runtime_candidate.py',
                '--checkpoint',croot+'/artifacts/evaluation/bevfusion/base5_inference_remote_261011/d25_smoke_inference.pth',
                '--sensor-config',croot+'/configs/sensors/Sensor_competition_local.json',
                '--output',out+'/gpu','--grid','common050','--variants','baseline',
                '--iterations',str(a.iterations),'--stage-iterations','8','--memory-budget-gib','12',
                '--cpu-threads',str(a.cpu_threads),'--camera-workers',str(a.camera_workers),
                '--opencv-threads','1','--ready-file',out+'/ready.json','--start-file',out+'/start.json']
            self.launch('inference',cmd)
        monitor=threading.Thread(target=self.monitor,daemon=True);monitor.start()
        if a.gpu_probe:
            self.wait_for(lambda:(a.output/'ready.json').exists(),180,'GPU warmup did not finish')
            self.container_pid=int(subprocess.check_output(['docker','inspect','--format',
                '{{.State.Pid}}',a.container_name],text=True).strip())
        player=self.launch('bag',['rosbag','play','--quiet','--clock','--rate=1','--duration='+str(a.duration),
                                 str(a.bag),'--topics',*topics])
        start=time.monotonic()
        if a.mode=='stack':
            self.wait_for(lambda:len(self.metrics.get('localization',[]))>=20,15,'No localization messages')
            self.route(ego)
            self.wait_for(lambda:any(n for _,n in self.metrics.get('trajectory',[])),20,'Planner never produced a path')
            self.wait_for(lambda:any(n for _,n in self.metrics.get('prediction',[])),10,'No predicted actors')
        if a.traffic_light:self.wait_for(lambda:len(self.metrics.get('traffic',[]))>=10,20,'No traffic-light outputs')
        for index in range(a.stress_cores):
            self.launch('stress_'+str(index),['/usr/bin/python3','-c',
                'import time; end=time.monotonic()+90; x=1\n'
                'while time.monotonic()<end: x=(x*1664525+1013904223)%4294967296'])
        time.sleep(max(0,a.settle_seconds-(time.monotonic()-start)))
        (a.output/'start.json').write_text(json.dumps(dict(monotonic=time.monotonic()))+'\n')
        if a.gpu_probe:
            p=dict(self.processes)['inference']
            if p.wait(timeout=180)!=0:raise RuntimeError('GPU probe failed')
            inference=json.loads((a.output/'gpu/report.json').read_text())['cases'][0]
            begin=inference['per_sample'][0]['monotonic_start'];end=inference['per_sample'][-1]['monotonic_end']
        else:
            begin=time.monotonic()
            time.sleep(max(0,min(a.duration-(begin-start)-2,35)));end=time.monotonic();inference=None
        if end-start>min(duration,a.duration):raise RuntimeError('Inference window exceeds live bag duration')
        rows=[r for r in self.resource_rows if begin<=r['monotonic']<=end]
        topic_stats={}
        for key,events in self.metrics.items():
            selected=[(t,n) for t,n in events if begin<=t<=end]
            times=[t for t,_ in selected]
            topic_stats[key]=dict(count=len(times),rate_hz=len(times)/(end-begin),
                nonempty=sum(n>0 for _,n in selected),object_or_point_count=stats([n for _,n in selected]),
                wall_interval_ms=stats([1000*(y-x) for x,y in zip(times,times[1:])]))
        resource=dict(samples=len(rows),per_core_mean=np.mean([r['per_cpu_percent'] for r in rows],axis=0).tolist(),
            total_cpu_percent=stats([np.mean(r['per_cpu_percent']) for r in rows]),
            busiest_core_percent=stats([max(r['per_cpu_percent']) for r in rows]),groups={})
        for key in set(k for row in rows for k in row['groups']):
            resource['groups'][key]=stats([r['groups'][key]['cpu_percent_one_core_100'] for r in rows if key in r['groups']])
        active=active_outputs(topic_stats,a.traffic_light)
        result=dict(valid=a.mode=='idle' or active,mode=a.mode,bag=str(a.bag),
            playback_rate=1,window_seconds=end-begin,measurement_start_after_play_sec=begin-start,
            traffic_light=a.traffic_light,synthetic_busy_cores=a.stress_cores,
            ros_master=self.env['ROS_MASTER_URI'],
            cpu_threads=a.cpu_threads,camera_workers=a.camera_workers,topics=topic_stats,resource=resource,
            inference={k:v for k,v in inference.items() if k not in ('per_sample','parity')} if inference else None,
            scope='Open-loop raw bag, live localization/tracking/prediction/planning/control. Independent cost-only Base5 frames, not end-to-end detector AP.',
            excluded=([] if a.traffic_light else ['traffic-light inference'])+['RViz','UDP network bridge','closed-loop dynamics'])
        (a.output/'resources.json').write_text(json.dumps(self.resource_rows)+'\n')
        (a.output/'topic_events.json').write_text(json.dumps(self.metrics)+'\n')
        (a.output/'report.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(dict(valid=result['valid'],mode=a.mode,traffic_light=a.traffic_light,
            stress_cores=a.stress_cores,window_seconds=end-begin,
            total_cpu=resource['total_cpu_percent'],
            topic_rates={key:row['rate_hz'] for key,row in topic_stats.items()},
            timing=inference['timing']['all']['timing'] if inference else None),indent=2))
        if not result['valid']:raise RuntimeError('Not all real stack outputs were active')

    def close(self):
        self.stop.set()
        if self.args.gpu_probe:
            subprocess.run(['docker','stop','-t','3',self.args.container_name],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,timeout=15)
        for _,p in reversed(self.processes):
            # An exited roslaunch parent can leave children in its own group.
            try:os.killpg(p.pid,signal.SIGINT)
            except ProcessLookupError:pass
        for _,p in reversed(self.processes):
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:os.killpg(p.pid,signal.SIGKILL)
                except ProcessLookupError:pass
        for file in self.files:file.close()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bag',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--mode',choices=['idle','stack'],required=True);p.add_argument('--gpu-probe',action='store_true')
    p.add_argument('--traffic-light',action='store_true')
    p.add_argument('--settle-seconds',type=float,default=25.)
    p.add_argument('--goal-fraction',type=float,default=1.,
                   help='Recorded Ego pose at this fraction of the bag provides the replay-only route goal')
    p.add_argument('--stress-cores',type=int,choices=[0,2,4],default=0,
                   help='Additional bounded synthetic CPU workers, not an actual ROS stack component')
    p.add_argument('--duration',type=float,default=60.);p.add_argument('--iterations',type=int,default=96)
    p.add_argument('--cpu-threads',type=int,choices=[2,4,8],default=4)
    p.add_argument('--camera-workers',type=int,choices=[2,4],default=4)
    a=p.parse_args();a.bag=a.bag.resolve();a.output=a.output.resolve()
    if a.output.exists() or not 10<=a.duration<=300:p.error('Fresh output and bounded duration required')
    if not 0<=a.settle_seconds<a.duration-5:p.error('Settle time must leave a live measurement window')
    if a.traffic_light and a.mode!='stack':p.error('Traffic-light inference requires the stack')
    if not .5<=a.goal_fraction<=1:p.error('Recorded route goal fraction must be in .5..1')
    if not 24<=a.iterations<=300:p.error('24..300 distinct requests required')
    a.output.mkdir(parents=True);a.container_name='ajou-cpu-probe-'+str(os.getpid())
    probe=Probe(a)
    try:probe.run()
    except Exception as exc:
        (a.output/'failure.json').write_text(json.dumps(dict(valid=False,error=repr(exc)))+'\n')
        (a.output/'resources.json').write_text(json.dumps(probe.resource_rows)+'\n')
        raise
    finally:probe.close()


if __name__=='__main__':main()
