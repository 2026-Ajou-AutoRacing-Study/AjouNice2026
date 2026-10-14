#!/usr/bin/env python3
"""CPU-only actual raw-bag bridge prep parity, no network/GPU/ROS nodes.

Imports the actual-class offline fixture. Fake transport returns one fixed
box: output parity here validates instrumentation, not model accuracy.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
import time
from unittest.mock import patch

import numpy as np

ROOT=Path(__file__).resolve().parents[2]
PACKAGE=ROOT/'base_ws/src/perception/bevfusion_ros'
sys.path[:0]=[str(PACKAGE/'test'),str(PACKAGE/'src'),str(ROOT/'base_ws/src/perception_object_pipeline/src')]
from test_frame_timing import bridge, module, wire
from bevfusion_ros.frame_timing import FrameTimingLedger
from profile_pipeline_cpu_offline import stats, file_digest


def normalize_metadata(metadata):
    result=dict(metadata)
    # Durations naturally differ between runs. Their definitions/positions are
    # separately qualified by the fixed-clock actual-class unit tests.
    for key in ('pointcloud_ms','host_preprocess_ms'):
        result.pop(key,None)
    return result


def main():
    import rosbag
    import rospy
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--frames',type=int,default=20)
    args=parser.parse_args()
    if not 1<=args.frames<=500:parser.error('frames must be 1..500')
    if args.output.exists():raise FileExistsError('Fresh output required')
    args.output.mkdir(parents=True)
    instances={}
    for enabled in (False,True):
        obj=bridge(enabled)
        obj.sensor_config=str(ROOT/'configs/sensors/Sensor_competition_local.json')
        obj.camera_topics,obj.camera_shapes,obj.lidar_topic,obj.lidar_position,obj.lidar_yaw=obj._load_topics()
        if enabled:obj.frame_timing=FrameTimingLedger()
        instances[enabled]=obj
    cameras={topic:frame for frame,topic in instances[False].camera_topics.items()}
    topics=list(cameras)+[instances[False].lidar_topic]
    costs={False:[],True:[]};counts=Counter();examples=[]
    with rosbag.Bag(str(args.bag))as bag:
        for topic,message,receipt in bag.read_messages(topics=topics):
            if topic in cameras:
                for obj in instances.values():obj._camera_callback(message,cameras[topic])
                continue
            counts['lidar_callbacks']+=1
            outputs={}
            for enabled in ((False,True)if counts['lidar_callbacks']%2 else(True,False)):
                obj=instances[enabled]
                with patch.object(module.rospy.Time,'now',return_value=receipt), \
                     patch.object(module.rospy,'logwarn_throttle'):
                    obj._lidar_callback(message)
                    if obj.raw_pending is None:continue
                    frame=obj.raw_pending;obj.raw_pending=None
                    if enabled:obj.frame_timing.stage(frame[-1],'preprocess_start')
                    start=time.perf_counter()
                    prepared=obj._prepare_frame(*frame)
                    costs[enabled].append((time.perf_counter()-start)*1000.)
                    if enabled:
                        obj.frame_timing.stage(frame[-1],'preprocess_end')
                        obj.frame_timing.stage(frame[-1],'prepared_enqueued')
                    if len(prepared)!=4:raise AssertionError('Prepared tuple changed')
                    obj._request_and_publish(prepared)
                outputs[enabled]=(normalize_metadata(prepared[0]),prepared[1],prepared[2].to_nsec(),
                                  wire(obj.objects_pub.messages[-1]),wire(obj.marker_pub.messages[-1]))
            if outputs.keys() != {False,True}:
                if outputs:raise AssertionError('ON/OFF selection accepted different input')
                counts['both_rejected']+=1;continue
            if outputs[False] != outputs[True]:raise AssertionError('Raw-bag input/output differs')
            counts['exact_frames']+=1
            if len(examples)<3:
                metadata,blobs,stamp,*_=outputs[True]
                examples.append(dict(stamp_ns=stamp,point_count=metadata['point_count'],
                                     blob_sha256={name:hashlib.sha256(value).hexdigest()for name,value in blobs.items()}))
            if counts['exact_frames']>=args.frames:break
    if counts['exact_frames']<args.frames:raise ValueError('Insufficient synchronized input')
    obj=instances[True]
    report=dict(bag=str(args.bag),bag_sha256=file_digest(args.bag),counts=dict(counts),
        sensor_config_sha256=obj.sensor_config_sha256,
        bridge_sha256=file_digest(PACKAGE/'scripts/bevfusion_ros_bridge.py'),
        helper_sha256=file_digest(PACKAGE/'src/bevfusion_ros/frame_timing.py'),
        host_prepare_ms={'off':stats(costs[False]),'on':stats(costs[True])},examples=examples,
        ledger_pending=len(obj.frame_timing.frames),ledger_active=len(obj.frame_timing.active),
        counters=obj.frame_timing.counters,gpu_inference_performed=False,
        equality_scope='Original JPEG/XYZI, semantic request metadata, selected prepared stamp, exact DetectedObjects/MarkerArray wire. Dynamic duration numbers excluded; legacy Float32 scope qualified separately by fixed-clock units.',
        limits=['20 input frames by default, not a latency distribution/certification.',
                'CPU prep only; fake transport fixed box; no real model inference/network.',
                'Saved camera receipt ordering and frozen ROS bag receipt clock; sequential simulation does not measure live queues.'])
    (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':main()
