#!/usr/bin/env python3
"""Exercise the actual host bridge on recorded ROS messages, without a master.

No mock sensor messages or copied image-preprocessing implementation. This
isolates bridge preparation/transport from real-time queue and LAN scheduling.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import importlib.util
import json
from pathlib import Path
import pickle
import sys
import time

import numpy as np
import rosbag

ROOT = Path(__file__).resolve().parents[4]
sys.path[:0] = [str(ROOT / 'base_ws/src/perception/bevfusion_ros/src'),
               str(ROOT / 'base_ws/src/perception_object_pipeline/src')]
from bevfusion_ros.protocol import encode_message, TYPE_REQUEST
from prepare_base5_rosbag_quick import nearest, CASES
from tracking_prediction_evaluation.pipeline_evaluator import message_timestamp


def build(inputs, output, image_encoding='compressed'):
    if output.exists():
        raise ValueError('Fresh parity output required')
    path = ROOT / 'base_ws/src/perception/bevfusion_ros/scripts/bevfusion_ros_bridge.py'
    spec = importlib.util.spec_from_file_location('actual_bevfusion_bridge', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    bridge = module.BevFusionRosBridge.__new__(module.BevFusionRosBridge)
    bridge.sensor_config = str(ROOT / 'configs/sensors/Sensor_4.json')
    bridge.camera_order = ['camera_front', 'camera_front_left', 'camera_front_right', 'camera_back']
    bridge.intrinsic_source = 'horizontal_fov'
    bridge.operation = 'inference'
    bridge.lidar_deskew_mode = 'off'
    bridge.temporal_feature_cache = False
    bridge.query_memory_context = False
    bridge.temporal_sweeps = 0
    bridge.sequence = 0
    bridge.model_score_threshold = .1
    bridge.max_detections = 100
    bridge.image_final_height = 512
    bridge.image_final_width = 1056
    bridge.image_resize = .825
    bridge.image_reference_width = 1280.
    bridge.image_encoding = image_encoding
    bridge.point_intensity_source = 'ros255'
    bridge.camera_executor = ThreadPoolExecutor(max_workers=4)
    (bridge.camera_topics, bridge.camera_shapes, bridge.lidar_topic,
     bridge.lidar_position, bridge.lidar_yaw) = bridge._load_topics()
    manifest = json.loads((inputs / 'manifest.json').read_text())
    with (inputs / 'infos_val.pkl').open('rb') as handle:
        rows = pickle.load(handle)['data_list']
    output.mkdir(parents=True)
    receipts = []
    try:
        for case in CASES:
            candidates = [r for r in rows if r['scene_id'] == case and r['instances']]
            selected = [candidates[int(i)] for i in np.unique(np.rint(
                np.linspace(0, len(candidates)-1, min(3, len(candidates)))).astype(int))]
            topics = list(bridge.camera_topics.values()) + [bridge.lidar_topic]
            messages = {topic: [] for topic in topics}
            with rosbag.Bag(manifest['cases'][case]['raw_bag']) as bag:
                for topic, msg, stamp in bag.read_messages(topics=topics):
                    messages[topic].append((message_timestamp(msg, stamp), msg))
            for topic in topics:
                messages[topic] = sorted({t: m for t, m in messages[topic]}.items())
            for row in selected:
                stamp = row['timestamp']
                lidar = nearest(messages[bridge.lidar_topic], stamp, 1e-6)[1]
                selected_images = {frame: nearest(messages[topic], stamp, .05)
                                   for frame, topic in bridge.camera_topics.items()}
                images = {frame: item[1] for frame, item in selected_images.items()}
                differences = {frame: item[0]-stamp for frame, item in selected_images.items()}
                metadata, blobs, _, _ = bridge._prepare_frame(
                    lidar, images, differences, None, None, stamp, time.monotonic())
                raw = np.fromfile(inputs / row['lidar_points']['lidar_path'], dtype='<f4')
                parsed = np.frombuffer(blobs['points'], dtype='<f4')
                if parsed.shape != raw.shape or not np.all((parsed == raw) | (np.isnan(parsed) & np.isnan(raw))):
                    raise ValueError('Actual bridge XYZI differs from native bag extraction')
                request = encode_message(TYPE_REQUEST, metadata, blobs)
                request_path = output / (row['token'] + '.request')
                request_path.write_bytes(request)
                receipts.append(dict(token=row['token'], request=request_path.name,
                    sha256=hashlib.sha256(request).hexdigest(), stamp=stamp,
                    pointcloud_exact=True, metadata=metadata))
    finally:
        bridge.camera_executor.shutdown(wait=True)
    report = dict(scope='Actual host bridge preparation and binary framing from raw ROS messages; no live ROS master or queue certification',
                  image_encoding=image_encoding,
                  bridge_sha256=hashlib.sha256(path.read_bytes()).hexdigest(), requests=receipts)
    (output / 'manifest.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(dict(event='actual_bridge_requests_complete', frames=len(receipts))))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--inputs', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--image-encoding',choices=('rgb8','compressed'),default='compressed')
    a = p.parse_args()
    build(a.inputs, a.output,a.image_encoding)
