#!/usr/bin/env python3
"""Adapt frozen E12 overlays to the shared map/vehicle RViz coordinate contract.

Offline bag I/O only. Existing exports/raw bags are never rewritten, inference
and Map-CV are not rerun, and no ROS master, GUI, controller or MORAI is started.
"""
import argparse
import copy
import gzip
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / 'artifacts/evaluation/bevfusion/e12_postprocessing_261023_v1'
CASES = {'H04': 'september/H04_162616', 'H03': 'core/H03_162301', 'R01': 'core/R01_163034'}
MARKER_TOPICS = tuple('/e12_view/' + name for name in (
    'gt', 'detections', 'tracks', 'velocity', 'prediction3', 'prediction5',
    'prefix5', 'gt_future', 'hud'))
TF_TOPIC = '/e12_view/tf'


def digest(path):
    result = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(8 << 20), b''):
            result.update(block)
    return result.hexdigest()


def read_json(path):
    with (gzip.open if str(path).endswith('.gz') else open)(path, 'rt') as stream:
        return json.load(stream)


def pose_matrix(xyz, quaternion):
    """Homogeneous pose; reject invalid ADD marker quaternion, never guess it."""
    x, y, z, w = map(float, quaternion)
    norm = x*x + y*y + z*z + w*w
    if not math.isfinite(norm) or norm < 1.e-20:
        raise ValueError('A finite nonzero pose quaternion is required')
    scale = 2. / norm
    result = np.eye(4)
    result[:3, :3] = [
        [1.-scale*(y*y+z*z), scale*(x*y-z*w), scale*(x*z+y*w)],
        [scale*(x*y+z*w), 1.-scale*(x*x+z*z), scale*(y*z-x*w)],
        [scale*(x*z-y*w), scale*(y*z+x*w), 1.-scale*(x*x+y*y)]]
    result[:3, 3] = xyz
    if not np.isfinite(result).all():
        raise ValueError('Nonfinite pose')
    return result


def quaternion_for_matrix(matrix):
    """Stable XYZW quaternion for a proper rotation matrix."""
    r = np.asarray(matrix, dtype=float)[:3, :3]
    candidates = np.array([1.+r[0,0]-r[1,1]-r[2,2],
        1.-r[0,0]+r[1,1]-r[2,2], 1.-r[0,0]-r[1,1]+r[2,2],
        1.+np.trace(r)])
    i = int(np.argmax(candidates))
    s = 2. * math.sqrt(max(0., candidates[i]))
    if s < 1.e-12:
        raise ValueError('Invalid rotation matrix')
    if i == 0:
        q = [s/4., (r[0,1]+r[1,0])/s, (r[0,2]+r[2,0])/s, (r[2,1]-r[1,2])/s]
    elif i == 1:
        q = [(r[0,1]+r[1,0])/s, s/4., (r[1,2]+r[2,1])/s, (r[0,2]-r[2,0])/s]
    elif i == 2:
        q = [(r[0,2]+r[2,0])/s, (r[1,2]+r[2,1])/s, s/4., (r[1,0]-r[0,1])/s]
    else:
        q = [(r[2,1]-r[1,2])/s, (r[0,2]-r[2,0])/s, (r[1,0]-r[0,1])/s, s/4.]
    return np.asarray(q) / np.linalg.norm(q)


def yaw_matrix(xyz, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    result = np.eye(4)
    result[:2, :2] = [[c, -s], [s, c]]
    result[:3, 3] = xyz
    return result


def map_from_morai(config):
    roll, pitch, yaw = map(float, config['rpy'])
    # Current surveyed alignment is planar. Fail explicitly if it changes.
    if abs(roll) > 1.e-12 or abs(pitch) > 1.e-12:
        raise ValueError('The audited map/MORAI alignment must remain planar')
    return yaw_matrix(config['xyz'], yaw)


def map_from_lidar(frame, extrinsic, alignment):
    ego = frame['ego']
    # Stored yaw already includes the recorded -1.347 degree correction once.
    return alignment @ yaw_matrix([ego[k] for k in ('x', 'y', 'z')], ego['yaw']) @ extrinsic


def marker_pose(marker):
    p, q = marker.pose.position, marker.pose.orientation
    return pose_matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])


def transform_array(message, transform):
    """Premultiply marker pose, not its local points as well (no double TF)."""
    output = copy.deepcopy(message)
    inverse = np.linalg.inv(transform)
    max_error = 0.
    for original, marker in zip(message.markers, output.markers):
        if original.header.frame_id.lstrip('/') != 'lidar1':
            raise ValueError('Source marker is not in the audited lidar1 frame')
        marker.header.frame_id = 'map'
        if marker.action != 0:  # DELETE/DELETEALL may have an unset quaternion.
            continue
        source_pose = marker_pose(original)
        new_pose = transform @ source_pose
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = new_pose[:3, 3]
        q = quaternion_for_matrix(new_pose)
        marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w = q
        restored = inverse @ marker_pose(marker)
        error = float(np.max(np.abs(restored-source_pose)))
        max_error = max(max_error, error)
        if error > 1.e-9:
            raise ValueError('Marker pose map/LiDAR roundtrip failed')
        # All non-pose fields must survive exactly, including IDs, colors,
        # text, local points, box scale, lifetime and receipt-time header stamp.
        audit = copy.deepcopy(marker)
        audit.header = copy.deepcopy(original.header)
        audit.pose = copy.deepcopy(original.pose)
        if audit != original:
            raise ValueError('Conversion changed non-pose marker content')
    return output, max_error


def source_receipts(raw_bag, frames):
    """Unique raw LiDAR header→receipt association, never zip clocks blindly."""
    import rosbag
    wanted = {float(frame['stamp']) for frame in frames}
    found = {}
    with rosbag.Bag(str(raw_bag), 'r') as bag:
        for _, message, receipt in bag.read_messages(topics=['/sensor/lidar1']):
            stamp = float(message.header.stamp.to_sec())
            if stamp not in wanted:
                continue
            if stamp in found or message.header.frame_id.lstrip('/') != 'lidar1':
                raise ValueError('Ambiguous LiDAR header or unexpected source frame')
            found[stamp] = (int(message.header.stamp.to_nsec()), int(receipt.to_nsec()))
    if set(found) != wanted:
        raise ValueError('Frozen pack contains unmatched raw LiDAR header stamps')
    result = [found[float(frame['stamp'])] for frame in frames]
    if any(b[1] <= a[1] for a, b in zip(result, result[1:])):
        raise ValueError('Source LiDAR receipt order is not strictly increasing')
    return result


def dynamic_tf(frame, header_nsec):
    import rospy
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    value = TransformStamped()
    value.header.frame_id, value.child_frame_id = 'morai_map', 'base_link'
    value.header.stamp = rospy.Time(header_nsec//10**9, header_nsec%10**9)
    ego = frame['ego']
    value.transform.translation.x, value.transform.translation.y, value.transform.translation.z = [ego[k] for k in ('x','y','z')]
    value.transform.rotation.z, value.transform.rotation.w = math.sin(ego['yaw']/2.), math.cos(ego['yaw']/2.)
    return TFMessage(transforms=[value])


def prepare(case_name, output, source=None, pack=None):
    import rosbag
    import rospy
    output = Path(output)
    if output.exists():
        raise FileExistsError('A fresh output directory is required')
    source = Path(source or BASE/'rviz_exports_v1'/case_name/'scene.json')
    pack_path = Path(pack or BASE/'tracker_v1/replay_pack.json.gz')
    scene, core = read_json(source), read_json(pack_path)
    if scene.get('schema') != 'e12_postprocessing_rviz_scene_v1' or scene.get('complete') is not True:
        raise ValueError('Completed original E12 viewer export required')
    if scene['case'] != case_name or set(scene['topics']) != set(MARKER_TOPICS) or scene['fixed_frame'] != 'lidar1':
        raise ValueError('Unexpected source viewer contract')
    if core.get('complete') is not True or core.get('schema') != 'e12_postprocessing_core_pack_v1':
        raise ValueError('Completed frozen core pack required')
    case = core['cases'][CASES[case_name]]
    original_bag = Path(scene['marker_bag'])
    if digest(original_bag) != scene['marker_bag_sha256']:
        raise ValueError('Original marker bag no longer matches its frozen receipt')
    pack_sha = digest(pack_path)
    if not any(Path(name).resolve() == pack_path.resolve() and sha == pack_sha
               for name, sha in scene['source_sha256'].items()):
        raise ValueError('Original viewer and current frozen pack differ')
    sensor = ROOT/'configs/sensors/Sensor_competition_lan.json'
    tf_path = ROOT/'configs/tf/morai_lanelet_static_tf.yaml'
    for path in (sensor, tf_path):
        if scene['source_sha256'].get(str(path)) != digest(path):
            raise ValueError('Source coordinate configuration differs: ' + str(path))
    sys.path.insert(0, str(ROOT/'docker/bevfusion'))
    from app.sensor4_calibration import load_sensor4_calibration
    extrinsic = np.asarray(load_sensor4_calibration(sensor)['base_from_lidar'], float)
    alignment = map_from_morai(yaml.safe_load(tf_path.read_text())['map_to_morai_map'])
    receipts = source_receipts(scene['raw_bag'], case['frames'])
    by_receipt = {receipt: (frame, header) for frame, (header, receipt) in zip(case['frames'], receipts)}
    if len(by_receipt) != len(case['frames']) or len(case['frames']) != scene['framecount']:
        raise ValueError('Frame count or source receipt is ambiguous')
    sources = {str(path.resolve()): digest(path) for path in (source, original_bag, pack_path, sensor, tf_path, Path(__file__))}
    counts, seen, max_error, tf_count = {}, set(), 0., 0
    output.mkdir(parents=True, exist_ok=False)
    target_bag = output/'markers.bag'
    with rosbag.Bag(str(original_bag), 'r') as reader, rosbag.Bag(str(target_bag), 'w') as writer:
        for topic, message, receipt in reader.read_messages():
            receipt_nsec = int(receipt.to_nsec())
            if topic not in MARKER_TOPICS or receipt_nsec not in by_receipt:
                raise ValueError('Unexpected marker source topic or timestamp')
            key = (receipt_nsec, topic)
            if key in seen:
                raise ValueError('Duplicate source marker topic/receipt')
            frame, header_nsec = by_receipt[receipt_nsec]
            if not any(stamp == receipt_nsec for stamp, _ in seen):
                writer.write(TF_TOPIC, dynamic_tf(frame, header_nsec), receipt)
                tf_count += 1
            if any(marker.header.stamp.to_nsec() != receipt_nsec for marker in message.markers):
                raise ValueError('Original marker header differs from frozen receipt')
            converted, error = transform_array(message, map_from_lidar(frame, extrinsic, alignment))
            max_error = max(max_error, error)
            writer.write(topic, converted, receipt)
            seen.add(key)
            counts[topic] = counts.get(topic, 0) + len(converted.markers)
    expected = {(receipt, topic) for receipt in by_receipt for topic in MARKER_TOPICS}
    if seen != expected or tf_count != len(by_receipt):
        raise ValueError('Incomplete common-view topic/frame coverage')
    if sources != {path: digest(path) for path in sources}:
        raise RuntimeError('Source changed during offline adaptation')
    result = dict(scene)
    result.update(schema='e12_postprocessing_common_rviz_scene_v1', fixed_frame='map',
        marker_bag=str(target_bag.resolve()), marker_bag_sha256=digest(target_bag),
        marker_topics=list(MARKER_TOPICS), topics=list(MARKER_TOPICS)+( [TF_TOPIC] ),
        tf_topic=TF_TOPIC, tf_parent='morai_map', tf_child='base_link',
        source_scene=str(source.resolve()), source_scene_sha256=digest(source),
        original_marker_bag=str(original_bag.resolve()), original_marker_bag_sha256=scene['marker_bag_sha256'],
        source_sha256=sources, source_raw_bag_sha256=scene['source_sha256'].get(str(Path(scene['raw_bag']).resolve())),
        geometry='map frame; premultiplied marker pose from original measurement-time lidar1; local points/size/colors/IDs unchanged',
        dynamic_tf_clock='exact source LiDAR measurement header, original bag receipt record time',
        dynamic_tf_remapping='play derived /e12_view/tf as /tf; do not replay raw /tf or raw /tf_static',
        static_tf_sources=[str(tf_path), str(sensor)], marker_counts=counts,
        dynamic_tf_count=tf_count, inverse_roundtrip_max_abs_error=max_error,
        heading_correction='frozen pack yaw already corrected once; no additional heading correction',
        ros_master_started=False, detector_rerun=False, complete=True)
    with (output/'scene.json').open('x') as stream:
        json.dump(result, stream, indent=2)
        stream.write('\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=CASES, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--source', type=Path)
    parser.add_argument('--pack', type=Path)
    args = parser.parse_args()
    result = prepare(args.case, args.output, args.source, args.pack)
    print(json.dumps({key: result[key] for key in ('case','framecount','dynamic_tf_count','inverse_roundtrip_max_abs_error','marker_bag')}, indent=2))


if __name__ == '__main__':
    main()
