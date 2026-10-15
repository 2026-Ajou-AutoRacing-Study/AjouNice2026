#!/usr/bin/env python3
"""Bounded installed C++ EKF origin/TF/30Hz gate on a private master.

Only this helper's localhost master, installed EKF process, synthetic /clock,
dynamic /tf and tagged detection messages run. No GPU/driver/planner/control.
"""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time
import xmlrpc.client

import numpy as np
import yaml

from audit_tracking_continuity import ROOT, stats
from audit_tracking_continuity_events import sha
from audit_tracking_continuity_box_origin_tf import ros_helpers


def private_uri(port):
    if not 1024 <= port <= 65535 or port == 11311:
        raise ValueError('Explicit private non-system port required')
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', port))
    return 'http://127.0.0.1:'+str(port)


def typed_world_center(center):
    """Mirror actual ROS-interface XYZ float32 narrowing, no policy change."""
    return np.asarray(center, dtype=np.float32).astype(np.float64)


def stop_owned(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=5.)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3.)


def process_group_members(group):
    result = subprocess.run(['ps', '-eo', 'pid=,pgid=,args='], text=True,
                            capture_output=True, check=True)
    return [line.strip() for line in result.stdout.splitlines()
            if len(line.split(None, 2)) >= 2 and int(line.split(None, 2)[1]) == group]


def wait(predicate, timeout, description):
    end = time.monotonic()+timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(.01)
    raise TimeoutError(description)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--master-port', type=int, default=11367)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError('Fresh private gate output required')
    uri = private_uri(args.master_port)
    args.output.mkdir(parents=True)
    install = ROOT/'base_ws/install_aw149'
    binary = install/'lib/ekf_multi_object_tracking/ekf_multi_object_tracking'
    config = install/'share/ekf_multi_object_tracking/config/config.yaml'
    source_config = ROOT/'base_ws/src/ekf-multi-object-tracking/config/config.yaml'
    if not binary.is_file() or sha(config) != sha(source_config):
        raise ValueError('Installed EKF/config missing or config differs from source')
    environment = dict(os.environ, ROS_MASTER_URI=uri, ROS_IP='127.0.0.1',
        ROS_NAMESPACE='/', ROS_LOG_DIR=str(args.output/'roslogs'), AJOUNICE_ROOT=str(ROOT))
    environment.pop('ROS_HOSTNAME', None)
    os.environ.update({k: v for k, v in environment.items() if k.startswith('ROS_')})
    os.environ.pop('ROS_HOSTNAME', None)
    # Import rospy only after binding its cached master URI to our private master.
    import rospy
    from autoware_perception_msgs.msg import TrackedObjects
    from rosgraph_msgs.msg import Clock
    from tf2_msgs.msg import TFMessage
    h = ros_helpers()
    owned = []
    handles = []
    report = dict(master_uri=uri, binary=str(binary), binary_sha256=sha(binary),
        config_sha256=sha(config), source_config_sha256=sha(source_config),
        production_changed=False, synthetic_inputs_per_mode=30, modes={},
        limits=['Synthetic localhost TF/clock and installed node only; no actual sensor/NN/MORAI/planner/control.',
                'No host-load latency or LAN/input-drop benchmark. Publish counts do not expose every internal detector update.',
                'Fresh tracker subprocess reset; no live planner-cache reset or closed-loop safety certification.',
                '30Hz copies are checked at synthetic ROS-time slots; wall-time runtime performance is not certified.'])
    report['source_sha256'] = {str(p): sha(p) for p in (
        Path(__file__).resolve(),
        ROOT/'base_ws/src/perception/bevfusion_ros/src/bevfusion_ros/box_center_origin.py',
        ROOT/'base_ws/src/ekf-multi-object-tracking/src/algorithm/ekf_multi_object_tracking.cpp',
        ROOT/'base_ws/src/ekf-multi-object-tracking/src/ekf_multi_object_tracking_node.cpp',
        ROOT/'base_ws/src/ekf-multi-object-tracking/include/ekf_multi_object_tracking_node.hpp')}
    failure = None
    master = None
    try:
        master_log = open(args.output/'master.log', 'w'); handles.append(master_log)
        master = subprocess.Popen(['roscore', '-p', str(args.master_port)], env=environment,
            stdout=master_log, stderr=subprocess.STDOUT, start_new_session=True)
        owned.append(master)
        proxy = xmlrpc.client.ServerProxy(uri)
        def master_ready():
            if master.poll() is not None:
                raise RuntimeError('Private master exited')
            try:
                return proxy.getPid('/origin_gate')[0] == 1
            except (ConnectionError, OSError):
                return False
        wait(master_ready, 10., 'Private master startup')
        rospy.init_node('origin_gate_driver', anonymous=False, disable_signals=True)
        rospy.set_param('/use_sim_time', True)
        params = yaml.safe_load(config.read_text())
        for key, value in params.items():
            rospy.set_param('/'+key, value)
        rospy.set_param('/configure/fixed_output_rate_hz', 30.)
        rospy.set_param('/configure/maximum_fixed_output_prediction_age_sec', .5)
        report['effective_configure'] = rospy.get_param('/configure')
        clock = rospy.Publisher('/clock', Clock, queue_size=20, latch=True)
        tf = rospy.Publisher('/tf', TFMessage, queue_size=100)
        from autoware_perception_msgs.msg import DetectedObjects
        detections = rospy.Publisher('/origin_gate/input', DetectedObjects, queue_size=10)
        received, lock = [], threading.Lock()
        def collect(message):
            with lock:
                received.append(dict(stamp=message.header.stamp.to_sec(), seq=message.header.seq,
                    frame_id=message.header.frame_id, objects=[dict(
                        id=list(obj.object_id.uuid),
                        center=[obj.kinematics.pose_with_covariance.pose.position.x,
                                obj.kinematics.pose_with_covariance.pose.position.y,
                                obj.kinematics.pose_with_covariance.pose.position.z],
                        height=obj.shape.dimensions.z,
                        velocity=[obj.kinematics.twist_with_covariance.twist.linear.x,
                                  obj.kinematics.twist_with_covariance.twist.linear.y]) for obj in message.objects]))
        subscriber = rospy.Subscriber('/origin_gate/output', TrackedObjects, collect, queue_size=200)
        quaternion = h['quaternion_from_euler'](*[math.radians(v) for v in (3., -5., 37.)])
        rotation = h['quaternion_matrix'](quaternion)[:3, :3]
        translation0 = np.asarray([400., 100., 27.])
        world_bottom = translation0+rotation@np.asarray([8., -2., -1.5])
        expected_delta = rotation[:, 2]*.9
        for mode_index, mode in enumerate(('legacy_passthrough', 'gravity_center_v1')):
            with lock:
                received.clear()
            start = 100.+mode_index*100.
            # Bind the next fresh child's initial ROS time before its TF
            # buffer starts. Forward clock jumps can clear TF history.
            clock.publish(Clock(clock=rospy.Time.from_sec(start)))
            time.sleep(.1)
            log = open(args.output/(mode+'.log'), 'w'); handles.append(log)
            node = subprocess.Popen([str(binary), '__name:=origin_gate_tracker',
                '_world_frame_id:=map', '_transform_timeout_sec:=0.1',
                '~input/objects:=/origin_gate/input', '~output/objects:=/origin_gate/output'],
                env=environment, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            owned.append(node)
            wait(lambda: detections.get_num_connections() > 0 and tf.get_num_connections() > 0,
                 8., 'Installed EKF input/TF connection')
            inputs = []
            for tick in range(360):
                if node.poll() is not None:
                    raise RuntimeError('Installed EKF exited: '+mode)
                now = start+tick*.01
                transform = h['TransformStamped']()
                transform.header = h['Header'](stamp=rospy.Time.from_sec(now), frame_id='map')
                transform.child_frame_id = 'lidar1'
                transform.transform.translation.x = 400.+2.*(now-start)
                transform.transform.translation.y = 100.
                transform.transform.translation.z = 27.
                for key, value in zip(('x', 'y', 'z', 'w'), quaternion):
                    setattr(transform.transform.rotation, key, float(value))
                tf.publish(TFMessage(transforms=[transform]))
                clock.publish(Clock(clock=rospy.Time.from_sec(now)))
                if tick >= 20 and (tick-20) % 10 == 0 and len(inputs) < 30:
                    stamp = now-.08
                    # Correct timestamp lookup yields a fixed world box. Using
                    # latest TF instead would shift map X by +0.16m.
                    translation = translation0+np.asarray([2.*(stamp-start), 0., 0.])
                    local_bottom = (rotation.T@(world_bottom-translation)).tolist()
                    raw = dict(box_center_origin='lidar_bottom_v1', detections=[dict(
                        center=local_bottom, size=[4.6, 1.9, 1.8], yaw=.2, score=.9, class_name='vehicle')])
                    adapted = h['response_for_ros'](raw, mode)
                    det = adapted['detections'][0]
                    header = h['Header'](seq=len(inputs), stamp=rospy.Time.from_sec(stamp), frame_id='lidar1')
                    message = h['make_detected_objects'](header, [det['center']+det['size']+[det['yaw']]], [.9], ['vehicle'])
                    detections.publish(message)
                    inputs.append(dict(stamp=stamp, published_ros_now=now, center=det['center']))
                time.sleep(.01)
            time.sleep(.1)
            stop_owned(node)
            with lock:
                output = copy.deepcopy(received)
            nonempty = [row for row in output if row['objects']]
            if not nonempty or any(len(row['objects']) != 1 for row in nonempty):
                raise AssertionError('No single confirmed track: '+mode)
            physical_expected = world_bottom+(expected_delta if mode == 'gravity_center_v1' else 0.)
            # Actual node's Object3DState narrows transformed world XYZ to
            # float before ConvertDetectObjectToMeastruct converts to double.
            # This is a source-defined oracle, not a tuned numeric tolerance.
            expected = typed_world_center(physical_expected)
            error = [float(np.linalg.norm(np.asarray(row['objects'][0]['center'])-expected)) for row in nonempty]
            physical_error = [float(np.linalg.norm(np.asarray(row['objects'][0]['center'])-physical_expected)) for row in nonempty]
            if max(error) > 1.e-7:
                raise AssertionError('Timestamped TF / origin center mismatch: '+mode+' '+str(max(error)))
            ids = {tuple(row['objects'][0]['id']) for row in nonempty}
            stamps = [row['stamp'] for row in output]
            diffs = [b-a for a, b in zip(stamps, stamps[1:])]
            if len(ids) != 1 or any(dt <= 0 for dt in diffs) or any(row['frame_id'] != 'map' for row in output):
                raise AssertionError('Output ID/stamp/frame contract')
            if nonempty[0]['stamp'] < inputs[2]['stamp']-1.e-7 or len(output) <= len(inputs):
                raise AssertionError('Confirmation / internal copy output absent')
            report['modes'][mode] = dict(inputs=inputs, outputs=output,
                published_inputs=len(inputs), outputs_count=len(output), nonempty_count=len(nonempty),
                expected_map_center=expected.tolist(), physical_double_map_center=physical_expected.tolist(),
                center_error_m=stats(error), center_error_vs_unquantized_tf_m=stats(physical_error), stable_ids=len(ids),
                first_confirmed_output_stamp=nonempty[0]['stamp'], third_input_stamp=inputs[2]['stamp'],
                output_delta_sec=stats(diffs),
                output_to_latest_input_header_age_sec=stats([row['stamp']-max(
                    stamp for stamp in [r['stamp'] for r in inputs] if stamp <= row['stamp']) for row in nonempty]),
                synthetic_full_rpy_deg=[3., -5., 37.], detector_header_lag_sec=.08,
                latest_instead_of_header_tf_would_shift_map_x_m=.16,
                transform_failures_in_log=[line for line in (args.output/(mode+'.log')).read_text().splitlines()
                                           if 'Cannot transform' in line])
        report['expected_gravity_minus_legacy_map_delta_m'] = expected_delta.tolist()
        report['typed_expected_gravity_minus_legacy_map_delta_m'] = (
            typed_world_center(world_bottom+expected_delta)-typed_world_center(world_bottom)).tolist()
        actual_delta = np.asarray(report['modes']['gravity_center_v1']['outputs'][-1]['objects'][0]['center'])-report['modes']['legacy_passthrough']['outputs'][-1]['objects'][0]['center']
        np.testing.assert_allclose(actual_delta, report['typed_expected_gravity_minus_legacy_map_delta_m'], rtol=0., atol=1.e-7)
        report['actual_gravity_minus_legacy_map_delta_m'] = actual_delta.tolist()
        report['node_xyz_narrowing'] = dict(source='base_ws/src/ekf-multi-object-tracking/include/ekf_multi_object_tracking_node.hpp:109',
            meaning='Actual transformed world x/y/z are stored as float32 before entering double core state. Physical double TF and typed core oracle are separately reported.')
        report['pass'] = True
        subscriber.unregister(); detections.unregister(); tf.unregister(); clock.unregister()
    except Exception as error:
        failure = error
        report.update(pass_result=False, error_type=type(error).__name__, error=str(error))
    finally:
        try:
            rospy.signal_shutdown('Owned private gate complete')
        except Exception:
            pass
        for process in reversed(owned):
            stop_owned(process)
        for handle in handles:
            handle.close()
        report['owned_cleanup'] = [dict(pid=p.pid, pgid=p.pid, returncode=p.poll(),
            remaining_group_members=process_group_members(p.pid)) for p in owned]
        report['owned_cleanup_clean'] = not any(r['remaining_group_members'] for r in report['owned_cleanup'])
        (args.output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    if failure:
        raise failure
    if not report['owned_cleanup_clean']:
        raise RuntimeError('Owned private process group not fully cleaned')
    print(json.dumps(dict(output=str(args.output), pass_result=True,
        modes={k: dict(inputs=v['published_inputs'], outputs=v['outputs_count'],
                        max_center_error=v['center_error_m']['maximum']) for k, v in report['modes'].items()},
        cleanup_clean=True)))


if __name__ == '__main__':
    main()
