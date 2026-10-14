#!/usr/bin/env python3
"""Actual private ROS proof that failed TF cannot overwrite pending input.

The clock is paused while valid A is committed and invalid-TF B is rejected.
Then the unchanged native MainLoop processes the pending input. Old/new
isolated binary snapshots are compared; no simulator or user graph is used.
"""
import argparse
import json
import os
from pathlib import Path
import sys
import threading
import time
import io
import hashlib

import yaml

from replay_vehicle_ros_candidates import validate_private_port

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / 'base_ws/src/planning_addons/planning_scenario_validator/src'),
               str(ROOT / 'base_ws/src/perception_object_pipeline/src')]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--old-build', type=Path, required=True)
    p.add_argument('--new-build', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--port', type=int, default=29381)
    p.add_argument('--case', choices=('tf_failure', 'late_stamp', 'duplicate_stamp'), default='tf_failure')
    p.add_argument('--old-preserves', action='store_true',
                   help='Compare two fixed source revisions: both must preserve A with identical complete wire output')
    a = p.parse_args()
    validate_private_port(a.port)
    if a.output.exists(): raise FileExistsError('Fresh probe directory required')
    a.output.mkdir(parents=True)
    previous = {key: os.environ.get(key) for key in ('ROS_MASTER_URI', 'ROS_IP')}
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:' + str(a.port)
    os.environ['ROS_IP'] = '127.0.0.1'
    from planning_scenario_validator.runtime import Processes
    from audit_tracking_continuity_events import sha
    processes = Processes(a.output / 'logs', os.environ.copy())
    subscriptions = []
    arrivals = {'old': [], 'new': []}; outputs = {'old': [], 'new': []}; rejected = set()
    wire_hashes = {'old': [], 'new': []}
    condition = threading.Condition()
    try:
        processes.start('master', ['roscore', '-p', str(a.port)])
        import rosgraph
        deadline = time.monotonic() + 15
        while not rosgraph.is_master_online():
            if time.monotonic() > deadline: raise RuntimeError('Private master unavailable')
            time.sleep(.05)
        import rospy
        from std_msgs.msg import Header
        from rosgraph_msgs.msg import Clock, Log
        from autoware_perception_msgs.msg import DetectedObjects, TrackedObjects
        from perception_object_pipeline.detected_objects import make_detected_objects
        from audit_tracking_continuity import object_row
        rospy.init_node('vehicle_detection_receipt_probe', disable_signals=True)
        rospy.set_param('/use_sim_time', True)
        clock = rospy.Publisher('/clock', Clock, queue_size=20, latch=True)
        start = rospy.Time(1000, 0)
        clock.publish(Clock(start))
        publisher = rospy.Publisher('/receipt_probe/detection', DetectedObjects, queue_size=20)
        configuration = yaml.safe_load((ROOT / 'base_ws/src/ekf-multi-object-tracking/config/config.yaml').read_text())
        configuration['configure'].update(fixed_output_rate_hz=0., output_confirmed_track=False,
            center_motion_velocity_fusion=False, compensate_output_to_current_time=False)
        rospy.set_param('/topic_name', dict(lidar_objects='/receipt_probe/detection', odometry='/unused_odometry',
            output_track_jsk='/output_track_jsk', output_track_marker='/output_track_marker'))
        def arrived(message, mode):
            with condition:
                arrivals[mode].append(message.stamp.to_sec()); condition.notify_all()
        def tracked(message, mode):
            with condition:
                wire = io.BytesIO(); message.serialize(wire)
                wire_hashes[mode].append(hashlib.sha256(wire.getvalue()).hexdigest())
                outputs[mode].append(dict(stamp=message.header.stamp.to_sec(),
                    objects=[object_row(o) for o in message.objects])); condition.notify_all()
        def log(message):
            reject = ('Cannot transform receipt_probe_missing_frame' in message.msg if a.case=='tf_failure'
                      else 'Dropping non-increasing detection commit' in message.msg)
            if reject:
                with condition:
                    rejected.add(message.name.split('/')[2]); condition.notify_all()
        subscriptions.append(rospy.Subscriber('/rosout', Log, log, queue_size=100))
        snapshots = {}
        for mode, directory in [('old', a.old_build), ('new', a.new_build)]:
            build = json.loads((directory / 'build_receipt.json').read_text())
            binary = Path(build['binaries']['baseline'])
            if sha(binary) != build['binaries_sha256']['baseline']:
                raise RuntimeError('Snapshot binary changed')
            snapshots[mode] = dict(binary=str(binary), sha256=sha(binary))
            namespace = '/receipt_probe/' + mode
            for key, value in configuration.items(): rospy.set_param(namespace + '/' + key, value)
            subscriptions.append(rospy.Subscriber(namespace + '/arrived', Header,
                lambda msg, mode=mode: arrived(msg, mode), queue_size=50))
            subscriptions.append(rospy.Subscriber(namespace + '/tracking', TrackedObjects,
                lambda msg, mode=mode: tracked(msg, mode), queue_size=50))
            processes.start(mode, [str(binary), '__name:=tracker', '__ns:=' + namespace,
                '_world_frame_id:=map', '_transform_timeout_sec:=0.0', '_research_direction_guard:=false',
                '~input/objects:=/receipt_probe/detection', '~output/objects:=' + namespace + '/tracking',
                '~diagnostics/input_arrived:=' + namespace + '/arrived',
                '/output_track_jsk:=' + namespace + '/jsk', '/output_track_marker:=' + namespace + '/marker'])
        deadline = time.monotonic()+10
        while publisher.get_num_connections() != 2:
            if time.monotonic() > deadline: raise RuntimeError('Both snapshots must subscribe')
            time.sleep(.05)
        time.sleep(.3)
        processes.start('recorder', ['rosbag', 'record', '--lz4', '-O', str(a.output/'probe.bag'),
            '/clock', '/receipt_probe/detection', '/receipt_probe/old/tracking', '/receipt_probe/new/tracking', '/rosout'])
        # A short probe can otherwise finish before rosbag opens its output.
        # Republish the same paused time, never advance the native worker.
        deadline = time.monotonic()+3
        while not (a.output/'probe.bag.active').exists():
            if time.monotonic()>deadline: raise RuntimeError('Recorder did not open before the pending-input probe')
            clock.publish(Clock(start)); time.sleep(.05)
        time.sleep(.2)
        valid = make_detected_objects(Header(seq=1, stamp=start, frame_id='map'),
            [[12., 34., 1., 4.5, 1.8, 1.7, .3]], [.9], ['vehicle'])
        publisher.publish(valid)
        deadline = time.monotonic()+3
        with condition:
            while any(values != [1000.] for values in arrivals.values()):
                if time.monotonic() > deadline: raise RuntimeError('Valid A did not atomically arrive at both snapshots')
                condition.wait(.05)
        bad_stamp = {'tf_failure': rospy.Time(1000, 1000000),
                     'late_stamp': rospy.Time(999, 999000000),
                     'duplicate_stamp': start}[a.case]
        invalid = make_detected_objects(Header(seq=2, stamp=bad_stamp,
            frame_id='receipt_probe_missing_frame' if a.case=='tf_failure' else 'map'),
            [[99., 99., 1., 4.5, 1.8, 1.7, .3]], [.9], ['vehicle'])
        publisher.publish(invalid)
        deadline = time.monotonic()+3
        with condition:
            while not (rejected=={'old','new'} if a.case=='tf_failure' or a.old_preserves else
                       'new' in rejected and len(arrivals['old'])==2):
                if time.monotonic() > deadline: raise RuntimeError('Invalid B rejection/old acceptance was not observed')
                condition.wait(.05)
        # The source clock has not moved: MainLoop is still sleeping, so this
        # proves pending-buffer integrity rather than an already-consumed A.
        if any(outputs.values()): raise RuntimeError('Input was processed before the paused-clock boundary')
        for index in range(1, 11):
            clock.publish(Clock(rospy.Time(1000, index*10000000))); time.sleep(.015)
        deadline = time.monotonic()+3
        with condition:
            while any(not values for values in outputs.values()):
                if time.monotonic() > deadline: raise RuntimeError('Snapshots did not process the pending boundary')
                condition.wait(.05)
        processes.stop('recorder')
        preserved = lambda frames: any(frame['stamp']==1000. and len(frame['objects'])==1 and
            frame['objects'][0]['center'][:2]==[12., 34.] for frame in frames)
        result = dict(case=a.case, snapshots=snapshots, accepted_input_stamps=arrivals,
            callback_rejection_nodes=sorted(rejected), outputs=outputs,
            old_pending_valid_overwritten=not preserved(outputs['old']),
            new_pending_valid_preserved=preserved(outputs['new']),
            old_expected_preserved=a.old_preserves,
            output_wire_sha256=wire_hashes,
            old_vs_new_complete_wire_exact=bool(wire_hashes['old']) and wire_hashes['old']==wire_hashes['new'],
            scope='Actual ROS adapter + native paused-clock pending input. TF timeout zero, guard OFF. Not a wall-load or planner test.')
        (a.output/'report.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
        old_ok = not result['old_pending_valid_overwritten'] if a.old_preserves else result['old_pending_valid_overwritten']
        if not old_ok or not result['new_pending_valid_preserved'] or (
                a.old_preserves and not result['old_vs_new_complete_wire_exact']):
            raise RuntimeError('The controlled failure did not reproduce old loss and preserve new input')
        print(json.dumps(result, indent=2))
    finally:
        for subscription in subscriptions: subscription.unregister()
        processes.close()
        for key, value in previous.items():
            if value is None: os.environ.pop(key, None)
            else: os.environ[key] = value


if __name__ == '__main__': main()
