#!/usr/bin/env python3
"""CPU-only actual tf2/message and original-core origin boundary fixtures.

No ROS master, operational nodes, GPU, planner policy change or source edit.
Source the Noetic/AW149 overlay: actual generated messages are intentional.
"""
import argparse
import copy
import gzip
import inspect
import json
import math
from pathlib import Path
import sys

import numpy as np

from audit_tracking_continuity import ROOT
from audit_tracking_continuity_events import replay, sha


def ros_helpers():
    sys.path.insert(0, str(ROOT/'base_ws/src/perception/bevfusion_ros/src'))
    sys.path.insert(0, str(ROOT/'base_ws/src/perception_object_pipeline/src'))
    import rospy
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from std_msgs.msg import Header
    from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix
    from tf2_geometry_msgs import do_transform_pose
    from bevfusion_ros.box_center_origin import response_for_ros, BOTTOM_ORIGIN, GRAVITY_ORIGIN
    from perception_object_pipeline.detected_objects import make_detected_objects
    return dict(rospy=rospy, PoseStamped=PoseStamped, TransformStamped=TransformStamped,
        Header=Header, quaternion_from_euler=quaternion_from_euler,
        quaternion_matrix=quaternion_matrix, euler_from_quaternion=euler_from_quaternion,
        do_transform_pose=do_transform_pose, response_for_ros=response_for_ros,
        BOTTOM_ORIGIN=BOTTOM_ORIGIN, GRAVITY_ORIGIN=GRAVITY_ORIGIN,
        make_detected_objects=make_detected_objects)


def transform_fixture(rpy_deg, height, bottom=(8., -2., -1.5)):
    """Actual DetectedObjects→PoseStamped→tf2 transform, plus math oracle."""
    h = ros_helpers()
    raw = dict(box_center_origin=h['BOTTOM_ORIGIN'], detections=[dict(
        center=list(bottom), size=[4.6, 1.9, float(height)], yaw=.2,
        score=.9, class_name='vehicle')])
    saved = copy.deepcopy(raw)
    corrected = h['response_for_ros'](raw, h['GRAVITY_ORIGIN'])
    if raw != saved or h['response_for_ros'](corrected, h['GRAVITY_ORIGIN']) is not corrected:
        raise AssertionError('Input mutation or origin converted twice')
    transform = h['TransformStamped']()
    transform.header = h['Header'](stamp=h['rospy'].Time.from_sec(100.), frame_id='map')
    transform.child_frame_id = 'lidar1'
    transform.transform.translation.x = 400.
    transform.transform.translation.y = 100.
    transform.transform.translation.z = 27.
    quaternion = h['quaternion_from_euler'](*[math.radians(v) for v in rpy_deg])
    for key, value in zip(('x', 'y', 'z', 'w'), quaternion):
        setattr(transform.transform.rotation, key, float(value))
    world = {}
    for name, response in (('legacy', raw), ('gravity', corrected)):
        detection = response['detections'][0]
        boxes = [detection['center']+detection['size']+[detection['yaw']]]
        message = h['make_detected_objects'](
            h['Header'](stamp=h['rospy'].Time.from_sec(100.), frame_id='lidar1'),
            boxes, [detection['score']], [detection['class_name']])
        stamped = h['PoseStamped'](header=message.header,
            pose=message.objects[0].kinematics.pose_with_covariance.pose)
        pose = h['do_transform_pose'](stamped, transform)
        q = pose.pose.orientation
        world[name] = dict(center=[pose.pose.position.x, pose.pose.position.y, pose.pose.position.z],
            yaw=float(h['euler_from_quaternion']([q.x, q.y, q.z, q.w])[2]),
            frame_id=pose.header.frame_id, stamp=pose.header.stamp.to_sec(),
            size=list(detection['size']), score=detection['score'], class_name=detection['class_name'])
    expected = (h['quaternion_matrix'](quaternion)[:3, :3][:, 2]*height*.5).tolist()
    measured = (np.asarray(world['gravity']['center'])-world['legacy']['center']).tolist()
    np.testing.assert_allclose(measured, expected, rtol=0., atol=1.e-10)
    return dict(rpy_deg=list(rpy_deg), height_m=height, world=world,
        expected_map_delta_m=expected, actual_map_delta_m=measured,
        actual_xy_shift_m=math.hypot(*measured[:2]),
        max_oracle_error_m=float(np.max(np.abs(np.asarray(measured)-expected))),
        actual_tf2=True, actual_generated_detected_objects=True)


def same_non_z_trace(a, b):
    def strip(rows):
        copied = copy.deepcopy(rows)
        for row in copied:
            for track in row['tracks']:
                track['center'][2] = 0.
        return copied
    return strip(a) == strip(b)


def core_ema_fixture(binary):
    fixture = transform_fixture((0., 0., 37.), 1.8)
    worlds = fixture['world']
    frames = lambda mode: [dict(stamp=100.+i*.1,
        world_objects=[dict(worlds['legacy' if mode == 'mixed' and i < 10 else
                                    'gravity' if mode in ('mixed', 'gravity') else 'legacy'])])
        for i in range(30)]
    histories = {mode: replay(binary, frames(mode)) for mode in ('legacy', 'gravity', 'mixed')}
    reset = replay(binary, frames('gravity')[10:])
    if not same_non_z_trace(histories['legacy'], histories['gravity']):
        raise AssertionError('Planar fresh runs differ outside observable Z')
    if not same_non_z_trace(histories['legacy'], histories['mixed']):
        raise AssertionError('Planar mixed run differs outside observable Z')
    target = worlds['gravity']['center'][2]
    gap = target-worlds['legacy']['center'][2]
    samples = []
    for index, row in enumerate(histories['mixed'][10:]):
        if len(row['tracks']) != 1:
            raise AssertionError('Unexpected fixture lifecycle')
        residual = target-row['tracks'][0]['center'][2]
        expected = gap*.7**(index+1)
        np.testing.assert_allclose(residual, expected, rtol=0., atol=1.e-10)
        samples.append(dict(associations_after_origin_switch=index+1, stamp=row['stamp'],
            actual_mixed_center_z_m=row['tracks'][0]['center'][2],
            target_gravity_z_m=target, residual_m=residual,
            expected_alpha_03_residual_m=expected))
    for row in reset:
        np.testing.assert_allclose(row['tracks'][0]['center'][2], target, rtol=0., atol=1.e-10)
    return dict(alpha=.3, binary=str(binary), binary_sha256=sha(binary),
        source_receipt=json.loads((binary.parent/'build_receipt.json').read_text()),
        planar_fresh_non_z_observable_trace_exact=True, planar_mixed_non_z_observable_trace_exact=True,
        mixed_origin_ema_samples=samples, fresh_reset_first_frame=reset[0],
        fresh_reset_immediate_correct_z=True,
        reset_note='Reset also restarts track age/confirmation/UUID allocation. This is not a safe live planner-cache migration or reset implementation.'), histories, reset


def map_future_fixture():
    ros_helpers()
    from perception_object_pipeline.map_based_prediction import LaneletNetwork, LaneletRecord, MapPredictionConfig
    horizons = (.5, 1., 1.5, 2.)
    network = LaneletNetwork([LaneletRecord(1, ((0., 0., 27.), (150., 0., 27.)))],
                            MapPredictionConfig(lateral_convergence_time_sec=1.5))
    legacy = network.predict(10., 0., 27., 0., 5., horizons)
    gravity = network.predict(10., 0., 27.9, 0., 5., horizons)
    if len(legacy) != 1 or len(gravity) != 1:
        raise AssertionError('Synthetic fixture must have one real mapped hypothesis')
    if gravity != network.predict(10., 0., 27.9, 0., 5., horizons):
        raise AssertionError('Map future fixture is not repeatable')
    values = []
    for t, low, high in zip(horizons, legacy[0].poses, gravity[0].poses):
        expected = max(0., 1.-t/1.5)*.9
        np.testing.assert_allclose(high[2]-low[2], expected, rtol=0., atol=1.e-10)
        if high[:2] != low[:2] or high[3] != low[3]:
            raise AssertionError('Z-only map fixture changed XY/yaw')
        values.append(dict(horizon_sec=t, legacy_future_z_m=low[2], gravity_future_z_m=high[2],
                           origin_delta_m=high[2]-low[2], expected_residual_times_half_height_m=expected))
    return dict(actual_pure_predictor=True, repeat_exact=True, source_policy_unchanged=True,
        synthetic_road_z_m=27., height_m=1.8, speed_mps=5., values=values,
        note='Mapped vehicle future offset converges to centerline: half-height is not preserved after 1.5s. Node prepends the current pose; fallback uses unchanged Z. No height-policy or planner closed-loop certification.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--core-binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError('Fresh output required')
    core_paths = [ROOT/'base_ws/src/ekf-multi-object-tracking/src/algorithm/ekf_multi_object_tracking.cpp',
                  ROOT/'base_ws/src/ekf-multi-object-tracking/include/algorithm/ekf_multi_object_tracking.hpp']
    before = {str(p): sha(p) for p in core_paths}
    fixtures = [transform_fixture(rpy, height, bottom) for rpy in
        ((0., 0., 37.), (3., -5., 37.), (-8., 6., -120.), (12., 7., 90.))
        for height in (.35, 1.8, 2.4) for bottom in ((8., -2., -1.5), (-60., 20., .5))]
    core, histories, reset = core_ema_fixture(args.core_binary)
    if core['source_receipt']['production_source_sha256'] != before:
        raise ValueError('Original binary receipt no longer matches production source')
    mapped = map_future_fixture()
    if before != {str(p): sha(p) for p in core_paths}:
        raise AssertionError('Production core changed during audit')
    source_paths = [Path(__file__).resolve(),
        ROOT/'base_ws/src/perception/bevfusion_ros/src/bevfusion_ros/box_center_origin.py',
        ROOT/'base_ws/src/perception_object_pipeline/src/perception_object_pipeline/detected_objects.py',
        ROOT/'base_ws/src/perception_object_pipeline/src/perception_object_pipeline/map_based_prediction.py',
        ROOT/'base_ws/src/perception_object_pipeline/scripts/map_based_prediction_node.py',
        ROOT/'base_ws/src/ekf-multi-object-tracking/include/ekf_multi_object_tracking_node.hpp',
        Path(inspect.getfile(ros_helpers()['do_transform_pose']))]
    report = dict(schema='box_origin_actual_tf2_core_ema_v1', production_changed=False,
        ros_master_contact=False, operational_nodes_started=False, gpu_used=False,
        transform_fixtures=fixtures, core_ema=core, map_future=mapped,
        source_sha256=before, adapter_tf2_map_audit_sources_sha256={str(p): sha(p) for p in source_paths},
        limits=['Direct actual tf2 geometry is not TF buffer lookup/arrival/time-bracketing/queue certification.',
                'Core traces preserve printed state/XY prior covariance fields, not full live ROS copy scheduler or planner memory.',
                'Fresh origin choice is required; mixed-origin input is demonstrated unsafe for immediate Z consistency.',
                'Nonplanar map XY intentionally changes by R[:,2]*h/2; planar non-Z exact does not prove nonplanar equivalence.',
                'Synthetic map route proves current future-Z behavior, not multi-lane closed-loop safety or a new Z policy.'])
    args.output.mkdir(parents=True)
    (args.output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    with gzip.open(args.output/'core_histories.json.gz', 'wt') as handle:
        json.dump(dict(histories=histories, fresh_reset=reset), handle, allow_nan=False)
    print(json.dumps(dict(output=str(args.output), tf2_fixtures=len(fixtures),
        oracle_max_error_m=max(f['max_oracle_error_m'] for f in fixtures),
        max_actual_nonplanar_xy_shift_m=max(f['actual_xy_shift_m'] for f in fixtures),
        fresh_reset_z_correct=True, mixed_ema_proved=True, map_future_offset_not_preserved=True)))


if __name__ == '__main__':
    main()
