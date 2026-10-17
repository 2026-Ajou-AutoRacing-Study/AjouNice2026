#!/usr/bin/env python3
"""격리된 정적 지면 QA 후보 분석. 기존 collector·판정·raw는 변경하지 않는다.

동일 SaveSensorData basename의 IMU와 native 여덟 꼭짓점을 이용한다. 이
보고서의 후보 분류는 지면 접촉 인증이나 본수집 승인으로 사용할 수 없다.
"""
import argparse
from collections import Counter
import copy
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from base5_capture_audit import rotation_matrix
from base5_capture_safety import atomic_json
from base5_native_box_corners import load_matching_corners
from base5_static_ground_policy import lightweight_static
import generate_base5_independent as gen

SCHEMA = 'base5_isolated_static_surface_attitude_candidate_v1'
POLICY = 'full-rpy-far-static-surface-uncertainty-candidate-v1'
COMPOSITES = {'BabyStroller1','BabyStroller2','BabyStroller3','Ped_Pet1'}
YAW_CORRECTION_DEG = -1.347


def require(condition, message):
    if not condition:
        raise ValueError(message)


def read(path):
    return json.loads(Path(path).read_text())


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda:stream.read(1024*1024), b''):
            h.update(chunk)
    return h.hexdigest()


def pins(paths):
    return {str(path.resolve()):dict(sha256=digest(path), bytes=path.stat().st_size,
        mtime_ns=path.stat().st_mtime_ns) for path in sorted(set(paths))}


def only(paths, message):
    paths = list(paths)
    require(len(paths) == 1, message)
    return paths[0]


def vector(value, length=3):
    result = np.asarray(value, float)
    require(result.shape == (length,) and np.isfinite(result).all(), 'Finite vector required')
    return result


def quaternion_matrix(quaternion):
    x,y,z,w = vector(quaternion, 4)
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    require(abs(norm-1.) <= .001, 'Quaternion must already be normalized within .001')
    x,y,z,w = np.asarray([x,y,z,w])/norm
    return np.asarray([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def attitude(root, metadata, telemetry, scenario, sensor):
    annotation = only(root.glob('LIDAR_5/*_Instance.txt'), 'One LiDAR5 native annotation required')
    basename = annotation.name[:-len('_Instance.txt')]
    imu = only(root.glob('IMU_7/*.txt'), 'One native IMU7 sample required')
    require(imu.name == basename+'.txt', 'IMU/LiDAR must share exact capture basename')
    fields = {}
    for line in imu.read_text().splitlines():
        if ':' in line:
            key,value = line.split(':',1);key=key.strip()
            require(key not in fields, 'Duplicate native IMU field')
            fields[key] = value.strip()
    quat = [float(fields['Orientation '+v]) for v in 'XYZW']
    gyro = vector([float(fields['Angular Velocity '+v]) for v in 'XYZ'])
    require(float(np.linalg.norm(gyro)) <= .01, 'IMU angular rate exceeds .01 rad/s')
    imu_sensor = only((s for s in sensor['IMUList'] if s['m_SensorUniqueID'] == 7), 'Exact IMU7 calibration required')
    lidar_sensor = only((s for s in sensor['Lidar3DList'] if s['m_SensorUniqueID'] == 5), 'Exact LiDAR5 calibration required')
    require(imu_sensor.get('m_TargetUniqueID') == lidar_sensor.get('m_TargetUniqueID') == 0,
            'Sensors must be mounted on Ego UID0')
    angles = lambda item:np.radians(vector([float(item['rot'][k]) for k in ('roll','pitch','yaw')]))
    mount = rotation_matrix(angles(imu_sensor))
    body_raw = quaternion_matrix(quat) @ mount.T
    transform = np.eye(4);transform[:3,:3] = rotation_matrix(angles(lidar_sensor))
    transform[:3,3] = [float(lidar_sensor['pos'][k]) for k in ('x','y','z')]
    require(np.allclose(np.asarray(metadata['lidar_to_ego']), transform, atol=1e-8, rtol=0),
            'Metadata LiDAR extrinsics differ from saved sensor reference')
    ego = telemetry['ego_before']
    require(metadata.get('capture_runtime_ego') == ego, 'Runtime Ego metadata/telemetry differs')
    require(scenario['egoVehicle']['initPosition']['gear'] == 1
            and type(telemetry.get('park_confirmed_monotonic')) in (float,int),
            'Collector Park confirmation and Park scenario required')
    for key in ('post_park_stationary_witness','pre_capture_stationary_witness'):
        require(telemetry.get(key, {}).get('verified') is True, 'Two verified stationary witnesses required')
    for key in ('ego_before','ego_after'):
        record = telemetry[key]
        require(np.linalg.norm(vector([record[k] for k in ('vx','vy','vz')])) <= .03
                and float(record['brake']) >= .99, 'Stopped Ego and applied brake required')
    collision = metadata.get('capture_safety', {}).get('collision', {})
    require(collision.get('observed') is True and collision.get('no_collision_verified') is True
            and not collision.get('non_ego_uids'), 'Fresh collision-free capture evidence required')
    imu_yaw = math.degrees(math.atan2(body_raw[1,0], body_raw[0,0]))
    yaw_delta = abs((imu_yaw-float(ego['heading'])+180)%360-180)
    require(yaw_delta <= .10, 'IMU raw yaw/Ego heading relationship not established')
    rotation = rotation_matrix([0.,0.,math.radians(YAW_CORRECTION_DEG)]) @ body_raw
    roll = math.degrees(math.atan2(body_raw[2,1],body_raw[2,2]))
    pitch = math.degrees(math.asin(float(np.clip(-body_raw[2,0],-1,1))))
    seconds, nanoseconds = fields.get('TimeStamp.Secs'), fields.get('TimeStamp.nSecs')
    stamp_valid = seconds is not None and nanoseconds is not None and 0 <= int(nanoseconds) < 10**9
    return rotation, transform, dict(imu_file=str(imu), imu_sha256=digest(imu), native_quaternion_xyzw=quat,
        sensor_mount_rotation_deg=imu_sensor['rot'], body_rpy_raw_deg=[roll,pitch,imu_yaw],
        raw_imu_minus_ego_heading_deg=yaw_delta, heading_relationship_verified=True,
        world_rotation_definition='Rz(-1.347deg) @ Rworld_imu(native quaternion) @ Rbody_imu(mount).T',
        correction_applied_once=True, yaw_correction_deg=YAW_CORRECTION_DEG,
        raw_timestamp_fields=dict(seconds=seconds,nanoseconds=nanoseconds),
        raw_nanoseconds_in_ros_range=stamp_valid, exact_source_time_synchronization_claimed=False,
        correspondence='Same SaveSensorData basename + collector Park/stationary witnesses + low IMU angular rate',
        angular_rate_radps=gyro.tolist(), translation_world_xyz_m=[ego[k] for k in ('x','y','z')],
        sensor_to_ego=transform.tolist(), world_rotation=rotation.tolist())


def world_corroboration(record, transform, rotation, ego):
    corners = np.asarray(record['corners_lidar_m'], float)
    require(corners.shape == (8,3) and np.isfinite(corners).all(), 'Eight finite native corners required')
    center = vector(record['annotation_center_lidar_m'])
    require(np.linalg.norm(corners.mean(0)-center) <= .001, 'Native corner/annotation center mismatch')
    angle_delta = (vector(record['rotation_rpy'])-vector(record['annotation_rotation_rpy'])+np.pi)%(2*np.pi)-np.pi
    require(np.max(abs(angle_delta)) <= 2e-5, 'Native corner/annotation rotations mismatch')
    local = (corners-center) @ rotation_matrix(record['annotation_rotation_rpy'])
    size = vector(record['annotation_size_m'])
    require((size > 0).all() and np.allclose(local.max(0)-local.min(0),size,atol=.001,rtol=.001),
            'Native corner/annotation dimensions mismatch')
    require(len({tuple(v) for v in (local >= 0).tolist()}) == 8
            and np.max(abs(abs(local)-size/2)) <= .002, 'Malformed native corner box')
    body = (np.c_[corners,np.ones(8)] @ transform.T)[:,:3]
    world = body @ rotation.T + vector([ego[k] for k in ('x','y','z')])
    return dict(corners_lidar_m=corners.tolist(), corners_ego_m=body.tolist(), corners_world_m=world.tolist(),
        lowest_native_world_z_m=float(world[:,2].min()), native_world_center_xy_m=world[:,:2].mean(0).tolist(),
        legacy_yaw_only_lowest_world_z_m=float(body[:,2].min()+ego['z']),
        native_parent_component_count=record['parent_component_count'], raw_class=record['raw_class'],
        all_eight_measured_corners_transformed=True, corner_source_files=record['source_files'])


def candidate_association(row, ground, metadata, corroboration, rotation, ego, *, allow_pedestrians=False,
                          map_hashes=None):
    """기존 수치 조건은 유지하고 world 대조만 실제 full RPY 관측으로 한다."""
    try:
        require(lightweight_static(metadata) and not metadata.get('ground_probe_references')
                and not metadata.get('static_ground_references'), 'Not ordinary lightweight static capture')
        require(row.get('reason') == 'actor_annotation_deep_below_ground' and row.get('verified') is False
                and not row.get('reference') and not row.get('used_independent_reference'), 'Not unverified below-plane evidence')
        obs, plane = row['surface_observation'], row['plane']
        xy, coef = vector(obs['actor_center_ego_xy_m'],2), vector(plane['coefficients'])
        count, lower, maximum = obs['patch_point_count'], obs['lower_by_one_m_count'], ground['max_range_m']
        require(np.isfinite(maximum) and maximum > 0 and np.linalg.norm(xy) > maximum
                and type(count) is int and 18 <= count <= 64 and type(lower) is int and 2 <= lower <= count,
                'Far sparse lower-return evidence required')
        require(row['minimum_annotation_ground_gap_m'] < -.40
                and obs['minimum_signed_plane_residual_m'] <= -1.
                and type(plane['inliers']) is int and 18 <= plane['inliers'] <= count
                and 0 <= plane['residual_rms_m'] <= .025
                and obs['native_parent_component_count'] == corroboration['native_parent_component_count'] == 1,
                'Original severe/fit/single-component gates differ')
        actor = only((a for a in metadata['runtime_actors'] if a.get('unique_id') == row['uid']), 'Unique planned actor required')
        require(actor.get('motion') in ('static','standing') and actor['name'] not in COMPOSITES
                and not actor.get('compound') and not actor.get('is_compound')
                and actor.get('component_count',1) == actor.get('annotation_component_count',1) == 1,
                'Dynamic and compound parents are excluded')
        physical = actor['source_list'] == 'objectList' and corroboration['raw_class'] == 'Obstacle'
        pedestrian = allow_pedestrians and actor['source_list'] == 'pedestrianList' and corroboration['raw_class'] == 'Pedestrian'
        require(physical or pedestrian, 'Pedestrian extension requires separate explicit opt-in')
        road = actor['map_ground_initializer'];foot = road['footprint'];height = float(road['height_m'])
        require(road.get('policy') == 'mgeo-segment-associated-boundary-v1' and road.get('usable') is True
                and road.get('confidence') == 'road_strip' and road.get('reason') is None
                and np.isfinite(height) and abs(float(actor['position']['z'])-height) <= .25
                and 0 <= road['nearest_road_distance_m'] <= 2.
                and foot.get('all_supported') is True and 0 <= foot['map_height_span_m'] <= .15
                and len(foot['samples']) >= 3
                and all(s.get('usable') is True and s.get('confidence') == 'road_strip'
                    and np.isfinite(s.get('height_m')) and abs(s['height_m']-height) <= .15 for s in foot['samples']),
                'Original MGeo road/footprint support gates differ')
        require(map_hashes and all(road.get(key) == value for key,value in map_hashes.items()),
                'MGeo provenance must match current pinned input bytes')
        planned = vector([actor['position'][k] for k in ('x','y')],2)
        expected = vector(actor['geometric_center_estimate'][:2],2)
        actual = vector(corroboration['native_world_center_xy_m'],2)
        require(np.linalg.norm(vector(road['query_xy_m'],2)-planned) <= .05
                and np.linalg.norm(actual-expected) <= .50, 'Native/planned world XY disagree')
        fitted = float(np.r_[xy,1.] @ coef)
        surface_world = (rotation @ np.r_[xy,fitted])[2]+ego['z']
        low_world = corroboration['lowest_native_world_z_m']
        require(np.isfinite(surface_world) and np.isfinite(low_world)
                and surface_world-height >= 2. and abs(low_world-height) <= .30,
                'Original upper-level >2m / native-to-road ±.30m gates not satisfied')
        return dict(eligible=True, candidate_reason='actor_ground_surface_association_unverified',
            surface_world_z_m=float(surface_world), map_road_z_m=height,
            fitted_minus_road_level_m=float(surface_world-height),
            native_lowest_corner_minus_road_level_m=float(low_world-height),
            legacy_yaw_only_lowest_corner_minus_road_level_m=float(corroboration['legacy_yaw_only_lowest_world_z_m']-height),
            native_to_planned_center_xy_delta_m=float(np.linalg.norm(actual-expected)),
            pedestrian_extension_opt_in=bool(pedestrian), original_plane_and_gap_preserved=True,
            verified_ground_or_mesh_contact=False, proposed_native_approval=False)
    except (ValueError,KeyError,TypeError,OverflowError) as error:
        return dict(eligible=False, reason=str(error), proposed_native_approval=False)


def analyze(root, *, allow_pedestrians=False, map_hashes=None, original_audit=None):
    root = Path(root).resolve(strict=True)
    metadata, telemetry, scenario, sensor = [read(root/name) for name in
        ('metadata.json','telemetry.json','scenario.json','sensor_reference.json')]
    require(original_audit is None or isinstance(original_audit,dict), 'RAM original audit must be a mapping')
    audit = read(root/'audit.json') if original_audit is None else copy.deepcopy(original_audit)
    original_reasons = audit.get('review_reasons', [])
    result = dict(capture_root=str(root), original_review_reasons=original_reasons,
        original_accepted_candidate=audit.get('accepted_candidate'), candidate_only=True,
        bulk_ready=False, native_qualified=False, source_mutation_performed=False, actors=[])
    try:
        rotation, transform, proof = attitude(root, metadata, telemetry, scenario, sensor)
        result['attitude'] = proof
        annotation = only(root.glob('LIDAR_5/*_Instance.txt'), 'One native annotation required')
        if not metadata.get('runtime_actors') and not audit.get('components'):
            companion = annotation.with_name(annotation.name[:-len('_Instance.txt')]+'_Instance_8Points.txt')
            require(not annotation.read_text().strip() and not companion.read_text().strip(),
                    'Empty background must not contain unmatched native components')
            records = {};result['empty_background_native_records'] = True
        else:
            records = load_matching_corners(annotation)
    except (ValueError,KeyError,TypeError,OverflowError,RuntimeError) as error:
        result.update(attitude_qualified=False, diagnostic_only_reason=str(error),
                      candidate_review_reasons=original_reasons, classification_changed=False)
        return result
    result['attitude_qualified'] = True
    ground = audit.get('ground_support') or {}
    changed = False
    below_results = []
    for row in ground.get('actors', []):
        item = dict(uid=row['uid'], original_reason=row.get('reason'),
                    original_gap_m=row.get('minimum_annotation_ground_gap_m'), eligible=False)
        matching = [record for key,record in records.items() if key[0] == row['uid']]
        try:
            require(len(matching) == 1, 'One measured component required')
            observation = world_corroboration(matching[0], transform, rotation, telemetry['ego_before'])
            item['full_rpy_world_corroboration'] = observation
            candidate = candidate_association(row, ground, metadata, observation, rotation,
                telemetry['ego_before'], allow_pedestrians=allow_pedestrians, map_hashes=map_hashes)
            item.update(candidate)
        except (ValueError,KeyError,TypeError,OverflowError,RuntimeError) as error:
            item['reason'] = str(error)
        if row.get('reason') == 'actor_annotation_deep_below_ground' and row.get('minimum_annotation_ground_gap_m',0) < -.40:
            below_results.append(item['eligible'])
        result['actors'].append(item)
    candidate_reasons = list(original_reasons)
    if 'static_ground_severe_below' in original_reasons and below_results and all(below_results):
        candidate_reasons.remove('static_ground_severe_below');changed=True
    result.update(candidate_review_reasons=candidate_reasons, classification_changed=changed,
        candidate_classification='unverified_surface_association_warning' if changed else 'unchanged',
        candidate_result_is_native_approval=False)
    return result


def build(roots, output, *, allow_pedestrians=False):
    roots = sorted(set(Path(root).resolve(strict=True) for root in roots))
    require(roots, 'At least one completed capture root required')
    output = Path(output).resolve()
    require(not output.exists() and not any(root == output or root in output.parents for root in roots),
            'Fresh output outside source captures required')
    map_paths = dict(source_map_link_sha256=gen.geo.DEFAULT_MGEO/'link_set.json',
                     source_map_boundary_sha256=gen.geo.DEFAULT_MGEO/'lane_boundary_set.json')
    map_hashes = {key:digest(path) for key,path in map_paths.items()}
    source_files = [path for root in roots for path in root.rglob('*') if path.is_file()]
    source_files += list(map_paths.values())
    before = pins(source_files)
    results = [analyze(root, allow_pedestrians=allow_pedestrians, map_hashes=map_hashes) for root in roots]
    final_source_files = [path for root in roots for path in root.rglob('*') if path.is_file()]
    final_source_files += list(map_paths.values())
    require(before == pins(final_source_files), 'Source files changed during candidate analysis')
    counts = Counter(result['candidate_classification'] if 'candidate_classification' in result
                     else 'attitude_unqualified_diagnostic_only' for result in results)
    report = dict(schema=SCHEMA, policy=POLICY, helper_sha256=digest(__file__),
        capture_count=len(results), results=results, counts=dict(counts),
        changed_captures=[r['capture_root'] for r in results if r.get('classification_changed')],
        source_files_before=before, source_snapshot_unchanged=True,
        allow_static_single_component_pedestrians=allow_pedestrians,
        production_qa_modified=False, existing_audit_modified=False, raw_gt_modified=False,
        source_runtime_changed=False, bulk_ready=False, native_qualified=False,
        candidate_results_are_capture_or_training_approval=False,
        numerical_guards=dict(severe_below_m=-.40, severe_above_m=.40, native_road_level_tolerance_m=.30,
            upper_competing_level_minimum_m=2., sparse_patch_points=[18,64], maximum_gyro_radps=.01),
        limitations=['No new road plane or contact ground truth is invented.',
            'Existing collector/runtime/approval and source annotations are unchanged.',
            'Native IMU timestamp text is preserved; same filename and static witnesses do not certify exact source synchronization.',
            'Any future runtime adoption requires new pins, tests and native requalification.'])
    # 분석 중 다른 실행이 만든 디렉터리·symlink의 보고서를 덮어쓰지 않는다.
    output.mkdir(parents=True, exist_ok=False)
    atomic_json(output/'report.json', report)
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture-root', type=Path, action='append', required=True,
        help='Completed native capture or a parent containing captures; may be repeated')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--allow-static-pedestrians', action='store_true',
        help='Separate candidate comparison only, never changes production policy')
    args = parser.parse_args()
    roots = []
    for path in args.capture_root:
        if (path/'audit.json').is_file():roots.append(path)
        else:roots.extend(p.parent for p in path.rglob('audit.json') if (p.parent/'telemetry.json').is_file())
    result = build(roots, args.output, allow_pedestrians=args.allow_static_pedestrians)
    print(json.dumps({k:result[k] for k in ('capture_count','counts','changed_captures','source_snapshot_unchanged')},indent=2))
