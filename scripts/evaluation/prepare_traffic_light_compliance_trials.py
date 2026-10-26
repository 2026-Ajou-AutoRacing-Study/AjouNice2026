#!/usr/bin/env python3
"""Prepare static A/B/C/D stopline trials, never load a scene or start ROS nodes.

Bag Ego is a development-only source for native scenario coordinates. Runtime
trials continue to use actual localization, route, YOLO/CNN and real control.
No operator-GT mode, forced light phase, pose provider or native API is enabled.
"""
import argparse
from bisect import bisect_left
import copy
import hashlib
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'scripts/evaluation'))
from traffic_light_stationary_probe import normalized_scene, validate_output_directory
from prepare_traffic_light_v5_trial import prepare as prepare_v5, BASE

GROUPS = ((201351, 'A', ['C1256W000077']), (201365, 'B', ['C1256W000091']),
          (201290, 'C', ['C1256W000016', 'C1256W000018']),
          (201326, 'D', ['C1256W000052']))
FOCUSED_MISSION_WAYPOINTS = (('A', 0), ('B', 1), ('C', 2), ('D', 3))
RAW_HEADING_CORRECTION_DEG = -1.347
TOPICS = ['/Ego_topic', '/Competition_topic', '/CollisionData', '/VehicleCollisionData',
          '/sensor/gps', '/sensor/imu', '/localization/gnss/odom',
          '/localization/odometry/local', '/localization/kinematic_state',
          '/localization/control_kinematic_state', '/tf', '/tf_static',
          '/planning/mission_planning/route', '/planning/route_state',
          '/planning/trajectory', '/planning/validation_status',
          '/control/command/control_cmd', '/control/command/steering', '/ctrl_cmd',
          '/purepursuit_autoware/debug/lateral_error',
          '/perception/traffic_light_recognition/traffic_signals',
          '/perception/traffic_light_recognition/route_group_id',
          '/perception/traffic_light_recognition/map_rois',
          '/traffic_light_node/debug_selection', '/traffic_light_node/debug_classifier',
          '/traffic_light_node/debug_runtime', '/traffic_light_node/debug_policy_reason',
          '/traffic_light_node/debug_observed_label', '/traffic_light_node/debug_raw_label',
          '/evaluation/traffic_light_policy/fault',
          '/evaluation/traffic_light_policy/source_signals',
          '/evaluation/traffic_light_policy/planner_fixture_signals',
          '/evaluation/traffic_light_policy/planner_fixture_status',
          '/diagnostics', '/rosout', '/rosout_agg']


def sha(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(4*1024**2), b''):
            value.update(chunk)
    return value.hexdigest()


def wrap_degrees(value):
    return (value+180.) % 360.-180.


def project_line(points, x, y):
    best, length = None, 0.
    for a, b in zip(points, points[1:]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        size = math.hypot(dx, dy)
        if size <= 1e-9:
            continue
        fraction = max(0., min(1., ((x-a[0])*dx+(y-a[1])*dy)/size**2))
        distance = math.hypot(x-a[0]-fraction*dx, y-a[1]-fraction*dy)
        candidate = (distance, length+fraction*size, math.atan2(dy, dx))
        if best is None or candidate[0] < best[0]:
            best = candidate
        length += size
    if best is None:
        raise ValueError('Line has no finite nonzero segment')
    return best[0], best[1], length, best[2]


def stopline_groups(map_path, projector_path, forward=None):
    """Read exact OSM ref_line nodes; injectable projection keeps tests pure."""
    if forward is None:
        sys.path.insert(0, str(ROOT/'base_ws/src/perception/traffic_light_detection/scripts'))
        from lanelet_route_matcher import _load_projector
        from lanelet2.core import GPSPoint
        projector = _load_projector(str(map_path), 'LocalCartesianUTM', str(projector_path))
        forward = lambda lat, lon: projector.forward(GPSPoint(lat, lon, 0.))
    tree = ET.parse(map_path).getroot()
    nodes = {node.get('id'): node for node in tree.findall('node')}
    ways = {way.get('id'): way for way in tree.findall('way')}
    relations = {int(relation.get('id')): relation for relation in tree.findall('relation')}
    result = []
    for group_id, mission, native_ids in GROUPS:
        relation = relations[group_id]
        tags = {tag.get('k'): tag.get('v') for tag in relation.findall('tag')}
        if tags.get('subtype') != 'traffic_light':
            raise ValueError('Mission relation is not a traffic light')
        lines = []
        for member in relation.findall('member'):
            if member.get('role') != 'ref_line' or member.get('type') != 'way':
                continue
            points = []
            for nd in ways[member.get('ref')].findall('nd'):
                node = nodes[nd.get('ref')]
                point = forward(float(node.get('lat')), float(node.get('lon')))
                xy = [float(point.x), float(point.y)]
                if not all(math.isfinite(value) for value in xy):
                    raise ValueError('Nonfinite stopline coordinate')
                points.append(xy)
            project_line(points, *points[0])
            lines.append(dict(way_id=int(member.get('ref')), points_xy_m=points))
        if not lines:
            raise ValueError('Mission stopline is missing: '+mission)
        if len(lines) != 1:
            # The observer/controller's flat polyline is unambiguous only for
            # one explicit ref_line. Never connect separate OSM stop lines.
            raise ValueError('Exactly one ref_line is required for flat stop_line_xy: '+mission)
        result.append(dict(group_id=group_id, mission=mission, native_ids=native_ids,
                           stop_lines=lines, stop_line_xy=copy.deepcopy(lines[0]['points_xy_m'])))
    return result


def route_projection(geometry, x, y, yaw_degrees):
    candidates = []
    for index, start, preferred_length, alternatives in geometry:
        for lane_id, points, _ in alternatives:
            distance, along, length, yaw = project_line(points, x, y)
            heading_error = abs(wrap_degrees(yaw_degrees-math.degrees(yaw)))
            if distance <= 8. and heading_error <= 60.:
                candidates.append((distance**2+4*(1-math.cos(math.radians(heading_error))),
                    dict(progress_m=start+preferred_length*along/length, lanelet_id=lane_id,
                         segment_index=index, lateral_m=distance, heading_error_degrees=heading_error)))
    if not candidates:
        raise ValueError('Pose is not aligned with the recorded actual route')
    return min(candidates, key=lambda row: row[0])[1]


def goal_check(goal, geometry, event_s, start_s):
    try:
        projection = route_projection(geometry, goal['x'], goal['y'], goal['yaw_deg'])
    except ValueError as error:
        return dict(valid=False, reason=str(error))
    after = projection['progress_m'] > event_s+5. and projection['progress_m'] > start_s
    return dict(valid=after, reason='after_stopline_on_recorded_route' if after else 'not_after_stopline',
                projection=projection, distance_after_stopline_m=projection['progress_m']-event_s)


def native_xy_in_map(x, y, transform):
    yaw = math.radians(transform['yaw_degrees'])
    tx, ty = transform['translation_xyz_m'][:2]
    return tx+math.cos(yaw)*x-math.sin(yaw)*y, ty+math.sin(yaw)*x+math.cos(yaw)*y


def select_start(ego_rows, localization_rows, geometry, group, event_s, transform):
    times = [row['receipt_sec'] for row in ego_rows]
    candidates = []
    for loc in localization_rows:
        # Cheap line-distance screening first: do not re-project the whole
        # route for every pose in a multi-minute bag.
        distance = min(project_line(line['points_xy_m'], *loc['xy_m'])[0]
                       for line in group['stop_lines'])
        if not 15. <= distance <= 20.:
            continue
        try:
            projected = route_projection(geometry, *loc['xy_m'], loc['yaw_degrees'])
        except ValueError:
            continue
        along = event_s-projected['progress_m']
        if not 10. <= along <= 30. or projected['lateral_m'] > 1.5:
            continue
        index = bisect_left(times, loc['receipt_sec'])
        nearby = [ego_rows[i] for i in (index-1, index) if 0 <= i < len(ego_rows)]
        if not nearby:
            continue
        ego = min(nearby, key=lambda row: abs(row['receipt_sec']-loc['receipt_sec']))
        dt = abs(ego['receipt_sec']-loc['receipt_sec'])
        native_map_xy = native_xy_in_map(*ego['xyz_m'][:2], transform)
        discrepancy = math.dist(native_map_xy, loc['xy_m'])
        map_yaw = ego['heading_degrees']+RAW_HEADING_CORRECTION_DEG+transform['yaw_degrees']
        yaw_error = abs(wrap_degrees(map_yaw-loc['yaw_degrees']))
        if dt > .05 or discrepancy > 1. or yaw_error > 5.:
            continue
        ego_distance = min(project_line(line['points_xy_m'], *native_map_xy)[0]
                           for line in group['stop_lines'])
        score = abs(distance-17.5)+discrepancy+10.*dt
        candidates.append((score, dict(native_pose=dict(zip(('x', 'y', 'z'), ego['xyz_m']),
            yaw=wrap_degrees(ego['heading_degrees']+RAW_HEADING_CORRECTION_DEG)),
            localization=loc, ego=ego, localization_stopline_distance_2d_m=distance,
            ego_stopline_distance_2d_m=ego_distance, ego_localization_distance_2d_m=discrepancy,
            paired_receipt_delta_sec=dt, corrected_heading_localization_delta_degrees=yaw_error,
            route_projection=projected, along_route_to_stopline_m=along)))
    if not candidates:
        raise ValueError('No recorded 15..20m approach pose with paired localization/Ego: '+group['mission'])
    return min(candidates, key=lambda row: row[0])[1]


def read_bag(path):
    import rosbag
    ego, localization, route, transforms = [], [], None, []
    with rosbag.Bag(str(path)) as bag:
        start = bag.get_start_time()
        for topic, msg, stamp in bag.read_messages(topics=['/Ego_topic', '/localization/kinematic_state',
                                                       '/planning/mission_planning/route', '/tf_static']):
            elapsed = stamp.to_sec()-start
            if topic == '/Ego_topic':
                ego.append(dict(receipt_sec=elapsed, source_stamp_ns=msg.header.stamp.to_nsec(),
                                xyz_m=[msg.position.x, msg.position.y, msg.position.z], heading_degrees=msg.heading))
            elif topic == '/localization/kinematic_state':
                if msg.header.frame_id.strip('/') != 'map':
                    raise ValueError('Expected map-frame localization')
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                yaw = math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
                localization.append(dict(receipt_sec=elapsed, source_stamp_ns=msg.header.stamp.to_nsec(),
                                         xy_m=[p.x, p.y], yaw_degrees=yaw))
            elif topic.endswith('/route'):
                if route is not None:
                    old = [[p.id for p in s.primitives] for s in route.segments]
                    if old != [[p.id for p in s.primitives] for s in msg.segments]:
                        raise ValueError('Multiple different routes require an explicit route-window implementation')
                route = msg
            else:
                for tf in msg.transforms:
                    if (tf.header.frame_id.strip('/'), tf.child_frame_id.strip('/')) != ('map', 'morai_map'):
                        continue
                    t, q = tf.transform.translation, tf.transform.rotation
                    if abs(q.x) > 1e-6 or abs(q.y) > 1e-6:
                        raise ValueError('Only a planar recorded map<-morai_map transform is supported')
                    transforms.append(dict(translation_xyz_m=[t.x, t.y, t.z],
                        yaw_degrees=math.degrees(2*math.atan2(q.z, q.w))))
    if not ego or not localization or route is None or not transforms or any(x != transforms[0] for x in transforms):
        raise ValueError('Require Ego, localization, one actual route and one consistent recorded static map transform')
    return ego, localization, route, transforms[0]


def trial_document(source, output, mission, scene_name, goal=None):
    doc = copy.deepcopy(source)
    doc['campaign_name'] = 'traffic_light_compliance_'+mission
    doc['scenarios'] = dict(directory=str(output/'scenarios'), glob='*.json', repeat=1, include=[scene_name])
    doc['recording'].update(bag_enabled=True, include_sensors=False, selected_topics=list(TOPICS),
                            minimum_free_disk_gib=4., video=dict(enabled=False))
    doc['upload']['enabled'] = False
    if goal is not None:
        doc['default_case'].update(goal=copy.deepcopy(goal), waypoints=[], timeout_sec=180)
    return doc


def prepare(args):
    output = validate_output_directory(args.output)
    inputs = [args.bag, args.map, args.projector, BASE,
              ROOT/'base_ws/src/vehicle/README.md', ROOT/'base_ws/src/vehicle/config/vehicle.yaml',
              Path(__file__), ROOT/'scripts/evaluation/prepare_traffic_light_v5_trial.py',
              ROOT/'scripts/evaluation/traffic_light_stationary_probe.py',
              ROOT/'base_ws/src/perception/traffic_light_detection/scripts/lanelet_route_matcher.py']
    fingerprints = {str(Path(p).resolve(strict=True)): sha(p) for p in inputs}
    groups = stopline_groups(args.map, args.projector)
    ego, localization, route, transform = read_bag(args.bag)
    sys.path.insert(0, str(ROOT/'base_ws/src/perception/traffic_light_detection/scripts'))
    from lanelet_route_matcher import LaneletRouteGroupIdMatcher
    matcher = LaneletRouteGroupIdMatcher(str(args.map), 'LocalCartesianUTM', str(args.projector), route_progress_guard=True)
    if not matcher.available:
        raise RuntimeError('Actual Lanelet2 route geometry is unavailable')
    matcher.lanelet_group_id = {k: v for k, v in matcher.lanelet_group_id.items() if v in {g[0] for g in GROUPS}}
    matcher._route_callback(route)
    if not matcher._route_geometry:
        raise ValueError('Recorded route geometry rejected: '+str(matcher._progress_debug))
    events = {event['group_id']: event['progress_m'] for event in matcher._route_events}
    starts = {g['mission']: select_start(ego, localization, matcher._route_geometry, g, events[g['group_id']], transform)
              for g in groups}
    output.mkdir(parents=True)
    base_trial = prepare_v5(output/'prepared_full_source', args.models_dir, args.python)
    doc = yaml.safe_load(base_trial.read_text())
    source_scene = Path(doc['scenarios']['directory'])/doc['scenarios']['include'][0]
    scene = json.loads(source_scene.read_text())
    (output/'scenarios').mkdir()
    (output/'scenarios/full_regression.json').write_text(json.dumps(scene, indent=2)+'\n')
    goal_receipts = {}
    for mission, requested_index in FOCUSED_MISSION_WAYPOINTS:
        group = next(g for g in groups if g['mission'] == mission)
        event_s = events[group['group_id']]
        pose = starts[mission]
        goal_candidates = doc['default_case']['waypoints']
        requested = goal_candidates[requested_index]
        requested_check = goal_check(requested, matcher._route_geometry, event_s, pose['route_projection']['progress_m'])
        checked = [(goal, goal_check(goal, matcher._route_geometry, event_s, pose['route_projection']['progress_m']))
                   for goal in goal_candidates]
        valid = [(goal, check) for goal, check in checked if check['valid']]
        if not valid:
            raise ValueError('No original full-course waypoint after stopline for '+mission)
        goal, check = (requested, requested_check) if requested_check['valid'] else min(valid, key=lambda row: row[1]['projection']['progress_m'])
        focused_scene = normalized_scene(scene, pose['native_pose'])
        scene_name = mission+'_stopline_approach.json'
        (output/'scenarios'/scene_name).write_text(json.dumps(focused_scene, indent=2)+'\n')
        focused = trial_document(doc, output, mission, scene_name, goal)
        (output/(mission+'_focused.yaml')).write_text(yaml.safe_dump(focused, allow_unicode=True, sort_keys=False))
        goal_receipts[mission] = dict(requested_original_waypoint_index=requested_index, requested_goal=requested,
                                     requested_check=requested_check, selected_goal=goal, selected_check=check)
    full = trial_document(doc, output, 'full_regression', 'full_regression.json')
    if full['default_case'] != doc['default_case']:
        raise AssertionError('Full regression goal/waypoints changed')
    (output/'full_regression.yaml').write_text(yaml.safe_dump(full, allow_unicode=True, sort_keys=False))
    vehicle = yaml.safe_load((ROOT/'base_ws/src/vehicle/config/vehicle.yaml').read_text())['vehicle']
    groups_doc = dict(schema_version=1, frame='map', groups=groups,
        vehicle_reference=dict(base_link='rear axle center', front_axle_offset_x_m=vehicle['wheelbase_m'],
            front_bumper_offset_x_m=vehicle['wheelbase_m']+vehicle['front_overhang_m'],
            note='3.000m is rear-to-front axle, not rear-to-front bumper; no runtime offset/policy changed'),
        map_sha256=sha(args.map), projector_sha256=sha(args.projector), source_sha256=sha(__file__))
    (output/'groups.json').write_text(json.dumps(groups_doc, indent=2)+'\n')
    if fingerprints != {path: sha(path) for path in fingerprints}:
        raise RuntimeError('An input/source changed during preparation; retain partial output as failed evidence')
    receipt = dict(schema='traffic_light_compliance_preparation_v1', executed=False, source_sha256=fingerprints,
        coordinate_contract=dict(recorded_map_from_morai_map=transform,
            native_scene_heading_correction_applied_once_degrees=RAW_HEADING_CORRECTION_DEG,
            Ego='development scenario start only; never operational pose/GT provider',
            localization='actual map-frame recorded localization, not corrected again'),
        selected_starts=starts, focused_goal_checks=goal_receipts,
        full_regression_goal_and_waypoints='unchanged from prepare_traffic_light_v5_trial output',
        guardrails=dict(operator_gt_enabled=False, native_phase_control=False, forced_group_id=False,
                        raw_camera_recording=False, minimum_free_disk_gib=4.),
        generated_sha256={str(p.relative_to(output)): sha(p) for p in sorted(output.rglob('*')) if p.is_file()},
        limitations=['Static preparation only; no scene load, ROS publisher, launch, install or live driving.',
                     'Original full-route topology is checked; a future planner-generated focused route must still be inspected.',
                     'Scene pose comes from observed development bag; road/rendering/network readiness remains a live check.',
                     'No signal compliance, classifier accuracy or checkpoint reset certification.'])
    (output/'preparation.json').write_text(json.dumps(receipt, indent=2, allow_nan=False)+'\n')
    return dict(output=str(output), groups=str(output/'groups.json'),
                **{'focused_'+mission: str(output/(mission+'_focused.yaml'))
                   for mission, _ in FOCUSED_MISSION_WAYPOINTS},
                full_regression=str(output/'full_regression.yaml'))


def main():
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--map', type=Path, default=ROOT/'kcity_map/lanelet_map/lanelet2_map_v87.osm')
    parser.add_argument('--projector', type=Path, default=ROOT/'kcity_map/lanelet_map/map_projector_info_v1.yaml')
    parser.add_argument('--models-dir', type=Path, required=True)
    parser.add_argument('--python', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(prepare(args), ensure_ascii=False))


if __name__ == '__main__':
    main()
