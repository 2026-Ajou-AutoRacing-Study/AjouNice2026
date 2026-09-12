"""ROS-independent policy. Invalid configurations fail before any ROS I/O."""
import math
import uuid

CASES = ('route_a_only', 'route_b_only', 'route_a_to_b',
         'route_a_to_fallback', 'route_a_to_b_with_fallback')
NORMAL = ('route_a_to_b', 'route_a_to_b_with_fallback')
FALLBACK = ('route_a_to_fallback', 'route_a_to_b_with_fallback')


def validate(config, routes):
    if config.get('route_operation_case') not in CASES:
        raise ValueError('route_operation_case must be one of ' + repr(CASES))
    for name in ('odometry_max_age', 'reroute_availability_max_age',
                 'reroute_available_hold_time', 'service_timeout',
                 'route_verification_timeout', 'trajectory_verification_timeout',
                 'retry_initial_interval', 'retry_reroute_interval', 'trajectory_max_age',
                 'operation_mode_max_age', 'goal_tolerance_m', 'route_a_refresh_max_speed_mps'):
        value = config.get(name)
        if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError('invalid positive parameter: ' + name)
    if config.get('never_clear_route') is not True or config.get('require_new_trajectory') is not True:
        raise ValueError('route preservation and trajectory verification are mandatory')
    if config.get('allow_runtime_case_change') is not False:
        raise ValueError('case changes require a restart')
    for name in ('initial_route_max_attempts', 'reroute_max_attempts'):
        if type(config.get(name)) is not int or config[name] < 0:
            raise ValueError('invalid attempt limit: ' + name)
    ids = set()
    for key in ('route_a', 'route_b', 'fallback_route'):
        r = routes[key]
        identity = str(uuid.UUID(r['uuid']))
        if identity in ids:
            raise ValueError('route UUIDs must be distinct')
        ids.add(identity)
        sequence = r['expected_path_lanelet_ids']
        if not sequence or len(set(sequence)) != len(sequence):
            raise ValueError('empty or looping route: ' + key)
        for ref in r.get('waypoint_refs', []) + [r['goal_ref']]:
            pose = routes['poses'][ref]
            if not all(math.isfinite(float(pose[k])) for k in ('x', 'y', 'yaw_deg')):
                raise ValueError('non-finite pose ' + ref)
    if routes['frame_id'] != 'map':
        raise ValueError('this frozen fixture requires map frame')


def fresh(now, stamp, maximum):
    return math.isfinite(stamp) and stamp > 0 and 0 <= now - stamp <= maximum


def allowed(case, key):
    return (key == 'route_a' and case != 'route_b_only' or
            key == 'route_b' and (case == 'route_b_only' or case in NORMAL) or
            key == 'fallback_route' and case in FALLBACK)


def route_matches(routes, key, identity, sequence, goal, tolerance):
    definition = routes[key]
    expected = definition['expected_path_lanelet_ids']
    valid_sequences = [expected]
    if key == 'route_a':
        valid_sequences.append([routes['start']['incoming_lanelet_id']] + expected)
    elif key == 'route_b':
        # A moving retry starts at the current lane, not the departed trigger.
        # Accept only the exact frozen suffix, and only starts before the deadline.
        for index, lane in enumerate(expected):
            if lane == 200346:
                break
            valid_sequences.append(expected[index:])
    target = routes['poses'][definition['goal_ref']]
    return (identity == str(uuid.UUID(definition['uuid'])) and sequence in valid_sequences and
            math.hypot(goal[0] - target['x'], goal[1] - target['y']) <= tolerance)


def in_window(routes, key, lane, arc):
    if key == 'route_b':
        sequence = routes['route_a']['expected_path_lanelet_ids']
        trigger = routes['poses']['wp_200178']
        if lane not in sequence:
            return False
        index = sequence.index(lane)
        return (sequence.index(trigger['lanelet_id']) <= index < sequence.index(200346) and
                (lane != trigger['lanelet_id'] or arc + 1e-6 >= trigger['lanelet_s_m']))
    if key == 'fallback_route':
        cutoff = routes['poses'][routes['fallback_route']['hard_cutoff_pose_ref']]
        return lane == cutoff['lanelet_id'] and arc + 1e-6 >= cutoff['lanelet_s_m']
    return lane in routes['start']['accepted_start_lanelet_ids']


def next_normal(case, active, lane, arc, routes):
    if not active:
        if case == 'route_b_only':
            # Standalone B has one exact start segment, not the later retry window.
            return 'route_b' if lane == 200178 and arc + 1e-6 >= 8 else None
        return 'route_a' if in_window(routes, 'route_a', lane, arc) else None
    if active == 'route_a' and case in NORMAL and in_window(routes, 'route_b', lane, arc):
        return 'route_b'
    return None
