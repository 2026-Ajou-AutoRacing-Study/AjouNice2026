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
        if key != 'fallback_route':
            waypoint_sequence = r.get('waypoint_path_lanelet_ids')
            if (not waypoint_sequence or len(waypoint_sequence) != len(sequence) or
                    len(set(waypoint_sequence)) != len(waypoint_sequence)):
                raise ValueError('invalid waypoint-planner route: ' + key)
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


def reroute_trigger(routes):
    config = routes['reroute_trigger']
    return routes['poses'][config['pose_ref']], float(config['minimum_lanelet_s_m'])


def startup_route_b_trigger(routes):
    config = routes['startup_route_b_trigger']
    return routes['poses'][config['pose_ref']], float(config['minimum_lanelet_s_m'])


def at_reroute_trigger(routes, lane, arc):
    trigger, minimum_arc = reroute_trigger(routes)
    return lane == trigger['lanelet_id'] and arc + 1e-6 >= minimum_arc


def accepted_sequences(routes, key, field='expected_path_lanelet_ids'):
    definition = routes[key]
    expected = definition[field]
    valid_sequences = [expected]
    if key == 'route_a':
        valid_sequences.append([routes['start']['incoming_lanelet_id']] + expected)
        # A planner session may start anywhere between the official start and
        # the first Route B trigger.  The waypoint planner then publishes the
        # exact Route A suffix beginning at the current lanelet.
        trigger_lane = startup_route_b_trigger(routes)[0]['lanelet_id']
        for index in range(1, expected.index(trigger_lane) + 1):
            valid_sequences.append(expected[index:])
    elif key == 'route_b':
        # A startup after the trigger starts at the current lane, not the
        # departed trigger. Accept only an exact frozen suffix.
        for index in range(1, len(expected)):
            valid_sequences.append(expected[index:])
    return valid_sequences


def route_matches(routes, key, identity, sequence, goal, tolerance,
                  field='expected_path_lanelet_ids'):
    definition = routes[key]
    target = routes['poses'][definition['goal_ref']]
    identity_and_goal_match = (
        identity == str(uuid.UUID(definition['uuid'])) and
        math.hypot(goal[0] - target['x'], goal[1] - target['y']) <= tolerance)
    if key == 'route_b':
        # RouteSelector may choose an equivalent parallel-lane branch when B
        # starts from the current vehicle pose.  B has its own fixed UUID and
        # goal, so do not reject that route only because its lanelet sequence
        # differs from the frozen Route A sequence/model.
        return identity_and_goal_match and bool(sequence)
    return identity_and_goal_match and sequence in accepted_sequences(routes, key, field)


def preferred_sequence(routes, key, waypoint_sequence):
    """Map an accepted waypoint-planner sequence to its final preferred lanes."""
    if key == 'fallback_route':
        return list(waypoint_sequence)
    initial = routes[key]['waypoint_path_lanelet_ids']
    final = routes[key]['expected_path_lanelet_ids']
    if key == 'route_a':
        incoming = routes['start']['incoming_lanelet_id']
        if waypoint_sequence == [incoming] + initial:
            return [incoming] + final
    for candidate in accepted_sequences(routes, key, 'waypoint_path_lanelet_ids'):
        if waypoint_sequence == candidate:
            offset = len(initial) - len(candidate)
            return final[offset:]
    if key == 'route_b' and waypoint_sequence:
        # Preserve RouteSelector's valid B branch when it cannot be mapped to
        # the frozen preferred-lane model.  Reapplying the observed preferred
        # primitives is a safe no-op and lets the normal UUID/goal and fresh
        # trajectory checks complete.
        return list(waypoint_sequence)
    return None


def in_window(routes, key, lane, arc, initial=False):
    if key == 'route_a':
        sequence = routes['route_a']['expected_path_lanelet_ids']
        trigger, minimum_arc = startup_route_b_trigger(routes)
        if lane == routes['start']['incoming_lanelet_id']:
            return arc + 1e-6 >= routes['start']['incoming_lanelet_end_s_m']
        if lane not in sequence:
            return False
        index = sequence.index(lane)
        trigger_index = sequence.index(trigger['lanelet_id'])
        return (index < trigger_index or
                index == trigger_index and arc + 1e-6 < minimum_arc)
    if key == 'route_b':
        sequence = routes['route_b']['waypoint_path_lanelet_ids']
        trigger, minimum_arc = (startup_route_b_trigger(routes) if initial
                                else reroute_trigger(routes))
        if lane not in sequence:
            return False
        route_a = routes['route_a']['expected_path_lanelet_ids']
        route_a_trigger_index = route_a.index(trigger['lanelet_id'])
        if lane in route_a[:route_a_trigger_index]:
            return False
        index = sequence.index(lane)
        trigger_index = sequence.index(trigger['lanelet_id'])
        if initial:
            end_lane = routes['startup_route_b_trigger']['end_lanelet_id']
            if index > sequence.index(end_lane):
                return False
            if lane == routes['start']['incoming_lanelet_id']:
                return arc + 1e-6 < routes['start']['incoming_lanelet_end_s_m']
        return (index > trigger_index or
                index == trigger_index and arc + 1e-6 >= minimum_arc)
    if key == 'fallback_route':
        cutoff = routes['poses'][routes['fallback_route']['hard_cutoff_pose_ref']]
        return lane == cutoff['lanelet_id'] and arc + 1e-6 >= cutoff['lanelet_s_m']
    return False


def next_normal(case, active, lane, arc, routes):
    if not active:
        if allowed(case, 'route_a') and in_window(routes, 'route_a', lane, arc):
            return 'route_a'
        if allowed(case, 'route_b') and in_window(routes, 'route_b', lane, arc, initial=True):
            return 'route_b'
        return None
    # An active Route A may switch only while the ego is on the configured
    # trigger lanelet. This prevents a repeated lanelet later in the course
    # from being mistaken for the A-to-B transition point.
    if active == 'route_a' and case in NORMAL and at_reroute_trigger(routes, lane, arc):
        return 'route_b'
    return None


def remaining_waypoint_refs(routes, key, current_lane):
    """Return only waypoints ahead of the current lane on a frozen route."""
    definition = routes[key]
    refs = definition.get('waypoint_refs', [])
    if key != 'route_b':
        return refs
    sequence = definition['waypoint_path_lanelet_ids']
    if current_lane not in sequence:
        return refs
    current_index = sequence.index(current_lane)
    lane_indexes = {lane: index for index, lane in enumerate(sequence)}
    return [ref for ref in refs
            if lane_indexes.get(routes['poses'][ref]['lanelet_id'], -1) > current_index]
