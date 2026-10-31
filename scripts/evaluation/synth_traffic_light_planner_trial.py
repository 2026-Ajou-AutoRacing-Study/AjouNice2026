#!/usr/bin/env python3
"""MANUAL FIXTURE, PLANNER-ONLY development test; NOT YOLO/lamp certification.

Publishes synthetic elemental signals ONLY on a fixed evaluation topic. The
root-owned exclusive relay, graph lock, validator, real planner/controller and
stop/P lifecycle are mandatory. No native API, vehicle control or stack launch.
"""
import argparse
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import signal
import threading
import time

from control_traffic_light_policy_trial import fresh_inputs
from observe_traffic_light_compliance import ROOT, JsonlWriter, finite, load_geometry, primitive, rotate, segment_crossing, stamp_ns

SIGNAL_TOPIC = '/evaluation/traffic_light_policy/planner_fixture_signals'
STATUS_TOPIC = '/evaluation/traffic_light_policy/planner_fixture_status'
ROUTE_TOPIC = '/perception/traffic_light_recognition/route_group_id'
MODES = ('stopped-yellow', 'moving-yellow-stoppable', 'moving-yellow-close',
         'loss-prefront', 'loss-postfront', 'postfront-red')
GROUPS = {'A': 201351, 'B': 201365, 'C': 201290, 'D': 201326}


class FixtureError(RuntimeError): pass
class FixtureNotApplicable(FixtureError): pass


def parse_group(value):
    value = str(value).upper()
    result = GROUPS[value] if value in GROUPS else int(value)
    if result not in GROUPS.values(): raise ValueError('Only A/B/C/D regulatory IDs are supported')
    return result


def phase_tokens(group_id, phase):
    if group_id not in GROUPS.values(): raise ValueError('Unknown fixture group')
    if phase == 'RED': return ['RED_CIRCLE']
    if phase == 'YELLOW': return ['RED_CIRCLE', 'YELLOW_CIRCLE'] if group_id == 201326 else ['YELLOW_CIRCLE']
    if phase == 'GO':
        if group_id == 201326: return ['RED_CIRCLE', 'GREEN_LEFT_ARROW']
        if group_id == 201290: return ['GREEN_CIRCLE', 'GREEN_LEFT_ARROW']
        return ['GREEN_CIRCLE']
    if phase is None: return []
    raise ValueError('Explicit manual fixture phase required')


def assign_fixture_stamp(message, seconds, nanoseconds):
    """Generated fixture time only; no perception libexec/Python-path dependency."""
    if type(seconds) is not int or seconds < 0 or type(nanoseconds) is not int or not 0 <= nanoseconds < 1000000000:
        raise ValueError('Explicit integer ROS fixture timestamp required')
    message.stamp.sec, message.stamp.nanosec = seconds, nanoseconds


def snapshot_with_current_clocks(lock, latest, monotonic, ros_now):
    """Copy inputs before taking their comparison clocks under the same lock.

    A callback may have waited for this lock with a newer receipt timestamp.
    Taking 'now' before locking could falsely mark that fresh input as future.
    The returned rows are immutable-by-convention callback replacements, not
    live ROS message objects; later callback replacements cannot alter them.
    """
    with lock:
        snapshot = dict(latest)
        now, ros_time = monotonic(), ros_now()
    return snapshot, now, ros_time


@dataclass(frozen=True)
class FixtureConfig:
    mode: str
    group_id: int
    probe_seconds: float = 2.
    front_offset_m: float = 3.
    trigger_min_m: float = None
    trigger_max_m: float = None
    trigger_min_speed_mps: float = None
    trigger_max_speed_mps: float = None

    def __post_init__(self):
        if self.mode not in MODES or self.group_id not in GROUPS.values(): raise ValueError('Explicit fixture mode/group required')
        if self.probe_seconds not in (2., 3.): raise ValueError('Yellow probe durations are explicit 2/3 second stress cases')
        if self.mode in ('loss-prefront', 'loss-postfront', 'postfront-red') and self.probe_seconds != 2.:
            raise ValueError('Loss/postfront probes are bounded at 2 seconds')
        if not 0 <= finite(self.front_offset_m) <= 6: raise ValueError('Invalid front offset')
        defaults = {'moving-yellow-stoppable': (10., 25., 1., 3.),
                    'moving-yellow-close': (.3, 1., 3., 25.),
                    'loss-prefront': (4., 12., .3, 25.)}
        values = defaults.get(self.mode, (.2, 2., 0., .1))
        for name, default in zip(('trigger_min_m', 'trigger_max_m', 'trigger_min_speed_mps', 'trigger_max_speed_mps'), values):
            if getattr(self, name) is None: object.__setattr__(self, name, default)
            finite(getattr(self, name))
        if not 0 < self.trigger_min_m <= self.trigger_max_m <= 30 or not 0 <= self.trigger_min_speed_mps < self.trigger_max_speed_mps <= 30:
            raise ValueError('Invalid bounded trigger window')
        if self.mode == 'moving-yellow-stoppable' and not (self.trigger_min_m >= 10 and self.trigger_min_speed_mps >= 1 and self.trigger_max_speed_mps <= 3):
            raise ValueError('Stoppable moving-Y requires distance>=10m and speed1..3m/s')
        if self.mode == 'moving-yellow-close' and not (self.trigger_min_m >= .3 and self.trigger_max_m <= 1 and self.trigger_min_speed_mps >= 3):
            raise ValueError('Close moving-Y requires front0.3..1m and speed>=3m/s')


def front_projection(odom, group, offset=3., segment_index=None, approach_sign=None):
    if odom.get('frame', '').strip('/') != 'map': raise FixtureError('Fixture requires corrected map-frame localization')
    position = list(map(finite, odom['position_xyz']))
    forward = rotate([1., 0., 0.], odom['quaternion_xyzw'])
    scale = math.hypot(*forward[:2])
    if scale < 1e-6: raise FixtureError('Invalid planar heading')
    forward = [value/scale for value in forward[:2]]
    front = [position[index]+offset*forward[index] for index in (0, 1)]
    candidates = []
    for index, (a, b) in enumerate(zip(group['stop_line_xy'], group['stop_line_xy'][1:])):
        if segment_index is not None and index != segment_index: continue
        vx, vy = b[0]-a[0], b[1]-a[1]
        length = math.hypot(vx, vy)
        if length < 1e-6: raise FixtureError('Degenerate stop segment')
        side = vx*(front[1]-a[1])-vy*(front[0]-a[0])
        derivative = vx*forward[1]-vy*forward[0]
        along = None
        distance = None
        if abs(derivative) > 1e-8:
            distance = -side/derivative  # Positive before, negative after along current heading.
            intersection = [front[axis]+distance*forward[axis] for axis in (0, 1)]
            along = ((intersection[0]-a[0])*vx+(intersection[1]-a[1])*vy)/(length*length)
        sign = approach_sign if approach_sign is not None else (1. if side >= 0 else -1.)
        candidates.append(dict(segment_index=index, front_xy_m=front, forward_xy=forward,
            raw_signed_side=side, approach_sign=sign, signed_distance_to_line_m=side/length*sign,
            forward_distance_m=distance, finite_segment_fraction=along,
            forward_ray_hits_finite_segment=along is not None and 0 <= along <= 1,
            segment_start_xy=a, segment_end_xy=b))
    usable = [row for row in candidates if row['forward_ray_hits_finite_segment']]
    if not usable and segment_index is None: raise FixtureNotApplicable('Heading ray misses the finite target stop segment')
    return min(usable or candidates, key=lambda row: abs(row['forward_distance_m']) if row['forward_distance_m'] is not None else float('inf'))


class PlannerFixture:
    """Pure condition-driven input program; no model/native/control dependencies."""
    def __init__(self, config, group):
        self.config, self.group = config, group
        self.stage, self.started_at, self.last_clock = 'waiting_inputs', None, None
        self.hold_since, self.armed, self.crossed = None, False, False
        self.segment_index, self.approach_sign, self.previous = None, None, None
        self.motion, self.last_odom_stamp, self.trigger = [], None, None
        self.probe_completed_at, self.settle_started_at, self.settled_stop = None, None, None
        self.settle_motion, self.settle_last_source = [], None
        self.stop_since, self.stop_first_source, self.stop_distinct_sources = None, None, set()
        self.complete = False

    def _decision(self, phase, **extra):
        return dict(planner_only=True, planner_only_fixture=True, source_kind='explicit_manual_program_not_perception_or_native_GT',
            mode=self.config.mode, group_id=self.config.group_id, stage=self.stage,
            manual_phase=phase, lamp_tokens=phase_tokens(self.config.group_id, phase),
            publish_signal=phase is not None, handoff_real=self.complete,
            fixture_complete=self.complete, behavior_PASS_certified=False,
            official_lamp_compliance_certified=False, **extra)

    def _sample_motion(self, latest, now, projection, speed):
        source = latest['odom']['source_stamp_ns']
        if source != self.last_odom_stamp:
            self.motion.append(dict(monotonic=now, odom_source_stamp_ns=source,
                                    front_xy_m=list(projection['front_xy_m']), speed_mps=speed))
            self.last_odom_stamp = source

    def _motion_summary(self, now):
        if len(self.motion) < 2: raise FixtureError('Probe ended without distinct fresh odometry motion samples')
        rows = self.motion
        gaps = [right['monotonic']-left['monotonic'] for left, right in zip(rows, rows[1:])]
        if rows[0]['monotonic']-self.started_at > .5 or now-rows[-1]['monotonic'] > .5 or max(gaps) > .5:
            raise FixtureError('Probe motion coverage has an unverified gap')
        first = rows[0]['front_xy_m']
        return dict(distinct_source_samples=len(rows), first_monotonic=rows[0]['monotonic'],
            last_monotonic=rows[-1]['monotonic'], maximum_gap_sec=max(gaps),
            maximum_speed_mps=max(row['speed_mps'] for row in rows),
            maximum_front_XY_displacement_m=max(math.dist(row['front_xy_m'], first) for row in rows))

    def _await_stopped_before_line(self, latest, now, projection, speed):
        """Separate RED settling proof; never append these samples to the probe."""
        if self.crossed or projection['signed_distance_to_line_m'] <= 0:
            raise FixtureError('Front crossed before verified prefront stop; do not qualify this fixture')
        if now-self.settle_started_at > 20.:
            raise FixtureError('RED settling exceeded bounded20seconds without verified prefront stop')
        source = latest['odom']['source_stamp_ns']
        if source != self.settle_last_source:
            self.settle_motion.append(dict(monotonic=now, odom_source_stamp_ns=source,
                                           front_xy_m=list(projection['front_xy_m']), speed_mps=speed))
            self.settle_last_source = source
        stopped = (speed < .1 and projection['forward_ray_hits_finite_segment']
                   and projection['forward_distance_m'] is not None and projection['forward_distance_m'] > 0)
        if not stopped:
            self.stop_since, self.stop_first_source = None, None
            self.stop_distinct_sources.clear()
        else:
            if self.stop_since is None:
                self.stop_since, self.stop_first_source = now, source
                self.stop_distinct_sources.clear()
            self.stop_distinct_sources.add(source)
        proof = dict(red_settle_started_monotonic=self.settle_started_at,
            red_settle_elapsed_sec=now-self.settle_started_at, source_samples=len(self.settle_motion),
            maximum_settle_speed_mps=max(row['speed_mps'] for row in self.settle_motion),
            maximum_settle_front_XY_displacement_m=max(math.dist(row['front_xy_m'], self.settle_motion[0]['front_xy_m'])
                                                       for row in self.settle_motion),
            stopped_hold_since_monotonic=self.stop_since,
            stopped_wall_span_sec=now-self.stop_since if self.stop_since is not None else 0.,
            stopped_source_span_sec=(source-self.stop_first_source)/1e9 if self.stop_first_source is not None else 0.,
            distinct_stopped_source_samples=len(self.stop_distinct_sources),
            front_projection=dict(projection), verified_prefront_stop=False)
        if (stopped and now-self.stop_since >= 1. and source-self.stop_first_source >= 1000000000
                and len(self.stop_distinct_sources) >= 2):
            proof.update(verified_prefront_stop=True, verified_monotonic=now)
            self.settled_stop = proof
            self.complete, self.stage = True, 'fixture_complete'
            return self._decision(None, status='fixture_complete', motion_stage_captured=True,
                motion_summary=self.completed_motion, probe_motion_summary=self.completed_motion,
                probe_completed_monotonic=self.probe_completed_at, red_settle_stop=proof,
                trigger=self.trigger, crossed_front=self.crossed)
        return self._decision('RED', status='await_stop_red', motion_stage_captured=True,
            motion_summary=self.completed_motion, probe_motion_summary=self.completed_motion,
            probe_completed_monotonic=self.probe_completed_at, red_settle_stop=proof,
            projection=projection, speed_mps=speed, trigger=self.trigger)

    def step(self, latest, now, ros_ns):
        if self.last_clock is not None and now < self.last_clock: raise FixtureError('Monotonic clock regressed')
        self.last_clock = now
        if self.complete:
            return self._decision(None, status='fixture_complete', motion_stage_captured=True,
                                  motion_summary=self.completed_motion, probe_motion_summary=self.completed_motion,
                                  probe_completed_monotonic=self.probe_completed_at,
                                  red_settle_stop=self.settled_stop, trigger=self.trigger)
        fresh, reason = fresh_inputs(latest, now, ros_ns)
        route = latest.get('route')
        route_fresh = route is not None and 0 <= now-route['receipt_monotonic'] < 1.
        if not fresh or not route_fresh:
            if self.armed: raise FixtureError('Active fixture input missing/stale: '+(reason if not fresh else 'route'))
            self.hold_since = None
            return self._decision(None, status='waiting_inputs', waiting_reason=reason if not fresh else 'route_missing_or_stale')
        if not self.armed and route['value'] != self.config.group_id:
            self.hold_since = None
            return self._decision(None, status='waiting_route', active_route_group_id=route['value'])
        speed = math.sqrt(sum(finite(value)**2 for value in latest['odom']['velocity_xyz']))
        projection = front_projection(latest['odom'], self.group, self.config.front_offset_m,
                                      self.segment_index, self.approach_sign)
        distance = projection['forward_distance_m']
        if not self.armed:
            self.stage = 'arming_red'
            if latest['gear']['value'] != 4:
                self.hold_since = None
                return self._decision('RED', status='waiting_D4', projection=projection)
            if self.config.mode == 'stopped-yellow':
                ready = projection['forward_ray_hits_finite_segment'] and distance is not None and .2 <= distance <= 2. and speed < .1
                self.hold_since = (now if self.hold_since is None else self.hold_since) if ready else None
                if self.hold_since is None or now-self.hold_since < 1.:
                    if distance is not None and distance < .2: raise FixtureNotApplicable('Stopped-Y front window already passed')
                    return self._decision('RED', status='waiting_stopped_hold', projection=projection, speed_mps=speed)
            else:
                if distance is None or distance < 8.: raise FixtureNotApplicable('Moving fixture initial8..25m window missed')
                if not projection['forward_ray_hits_finite_segment'] or distance > 25. or speed >= .1:
                    return self._decision('RED', status='waiting_initial_stopped8_25m', projection=projection, speed_mps=speed)
            self.armed, self.segment_index, self.approach_sign = True, projection['segment_index'], projection['approach_sign']
            self.previous = (now, projection['front_xy_m'], latest['odom']['source_stamp_ns'])
            if self.config.mode == 'stopped-yellow': self._begin_probe(now, projection, speed)
            else: self.stage = 'fixture_GO_approach'
        if latest['gear']['value'] != 4: raise FixtureError('Active fixture lost D4 gear')
        if self.previous:
            previous_time, previous_point, previous_source = self.previous
            source = latest['odom']['source_stamp_ns']
            if source < previous_source: raise FixtureError('Odometry source time regressed')
            if now-previous_time > .5 or math.dist(previous_point, projection['front_xy_m']) > 20:
                raise FixtureError('Unverified ego interval or position jump')
            if source != previous_source:
                crossing = segment_crossing(previous_point, projection['front_xy_m'],
                                            projection['segment_start_xy'], projection['segment_end_xy'])
                self.crossed = self.crossed or crossing is not None
                self.previous = (now, projection['front_xy_m'], source)
        needs_prefront_stop = self.config.mode in ('moving-yellow-stoppable', 'loss-prefront')
        if needs_prefront_stop and self.stage in ('manual_fixture_probe', 'await_stop_red') and (
                self.crossed or projection['signed_distance_to_line_m'] <= 0):
            raise FixtureError('Front crossed before verified prefront stop; do not qualify this fixture')
        if self.stage == 'await_stop_red':
            return self._await_stopped_before_line(latest, now, projection, speed)
        if self.stage == 'fixture_GO_approach':
            post = self.config.mode in ('loss-postfront', 'postfront-red')
            if post:
                ready = self.crossed and projection['signed_distance_to_line_m'] <= 0
            else:
                ready = (projection['forward_ray_hits_finite_segment'] and distance is not None
                    and self.config.trigger_min_m <= distance <= self.config.trigger_max_m
                    and self.config.trigger_min_speed_mps <= speed <= self.config.trigger_max_speed_mps)
                if self.crossed or distance is not None and distance < self.config.trigger_min_m:
                    raise FixtureNotApplicable('Trigger distance/speed window missed; no forced yellow/loss issued')
            if ready: self._begin_probe(now, projection, speed)
            else: return self._decision('GO', status='approach', projection=projection, speed_mps=speed)
        self._sample_motion(latest, now, projection, speed)
        elapsed = now-self.started_at
        if elapsed >= self.config.probe_seconds:
            self.completed_motion = self._motion_summary(now)
            self.probe_completed_at = now
            if needs_prefront_stop:
                self.stage, self.settle_started_at = 'await_stop_red', now
                return self._await_stopped_before_line(latest, now, projection, speed)
            self.complete, self.stage = True, 'fixture_complete'
            return self._decision(None, status='fixture_complete', motion_stage_captured=True,
                motion_summary=self.completed_motion, trigger=self.trigger, crossed_front=self.crossed,
                probe_elapsed_sec=elapsed)
        phase = ('YELLOW' if 'yellow' in self.config.mode else 'RED' if self.config.mode == 'postfront-red' else None)
        return self._decision(phase, status='probe_running', projection=projection, speed_mps=speed,
                              trigger=self.trigger, crossed_front=self.crossed, probe_elapsed_sec=elapsed)

    def _begin_probe(self, now, projection, speed):
        if self.config.mode.startswith('moving-yellow') and not (
                projection['forward_ray_hits_finite_segment'] and projection['forward_distance_m'] is not None
                and self.config.trigger_min_m <= projection['forward_distance_m'] <= self.config.trigger_max_m
                and self.config.trigger_min_speed_mps <= speed <= self.config.trigger_max_speed_mps):
            raise FixtureError('Yellow requested outside its explicit movement gate')
        self.stage, self.started_at = 'manual_fixture_probe', now
        self.trigger = dict(monotonic=now, projection=dict(projection), speed_mps=speed,
                            fixture_phase_not_native_GT=True)

    def finish_on_signal(self, signum):
        if signum != signal.SIGTERM or not self.complete:
            raise FixtureError('Owner stop before fixture completion; partial, not a behavior result')
        return True


def run(args):
    import rospy
    import rosgraph
    from roslib.message import get_message_class
    from std_msgs.msg import String
    from autoware_perception_msgs.msg import TrafficLightElement, TrafficLightGroup, TrafficLightGroupArray

    groups, geometry = load_geometry(args.geometry)
    group_id = parse_group(args.group_id)
    group = next(row for row in groups if row['group_id'] == group_id)
    config = FixtureConfig(args.mode, group_id, args.probe_seconds, args.front_axle_offset_m,
        args.trigger_min_distance_m, args.trigger_max_distance_m, args.trigger_min_speed_mps, args.trigger_max_speed_mps)
    if not 0 < finite(args.max_seconds) <= 600: raise ValueError('Bounded max_seconds0..600 required')
    output = args.output.expanduser().resolve()
    output.relative_to(ROOT/'artifacts/evaluation')
    if output.exists(): raise FileExistsError('Fresh planner-only evidence directory required')
    publishers = dict(rosgraph.Master('/planner_fixture_preflight').getSystemState()[0])
    if publishers.get(SIGNAL_TOPIC) or publishers.get(STATUS_TOPIC): raise FixtureError('Another fixture publisher exists')
    output.mkdir(parents=True)
    metadata = dict(planner_only_fixture=True, perception_accuracy_certified=False, official_lamp_compliance_certified=False,
        fixture_config=config.__dict__, geometry=geometry, signal_topic=SIGNAL_TOPIC, status_topic=STATUS_TOPIC,
        post_probe_RED_stop_proof_required=config.mode in ('moving-yellow-stoppable', 'loss-prefront'),
        post_probe_RED_settle_max_seconds=20., post_probe_distinct_stop_hold_seconds=1.,
        owner_must_hold_graph_lock=True, exclusive_relay_no_real_fallback_until_explicit_handoff=True,
        requires_real_planner_controller_and_owner_stop_P=True, native_API_used=False, vehicle_controls_published=False,
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        limitations=['Synthetic manual fixture signals are planner inputs, NOT sensor observations or native lamp GT.',
            '2/3 second yellow durations are stress inputs, not assumed competition timings.',
            'fixture_complete means input program and motion coverage captured, not a behavior PASS or goal SUCCESS.',
            'Only owner may explicitly hand the relay back to real YOLO after handoff_real=true.',
            'No native SET/GET, no vehicle controls, no stack start or global-lock bypass.'])
    (output/'metadata.json').write_text(json.dumps(metadata, indent=2)+'\n')
    writer, stop, lock, latest = JsonlWriter(output/'events.jsonl'), threading.Event(), threading.RLock(), {}
    machine, success, end_signal = PlannerFixture(config, group), False, {'value': None}
    rospy.init_node('synthetic_planner_signal_fixture', anonymous=True, disable_signals=True)
    def interrupted(signum, _): end_signal['value'] = signum; stop.set()
    for sig in (signal.SIGINT, signal.SIGTERM): signal.signal(sig, interrupted)
    def receive(name):
        def callback(raw):
            with lock:
                if stop.is_set(): return
                mono, ros_ns = time.monotonic(), rospy.Time.now().to_nsec()
                try:
                    cls = get_message_class(raw._connection_header['type'])
                    if cls is None: raise FixtureError('ROS message type unavailable')
                    message = cls().deserialize(raw._buff)
                    source = stamp_ns(getattr(getattr(message, 'header', None), 'stamp', None))
                    row = dict(receipt_monotonic=mono, receipt_ros_ns=ros_ns, source_stamp_ns=source)
                    if name == 'odom':
                        p, q, v = message.pose.pose.position, message.pose.pose.orientation, message.twist.twist.linear
                        row.update(frame=message.header.frame_id, position_xyz=[p.x, p.y, p.z],
                                   quaternion_xyzw=[q.x, q.y, q.z, q.w], velocity_xyz=[v.x, v.y, v.z])
                    elif name in ('gear', 'route'): row['value'] = message.data
                    else: row['control_message'] = primitive(message)
                    latest[name] = row
                    writer.write(dict(event='fixture_input', input_name=name, **row))
                except Exception as error:
                    latest.pop(name, None)
                    writer.write(dict(event='fixture_input_error', input_name=name, error=str(error)))
        return callback
    subscriptions = [rospy.Subscriber(topic, rospy.AnyMsg, receive(name), queue_size=10)
        for name, topic in (('odom', '/localization/kinematic_state'), ('gear', '/vehicle/status/gear'),
                            ('ctrl', '/ctrl_cmd'), ('route', ROUTE_TOPIC))]
    signal_publisher = rospy.Publisher(SIGNAL_TOPIC, TrafficLightGroupArray, queue_size=1, latch=False)
    status_publisher = rospy.Publisher(STATUS_TOPIC, String, queue_size=1, latch=False)
    deadline, last_pub, last_status, last_phase, last_stage = time.monotonic()+args.max_seconds, -float('inf'), -float('inf'), object(), None
    def status(body):
        payload = dict(body, planner_only=True, group_id=group_id, mode=args.mode,
                       fixture_signal_topic=SIGNAL_TOPIC, monotonic=time.monotonic(), generated_ros_ns=rospy.Time.now().to_nsec())
        status_publisher.publish(String(data=json.dumps(payload, allow_nan=False)))
        writer.write(dict(event='fixture_status', **payload))
    try:
        while True:
            if stop.is_set() or rospy.is_shutdown():
                machine.finish_on_signal(end_signal['value'])
                success = True
                break
            current_inputs, now, ros_now = snapshot_with_current_clocks(
                lock, latest, time.monotonic, rospy.Time.now)
            if now >= deadline: raise FixtureError('Fixture global deadline expired')
            decision = machine.step(current_inputs, now, ros_now.to_nsec())
            stage_changed = decision['stage'] != last_stage
            if decision['publish_signal'] and (decision['manual_phase'] != last_phase or now-last_pub >= .1):
                message, group_message = TrafficLightGroupArray(), TrafficLightGroup()
                assign_fixture_stamp(message, ros_now.secs, ros_now.nsecs)
                group_message.traffic_light_group_id = group_id
                for token in decision['lamp_tokens']:
                    element = TrafficLightElement()
                    element.color = {'RED_CIRCLE': element.RED, 'YELLOW_CIRCLE': element.AMBER,
                                     'GREEN_CIRCLE': element.GREEN, 'GREEN_LEFT_ARROW': element.GREEN}[token]
                    element.shape = element.LEFT_ARROW if token == 'GREEN_LEFT_ARROW' else element.CIRCLE
                    element.status, element.confidence = element.SOLID_ON, 1.
                    group_message.elements.append(element)
                message.traffic_light_groups.append(group_message)
                signal_publisher.publish(message)
                writer.write(dict(event='manual_fixture_signal_published', generated_source_stamp_ns=ros_now.to_nsec(),
                                  monotonic=now, **decision))
                last_pub = now
            if stage_changed or now-last_status >= .1:
                status(decision)
                last_status = now
            last_phase, last_stage = decision['manual_phase'], decision['stage']
            stop.wait(.02)
    except BaseException as error:
        try: status(dict(planner_only_fixture=True, stage='not_applicable' if isinstance(error, FixtureNotApplicable) else 'error',
                         handoff_real=False, fixture_complete=machine.complete, error=type(error).__name__+':'+str(error)))
        finally: raise
    finally:
        stop.set()
        for subscription in subscriptions: subscription.unregister()
        signal_publisher.unregister(); status_publisher.unregister()
        with lock:
            (output/'summary.json').write_text(json.dumps(dict(planner_only_fixture=True, owner_stop_completed=success,
                fixture_complete=machine.complete, handoff_real=success and machine.complete,
                handoff_real_was_authorized=machine.complete,
                final_stage=machine.stage, behavior_PASS_certified=False,
                official_lamp_compliance_certified=False), indent=2)+'\n')
        writer.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--geometry', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--mode', choices=MODES, required=True)
    parser.add_argument('--group-id', required=True)
    parser.add_argument('--probe-seconds', type=float, choices=(2., 3.), default=2.)
    parser.add_argument('--max-seconds', type=float, default=180.)
    parser.add_argument('--front-axle-offset-m', type=float, default=3.)
    for option in ('trigger-min-distance-m', 'trigger-max-distance-m', 'trigger-min-speed-mps', 'trigger-max-speed-mps'):
        parser.add_argument('--'+option, type=float)
    run(parser.parse_args())


if __name__ == '__main__': main()
