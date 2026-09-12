"""Serialized route service gateway and two independent policy processes.

No LaneletRoute publisher, ClearRoute, safety overrides, or fabricated inputs.
An ambiguous RPC outcome latches a reconciliation fault, never a blind retry.
"""
import json
import math
import threading
import time
import uuid
from collections import deque
from pathlib import Path

import rospy
import yaml
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_map_msgs.msg import LaneletMapBin
from autoware_planning_msgs.msg import LaneletRoute, LaneletSegment, LaneletPrimitive, RouteState, Trajectory
from autoware_planning_msgs.srv import SetLaneletRoute, SetLaneletRouteRequest
from autoware_internal_planning_msgs.srv import SetWaypointRoute, SetWaypointRouteRequest
from autoware_planning_validator.msg import PlanningValidatorStatus
from tier4_planning_msgs.msg import RerouteAvailability
from competition_route_requester.srv import RequestRoute, RequestRouteResponse
from . import policy
from .geometry import Geometry


def stamp(value):
    return value.to_sec() if hasattr(value, 'to_sec') else value.sec + value.nanosec * 1e-9


def pose(value):
    result = Pose()
    result.position.x, result.position.y = value['x'], value['y']
    result.position.z = value.get('z', 0.0)
    yaw = math.radians(value['yaw_deg'])
    result.orientation.z, result.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return result


class Inputs:
    def __init__(self):
        cfg = Path(rospy.get_param('~config_file'))
        self.cfg = yaml.safe_load(cfg.read_text())
        self.routes = yaml.safe_load((cfg.parent / self.cfg['route_definition_file']).read_text())
        policy.validate(self.cfg, self.routes)  # MUST precede all ROS I/O.
        self.geometry = Geometry(rospy.get_param('~repo_root'), self.routes)
        self.case = self.cfg['route_operation_case']
        self.odom = self.mode = self.route = self.state = self.map = None
        self.availability = None
        self.held_since = None
        self.active = None
        self.uncertain = False
        self.route_count = 0
        self.subscribers = []
        for field, topic, cls in (
                ('odom', 'odometry_topic', Odometry), ('mode', 'operation_mode_topic', OperationModeState),
                ('route', 'route_topic', LaneletRoute), ('state', 'route_state_topic', RouteState),
                ('map', 'map_topic', LaneletMapBin)):
            self.subscribers.append(rospy.Subscriber(self.cfg[topic], cls,
                lambda msg, key=field: self.receive(key, msg), queue_size=1))
        self.subscribers.append(rospy.Subscriber(self.cfg['reroute_availability_topic'],
            RerouteAvailability, self.available, queue_size=1))

    def receive(self, field, msg):
        setattr(self, field, msg)
        if field == 'route':
            self.route_count += 1

    def available(self, msg):
        now = rospy.Time.now().to_sec()
        previous = self.availability
        self.availability = msg
        if not msg.availability or not policy.fresh(now, stamp(msg.stamp), self.cfg['reroute_availability_max_age']):
            self.held_since = None
        elif self.held_since is None or previous is None or not policy.fresh(now, stamp(previous.stamp), self.cfg['reroute_availability_max_age']):
            self.held_since = now

    def progress(self):
        msg = self.odom
        if msg is None or msg.header.frame_id != 'map' or msg.child_frame_id != 'base_link':
            return None
        if not policy.fresh(rospy.Time.now().to_sec(), stamp(msg.header.stamp), self.cfg['odometry_max_age']):
            return None
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if not all(math.isfinite(x) for x in (p.x, p.y, q.x, q.y, q.z, q.w)):
            return None
        if abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) > 0.01:
            return None
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
        return self.geometry.locate(p.x, p.y, yaw)

    def ego_speed(self):
        if self.odom is None:
            return None
        velocity = self.odom.twist.twist.linear
        values = (velocity.x, velocity.y, velocity.z)
        if not all(math.isfinite(value) for value in values):
            return None
        return math.sqrt(sum(value * value for value in values))

    def guard(self, key):
        progress = self.progress()
        if not policy.allowed(self.case, key) or progress is None or not policy.in_window(self.routes, key, *progress):
            return False, 'case, fresh odometry, or request window not satisfied'
        now = rospy.Time.now().to_sec()
        if self.map is None or not self.map.data or self.state is None or self.mode is None:
            return False, 'map, route state, or operation mode unavailable'
        if bytes(self.map.data[:64]).lstrip().startswith(b'<'):
            return False, 'raw OSM payload is not the planning binary map contract'
        if not policy.fresh(now, stamp(self.mode.stamp), self.cfg['operation_mode_max_age']):
            return False, 'stale operation mode'
        if self.state.state == RouteState.UNSET:
            if key == 'route_a':
                start = self.routes['start']
                p = self.odom.pose.pose.position
                if math.hypot(p.x-start['map_pose']['x'], p.y-start['map_pose']['y']) > start['expected_pose_tolerance_m']:
                    return False, 'outside official start tolerance'
            elif key != 'route_b' or self.case != 'route_b_only' or progress[0] != 200178:
                return False, 'invalid initial route'
            return True, 'initial route ready'
        if key == 'route_a':
            if (self.active not in ('route_a', 'route_b', 'fallback_route') or
                    self.state.state != RouteState.SET or not self.matches(self.active)):
                return False, 'only a route verified by this gateway can be replaced with Route A'
            start = self.routes['start']
            p = self.odom.pose.pose.position
            if math.hypot(p.x-start['map_pose']['x'], p.y-start['map_pose']['y']) > start['expected_pose_tolerance_m']:
                return False, 'Route A restart requires the official start tolerance'
            speed = self.ego_speed()
            if speed is None or speed > self.cfg['route_a_refresh_max_speed_mps']:
                return False, 'Route A restart requires a stopped vehicle'
            return True, 'Route A restart ready at the stopped official start'
        if self.state.state != RouteState.SET or not self.matches('route_a'):
            return False, 'current route is not verified Route A; reconcile before rerouting'
        a = self.availability
        ready = (self.mode.mode == OperationModeState.AUTONOMOUS and self.mode.is_autoware_control_enabled and
                 not self.mode.is_in_transition and
                 a is not None and a.availability and
                 policy.fresh(now, stamp(a.stamp), self.cfg['reroute_availability_max_age']) and
                 self.held_since is not None and now-self.held_since >= self.cfg['reroute_available_hold_time'])
        return bool(ready), 'autonomous mode and held reroute availability required'

    def matches(self, key):
        r = self.route
        if r is None or r.header.frame_id != 'map':
            return False
        return policy.route_matches(self.routes, key, str(uuid.UUID(bytes=bytes(r.uuid.uuid))),
            [s.preferred_primitive.id for s in r.segments],
            (r.goal_pose.position.x, r.goal_pose.position.y), self.cfg['goal_tolerance_m'])


class Gateway(Inputs):
    def __init__(self):
        super().__init__()
        self.lock = threading.Lock()
        self.busy = False
        self.pending_fallback = False
        self.generation = 0
        self.trajectory = self.validation = None
        self.traj_count = self.validation_count = 0
        self.trajectory_samples = deque(maxlen=100)
        self.validation_samples = deque(maxlen=100)
        self.publisher = rospy.Publisher('/competition_route_requester/request_status', String, queue_size=10, latch=True)
        self.subscribers.extend([
            rospy.Subscriber(self.cfg['trajectory_topic'], Trajectory, self.on_trajectory, queue_size=1),
            rospy.Subscriber(self.cfg['validation_topic'], PlanningValidatorStatus, self.on_validation, queue_size=10)])
        self.service = rospy.Service('/competition_route_requester/request', RequestRoute, self.enqueue)
        self.report('READY', '', 'gateway ready; no route adopted from a previous process')

    def on_trajectory(self, msg):
        self.trajectory = msg
        self.traj_count += 1
        self.trajectory_samples.append((self.traj_count, msg))

    def on_validation(self, msg):
        self.validation = msg
        self.validation_count += 1
        self.validation_samples.append((self.validation_count, msg))

    def validated_pairs_after(self, trajectory_count, validation_count):
        """Pair each buffered trajectory with the first later validation stamp once.

        ROS1 subscriber callbacks are independent.  Looking only at both latest
        messages can permanently observe trajectory[N+1] beside validation[N]
        even though every validation is valid.  Bounded queues preserve the
        frozen post-route stamp and freshness checks without accepting a sample
        twice or weakening any validator flag.
        """
        pairs = []
        used_trajectory_count = trajectory_count
        used_validation_count = validation_count
        trajectories = list(self.trajectory_samples)
        validations = list(self.validation_samples)
        for next_trajectory_count, trajectory in trajectories:
            if next_trajectory_count <= used_trajectory_count:
                continue
            trajectory_stamp = stamp(trajectory.header.stamp)
            match = next((item for item in validations
                if item[0] > used_validation_count and stamp(item[1].stamp) >= trajectory_stamp), None)
            if match is None:
                break
            next_validation_count, validation = match
            pairs.append((next_trajectory_count, next_validation_count, trajectory, validation))
            used_trajectory_count = next_trajectory_count
            used_validation_count = next_validation_count
        return pairs

    def report(self, phase, key, detail):
        self.last_report = (phase, key, detail)
        self.publisher.publish(String(data=json.dumps(dict(phase=phase, route_key=key, detail=detail,
            generation=self.generation, verified_active=self.active, uncertain=self.uncertain,
            busy=self.busy, stamp=rospy.Time.now().to_sec()))))
        rospy.loginfo('%s %s: %s', phase, key, detail)

    def enqueue(self, request):
        key = request.route_key
        with self.lock:
            if self.uncertain:
                return RequestRouteResponse(False, 'reconciliation required; no blind retry')
            if not policy.allowed(self.case, key):
                return RequestRouteResponse(False, 'route not allowed by frozen case')
            if self.busy:
                if key == 'fallback_route':
                    self.pending_fallback = True
                return RequestRouteResponse(False, 'request already in flight')
            restart_a = key == 'route_a' and self.active in ('route_a', 'route_b', 'fallback_route')
            if ((self.active in ('route_b', 'fallback_route') and not restart_a) or
                    (key == self.active and not restart_a)):
                return RequestRouteResponse(False, 'already verified; do not overwrite')
            if self.pending_fallback and key != 'fallback_route' and not restart_a:
                return RequestRouteResponse(False, 'fallback has priority')
            ready, reason = self.guard(key)
            if not ready:
                return RequestRouteResponse(False, reason)
            if restart_a:
                # A deliberate stopped-at-start Route A request starts a new run.
                # Do not let a fallback queued by the previous run overwrite it.
                self.pending_fallback = False
            self.busy = True
            self.generation += 1
            threading.Thread(target=self.run_request, args=(key,), daemon=True).start()
            return RequestRouteResponse(True, 'queued')

    def build_request(self, key):
        definition = self.routes[key]
        request = SetLaneletRouteRequest() if key == 'fallback_route' else SetWaypointRouteRequest()
        request.header.frame_id = 'map'
        request.header.stamp = rospy.Time.now()
        request.goal_pose = pose(self.routes['poses'][definition['goal_ref']])
        request.uuid.uuid = list(uuid.UUID(definition['uuid']).bytes)
        request.allow_modification = definition['allow_modification']
        if key == 'fallback_route':
            for lane in definition['expected_path_lanelet_ids']:
                primitive = LaneletPrimitive(id=lane, primitive_type='lane')
                request.segments.append(LaneletSegment(preferred_primitive=primitive, primitives=[primitive]))
        else:
            request.waypoints = [pose(self.routes['poses'][ref]) for ref in definition['waypoint_refs']]
        return request

    def fault(self, key, reason):
        self.uncertain = True
        self.report('RECONCILIATION_REQUIRED', key, reason)

    def run_request(self, key):
        dispatched = False
        try:
            srv_type = SetLaneletRoute if key == 'fallback_route' else SetWaypointRoute
            name = self.cfg['set_lanelet_route_service_name' if key == 'fallback_route' else 'set_waypoint_route_service_name']
            rospy.wait_for_service(name, timeout=self.cfg['service_timeout'])
            ready, reason = self.guard(key)
            if not ready:
                self.report('REJECTED', key, reason)
                return
            request = self.build_request(key)
            baseline = self.route_count
            result, done = {}, threading.Event()
            def invoke():
                try:
                    result['response'] = rospy.ServiceProxy(name, srv_type)(request)
                except Exception as error:
                    result['error'] = str(error)
                finally:
                    done.set()
            dispatched = True
            self.report('DISPATCHED', key, 'main RouteSelector service only')
            threading.Thread(target=invoke, daemon=True).start()
            if not done.wait(self.cfg['service_timeout']):
                self.fault(key, 'RPC timeout: server may still finish; automatic retry disabled')
                return
            if 'error' in result:
                self.fault(key, 'RPC transport failure: ' + result['error'])
                return
            response = result['response']
            if not response.status.success:
                self.report('REJECTED', key, str(response.status.message))
                return
            deadline = time.monotonic() + self.cfg['route_verification_timeout']
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                route_is_new = (self.route is not None and
                                stamp(self.route.header.stamp) >= stamp(request.header.stamp))
                if self.route_count > baseline and route_is_new and self.matches(key):
                    break
                time.sleep(0.02)
            else:
                self.fault(key, 'service accepted but route UUID/goal/sequence not confirmed')
                return
            # Only outputs after the confirmed route/response fence count.
            fence = rospy.Time.now().to_sec()
            tc, vc, good = self.traj_count, self.validation_count, 0
            deadline = time.monotonic() + self.cfg['trajectory_verification_timeout']
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                if not self.matches(key):
                    self.fault(key, 'route changed during verification')
                    return
                if self.progress() is None:
                    self.fault(key, 'fresh valid odometry lost during verification')
                    return
                if key != 'route_a':
                    progress = self.progress()
                    if progress is None or not policy.in_window(self.routes, key, *progress):
                        self.fault(key, 'verification lost odometry or passed route deadline')
                        return
                for next_tc, next_vc, trajectory, status in self.validated_pairs_after(tc, vc):
                    ts, vs = stamp(trajectory.header.stamp), stamp(status.stamp)
                    tc, vc = next_tc, next_vc
                    if vs >= ts and ts >= fence:
                        flags = [getattr(status, field) for field in status.__slots__ if field.startswith('is_valid_')]
                        finite = all(math.isfinite(v) for p in trajectory.points for v in
                            (p.pose.position.x, p.pose.position.y, p.longitudinal_velocity_mps))
                        valid = (len(flags) == 20 and all(flags) and finite and len(trajectory.points) >= 2 and
                                 trajectory.header.frame_id == 'map' and
                                 policy.fresh(rospy.Time.now().to_sec(), ts, self.cfg['trajectory_max_age']))
                        good = good + 1 if valid else 0
                        if good >= 3:
                            self.active = key
                            self.report('VERIFIED', key, 'route identity and three fresh validated outputs; not a closed-loop proof')
                            return
                    else:
                        good = 0
                time.sleep(0.02)
            self.fault(key, 'new trajectory/validator evidence not confirmed before timeout')
        except Exception as error:
            if dispatched:
                self.fault(key, str(error))
            else:
                self.report('REJECTED', key, str(error))
        finally:
            with self.lock:
                self.busy = False
            self.report(*self.last_report)


class Supervisor(Inputs):
    def __init__(self, finish_guard=False):
        super().__init__()
        self.finish_guard = finish_guard
        self.last_attempt = 0
        self.attempts = {}
        self.client = rospy.ServiceProxy('/competition_route_requester/request', RequestRoute)
        self.subscribers.append(rospy.Subscriber('/competition_route_requester/request_status', String, self.on_status, queue_size=10))
        self.timer = rospy.Timer(rospy.Duration(0.1), self.tick)

    def on_status(self, msg):
        value = json.loads(msg.data)
        self.active, self.uncertain = value['verified_active'], value['uncertain']

    def tick(self, _event):
        if self.uncertain:
            return
        progress = self.progress()
        if progress is None:
            return
        if self.finish_guard:
            key = ('fallback_route' if self.case in policy.FALLBACK and self.active == 'route_a' and
                   policy.in_window(self.routes, 'fallback_route', *progress) else None)
        else:
            if not self.active and not self.cfg['auto_request_initial_route']:
                return
            key = policy.next_normal(self.case, self.active, *progress, self.routes)
        if key is None:
            return
        initial = self.active is None
        limit = self.cfg['initial_route_max_attempts' if initial else 'reroute_max_attempts']
        interval = self.cfg['retry_initial_interval' if initial else 'retry_reroute_interval']
        if limit and self.attempts.get(key, 0) >= limit or time.monotonic()-self.last_attempt < interval:
            return
        ready, _reason = self.guard(key)
        if not ready:
            return
        self.last_attempt = time.monotonic()
        try:
            response = self.client(key)
            if response.accepted:
                self.attempts[key] = self.attempts.get(key, 0) + 1
        except rospy.ServiceException as error:
            rospy.logwarn_throttle(5, 'route gateway unavailable: %s', error)
