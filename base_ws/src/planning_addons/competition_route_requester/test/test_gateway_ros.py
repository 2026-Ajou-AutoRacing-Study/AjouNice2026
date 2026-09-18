#!/usr/bin/env python3
"""Real ROS transport with mock RouteSelector, NOT a planner/closed-loop test.

Run ONLY through planning_ws/tools/test_requester.sh (private master).
"""
import io
import math
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest import mock
import uuid
import yaml
import rospy
from nav_msgs.msg import Odometry
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_map_msgs.msg import LaneletMapBin
from autoware_planning_msgs.msg import LaneletRoute, LaneletSegment, LaneletPrimitive, RouteState, Trajectory, TrajectoryPoint
from autoware_internal_planning_msgs.srv import SetWaypointRoute, SetWaypointRouteResponse
from autoware_planning_msgs.srv import (SetLaneletRoute, SetLaneletRouteResponse,
                                        SetPreferredPrimitive, SetPreferredPrimitiveResponse)
from tier4_planning_msgs.msg import RerouteAvailability
from autoware_planning_validator.msg import PlanningValidatorStatus
from competition_route_requester.srv import RequestRoute
from competition_route_requester.runtime import Gateway, pose

PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[3]


class GatewayROS(unittest.TestCase):
    def setUp(self):
        self.cfg = yaml.safe_load((PACKAGE/'config/competition_route_requester.param.yaml').read_text())
        self.routes = yaml.safe_load((PACKAGE/'config/kcity_v81_competition_route.yaml').read_text())
        self.cfg.update(route_definition_file=str(PACKAGE/'config/kcity_v81_competition_route.yaml'),
                        service_timeout=.2, route_verification_timeout=.4, trajectory_verification_timeout=.5)
        if self._testMethodName in ('test_b_trigger_transport', 'test_fallback_transport',
                                    'test_manual_a_restart_from_b',
                                    'test_manual_a_restart_from_fallback'):
            self.cfg['route_operation_case'] = 'route_a_to_b_with_fallback'
        self.tmp = tempfile.TemporaryDirectory(prefix='requester-config-')
        path = Path(self.tmp.name)/'config.yaml'
        path.write_text(yaml.safe_dump(self.cfg))
        rospy.set_param('~config_file', str(path))
        rospy.set_param('~repo_root', str(REPO))
        self.gateway = Gateway()
        self.route = None
        self.ego_pose = self.routes['start']['map_pose']
        self.ego_speed = 0.0
        self.behavior = 'good'
        self.calls = 0
        self.preferred_calls = 0
        self.response_done = threading.Event()
        self.service = rospy.Service(self.cfg['set_waypoint_route_service_name'], SetWaypointRoute, self.serve)
        self.lane_service = rospy.Service(self.cfg['set_lanelet_route_service_name'], SetLaneletRoute,
                                         lambda req: self.serve(req, True))
        self.preferred_service = rospy.Service(self.cfg['set_preferred_primitive_service_name'],
                                               SetPreferredPrimitive, self.serve_preferred)
        self.pubs = {}
        for field, cls in [('odometry_topic', Odometry), ('operation_mode_topic', OperationModeState),
                           ('map_topic', LaneletMapBin), ('route_state_topic', RouteState),
                           ('route_topic', LaneletRoute), ('trajectory_topic', Trajectory),
                           ('validation_topic', PlanningValidatorStatus), ('reroute_availability_topic', RerouteAvailability)]:
            self.pubs[field] = rospy.Publisher(self.cfg[field], cls, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(.03), self.publish)
        self.client = rospy.ServiceProxy('/competition_route_requester/request', RequestRoute)
        self.wait(lambda: self.gateway.guard('route_a')[0], 3)

    def tearDown(self):
        deadline = time.monotonic()+2
        while self.gateway.busy and time.monotonic()<deadline:
            time.sleep(.02)
        if self.calls:
            self.response_done.wait(1)
        self.timer.shutdown()
        self.service.shutdown()
        self.lane_service.shutdown()
        self.preferred_service.shutdown()
        self.gateway.service.shutdown()
        for sub in self.gateway.subscribers:
            sub.unregister()
        self.gateway.publisher.unregister()
        for pub in self.pubs.values():
            pub.unregister()
        self.tmp.cleanup()

    def wait(self, predicate, timeout=2):
        end = time.monotonic()+timeout
        while time.monotonic()<end:
            if predicate():
                return
            time.sleep(.01)
        self.fail('condition timed out')

    def publish(self, _event):
        now = rospy.Time.now()
        odom = Odometry()
        odom.header.frame_id, odom.child_frame_id = 'map', 'base_link'
        odom.header.stamp = now
        odom.pose.pose = pose(self.ego_pose)
        odom.twist.twist.linear.x = self.ego_speed
        mode = OperationModeState()
        mode.stamp.sec, mode.stamp.nanosec = now.secs, now.nsecs
        mode.mode = mode.AUTONOMOUS
        mode.is_autoware_control_enabled = True
        state = RouteState(state=RouteState.SET if self.route else RouteState.UNSET)
        self.pubs['odometry_topic'].publish(odom)
        self.pubs['operation_mode_topic'].publish(mode)
        self.pubs['route_state_topic'].publish(state)
        self.pubs['map_topic'].publish(LaneletMapBin(data=b'mock binary; not a map-loader test'))
        availability = RerouteAvailability(availability=True)
        availability.stamp.sec, availability.stamp.nanosec = now.secs, now.nsecs
        self.pubs['reroute_availability_topic'].publish(availability)
        if self.route:
            self.pubs['route_topic'].publish(self.route)
        trajectory = Trajectory()
        trajectory.header.frame_id = 'map'
        trajectory.header.stamp = now-rospy.Duration(10) if self.behavior == 'stale' else now
        point = TrajectoryPoint()
        point.pose.orientation.w = 1
        trajectory.points = [point, point]
        status = PlanningValidatorStatus()
        status.stamp = now
        for field in status.__slots__:
            if field.startswith('is_valid_'):
                setattr(status, field, True)
        if self.behavior == 'invalid':
            status.is_valid_latency = False
        self.pubs['trajectory_topic'].publish(trajectory)
        self.pubs['validation_topic'].publish(status)

    def serve(self, req, lane=False):
        self.calls += 1
        # Actual genpy serialization, including nested builtin Time/Duration.
        req.serialize(io.BytesIO())
        response = SetLaneletRouteResponse() if lane else SetWaypointRouteResponse()
        if self.behavior == 'reject':
            response.status.success = False
            response.status.message = 'mock safety rejection'
        else:
            if self.behavior == 'timeout':
                time.sleep(.5)
            key = next(k for k in ('route_a', 'route_b', 'fallback_route')
                       if self.routes[k]['uuid'] == str(uuid.UUID(bytes=bytes(req.uuid.uuid))))
            route = LaneletRoute()
            route.header = req.header
            route.uuid = req.uuid
            if self.behavior == 'wrong_uuid':
                route.uuid.uuid = [0]*16
            route.goal_pose = req.goal_pose
            final = self.routes[key]['expected_path_lanelet_ids']
            initial = self.routes[key].get('waypoint_path_lanelet_ids', final)
            route.segments = []
            for source, target in zip(initial, final):
                preferred = LaneletPrimitive(id=source, primitive_type='lane')
                primitives = [preferred]
                if target != source:
                    primitives.append(LaneletPrimitive(id=target, primitive_type='lane'))
                route.segments.append(LaneletSegment(preferred_primitive=preferred, primitives=primitives))
            self.route = route
            response.status.success = True
        self.response_done.set()
        return response

    def serve_preferred(self, req):
        self.preferred_calls += 1
        req.serialize(io.BytesIO())
        response = SetPreferredPrimitiveResponse()
        if self.route is None or req.uuid != self.route.uuid:
            response.status.success = False
            response.status.message = 'mock route mismatch'
            return response
        if len(req.preferred_primitives) != len(self.route.segments):
            response.status.success = False
            response.status.message = 'mock segment count mismatch'
            return response
        for segment, preferred in zip(self.route.segments, req.preferred_primitives):
            if preferred.id not in [primitive.id for primitive in segment.primitives]:
                response.status.success = False
                response.status.message = 'mock primitive mismatch'
                return response
            segment.preferred_primitive = preferred
        self.route.header.stamp = rospy.Time.now()
        response.status.success = True
        return response

    def test_a_service_and_fresh_output(self):
        self.assertTrue(self.client('route_a').accepted)
        self.wait(lambda: not self.gateway.busy)
        self.assertEqual(self.gateway.active, 'route_a')
        self.assertFalse(self.gateway.uncertain)
        self.assertTrue(self.client('route_a').accepted)
        self.wait(lambda: not self.gateway.busy)
        self.assertEqual(self.gateway.active, 'route_a')
        self.assertEqual(self.gateway.generation, 2)
        self.assertEqual(self.calls, 2)
        self.assertEqual(self.preferred_calls, 2)
        self.assertEqual([segment.preferred_primitive.id for segment in self.route.segments],
                         self.routes['route_a']['expected_path_lanelet_ids'])

    def test_a_refresh_rejected_while_moving(self):
        self.assertTrue(self.client('route_a').accepted)
        self.wait(lambda: not self.gateway.busy)
        self.ego_speed = self.cfg['route_a_refresh_max_speed_mps'] + 0.01
        self.wait(lambda: not self.gateway.guard('route_a')[0])
        response = self.client('route_a')
        self.assertFalse(response.accepted)
        self.assertIn('stopped vehicle', response.detail)
        self.assertEqual(self.calls, 1)

    def test_a_refresh_rejected_outside_official_start(self):
        self.assertTrue(self.client('route_a').accepted)
        self.wait(lambda: not self.gateway.busy)
        self.ego_pose = self.routes['poses']['wp_200178']
        self.wait(lambda: not self.gateway.guard('route_a')[0])
        self.assertFalse(self.client('route_a').accepted)
        self.assertEqual(self.calls, 1)

    def test_rejection_is_not_completion(self):
        self.behavior = 'reject'
        self.assertTrue(self.client('route_a').accepted)
        self.wait(lambda: not self.gateway.busy)
        self.assertIsNone(self.gateway.active)
        self.assertFalse(self.gateway.uncertain)

    def test_startup_replaces_an_existing_unverified_route(self):
        old = LaneletRoute()
        old.header.frame_id = 'map'
        old.header.stamp = rospy.Time.now()
        old.uuid.uuid = [0] * 16
        self.route = old
        self.wait(lambda: self.gateway.state is not None and
                  self.gateway.state.state == RouteState.SET)
        self.assertIsNone(self.gateway.active)
        response = self.client('route_a')
        self.assertTrue(response.accepted, response.detail)
        self.wait(lambda: not self.gateway.busy)
        self.assertEqual(self.gateway.active, 'route_a')
        self.assertEqual(self.calls, 1)

    def test_startup_selects_route_b_after_first_trigger(self):
        self.ego_pose = self.routes['poses']['wp_200258']
        self.wait(lambda: self.gateway.guard('route_b')[0])
        response = self.client('route_b')
        self.assertTrue(response.accepted, response.detail)
        self.wait(lambda: not self.gateway.busy)
        self.assertEqual(self.gateway.active, 'route_b')

    def test_dispatch_guard_recovers_transient_input_gap(self):
        checks = iter([
            (False, 'fresh valid odometry unavailable'),
            (False, 'stale operation mode'),
            (True, 'startup route_a ready'),
        ])
        with mock.patch.object(self.gateway, 'guard', side_effect=lambda _key: next(checks)):
            self.assertEqual(self.gateway.wait_for_dispatch_guard('route_a'),
                             (True, 'startup route_a ready'))

    def test_timeout_disables_retry_and_concurrency(self):
        self.behavior = 'timeout'
        self.assertTrue(self.client('route_a').accepted)
        self.assertFalse(self.client('route_a').accepted)
        self.wait(lambda: self.gateway.uncertain)
        self.assertFalse(self.client('route_a').accepted)
        self.assertIsNone(self.gateway.active)
        self.assertEqual(self.calls, 1)

    def test_wrong_uuid_not_verified(self):
        self.behavior = 'wrong_uuid'
        self.assertTrue(self.client('route_a').accepted)
        self.wait(lambda: self.gateway.uncertain)
        self.assertIsNone(self.gateway.active)
        self.assertIn('route UUID/goal/sequence', self.gateway.last_report[2])

    def test_stale_output_not_verified(self):
        self.behavior = 'stale'
        self.assertTrue(self.client('route_a').accepted)
        self.wait(lambda: self.gateway.uncertain)
        self.assertIsNone(self.gateway.active)

    def test_invalid_validator_not_verified(self):
        self.behavior = 'invalid'
        self.assertTrue(self.client('route_a').accepted)
        self.wait(lambda: self.gateway.uncertain)
        self.assertIsNone(self.gateway.active)

    def test_buffered_trajectory_validation_pairing_is_ordered_and_single_use(self):
        baseline_trajectory = self.gateway.traj_count
        baseline_validation = self.gateway.validation_count
        first_stamp = rospy.Time.now()
        for offset in (0.0, 0.01, 0.02):
            trajectory = Trajectory()
            trajectory.header.stamp = first_stamp + rospy.Duration(offset)
            status = PlanningValidatorStatus()
            status.stamp = first_stamp + rospy.Duration(offset + 0.005)
            self.gateway.on_trajectory(trajectory)
            self.gateway.on_validation(status)
        pairs = self.gateway.validated_pairs_after(baseline_trajectory, baseline_validation)
        self.assertEqual(len(pairs), 3)
        self.assertEqual([item[0] for item in pairs], list(range(baseline_trajectory + 1, baseline_trajectory + 4)))
        self.assertEqual([item[1] for item in pairs], list(range(baseline_validation + 1, baseline_validation + 4)))
        self.assertEqual(self.gateway.validated_pairs_after(pairs[-1][0], pairs[-1][1]), [])

    def test_disallowed_case_has_no_rpc(self):
        self.assertFalse(self.client('route_b').accepted)
        self.assertFalse(self.client('fallback_route').accepted)
        self.assertEqual(self.calls, 0)

    def test_fallback_builder_serialization(self):
        request = self.gateway.build_request('fallback_route')
        request.serialize(io.BytesIO())
        self.assertEqual(len(request.segments), len(self.routes['fallback_route']['expected_path_lanelet_ids']))
        self.assertEqual([x.preferred_primitive.id for x in request.segments],
                         self.routes['fallback_route']['expected_path_lanelet_ids'])
        self.assertEqual(str(uuid.UUID(bytes=bytes(request.uuid.uuid))), self.routes['fallback_route']['uuid'])

    def transition(self, key, ref):
        self.assertTrue(self.client('route_a').accepted)
        self.wait(lambda: not self.gateway.busy)
        self.assertEqual(self.gateway.active, 'route_a')
        self.ego_pose = self.routes['poses'][ref]
        self.wait(lambda: self.gateway.guard(key)[0])
        self.assertTrue(self.client(key).accepted)
        self.wait(lambda: not self.gateway.busy)
        self.assertEqual(self.gateway.active, key)
        self.assertFalse(self.gateway.uncertain)
        self.assertEqual(self.calls, 2)

    def test_b_trigger_transport(self):
        # Pose advances to trigger. This checks policy/transport, not Mission safety.
        self.transition('route_b', 'wp_200258')

    def test_fallback_transport(self):
        self.transition('fallback_route', self.routes['fallback_route']['hard_cutoff_pose_ref'])

    def restart_a_from(self, key, ref):
        self.transition(key, ref)
        self.ego_pose = self.routes['start']['map_pose']
        self.ego_speed = 0.0
        self.gateway.pending_fallback = True
        self.wait(lambda: self.gateway.guard('route_a')[0])
        self.assertTrue(self.client('route_a').accepted)
        self.wait(lambda: not self.gateway.busy)
        self.assertEqual(self.gateway.active, 'route_a')
        self.assertFalse(self.gateway.pending_fallback)
        self.assertFalse(self.gateway.uncertain)
        self.assertEqual(self.gateway.generation, 3)
        self.assertEqual(self.calls, 3)

    def test_manual_a_restart_from_b(self):
        self.restart_a_from('route_b', 'wp_200258')

    def test_manual_a_restart_from_fallback(self):
        self.restart_a_from('fallback_route', self.routes['fallback_route']['hard_cutoff_pose_ref'])


if __name__ == '__main__':
    rospy.init_node('requester_transport_test')
    unittest.main(verbosity=2)
