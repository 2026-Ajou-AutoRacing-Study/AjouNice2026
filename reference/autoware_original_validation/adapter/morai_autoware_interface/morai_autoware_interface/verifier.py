"""Run a non-driving integration check against the live MORAI bridge."""

from __future__ import annotations

import math
import time

from autoware_control_msgs.msg import Control
from autoware_perception_msgs.msg import ObjectClassification
from autoware_perception_msgs.msg import PredictedObjects
from autoware_vehicle_msgs.msg import SteeringReport
from autoware_vehicle_msgs.msg import VelocityReport
from geometry_msgs.msg import AccelWithCovarianceStamped
from morai_msgs.msg import CtrlCmd
from morai_msgs.msg import EgoVehicleStatus
from morai_msgs.msg import ObjectStatusList
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy

from .conversions import heading_deg_to_yaw
from .conversions import normalize_angle


class MoraiAutowareVerifier(Node):
    """Compare live MORAI data with isolated adapter outputs."""

    TEST_DURATION_SEC = 5.0
    COMMAND_START_SEC = 1.0
    COMMAND_STOP_SEC = 2.0
    EXPECTED_VELOCITY_MPS = 2.0
    EXPECTED_ACCELERATION_MPS2 = 0.5
    EXPECTED_STEERING_RAD = 0.1

    def __init__(self) -> None:
        super().__init__('morai_autoware_verifier')
        self._started_at = time.monotonic()
        self._last_command_at = 0.0
        self._ego_count = 0
        self._velocity_count = 0
        self._passed: set[str] = set()
        self._failed_details: dict[str, str] = {}
        self._warnings: list[str] = []
        self._saw_target_control = False

        self._ego_by_stamp: dict[tuple[int, int], EgoVehicleStatus] = {}
        self._velocity_by_stamp: dict[tuple[int, int], VelocityReport] = {}
        self._steering_by_stamp: dict[tuple[int, int], SteeringReport] = {}
        self._odometry_by_stamp: dict[tuple[int, int], Odometry] = {}
        self._acceleration_by_stamp: dict[
            tuple[int, int], AccelWithCovarianceStamped
        ] = {}
        self._objects_in_by_stamp: dict[
            tuple[int, int], ObjectStatusList
        ] = {}
        self._objects_out_by_stamp: dict[
            tuple[int, int], PredictedObjects
        ] = {}

        morai_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        report_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            EgoVehicleStatus, '/Ego_topic', self._on_ego, morai_qos
        )
        self.create_subscription(
            ObjectStatusList, '/Object_topic', self._on_objects_in, morai_qos
        )
        self.create_subscription(
            VelocityReport,
            '/verification/vehicle/status/velocity_status',
            self._on_velocity,
            report_qos,
        )
        self.create_subscription(
            SteeringReport,
            '/verification/vehicle/status/steering_status',
            self._on_steering,
            report_qos,
        )
        self.create_subscription(
            Odometry,
            '/verification/localization/kinematic_state',
            self._on_odometry,
            report_qos,
        )
        self.create_subscription(
            AccelWithCovarianceStamped,
            '/verification/localization/acceleration',
            self._on_acceleration,
            report_qos,
        )
        self.create_subscription(
            PredictedObjects,
            '/verification/perception/object_recognition/objects',
            self._on_objects_out,
            report_qos,
        )
        self.create_subscription(
            CtrlCmd,
            '/verification/morai_ctrl_cmd',
            self._on_control_output,
            morai_qos,
        )
        self._control_publisher = self.create_publisher(
            Control,
            '/verification/autoware_control_cmd',
            command_qos,
        )

    @staticmethod
    def _stamp_key(stamp: object) -> tuple[int, int]:
        return (int(stamp.sec), int(stamp.nanosec))

    @staticmethod
    def _close(left: float, right: float, tolerance: float = 1.0e-5) -> bool:
        return math.isclose(left, right, rel_tol=tolerance, abs_tol=tolerance)

    @staticmethod
    def _remember(mapping: dict, key: tuple[int, int], message: object) -> None:
        mapping[key] = message
        while len(mapping) > 500:
            mapping.pop(next(iter(mapping)))

    def _observe(self, name: str, passed: bool, detail: str) -> None:
        if passed:
            self._passed.add(name)
            self._failed_details.pop(name, None)
        elif name not in self._passed:
            self._failed_details[name] = detail

    def _on_ego(self, message: EgoVehicleStatus) -> None:
        self._ego_count += 1
        key = self._stamp_key(message.header.stamp)
        self._remember(self._ego_by_stamp, key, message)
        self._match_ego_outputs(key)

    def _on_velocity(self, message: VelocityReport) -> None:
        self._velocity_count += 1
        key = self._stamp_key(message.header.stamp)
        self._remember(self._velocity_by_stamp, key, message)
        self._match_ego_outputs(key)

    def _on_steering(self, message: SteeringReport) -> None:
        key = self._stamp_key(message.stamp)
        self._remember(self._steering_by_stamp, key, message)
        self._match_ego_outputs(key)

    def _on_odometry(self, message: Odometry) -> None:
        key = self._stamp_key(message.header.stamp)
        self._remember(self._odometry_by_stamp, key, message)
        self._match_ego_outputs(key)

    def _on_acceleration(self, message: AccelWithCovarianceStamped) -> None:
        key = self._stamp_key(message.header.stamp)
        self._remember(self._acceleration_by_stamp, key, message)
        self._match_ego_outputs(key)

    def _match_ego_outputs(self, key: tuple[int, int]) -> None:
        ego = self._ego_by_stamp.get(key)
        if ego is None:
            return

        velocity = self._velocity_by_stamp.get(key)
        if velocity is not None:
            passed = self._close(
                velocity.longitudinal_velocity,
                float(ego.velocity.x),
            ) and self._close(
                velocity.lateral_velocity,
                float(ego.velocity.y),
            )
            self._observe(
                'Ego velocity m/s passthrough',
                passed,
                'converted velocity does not match the same Ego_topic stamp',
            )

        steering = self._steering_by_stamp.get(key)
        if steering is not None:
            self._observe(
                'steering report conversion',
                self._close(
                    steering.steering_tire_angle,
                    math.radians(float(ego.wheel_angle)),
                ),
                'steering report does not match MORAI wheel_angle deg to rad',
            )

        odometry = self._odometry_by_stamp.get(key)
        if odometry is not None:
            pose = odometry.pose.pose
            position_matches = all(
                (
                    self._close(pose.position.x, float(ego.position.x)),
                    self._close(pose.position.y, float(ego.position.y)),
                    self._close(pose.position.z, float(ego.position.z)),
                )
            )
            quaternion = pose.orientation
            converted_yaw = math.atan2(
                2.0 * quaternion.w * quaternion.z,
                1.0 - 2.0 * quaternion.z * quaternion.z,
            )
            expected_yaw = heading_deg_to_yaw(float(ego.heading))
            heading_matches = self._close(
                normalize_angle(converted_yaw - expected_yaw),
                0.0,
            )
            self._observe(
                'position and heading conversion',
                position_matches and heading_matches,
                'odometry position or quaternion differs from Ego_topic',
            )

        acceleration = self._acceleration_by_stamp.get(key)
        if acceleration is not None:
            linear = acceleration.accel.accel.linear
            passed = all(
                (
                    self._close(linear.x, float(ego.acceleration.x)),
                    self._close(linear.y, float(ego.acceleration.y)),
                    self._close(linear.z, float(ego.acceleration.z)),
                )
            )
            self._observe(
                'acceleration conversion',
                passed,
                'acceleration differs from the same Ego_topic stamp',
            )

    def _on_objects_in(self, message: ObjectStatusList) -> None:
        key = self._stamp_key(message.header.stamp)
        self._remember(self._objects_in_by_stamp, key, message)
        self._match_objects(key)

    def _on_objects_out(self, message: PredictedObjects) -> None:
        key = self._stamp_key(message.header.stamp)
        self._remember(self._objects_out_by_stamp, key, message)
        self._match_objects(key)

    def _match_objects(self, key: tuple[int, int]) -> None:
        incoming = self._objects_in_by_stamp.get(key)
        outgoing = self._objects_out_by_stamp.get(key)
        if incoming is None or outgoing is None:
            return
        expected_count = (
            len(incoming.npc_list)
            + len(incoming.pedestrian_list)
            + len(incoming.obstacle_list)
        )
        count_matches = len(outgoing.objects) == expected_count
        labels = (
            [ObjectClassification.CAR] * len(incoming.npc_list)
            + [ObjectClassification.PEDESTRIAN]
            * len(incoming.pedestrian_list)
            + [ObjectClassification.UNKNOWN] * len(incoming.obstacle_list)
        )
        labels_match = all(
            output.classification
            and output.classification[0].label == expected_label
            for output, expected_label in zip(outgoing.objects, labels)
        )
        shapes_valid = all(
            output.shape.dimensions.x > 0.0
            and output.shape.dimensions.y > 0.0
            and output.shape.dimensions.z > 0.0
            for output in outgoing.objects
        )
        self._observe(
            'ground-truth object conversion',
            count_matches and labels_match and shapes_valid,
            'object count, class, or dimensions do not match Object_topic',
        )
        if expected_count > 0:
            self._passed.add('non-empty object payload')

    def _on_control_output(self, message: CtrlCmd) -> None:
        safe_stop = (
            message.longl_cmd_type == 1
            and message.brake >= 0.99
            and message.accel <= 0.01
        )
        if safe_stop and self._saw_target_control:
            self._passed.add('command timeout watchdog')
        elif safe_stop:
            self._passed.add('safe startup output')

        target_matches = (
            message.longl_cmd_type == 2
            and self._close(message.velocity, 7.2)
            and self._close(
                message.acceleration,
                self.EXPECTED_ACCELERATION_MPS2,
            )
            and self._close(message.steering, self.EXPECTED_STEERING_RAD)
            and message.accel <= 0.01
            and message.brake <= 0.01
        )
        if target_matches:
            self._saw_target_control = True
            self._passed.add('isolated Autoware to MORAI command conversion')

    def publish_test_command_if_needed(self) -> None:
        elapsed = time.monotonic() - self._started_at
        should_publish = self.COMMAND_START_SEC <= elapsed < self.COMMAND_STOP_SEC
        enough_time_elapsed = time.monotonic() - self._last_command_at >= 0.05
        if not should_publish or not enough_time_elapsed:
            return
        message = Control()
        message.stamp = self.get_clock().now().to_msg()
        message.longitudinal.velocity = self.EXPECTED_VELOCITY_MPS
        message.longitudinal.acceleration = self.EXPECTED_ACCELERATION_MPS2
        message.longitudinal.is_defined_acceleration = True
        message.lateral.steering_tire_angle = self.EXPECTED_STEERING_RAD
        self._control_publisher.publish(message)
        self._last_command_at = time.monotonic()

    def finish_graph_checks(self) -> None:
        topics = dict(self.get_topic_names_and_types())
        services = dict(self.get_service_names_and_types())
        expected_topics = {
            '/Ego_topic': 'morai_msgs/msg/EgoVehicleStatus',
            '/Object_topic': 'morai_msgs/msg/ObjectStatusList',
            '/ctrl_cmd': 'morai_msgs/msg/CtrlCmd',
        }
        topic_contract_valid = all(
            expected_type in topics.get(name, [])
            for name, expected_type in expected_topics.items()
        )
        self._observe(
            'MORAI topic type contract',
            topic_contract_valid,
            'one or more MORAI topic names/types are missing',
        )
        service_contract_valid = (
            'morai_msgs/srv/MoraiEventCmdSrv'
            in services.get('/Service_MoraiEventCmd', [])
        )
        self._observe(
            'MORAI event service contract',
            service_contract_valid,
            '/Service_MoraiEventCmd is missing or has a different type',
        )

        elapsed = max(0.001, time.monotonic() - self._started_at)
        ego_rate = self._ego_count / elapsed
        velocity_rate = self._velocity_count / elapsed
        self._observe(
            'live Ego_topic rate',
            ego_rate >= 10.0,
            f'Ego_topic rate is only {ego_rate:.1f} Hz',
        )
        self._observe(
            'live converted status rate',
            velocity_rate >= 10.0,
            f'converted velocity rate is only {velocity_rate:.1f} Hz',
        )
        if 'non-empty object payload' not in self._passed:
            self._warnings.append(
                'No non-empty object list was observed; object structure was '
                'checked, but a populated scenario is needed for payload validation.'
            )

    def report(self) -> bool:
        required_checks = (
            'MORAI topic type contract',
            'MORAI event service contract',
            'live Ego_topic rate',
            'live converted status rate',
            'Ego velocity m/s passthrough',
            'steering report conversion',
            'position and heading conversion',
            'acceleration conversion',
            'ground-truth object conversion',
            'safe startup output',
            'isolated Autoware to MORAI command conversion',
            'command timeout watchdog',
        )
        print('\nMORAI–Autoware safe verification')
        print('=' * 36)
        success = True
        for name in required_checks:
            if name in self._passed:
                print(f'[PASS] {name}')
                continue
            success = False
            detail = self._failed_details.get(name, 'no matching sample received')
            print(f'[FAIL] {name}: {detail}')
        for warning in self._warnings:
            print(f'[WARN] {warning}')
        print('=' * 36)
        print('RESULT: PASS (vehicle output isolated)' if success else 'RESULT: FAIL')
        return success


def main(args: list[str] | None = None) -> int:
    """Spin the safe verifier and return a shell-friendly result code."""
    rclpy.init(args=args)
    node = MoraiAutowareVerifier()
    try:
        while time.monotonic() - node._started_at < node.TEST_DURATION_SEC:
            node.publish_test_command_if_needed()
            rclpy.spin_once(node, timeout_sec=0.02)
        node.finish_graph_checks()
        return 0 if node.report() else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
