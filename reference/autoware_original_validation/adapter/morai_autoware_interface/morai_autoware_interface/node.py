"""Translate MORAI competition messages to the Autoware vehicle contract."""

from __future__ import annotations

from functools import partial
import math
import time
from typing import Callable

from autoware_adapi_v1_msgs.msg import LocalizationInitializationState
from autoware_control_msgs.msg import Control
from autoware_perception_msgs.msg import ObjectClassification
from autoware_perception_msgs.msg import PredictedObject
from autoware_perception_msgs.msg import PredictedObjects
from autoware_perception_msgs.msg import PredictedPath
from autoware_perception_msgs.msg import Shape
from autoware_vehicle_msgs.msg import ControlModeReport
from autoware_vehicle_msgs.msg import GearCommand
from autoware_vehicle_msgs.msg import GearReport
from autoware_vehicle_msgs.msg import HazardLightsCommand
from autoware_vehicle_msgs.msg import HazardLightsReport
from autoware_vehicle_msgs.msg import SteeringReport
from autoware_vehicle_msgs.msg import TurnIndicatorsCommand
from autoware_vehicle_msgs.msg import TurnIndicatorsReport
from autoware_vehicle_msgs.msg import VelocityReport
from autoware_vehicle_msgs.srv import ControlModeCommand
from geometry_msgs.msg import AccelWithCovarianceStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from morai_msgs.msg import CtrlCmd
from morai_msgs.msg import EgoVehicleStatus
from morai_msgs.msg import ObjectStatus
from morai_msgs.msg import ObjectStatusList
from morai_msgs.srv import MoraiEventCmdSrv
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from tf2_ros import TransformBroadcaster
from tier4_vehicle_msgs.msg import ActuationStatusStamped

from .conversions import clamp
from .conversions import deterministic_uuid
from .conversions import heading_deg_to_yaw
from .conversions import kmh_to_mps
from .conversions import mps_to_kmh
from .conversions import normalize_angle
from .conversions import quaternion_from_yaw


EventSuccessCallback = Callable[[object], None]


class MoraiAutowareInterface(Node):
    """Bidirectional MORAI/Autoware vehicle and ground-truth adapter."""

    MORAI_OPTION_CONTROL_MODE = 1
    MORAI_OPTION_GEAR = 2
    MORAI_OPTION_LAMPS = 4

    MORAI_CONTROL_KEYBOARD = 1
    MORAI_CONTROL_AUTO = 3

    MORAI_GEAR_PARK = 1
    MORAI_GEAR_REVERSE = 2
    MORAI_GEAR_NEUTRAL = 3
    MORAI_GEAR_DRIVE = 4

    def __init__(self) -> None:
        super().__init__('morai_autoware_interface')
        self._read_parameters()
        self._create_qos_profiles()
        self._create_publishers()
        self._create_subscriptions()
        self._create_services()

        self._tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self._latest_control: Control | None = None
        self._latest_control_time = 0.0
        self._last_watchdog_warning = 0.0
        self._last_yaw: float | None = None
        self._last_ego_time: float | None = None
        self._latest_stamp = self.get_clock().now().to_msg()
        self._localization_initialized = False

        self._control_mode_report = self._initial_control_mode()
        self._gear_report = self._initial_gear()
        self._turn_report = TurnIndicatorsReport.DISABLE
        self._hazard_report = HazardLightsReport.DISABLE
        self._morai_control_mode = self._autoware_mode_to_morai(
            self._control_mode_report
        )
        self._morai_gear = self._autoware_gear_to_morai(self._gear_report)
        self._pending_gear_report: int | None = None
        self._last_gear_request_time = 0.0

        self._status_timer = self.create_timer(
            1.0 / self.status_publish_rate_hz,
            self._publish_discrete_status,
        )
        self._command_timer = self.create_timer(
            1.0 / self.command_publish_rate_hz,
            self._publish_control_command,
        )

        self._initialization_requested = False
        if self.set_autonomous_and_drive_on_start:
            self._initialization_timer = self.create_timer(
                1.0,
                self._request_startup_mode,
            )

        self.get_logger().info(
            'MORAI interface ready: /Ego_topic -> Autoware status, '
            '/control/command/control_cmd -> /ctrl_cmd'
        )
        if not self.set_autonomous_and_drive_on_start:
            self.get_logger().info(
                'Safe startup is enabled; MORAI control mode is not changed automatically'
            )

    def _read_parameters(self) -> None:
        self.ego_topic = self.declare_parameter(
            'morai.ego_topic', '/Ego_topic'
        ).value
        self.object_topic = self.declare_parameter(
            'morai.object_topic', '/Object_topic'
        ).value
        self.ctrl_topic = self.declare_parameter(
            'morai.ctrl_topic', '/ctrl_cmd'
        ).value
        self.event_service = self.declare_parameter(
            'morai.event_service', '/Service_MoraiEventCmd'
        ).value

        self.control_topic = self.declare_parameter(
            'autoware.control_topic', '/control/command/control_cmd'
        ).value
        self.gear_topic = self.declare_parameter(
            'autoware.gear_topic', '/control/command/gear_cmd'
        ).value
        self.turn_topic = self.declare_parameter(
            'autoware.turn_topic', '/control/command/turn_indicators_cmd'
        ).value
        self.hazard_topic = self.declare_parameter(
            'autoware.hazard_topic', '/control/command/hazard_lights_cmd'
        ).value
        self.control_mode_service = self.declare_parameter(
            'autoware.control_mode_service', '/control/control_mode_request'
        ).value

        self.map_frame = self.declare_parameter('frames.map', 'map').value
        self.base_frame = self.declare_parameter(
            'frames.base_link', 'base_link'
        ).value

        self.publish_tf = bool(self.declare_parameter('publish.tf', True).value)
        self.publish_kinematic_state = bool(
            self.declare_parameter('publish.kinematic_state', True).value
        )
        self.publish_objects = bool(
            self.declare_parameter('publish.objects', True).value
        )
        self.publish_localization_initialization_state = bool(
            self.declare_parameter(
                'publish.localization_initialization_state', True
            ).value
        )

        self.status_publish_rate_hz = float(
            self.declare_parameter('status_publish_rate_hz', 20.0).value
        )
        self.command_publish_rate_hz = float(
            self.declare_parameter('command.publish_rate_hz', 30.0).value
        )
        self.command_timeout_sec = float(
            self.declare_parameter('command.timeout_sec', 0.5).value
        )
        self.max_velocity_kmh = float(
            self.declare_parameter('command.max_velocity_kmh', 30.0).value
        )
        self.max_steering_angle = float(
            self.declare_parameter('command.max_steering_tire_angle', 0.70).value
        )
        self.steering_sign = float(
            self.declare_parameter('command.steering_sign', 1.0).value
        )
        self.steering_report_sign = float(
            self.declare_parameter('report.steering_sign', 1.0).value
        )
        self.ego_velocity_is_kmh = bool(
            self.declare_parameter('report.ego_velocity_is_kmh', False).value
        )
        self.object_velocity_is_kmh = bool(
            self.declare_parameter('report.object_velocity_is_kmh', True).value
        )
        self.morai_steering_is_degrees = bool(
            self.declare_parameter(
                'report.morai_steering_is_degrees', True
            ).value
        )
        self.initial_control_mode = str(
            self.declare_parameter('initial.control_mode', 'manual').value
        ).lower()
        self.initial_gear = str(
            self.declare_parameter('initial.gear', 'park').value
        ).lower()
        self.set_autonomous_and_drive_on_start = bool(
            self.declare_parameter(
                'initial.set_autonomous_and_drive_on_start', False
            ).value
        )

        for name, value in (
            ('status_publish_rate_hz', self.status_publish_rate_hz),
            ('command.publish_rate_hz', self.command_publish_rate_hz),
            ('command.timeout_sec', self.command_timeout_sec),
        ):
            if value <= 0.0:
                raise ValueError(f'{name} must be greater than zero')

    def _create_qos_profiles(self) -> None:
        self._morai_input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._morai_output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._command_input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._latched_status_output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

    def _create_publishers(self) -> None:
        self._ctrl_pub = self.create_publisher(
            CtrlCmd, self.ctrl_topic, self._morai_output_qos
        )
        self._velocity_pub = self.create_publisher(
            VelocityReport,
            '/vehicle/status/velocity_status',
            self._status_output_qos,
        )
        self._steering_pub = self.create_publisher(
            SteeringReport,
            '/vehicle/status/steering_status',
            self._status_output_qos,
        )
        self._control_mode_pub = self.create_publisher(
            ControlModeReport,
            '/vehicle/status/control_mode',
            self._status_output_qos,
        )
        self._gear_pub = self.create_publisher(
            GearReport,
            '/vehicle/status/gear_status',
            self._status_output_qos,
        )
        self._turn_pub = self.create_publisher(
            TurnIndicatorsReport,
            '/vehicle/status/turn_indicators_status',
            self._status_output_qos,
        )
        self._hazard_pub = self.create_publisher(
            HazardLightsReport,
            '/vehicle/status/hazard_lights_status',
            self._status_output_qos,
        )
        self._actuation_pub = self.create_publisher(
            ActuationStatusStamped,
            '/vehicle/status/actuation_status',
            self._status_output_qos,
        )
        self._odom_pub = self.create_publisher(
            Odometry,
            '/localization/kinematic_state',
            self._status_output_qos,
        )
        self._acceleration_pub = self.create_publisher(
            AccelWithCovarianceStamped,
            '/localization/acceleration',
            self._status_output_qos,
        )
        self._objects_pub = self.create_publisher(
            PredictedObjects,
            '/perception/object_recognition/objects',
            self._status_output_qos,
        )
        self._localization_state_pub = None
        if self.publish_localization_initialization_state:
            self._localization_state_pub = self.create_publisher(
                LocalizationInitializationState,
                '/localization/initialization_state',
                self._latched_status_output_qos,
            )

    def _create_subscriptions(self) -> None:
        self._ego_sub = self.create_subscription(
            EgoVehicleStatus,
            self.ego_topic,
            self._on_ego_status,
            self._morai_input_qos,
        )
        if self.publish_objects:
            self._object_sub = self.create_subscription(
                ObjectStatusList,
                self.object_topic,
                self._on_object_status,
                self._morai_input_qos,
            )
        self._control_sub = self.create_subscription(
            Control,
            self.control_topic,
            self._on_control,
            self._command_input_qos,
        )
        self._gear_sub = self.create_subscription(
            GearCommand,
            self.gear_topic,
            self._on_gear,
            self._command_input_qos,
        )
        self._turn_sub = self.create_subscription(
            TurnIndicatorsCommand,
            self.turn_topic,
            self._on_turn_indicators,
            self._command_input_qos,
        )
        self._hazard_sub = self.create_subscription(
            HazardLightsCommand,
            self.hazard_topic,
            self._on_hazard_lights,
            self._command_input_qos,
        )

    def _create_services(self) -> None:
        self._event_client = self.create_client(
            MoraiEventCmdSrv,
            self.event_service,
        )
        self._control_mode_server = self.create_service(
            ControlModeCommand,
            self.control_mode_service,
            self._on_control_mode_request,
        )

    def _initial_control_mode(self) -> int:
        if self.initial_control_mode in ('auto', 'autonomous'):
            return ControlModeReport.AUTONOMOUS
        return ControlModeReport.MANUAL

    def _initial_gear(self) -> int:
        return {
            'drive': GearReport.DRIVE,
            'reverse': GearReport.REVERSE,
            'neutral': GearReport.NEUTRAL,
            'park': GearReport.PARK,
        }.get(self.initial_gear, GearReport.PARK)

    @staticmethod
    def _autoware_mode_to_morai(mode: int) -> int:
        if mode in (
            ControlModeReport.AUTONOMOUS,
            ControlModeReport.AUTONOMOUS_STEER_ONLY,
            ControlModeReport.AUTONOMOUS_VELOCITY_ONLY,
        ):
            return MoraiAutowareInterface.MORAI_CONTROL_AUTO
        return MoraiAutowareInterface.MORAI_CONTROL_KEYBOARD

    @staticmethod
    def _autoware_gear_to_morai(gear: int) -> int:
        if gear == GearReport.PARK:
            return MoraiAutowareInterface.MORAI_GEAR_PARK
        if gear in (GearReport.REVERSE, GearReport.REVERSE_2):
            return MoraiAutowareInterface.MORAI_GEAR_REVERSE
        if gear == GearReport.NEUTRAL:
            return MoraiAutowareInterface.MORAI_GEAR_NEUTRAL
        return MoraiAutowareInterface.MORAI_GEAR_DRIVE

    @staticmethod
    def _morai_gear_to_autoware(gear: int) -> int:
        return {
            MoraiAutowareInterface.MORAI_GEAR_PARK: GearReport.PARK,
            MoraiAutowareInterface.MORAI_GEAR_REVERSE: GearReport.REVERSE,
            MoraiAutowareInterface.MORAI_GEAR_NEUTRAL: GearReport.NEUTRAL,
            MoraiAutowareInterface.MORAI_GEAR_DRIVE: GearReport.DRIVE,
        }.get(gear, GearReport.NONE)

    @staticmethod
    def _stamp_to_seconds(stamp: object) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

    def _usable_stamp(self, stamp: object) -> object:
        if stamp.sec == 0 and stamp.nanosec == 0:
            return self.get_clock().now().to_msg()
        return stamp

    def _on_ego_status(self, msg: EgoVehicleStatus) -> None:
        stamp = self._usable_stamp(msg.header.stamp)
        self._latest_stamp = stamp
        if not self._localization_initialized:
            self._publish_localization_initialized(stamp)
        yaw = heading_deg_to_yaw(msg.heading)
        stamp_seconds = self._stamp_to_seconds(stamp)
        yaw_rate = 0.0
        if self._last_yaw is not None and self._last_ego_time is not None:
            delta_time = stamp_seconds - self._last_ego_time
            if 0.0 < delta_time < 1.0:
                yaw_rate = normalize_angle(yaw - self._last_yaw) / delta_time
        self._last_yaw = yaw
        self._last_ego_time = stamp_seconds

        longitudinal_velocity = float(msg.velocity.x)
        lateral_velocity = float(msg.velocity.y)
        if self.ego_velocity_is_kmh:
            longitudinal_velocity = kmh_to_mps(longitudinal_velocity)
            lateral_velocity = kmh_to_mps(lateral_velocity)

        steering_tire_angle = float(msg.wheel_angle)
        if self.morai_steering_is_degrees:
            steering_tire_angle = math.radians(steering_tire_angle)
        steering_tire_angle *= self.steering_report_sign

        velocity = VelocityReport()
        velocity.header.stamp = stamp
        velocity.header.frame_id = self.base_frame
        velocity.longitudinal_velocity = longitudinal_velocity
        velocity.lateral_velocity = lateral_velocity
        velocity.heading_rate = yaw_rate
        self._velocity_pub.publish(velocity)

        steering = SteeringReport()
        steering.stamp = stamp
        steering.steering_tire_angle = steering_tire_angle
        self._steering_pub.publish(steering)

        actuation = ActuationStatusStamped()
        actuation.header.stamp = stamp
        actuation.header.frame_id = self.base_frame
        actuation.status.accel_status = float(msg.accel)
        actuation.status.brake_status = float(msg.brake)
        actuation.status.steer_status = steering_tire_angle
        self._actuation_pub.publish(actuation)

        if self.publish_kinematic_state:
            self._publish_kinematics(
                msg,
                stamp,
                yaw,
                yaw_rate,
                longitudinal_velocity,
                lateral_velocity,
            )

    def _publish_kinematics(
        self,
        msg: EgoVehicleStatus,
        stamp: object,
        yaw: float,
        yaw_rate: float,
        longitudinal_velocity: float,
        lateral_velocity: float,
    ) -> None:
        quaternion = quaternion_from_yaw(yaw)

        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = self.map_frame
        odometry.child_frame_id = self.base_frame
        odometry.pose.pose.position.x = float(msg.position.x)
        odometry.pose.pose.position.y = float(msg.position.y)
        odometry.pose.pose.position.z = float(msg.position.z)
        odometry.pose.pose.orientation.x = quaternion[0]
        odometry.pose.pose.orientation.y = quaternion[1]
        odometry.pose.pose.orientation.z = quaternion[2]
        odometry.pose.pose.orientation.w = quaternion[3]
        odometry.twist.twist.linear.x = longitudinal_velocity
        odometry.twist.twist.linear.y = lateral_velocity
        vertical_velocity = float(msg.velocity.z)
        if self.ego_velocity_is_kmh:
            vertical_velocity = kmh_to_mps(vertical_velocity)
        odometry.twist.twist.linear.z = vertical_velocity
        odometry.twist.twist.angular.z = yaw_rate
        for index in (0, 7, 14, 21, 28, 35):
            odometry.pose.covariance[index] = 1.0e-4
            odometry.twist.covariance[index] = 1.0e-3
        self._odom_pub.publish(odometry)

        acceleration = AccelWithCovarianceStamped()
        acceleration.header.stamp = stamp
        acceleration.header.frame_id = self.base_frame
        acceleration.accel.accel.linear.x = float(msg.acceleration.x)
        acceleration.accel.accel.linear.y = float(msg.acceleration.y)
        acceleration.accel.accel.linear.z = float(msg.acceleration.z)
        self._acceleration_pub.publish(acceleration)

        if self._tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.map_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = float(msg.position.x)
            transform.transform.translation.y = float(msg.position.y)
            transform.transform.translation.z = float(msg.position.z)
            transform.transform.rotation.x = quaternion[0]
            transform.transform.rotation.y = quaternion[1]
            transform.transform.rotation.z = quaternion[2]
            transform.transform.rotation.w = quaternion[3]
            self._tf_broadcaster.sendTransform(transform)

    def _publish_localization_initialized(self, stamp: object) -> None:
        if self._localization_state_pub is None:
            return
        state = LocalizationInitializationState()
        state.stamp = stamp
        state.state = LocalizationInitializationState.INITIALIZED
        self._localization_state_pub.publish(state)
        self._localization_initialized = True
        self.get_logger().info(
            'MORAI ego pose received; localization state is INITIALIZED'
        )

    def _on_object_status(self, msg: ObjectStatusList) -> None:
        output = PredictedObjects()
        output.header.stamp = self._usable_stamp(msg.header.stamp)
        output.header.frame_id = self.map_frame

        for status in msg.npc_list:
            output.objects.append(
                self._convert_object(status, ObjectClassification.CAR, 1)
            )
        for status in msg.pedestrian_list:
            output.objects.append(
                self._convert_object(status, ObjectClassification.PEDESTRIAN, 2)
            )
        for status in msg.obstacle_list:
            output.objects.append(
                self._convert_object(status, ObjectClassification.UNKNOWN, 3)
            )
        self._objects_pub.publish(output)

    def _convert_object(
        self,
        status: ObjectStatus,
        classification_label: int,
        namespace: int,
    ) -> PredictedObject:
        output = PredictedObject()
        output.object_id.uuid = deterministic_uuid(status.unique_id, namespace)
        output.existence_probability = 1.0

        classification = ObjectClassification()
        classification.label = classification_label
        classification.probability = 1.0
        output.classification = [classification]

        pose = output.kinematics.initial_pose_with_covariance.pose
        pose.position.x = float(status.position.x)
        pose.position.y = float(status.position.y)
        pose.position.z = float(status.position.z)
        quaternion = quaternion_from_yaw(heading_deg_to_yaw(status.heading))
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

        velocity_scale = 1.0
        if self.object_velocity_is_kmh:
            velocity_scale = kmh_to_mps(1.0)
        twist = output.kinematics.initial_twist_with_covariance.twist
        twist.linear.x = float(status.velocity.x) * velocity_scale
        twist.linear.y = float(status.velocity.y) * velocity_scale
        twist.linear.z = float(status.velocity.z) * velocity_scale
        accel = output.kinematics.initial_acceleration_with_covariance.accel
        accel.linear.x = float(status.acceleration.x)
        accel.linear.y = float(status.acceleration.y)
        accel.linear.z = float(status.acceleration.z)

        predicted_path = PredictedPath()
        predicted_path.time_step.nanosec = 500_000_000
        predicted_path.confidence = 1.0
        object_yaw = heading_deg_to_yaw(status.heading)
        map_velocity_x = (
            twist.linear.x * math.cos(object_yaw)
            - twist.linear.y * math.sin(object_yaw)
        )
        map_velocity_y = (
            twist.linear.x * math.sin(object_yaw)
            + twist.linear.y * math.cos(object_yaw)
        )
        for index in range(11):
            predicted_pose = Pose()
            elapsed = 0.5 * index
            predicted_pose.position.x = pose.position.x + map_velocity_x * elapsed
            predicted_pose.position.y = pose.position.y + map_velocity_y * elapsed
            predicted_pose.position.z = pose.position.z + twist.linear.z * elapsed
            predicted_pose.orientation = pose.orientation
            predicted_path.path.append(predicted_pose)
        output.kinematics.predicted_paths = [predicted_path]

        output.shape.type = Shape.BOUNDING_BOX
        output.shape.dimensions.x = max(0.01, float(status.size.x))
        output.shape.dimensions.y = max(0.01, float(status.size.y))
        output.shape.dimensions.z = max(0.01, float(status.size.z))
        return output

    def _on_control(self, msg: Control) -> None:
        self._latest_control = msg
        self._latest_control_time = time.monotonic()

    def _publish_control_command(self) -> None:
        if self._control_mode_report != ControlModeReport.AUTONOMOUS:
            return

        command_age = time.monotonic() - self._latest_control_time
        if self._latest_control is None or command_age > self.command_timeout_sec:
            self.publish_safe_stop()
            now = time.monotonic()
            if now - self._last_watchdog_warning > 5.0:
                self.get_logger().warning(
                    'Autoware control command is stale; applying full brake'
                )
                self._last_watchdog_warning = now
            return

        msg = self._latest_control
        output = CtrlCmd()
        output.longl_cmd_type = 2
        target_velocity = abs(mps_to_kmh(float(msg.longitudinal.velocity)))
        if self.max_velocity_kmh > 0.0:
            target_velocity = min(target_velocity, self.max_velocity_kmh)
        if self._gear_report in (GearReport.PARK, GearReport.NEUTRAL):
            target_velocity = 0.0
        output.velocity = target_velocity
        output.acceleration = float(msg.longitudinal.acceleration)
        output.steering = clamp(
            self.steering_sign * float(msg.lateral.steering_tire_angle),
            -self.max_steering_angle,
            self.max_steering_angle,
        )
        output.accel = 0.0
        output.brake = 0.0
        self._ctrl_pub.publish(output)

    def publish_safe_stop(self) -> None:
        """Publish the MORAI throttle-mode full-brake command."""
        output = CtrlCmd()
        output.longl_cmd_type = 1
        output.accel = 0.0
        output.brake = 1.0
        output.steering = 0.0
        self._ctrl_pub.publish(output)

    def _publish_discrete_status(self) -> None:
        stamp = self._latest_stamp

        control_mode = ControlModeReport()
        control_mode.stamp = stamp
        control_mode.mode = self._control_mode_report
        self._control_mode_pub.publish(control_mode)

        gear = GearReport()
        gear.stamp = stamp
        gear.report = self._gear_report
        self._gear_pub.publish(gear)

        turn = TurnIndicatorsReport()
        turn.stamp = stamp
        turn.report = self._turn_report
        self._turn_pub.publish(turn)

        hazard = HazardLightsReport()
        hazard.stamp = stamp
        hazard.report = self._hazard_report
        self._hazard_pub.publish(hazard)

    def _on_control_mode_request(
        self,
        request: ControlModeCommand.Request,
        response: ControlModeCommand.Response,
    ) -> ControlModeCommand.Response:
        if request.mode in (
            ControlModeCommand.Request.AUTONOMOUS,
            ControlModeCommand.Request.AUTONOMOUS_STEER_ONLY,
            ControlModeCommand.Request.AUTONOMOUS_VELOCITY_ONLY,
        ):
            requested_report = ControlModeReport.AUTONOMOUS
            morai_mode = self.MORAI_CONTROL_AUTO
        elif request.mode == ControlModeCommand.Request.MANUAL:
            requested_report = ControlModeReport.MANUAL
            morai_mode = self.MORAI_CONTROL_KEYBOARD
        else:
            response.success = False
            return response

        if requested_report == self._control_mode_report:
            response.success = True
            return response

        response.success = self._send_event(
            option=self.MORAI_OPTION_CONTROL_MODE,
            control_mode=morai_mode,
            description='control mode',
            on_success=partial(self._accept_control_mode, requested_report),
        )
        return response

    def _on_gear(self, msg: GearCommand) -> None:
        if msg.command == GearCommand.NONE:
            return
        requested_report = self._normalize_gear_report(msg.command)
        if requested_report == self._gear_report:
            return
        now = time.monotonic()
        if (
            requested_report == self._pending_gear_report
            and now - self._last_gear_request_time < 1.0
        ):
            return
        morai_gear = self._autoware_gear_to_morai(requested_report)
        accepted = self._send_event(
            option=self.MORAI_OPTION_GEAR,
            gear=morai_gear,
            description='gear',
            on_success=partial(self._accept_gear, requested_report),
        )
        if accepted:
            self._pending_gear_report = requested_report
            self._last_gear_request_time = now

    @staticmethod
    def _normalize_gear_report(command: int) -> int:
        if command == GearCommand.PARK:
            return GearReport.PARK
        if command == GearCommand.NEUTRAL:
            return GearReport.NEUTRAL
        if command in (GearCommand.REVERSE, GearCommand.REVERSE_2):
            return GearReport.REVERSE
        return GearReport.DRIVE

    def _on_turn_indicators(self, msg: TurnIndicatorsCommand) -> None:
        if msg.command == TurnIndicatorsCommand.ENABLE_LEFT:
            requested_turn = TurnIndicatorsReport.ENABLE_LEFT
        elif msg.command == TurnIndicatorsCommand.ENABLE_RIGHT:
            requested_turn = TurnIndicatorsReport.ENABLE_RIGHT
        else:
            requested_turn = TurnIndicatorsReport.DISABLE
        if requested_turn == self._turn_report:
            return
        self._request_lamps(requested_turn, self._hazard_report)

    def _on_hazard_lights(self, msg: HazardLightsCommand) -> None:
        requested_hazard = (
            HazardLightsReport.ENABLE
            if msg.command == HazardLightsCommand.ENABLE
            else HazardLightsReport.DISABLE
        )
        if requested_hazard == self._hazard_report:
            return
        self._request_lamps(self._turn_report, requested_hazard)

    def _request_lamps(self, turn_report: int, hazard_report: int) -> None:
        turn_signal = {
            TurnIndicatorsReport.ENABLE_LEFT: 1,
            TurnIndicatorsReport.ENABLE_RIGHT: 2,
        }.get(turn_report, 0)
        emergency_signal = 1 if hazard_report == HazardLightsReport.ENABLE else 0
        self._send_event(
            option=self.MORAI_OPTION_LAMPS,
            turn_signal=turn_signal,
            emergency_signal=emergency_signal,
            description='lamps',
            on_success=partial(self._accept_lamps, turn_report, hazard_report),
        )

    def _send_event(
        self,
        option: int,
        description: str,
        on_success: EventSuccessCallback,
        control_mode: int | None = None,
        gear: int | None = None,
        turn_signal: int = 0,
        emergency_signal: int = 0,
    ) -> bool:
        if not self._event_client.service_is_ready():
            self._event_client.wait_for_service(timeout_sec=0.0)
        if not self._event_client.service_is_ready():
            self.get_logger().warning(
                f'MORAI event service unavailable; rejected {description} request'
            )
            return False

        request = MoraiEventCmdSrv.Request()
        request.request.option = option
        request.request.ctrl_mode = (
            self._morai_control_mode if control_mode is None else control_mode
        )
        request.request.gear = self._morai_gear if gear is None else gear
        request.request.lamps.header.stamp = self.get_clock().now().to_msg()
        request.request.lamps.turn_signal = turn_signal
        request.request.lamps.emergency_signal = emergency_signal
        request.request.set_pause = False

        future = self._event_client.call_async(request)
        future.add_done_callback(
            partial(self._finish_event, description, on_success)
        )
        return True

    def _finish_event(
        self,
        description: str,
        on_success: EventSuccessCallback,
        future: object,
    ) -> None:
        try:
            result = future.result()
        except Exception as error:  # noqa: BLE001 - ROS future errors vary by RMW
            self.get_logger().error(f'MORAI {description} request failed: {error}')
            return
        if result is None:
            self.get_logger().error(f'MORAI {description} request returned no response')
            return
        on_success(result.response)

    def _accept_control_mode(self, requested_report: int, response: object) -> None:
        self._morai_control_mode = int(response.ctrl_mode)
        if response.ctrl_mode == self.MORAI_CONTROL_AUTO:
            self._control_mode_report = ControlModeReport.AUTONOMOUS
        elif response.ctrl_mode == self.MORAI_CONTROL_KEYBOARD:
            self._control_mode_report = ControlModeReport.MANUAL
        else:
            self._control_mode_report = requested_report
        self.get_logger().info(
            f'MORAI control mode accepted: {self._control_mode_report}'
        )

    def _accept_gear(self, requested_report: int, response: object) -> None:
        self._pending_gear_report = None
        self._morai_gear = int(response.gear)
        reported = self._morai_gear_to_autoware(self._morai_gear)
        self._gear_report = reported if reported != GearReport.NONE else requested_report
        self.get_logger().info(f'MORAI gear accepted: {self._gear_report}')

    def _accept_lamps(
        self,
        turn_report: int,
        hazard_report: int,
        response: object,
    ) -> None:
        del response
        self._turn_report = turn_report
        self._hazard_report = hazard_report

    def _request_startup_mode(self) -> None:
        if self._initialization_requested:
            return
        accepted = self._send_event(
            option=self.MORAI_OPTION_CONTROL_MODE | self.MORAI_OPTION_GEAR,
            control_mode=self.MORAI_CONTROL_AUTO,
            gear=self.MORAI_GEAR_DRIVE,
            description='startup auto/drive',
            on_success=self._accept_startup_mode,
        )
        if accepted:
            self._initialization_requested = True

    def _accept_startup_mode(self, response: object) -> None:
        self._morai_control_mode = int(response.ctrl_mode)
        self._morai_gear = int(response.gear)
        self._control_mode_report = ControlModeReport.AUTONOMOUS
        self._gear_report = GearReport.DRIVE
        self.get_logger().info('MORAI startup auto/drive mode accepted')


def main(args: list[str] | None = None) -> None:
    """Run the MORAI Autoware interface node."""
    rclpy.init(args=args)
    node = MoraiAutowareInterface()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if (
            rclpy.ok()
            and node._control_mode_report == ControlModeReport.AUTONOMOUS
        ):
            node.publish_safe_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
