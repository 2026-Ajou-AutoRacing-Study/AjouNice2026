#!/usr/bin/env python3

import bisect
import copy
import json
import math
import os
import re
import threading
import time

import rospy
from autoware_internal_planning_msgs.msg import PathWithLaneId
from autoware_planning_msgs.msg import LaneletRoute, Trajectory
from geometry_msgs.msg import Point, Pose, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray


START_POSE = (2468.200906609, 23725.455665620, 28.5509, 1.060582567)
GOAL_POSE = (2458.928634542, 23708.828267064, 28.5485, 1.055589773)
EXPECTED_PREFERRED_LANE_IDS = [
    200889,
    200392,
    200178,
    200337,
    200287,
    200348,
    200346,
    200291,
    200292,
    200305,
    200841,
    200950,
    200310,
    200308,
    200258,
    200261,
    200315,
    200445,
    200311,
    200968,
    201095,
    201098,
    201096,
    201087,
    201086,
    200101,
    200128,
    200102,
    200134,
    200360,
    201399,
    200088,
    201401,
    200183,
    200037,
    200471,
    201071,
    200182,
    200361,
    200181,
    200892,
]


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_from_yaw(yaw):
    quaternion = Quaternion()
    quaternion.z = math.sin(yaw * 0.5)
    quaternion.w = math.cos(yaw * 0.5)
    return quaternion


def yaw_from_quaternion(quaternion):
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def make_pose(values):
    x, y, z, yaw = values
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation = quaternion_from_yaw(yaw)
    return pose


def distance_2d(first, second):
    return math.hypot(first.x - second.x, first.y - second.y)


class CourseProjection:
    def __init__(self, station, distance, segment_index, ratio, yaw):
        self.station = station
        self.distance = distance
        self.segment_index = segment_index
        self.ratio = ratio
        self.yaw = yaw


class PolylineCourse:
    def __init__(self, poses):
        if len(poses) < 2:
            raise ValueError("course needs at least two poses")
        self.poses = [copy.deepcopy(pose) for pose in poses]
        self.stations = [0.0]
        for first, second in zip(self.poses[:-1], self.poses[1:]):
            self.stations.append(
                self.stations[-1]
                + distance_2d(first.position, second.position)
            )
        self.length = self.stations[-1]

    def sample(self, station):
        station = max(0.0, min(float(station), self.length))
        index = max(0, bisect.bisect_right(self.stations, station) - 1)
        index = min(index, len(self.poses) - 2)
        start_s = self.stations[index]
        end_s = self.stations[index + 1]
        segment_length = max(end_s - start_s, 1.0e-9)
        ratio = max(0.0, min((station - start_s) / segment_length, 1.0))
        first = self.poses[index].position
        second = self.poses[index + 1].position
        yaw = math.atan2(second.y - first.y, second.x - first.x)
        pose = Pose()
        pose.position.x = first.x + ratio * (second.x - first.x)
        pose.position.y = first.y + ratio * (second.y - first.y)
        pose.position.z = first.z + ratio * (second.z - first.z)
        pose.orientation = quaternion_from_yaw(yaw)
        return pose

    def project(
        self,
        position,
        minimum_station=0.0,
        maximum_station=None,
        yaw=None,
        yaw_threshold=None,
    ):
        maximum_station = self.length if maximum_station is None else maximum_station
        minimum_station = max(0.0, float(minimum_station))
        maximum_station = min(self.length, float(maximum_station))
        if maximum_station < minimum_station:
            return None

        first_index = max(
            0, bisect.bisect_right(self.stations, minimum_station) - 1
        )
        last_index = min(
            len(self.poses) - 2,
            bisect.bisect_left(self.stations, maximum_station),
        )
        best = None
        for index in range(first_index, last_index + 1):
            first = self.poses[index].position
            second = self.poses[index + 1].position
            dx = second.x - first.x
            dy = second.y - first.y
            segment_length = math.hypot(dx, dy)
            if segment_length < 1.0e-6:
                continue
            segment_yaw = math.atan2(dy, dx)
            if (
                yaw is not None
                and yaw_threshold is not None
                and abs(normalize_angle(segment_yaw - yaw)) > yaw_threshold
            ):
                continue
            ratio = (
                (position.x - first.x) * dx + (position.y - first.y) * dy
            ) / (segment_length * segment_length)
            segment_min = max(
                0.0,
                (minimum_station - self.stations[index]) / segment_length,
            )
            segment_max = min(
                1.0,
                (maximum_station - self.stations[index]) / segment_length,
            )
            ratio = max(segment_min, min(ratio, segment_max))
            closest_x = first.x + ratio * dx
            closest_y = first.y + ratio * dy
            distance = math.hypot(position.x - closest_x, position.y - closest_y)
            projection = CourseProjection(
                self.stations[index] + ratio * segment_length,
                distance,
                index,
                ratio,
                segment_yaw,
            )
            if best is None or projection.distance < best.distance:
                best = projection
        return best

    def trim_from_station(self, station):
        station = max(0.0, min(float(station), self.length))
        poses = [self.sample(station)]
        for pose, pose_station in zip(self.poses, self.stations):
            if pose_station > station + 1.0e-6:
                poses.append(copy.deepcopy(pose))
        return PolylineCourse(poses)


def path_point_poses(message):
    return [copy.deepcopy(path_point.point.pose) for path_point in message.points]


def trajectory_length(message):
    return sum(
        distance_2d(first.pose.position, second.pose.position)
        for first, second in zip(message.points[:-1], message.points[1:])
    )


def color(red, green, blue, alpha=1.0):
    value = ColorRGBA()
    value.r = float(red)
    value.g = float(green)
    value.b = float(blue)
    value.a = float(alpha)
    return value


class KcityDynamicEgoValidator:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.update_rate = max(float(rospy.get_param("~update_rate", 20.0)), 1.0)
        self.step_distance = max(float(rospy.get_param("~step_distance", 3.0)), 0.1)
        self.nominal_speed = max(float(rospy.get_param("~nominal_speed", 5.0)), 0.1)
        self.output_horizon = max(float(rospy.get_param("~output_horizon", 200.0)), 10.0)
        self.yaw_threshold = max(float(rospy.get_param("~yaw_threshold", 1.0472)), 0.0)
        self.behind_tolerance = max(
            float(rospy.get_param("~behind_tolerance", 0.75)), 0.0
        )
        self.lateral_tolerance = max(
            float(rospy.get_param("~lateral_tolerance", 3.0)), 0.1
        )
        self.step_timeout = max(float(rospy.get_param("~step_timeout", 30.0)), 1.0)
        self.startup_timeout = max(
            float(rospy.get_param("~startup_timeout", 120.0)), 5.0
        )
        self.keep_alive = bool(rospy.get_param("~keep_alive", True))
        self.reference_path_topic = rospy.get_param(
            "~reference_path_topic",
            "/behavior_path_planner/debug/full_route_path",
        )
        self.report_path = os.path.expanduser(
            rospy.get_param(
                "~report_path", "/tmp/kcity_dynamic_ego_validation.report.json"
            )
        )
        self.start_pose = make_pose(START_POSE)
        self.goal_pose = make_pose(GOAL_POSE)

        self.lock = threading.RLock()
        self.route = None
        self.latest_path = None
        self.course = None
        self.current_station = 0.0
        self.generation = 0
        self.validated_generation = -1
        self.required_trajectory_stamp = rospy.Time(0)
        self.last_trajectory_wall_time = None
        self.last_validation = None
        self.last_trajectory_stamp = rospy.Time(0)
        self.hard_failure = None
        self.validation_samples = 0
        self.trajectory_updates = 0
        self.max_behind_distance = 0.0
        self.max_first_point_error = 0.0
        self.max_projection_error = 0.0
        self.max_output_length = 0.0
        self.route_section_by_lane = {}
        self.maximum_root_section = -1
        self.root_regressions = []
        self.state = "WAITING_FOR_ROUTE"
        self.last_progress_log = -10

        self.odometry_pub = rospy.Publisher(
            "/localization/kinematic_state", Odometry, queue_size=1
        )
        self.marker_pub = rospy.Publisher(
            "/planning/debug/dynamic_ego/markers",
            MarkerArray,
            queue_size=1,
            latch=True,
        )
        rospy.Subscriber(
            "/planning/mission_planning/route",
            LaneletRoute,
            self.on_route,
            queue_size=1,
        )
        rospy.Subscriber(
            self.reference_path_topic,
            PathWithLaneId,
            self.on_path,
            queue_size=1,
        )
        rospy.Subscriber(
            "/planning/scenario_planning/lane_driving/motion_planning/"
            "motion_velocity_planner/trajectory",
            Trajectory,
            self.on_trajectory,
            queue_size=1,
        )
        rospy.Subscriber(
            "/planning/debug/bpp/module_status",
            String,
            self.on_module_status,
            queue_size=10,
        )

    def on_route(self, message):
        preferred_ids = [
            int(segment.preferred_primitive.id) for segment in message.segments
        ]
        with self.lock:
            self.route = message
            self.route_section_by_lane = {}
            for index, segment in enumerate(message.segments):
                lane_ids = [int(item.id) for item in segment.primitives]
                lane_ids.append(int(segment.preferred_primitive.id))
                for lane_id in lane_ids:
                    if lane_id:
                        self.route_section_by_lane[lane_id] = index
            if preferred_ids != EXPECTED_PREFERRED_LANE_IDS:
                self.hard_failure = (
                    "Mission Planner route differs from the K-City v44 course: {}"
                ).format(preferred_ids)
            else:
                self.state = "WAITING_FOR_BASELINE_PATH"

    def on_path(self, message):
        if len(message.points) < 2:
            return
        with self.lock:
            self.latest_path = message
            if self.course is not None or self.route is None or self.hard_failure:
                return
            end = message.points[-1].point.pose.position
            if distance_2d(end, self.goal_pose.position) > 2.0:
                return
            try:
                raw_course = PolylineCourse(path_point_poses(message))
            except ValueError:
                return
            start_projection = raw_course.project(
                self.start_pose.position,
                yaw=yaw_from_quaternion(self.start_pose.orientation),
                yaw_threshold=self.yaw_threshold,
            )
            if start_projection is None or start_projection.distance > 2.0:
                return
            course = raw_course.trim_from_station(start_projection.station)
            if course.length < 1000.0:
                self.hard_failure = "baseline course is unexpectedly short: {:.1f} m".format(
                    course.length
                )
                return
            maximum_jump = max(
                distance_2d(first.position, second.position)
                for first, second in zip(course.poses[:-1], course.poses[1:])
            )
            if maximum_jump > 5.0:
                self.hard_failure = "baseline course contains a {:.2f} m jump".format(
                    maximum_jump
                )
                return
            self.course = course
            self.state = "WAITING_FOR_FIRST_TRAJECTORY"
            rospy.loginfo(
                "K-City dynamic validation baseline ready: %.1f m, %d points",
                course.length,
                len(course.poses),
            )

    def on_module_status(self, message):
        match = re.search(r"root_lanelet=(\d+)", message.data)
        if match is None:
            return
        lane_id = int(match.group(1))
        with self.lock:
            if self.state not in ("MOVING", "FINALIZING"):
                return
            section = self.route_section_by_lane.get(lane_id)
            if section is None:
                return
            if section + 1 < self.maximum_root_section:
                self.root_regressions.append(
                    {
                        "lane_id": lane_id,
                        "section": section,
                        "previous_maximum": self.maximum_root_section,
                        "ego_station": self.current_station,
                    }
                )
                self.hard_failure = (
                    "BPP current route lanelet regressed from section {} to {}"
                ).format(self.maximum_root_section, section)
            self.maximum_root_section = max(self.maximum_root_section, section)

    def validate_trajectory(self, message, ego_station):
        if self.course is None:
            return False, "course is not ready", None
        if len(message.points) < 2:
            if self.course.length - ego_station <= 1.0 and message.points:
                return True, "goal point", {
                    "first_relative_station": 0.0,
                    "minimum_relative_station": 0.0,
                    "maximum_projection_error": 0.0,
                    "length": 0.0,
                }
            return False, "trajectory has fewer than two points", None

        minimum_station = max(0.0, ego_station - self.behind_tolerance - 2.0)
        maximum_station = min(
            self.course.length, ego_station + self.output_horizon + 20.0
        )
        stations = []
        maximum_projection_error = 0.0
        previous_station = minimum_station
        for index, point in enumerate(message.points):
            point_yaw = yaw_from_quaternion(point.pose.orientation)
            projection = self.course.project(
                point.pose.position,
                minimum_station=max(minimum_station, previous_station - 1.0),
                maximum_station=maximum_station,
                yaw=point_yaw,
                yaw_threshold=self.yaw_threshold,
            )
            if projection is None:
                return False, "trajectory point {} cannot be projected".format(index), None
            if projection.distance > self.lateral_tolerance:
                return (
                    False,
                    "trajectory point {} is {:.2f} m from the course".format(
                        index, projection.distance
                    ),
                    None,
                )
            if stations and projection.station + 0.5 < stations[-1]:
                return (
                    False,
                    "trajectory station regressed at point {}: {:.2f} -> {:.2f}".format(
                        index, stations[-1], projection.station
                    ),
                    None,
                )
            stations.append(projection.station)
            previous_station = max(previous_station, projection.station)
            maximum_projection_error = max(
                maximum_projection_error, projection.distance
            )

        first_relative = stations[0] - ego_station
        minimum_relative = min(stations) - ego_station
        remaining = self.course.length - ego_station
        output_length = trajectory_length(message)
        if minimum_relative < -self.behind_tolerance:
            return (
                False,
                "trajectory extends {:.2f} m behind ego".format(-minimum_relative),
                None,
            )
        if first_relative > 3.0:
            return (
                False,
                "trajectory begins {:.2f} m ahead of ego".format(first_relative),
                None,
            )
        if remaining > 25.0 and stations[-1] - ego_station < 20.0:
            return (
                False,
                "trajectory forward horizon is only {:.2f} m".format(
                    stations[-1] - ego_station
                ),
                None,
            )
        if output_length > self.output_horizon + 5.0:
            return (
                False,
                "trajectory length {:.2f} m exceeds configured horizon".format(
                    output_length
                ),
                None,
            )
        return True, "ok", {
            "first_relative_station": first_relative,
            "minimum_relative_station": minimum_relative,
            "maximum_projection_error": maximum_projection_error,
            "length": output_length,
        }

    def on_trajectory(self, message):
        receive_time = time.monotonic()
        with self.lock:
            self.trajectory_updates += 1
            self.last_trajectory_wall_time = receive_time
            self.last_trajectory_stamp = message.header.stamp
            if self.course is None or self.hard_failure:
                return
            if message.header.stamp < self.required_trajectory_stamp:
                return
            ego_station = self.current_station
            valid, reason, metrics = self.validate_trajectory(message, ego_station)
            self.last_validation = reason
            if not valid:
                self.hard_failure = "generation {}: {}".format(
                    self.generation, reason
                )
                return
            self.validation_samples += 1
            self.validated_generation = self.generation
            self.max_behind_distance = max(
                self.max_behind_distance,
                max(0.0, -metrics["minimum_relative_station"]),
            )
            self.max_first_point_error = max(
                self.max_first_point_error,
                abs(metrics["first_relative_station"]),
            )
            self.max_projection_error = max(
                self.max_projection_error, metrics["maximum_projection_error"]
            )
            self.max_output_length = max(self.max_output_length, metrics["length"])

    def ego_pose(self):
        if self.course is None:
            return copy.deepcopy(self.start_pose)
        return self.course.sample(self.current_station)

    def publish_odometry(self, stamp):
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = self.frame_id
        message.child_frame_id = "base_link"
        message.pose.pose = self.ego_pose()
        if self.state == "MOVING":
            message.twist.twist.linear.x = self.nominal_speed
        self.odometry_pub.publish(message)

    def make_marker(self, namespace, marker_id, marker_type, stamp):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        return marker

    def publish_markers(self, stamp):
        markers = []
        delete_all = Marker()
        delete_all.header.stamp = stamp
        delete_all.header.frame_id = self.frame_id
        delete_all.action = Marker.DELETEALL
        markers.append(delete_all)

        ego = self.make_marker("dynamic_ego", 0, Marker.ARROW, stamp)
        ego.pose = self.ego_pose()
        ego.pose.position.z += 0.8
        ego.scale.x = 5.0
        ego.scale.y = 2.0
        ego.scale.z = 1.5
        ego.color = color(0.1, 1.0, 0.25, 1.0)
        markers.append(ego)

        label = self.make_marker("dynamic_ego", 1, Marker.TEXT_VIEW_FACING, stamp)
        label.pose = self.ego_pose()
        label.pose.position.z += 4.0
        label.scale.z = 2.2
        label.color = color(0.9, 1.0, 0.9, 1.0)
        progress = 0.0 if self.course is None else 100.0 * self.current_station / self.course.length
        label.text = "DYNAMIC EGO  {:5.1f}%  {}".format(progress, self.state)
        markers.append(label)

        if self.course is not None:
            course = self.make_marker("dynamic_ego_course", 0, Marker.LINE_STRIP, stamp)
            course.scale.x = 0.25
            course.color = color(0.05, 0.7, 1.0, 0.35)
            course.points = [copy.deepcopy(pose.position) for pose in self.course.poses]
            markers.append(course)
        self.marker_pub.publish(MarkerArray(markers=markers))

    def set_generation_stamp(self):
        self.required_trajectory_stamp = rospy.Time.now()
        self.validated_generation = -1

    def wait_for_generation(self, rate):
        deadline = time.monotonic() + self.step_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self.lock:
                if self.hard_failure:
                    return False
                if self.validated_generation == self.generation:
                    return True
                stamp = rospy.Time.now()
                self.publish_odometry(stamp)
                self.publish_markers(stamp)
            rate.sleep()
        with self.lock:
            self.hard_failure = (
                "timed out waiting for trajectory generation {} (last={})"
            ).format(self.generation, self.last_validation)
        return False

    def report(self, passed):
        with self.lock:
            report = {
                "passed": bool(passed),
                "state": self.state,
                "failure": self.hard_failure,
                "course_length_m": self.course.length if self.course else 0.0,
                "ego_station_m": self.current_station,
                "lap_completion_ratio": (
                    self.current_station / self.course.length if self.course else 0.0
                ),
                "trajectory_updates": self.trajectory_updates,
                "validated_samples": self.validation_samples,
                "max_trajectory_behind_ego_m": self.max_behind_distance,
                "max_first_point_station_error_m": self.max_first_point_error,
                "max_course_projection_error_m": self.max_projection_error,
                "max_output_trajectory_length_m": self.max_output_length,
                "maximum_bpp_route_section": self.maximum_root_section,
                "expected_final_route_section": len(EXPECTED_PREFERRED_LANE_IDS) - 1,
                "bpp_root_regressions": self.root_regressions,
                "last_trajectory_stamp": self.last_trajectory_stamp.to_sec(),
            }
        directory = os.path.dirname(self.report_path)
        if directory and not os.path.isdir(directory):
            os.makedirs(directory)
        with open(self.report_path, "w", encoding="utf-8") as stream:
            json.dump(report, stream, indent=2, sort_keys=True)
            stream.write("\n")
        return report

    def run(self):
        rate = rospy.Rate(self.update_rate)
        startup_deadline = time.monotonic() + self.startup_timeout
        while not rospy.is_shutdown():
            with self.lock:
                stamp = rospy.Time.now()
                self.publish_odometry(stamp)
                self.publish_markers(stamp)
                ready = self.course is not None
                failure = self.hard_failure
            if failure:
                break
            if ready:
                break
            if time.monotonic() >= startup_deadline:
                with self.lock:
                    self.hard_failure = "timed out waiting for route and complete BPP path"
                break
            rate.sleep()

        with self.lock:
            if not self.hard_failure:
                self.state = "WAITING_FOR_FIRST_TRAJECTORY"
                self.required_trajectory_stamp = rospy.Time(0)
                self.validated_generation = -1
        if not self.hard_failure and not self.wait_for_generation(rate):
            pass

        with self.lock:
            if not self.hard_failure:
                self.state = "MOVING"
        while not rospy.is_shutdown():
            with self.lock:
                if self.hard_failure:
                    break
                if self.current_station >= self.course.length - 1.0e-6:
                    break
                self.current_station = min(
                    self.course.length, self.current_station + self.step_distance
                )
                self.generation += 1
                self.set_generation_stamp()
                progress = int(100.0 * self.current_station / self.course.length)
                if progress >= self.last_progress_log + 10:
                    self.last_progress_log = progress
                    rospy.loginfo(
                        "K-City dynamic lap: %d%% (%.1f / %.1f m), generation=%d",
                        progress,
                        self.current_station,
                        self.course.length,
                        self.generation,
                    )
            if not self.wait_for_generation(rate):
                break

        with self.lock:
            if not self.hard_failure:
                self.state = "FINALIZING"
                if self.maximum_root_section < len(EXPECTED_PREFERRED_LANE_IDS) - 2:
                    self.hard_failure = (
                        "BPP root lanelet did not reach the final route sections: {}"
                    ).format(self.maximum_root_section)
                elif self.root_regressions:
                    self.hard_failure = "BPP root lanelet regressed during the lap"

        passed = self.hard_failure is None and not rospy.is_shutdown()
        with self.lock:
            self.state = "PASS" if passed else "FAIL"
        report = self.report(passed)
        if passed:
            rospy.loginfo(
                "K-CITY DYNAMIC EGO VALIDATION: PASS | lap=%.1f m samples=%d "
                "max_behind=%.3f m max_first_error=%.3f m",
                report["course_length_m"],
                report["validated_samples"],
                report["max_trajectory_behind_ego_m"],
                report["max_first_point_station_error_m"],
            )
        else:
            rospy.logfatal(
                "K-CITY DYNAMIC EGO VALIDATION: FAIL | %s",
                report["failure"],
            )
            return 1

        if self.keep_alive:
            while not rospy.is_shutdown():
                stamp = rospy.Time.now()
                self.publish_odometry(stamp)
                self.publish_markers(stamp)
                rate.sleep()
        return 0


def main():
    rospy.init_node("kcity_dynamic_ego_validator")
    raise SystemExit(KcityDynamicEgoValidator().run())


if __name__ == "__main__":
    main()
