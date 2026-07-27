#!/usr/bin/env python3

import math
import time
import uuid

import rospy
from autoware_internal_planning_msgs.srv import SetWaypointRoute, SetWaypointRouteRequest
from autoware_planning_msgs.msg import LaneletRoute, Trajectory
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


COURSE_POSES = {
    # K-City v44 lanelet 200889, shortly after the shared start/finish area.
    "start": (2468.200906609, 23725.455665620, 28.5509, 1.060582567),
    # Lanelet 200445 forces the outer turn instead of the shorter 200272 branch.
    "outer_turn": (2593.382702255, 24248.075945764, 28.4239, 0.766356166),
    # About 35% through the v44 competition route, on lanelet 200311.
    "early_middle_start": (2520.860484, 24344.294037, 28.4247, 2.525353892),
    # About 50% through the v44 competition route, on lanelet 201086.
    "middle_start": (2631.218601, 24478.105281, 28.3709, -0.704125321),
    # About 75% through the v44 competition route, on lanelet 200360.
    "late_start": (2665.295019, 23859.059530, 28.3568, -1.571568602),
    # Lanelet 200892 is immediately before 200889, completing the course loop.
    "goal": (2458.928634542, 23708.828267064, 28.5485, 1.055589773),
}

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

ROUTE_PRESETS = {
    "full": {
        "start": COURSE_POSES["start"],
        "start_label": "START FULL",
        "waypoints": [("OUTER TURN", COURSE_POSES["outer_turn"])],
        "expected_lane_ids": EXPECTED_PREFERRED_LANE_IDS,
    },
    "early_middle": {
        "start": COURSE_POSES["early_middle_start"],
        "start_label": "START EARLY-MIDDLE",
        "waypoints": [],
        "expected_lane_ids": EXPECTED_PREFERRED_LANE_IDS[18:],
    },
    "middle": {
        "start": COURSE_POSES["middle_start"],
        "start_label": "START MIDDLE",
        "waypoints": [],
        "expected_lane_ids": EXPECTED_PREFERRED_LANE_IDS[24:],
    },
    "late": {
        "start": COURSE_POSES["late_start"],
        "start_label": "START LATE",
        "waypoints": [],
        "expected_lane_ids": EXPECTED_PREFERRED_LANE_IDS[29:],
    },
}

ROUTE_PRESET_ALIASES = {
    "early_mid": "early_middle",
    "early-middle": "early_middle",
    "mid": "middle",
    "half": "middle",
    "final": "late",
}


def color(red, green, blue, alpha=1.0):
    value = ColorRGBA()
    value.r = red
    value.g = green
    value.b = blue
    value.a = alpha
    return value


def make_pose(values):
    x, y, z, yaw = values
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.z = math.sin(yaw * 0.5)
    pose.orientation.w = math.cos(yaw * 0.5)
    return pose


def trajectory_length(message):
    return sum(
        math.hypot(
            second.pose.position.x - first.pose.position.x,
            second.pose.position.y - first.pose.position.y,
        )
        for first, second in zip(message.points[:-1], message.points[1:])
    )


class KcityCompetitionRoutePublisher:
    def __init__(self):
        requested_preset = str(rospy.get_param("~route_preset", "full")).strip().lower()
        self.route_preset = ROUTE_PRESET_ALIASES.get(requested_preset, requested_preset)
        if self.route_preset not in ROUTE_PRESETS:
            raise RuntimeError(
                "unknown route_preset '{}'; use full, early_middle, middle, or late".format(
                    requested_preset
                )
            )
        preset = ROUTE_PRESETS[self.route_preset]
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.initial_pose_topic = rospy.get_param("~initial_pose_topic", "/initialpose")
        self.odometry_topic = rospy.get_param(
            "~odometry_topic", "/localization/kinematic_state"
        )
        self.route_topic = rospy.get_param(
            "~route_topic", "/planning/mission_planning/route"
        )
        self.route_service = rospy.get_param(
            "~route_service",
            "/planning/mission_planning/mission_planner/set_waypoint_route",
        )
        self.trajectory_topic = rospy.get_param(
            "~trajectory_topic",
            "/planning/scenario_planning/lane_driving/motion_planning/"
            "motion_velocity_planner/trajectory",
        )
        self.route_timeout = float(rospy.get_param("~route_timeout", 10.0))
        self.initial_odometry_timeout = float(
            rospy.get_param("~initial_odometry_timeout", 15.0)
        )
        self.route_service_retry_timeout = float(
            rospy.get_param("~route_service_retry_timeout", 20.0)
        )
        self.trajectory_timeout = float(rospy.get_param("~trajectory_timeout", 120.0))
        self.validate_trajectory = bool(
            rospy.get_param("~validate_trajectory", True)
        )
        self.keep_alive = bool(rospy.get_param("~keep_alive", False))
        self.require_goal_in_trajectory = bool(
            rospy.get_param("~require_goal_in_trajectory", True)
        )

        self.start_pose = make_pose(preset["start"])
        self.start_label = preset["start_label"]
        self.waypoint_entries = [
            (label, make_pose(values)) for label, values in preset["waypoints"]
        ]
        self.expected_lane_ids = list(preset["expected_lane_ids"])
        self.goal_pose = make_pose(COURSE_POSES["goal"])

        self.initial_pose_pub = rospy.Publisher(
            self.initial_pose_topic, PoseWithCovarianceStamped, queue_size=1
        )
        self.marker_pub = rospy.Publisher(
            "/planning/mission_planning/competition_route/checkpoints",
            MarkerArray,
            queue_size=1,
            latch=True,
        )

    def wait_for_initial_pose_subscriber(self):
        deadline = rospy.Time.now() + rospy.Duration(5.0)
        rate = rospy.Rate(20.0)
        while (
            not rospy.is_shutdown()
            and self.initial_pose_pub.get_num_connections() == 0
            and rospy.Time.now() < deadline
        ):
            rate.sleep()

    def publish_initial_pose(self):
        message = PoseWithCovarianceStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.frame_id
        message.pose.pose = self.start_pose
        self.initial_pose_pub.publish(message)

    def initialize_ego(self):
        deadline = time.monotonic() + self.initial_odometry_timeout
        latest_error = float("inf")
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self.publish_initial_pose()
            try:
                odometry = rospy.wait_for_message(
                    self.odometry_topic, Odometry, timeout=1.0
                )
            except rospy.ROSException:
                continue
            position = odometry.pose.pose.position
            latest_error = math.hypot(
                position.x - self.start_pose.position.x,
                position.y - self.start_pose.position.y,
            )
            if latest_error <= 2.0:
                return
        raise RuntimeError(
            "ego odometry did not initialize at the '{}' competition start "
            "(latest error {:.2f} m)".format(self.route_preset, latest_error)
        )

    def checkpoint_markers(self):
        clear = Marker()
        clear.header.frame_id = self.frame_id
        clear.header.stamp = rospy.Time.now()
        clear.action = Marker.DELETEALL
        markers = [clear]
        entries = [(self.start_label, self.start_pose, color(0.1, 1.0, 0.2, 1.0))]
        entries.extend(
            (label, pose, color(1.0, 0.75, 0.05, 1.0))
            for label, pose in self.waypoint_entries
        )
        entries.append(("GOAL", self.goal_pose, color(1.0, 0.1, 0.1, 1.0)))
        for marker_id, (label, pose, marker_color) in enumerate(entries):
            sphere = Marker()
            sphere.header.frame_id = self.frame_id
            sphere.header.stamp = rospy.Time.now()
            sphere.ns = "kcity_competition_checkpoints"
            sphere.id = marker_id * 2
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose = pose
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 4.0
            sphere.color = marker_color
            markers.append(sphere)

            text = Marker()
            text.header = sphere.header
            text.ns = "kcity_competition_checkpoint_labels"
            text.id = marker_id * 2 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose = pose
            text.pose.position.z += 4.0
            text.scale.z = 2.8
            text.color = marker_color
            text.text = label
            markers.append(text)
        return MarkerArray(markers=markers)

    @staticmethod
    def preferred_lane_ids(route):
        return [int(segment.preferred_primitive.id) for segment in route.segments]

    def wait_for_trajectory(self, minimum_stamp):
        deadline = rospy.Time.now() + rospy.Duration(self.trajectory_timeout)
        latest_error = float("inf")
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            remaining = max((deadline - rospy.Time.now()).to_sec(), 0.1)
            try:
                trajectory = rospy.wait_for_message(
                    self.trajectory_topic,
                    Trajectory,
                    timeout=min(remaining, 5.0),
                )
            except rospy.ROSException:
                continue
            if not trajectory.points:
                continue
            if trajectory.header.stamp < minimum_stamp:
                continue
            end = trajectory.points[-1].pose.position
            latest_error = math.hypot(
                end.x - self.goal_pose.position.x, end.y - self.goal_pose.position.y
            )
            if not self.require_goal_in_trajectory or latest_error <= 2.0:
                return trajectory, latest_error
        if not self.require_goal_in_trajectory:
            raise RuntimeError("competition trajectory was not published")
        raise RuntimeError(
            "trajectory did not reach the competition goal (latest error {:.2f} m)".format(
                latest_error
            )
        )

    def set_route(self):
        rospy.wait_for_service(self.route_service, timeout=10.0)
        request = SetWaypointRouteRequest()
        request.header.frame_id = self.frame_id
        request.goal_pose = self.goal_pose
        request.waypoints = [pose for _label, pose in self.waypoint_entries]
        request.uuid.uuid = list(uuid.uuid4().bytes)
        request.allow_modification = False
        service = rospy.ServiceProxy(self.route_service, SetWaypointRoute)
        deadline = time.monotonic() + self.route_service_retry_timeout
        last_message = "route service did not become ready"
        while not rospy.is_shutdown():
            request.header.stamp = rospy.Time.now()
            try:
                response = service(request)
                if response.status.success:
                    return
                last_message = response.status.message
            except rospy.ServiceException as error:
                last_message = str(error)
            if time.monotonic() >= deadline:
                break
            rospy.logwarn_throttle(
                2.0, "K-City route waiting for Mission Planner: %s", last_message
            )
            rospy.sleep(0.5)
        raise RuntimeError(last_message)

    def run(self):
        self.wait_for_initial_pose_subscriber()
        self.initialize_ego()
        self.marker_pub.publish(self.checkpoint_markers())

        # mission_planner_rviz_input updates odometry and clears the previous route.
        rospy.sleep(1.0)
        self.set_route()

        route = rospy.wait_for_message(
            self.route_topic, LaneletRoute, timeout=self.route_timeout
        )
        preferred_ids = self.preferred_lane_ids(route)
        if preferred_ids != self.expected_lane_ids:
            rospy.logwarn(
                "K-City '%s' route differs from the v44 preset: %s",
                self.route_preset,
                preferred_ids,
            )
        else:
            rospy.loginfo(
                "K-City '%s' competition route set: %d preferred lanelets",
                self.route_preset,
                len(preferred_ids),
            )

        if not self.validate_trajectory:
            rospy.loginfo(
                "K-City '%s' static competition route ready: %d preferred lanelets",
                self.route_preset,
                len(preferred_ids),
            )
            if self.keep_alive:
                rospy.spin()
            return

        trajectory, goal_error = self.wait_for_trajectory(route.header.stamp)
        if self.require_goal_in_trajectory:
            rospy.loginfo(
                "K-City competition trajectory ready: points=%d length=%.1f m "
                "goal_error=%.2f m",
                len(trajectory.points),
                trajectory_length(trajectory),
                goal_error,
            )
        else:
            rospy.loginfo(
                "K-City competition trajectory ready: points=%d length=%.1f m",
                len(trajectory.points),
                trajectory_length(trajectory),
            )


def main():
    rospy.init_node("kcity_competition_route_publisher")
    try:
        KcityCompetitionRoutePublisher().run()
    except (rospy.ROSException, rospy.ServiceException, RuntimeError) as error:
        rospy.logfatal("Failed to set K-City competition route: %s", error)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
