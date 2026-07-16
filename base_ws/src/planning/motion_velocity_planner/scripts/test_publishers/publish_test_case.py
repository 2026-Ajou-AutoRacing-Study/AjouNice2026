#!/usr/bin/env python3
"""
Simple test publisher for motion_velocity_planner integration tests.
Publishes:
 - Path Optimizer trajectory (autoware_planning_msgs/Trajectory)
 - vehicle odometry (nav_msgs/Odometry)
 - predicted objects (autoware_perception_msgs/PredictedObjects)

Behavior:
 - publishes a straight path with constant initial velocity
 - after `object_spawn_time` seconds, publishes a static obstacle on the path
"""

import math
import rospy

from nav_msgs.msg import Odometry

from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from autoware_perception_msgs.msg import (
    PredictedObject,
    PredictedObjects,
    ObjectClassification,
    Shape,
)


def quaternion_from_yaw(yaw):
    from geometry_msgs.msg import Quaternion

    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.z = math.sin(yaw * 0.5)
    return q


def make_trajectory(num_points=121, spacing=1.0, default_speed=8.0):
    trajectory = Trajectory()
    trajectory.header.frame_id = "map"
    for i in range(num_points):
        point = TrajectoryPoint()
        point.pose.position.x = float(i) * spacing
        point.pose.position.y = 0.0
        point.pose.orientation = quaternion_from_yaw(0.0)
        point.longitudinal_velocity_mps = float(default_speed)
        trajectory.points.append(point)
    return trajectory


def make_odometry(speed=5.0):
    msg = Odometry()
    msg.header.frame_id = "map"
    msg.pose.pose.position.x = 0.0
    msg.pose.pose.orientation = quaternion_from_yaw(0.0)
    msg.twist.twist.linear.x = float(speed)
    return msg


def make_object(x=30.0, y=0.0, yaw=math.pi, speed=0.0, object_id=1):
    obj = PredictedObject()
    obj.object_id.uuid = [object_id] + [0] * 15
    cls = ObjectClassification()
    # Use generic CAR label if available
    try:
        cls.label = ObjectClassification.CAR
    except Exception:
        cls.label = 1
    cls.probability = 1.0
    obj.classification.append(cls)
    obj.shape.type = Shape.BOUNDING_BOX
    obj.shape.dimensions.x = 4.5
    obj.shape.dimensions.y = 2.0
    obj.shape.dimensions.z = 1.6
    pose = obj.kinematics.initial_pose_with_covariance.pose
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.orientation = quaternion_from_yaw(yaw)
    obj.kinematics.initial_twist_with_covariance.twist.linear.x = float(speed)
    return obj


def make_objects_message(*objs):
    msg = PredictedObjects()
    msg.objects = list(objs)
    return msg


def main():
    rospy.init_node("mvp_test_publisher")

    trajectory_pub = rospy.Publisher(
        "/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory",
        Trajectory,
        queue_size=1,
        latch=True,
    )
    odom_pub = rospy.Publisher(
        "/localization/kinematic_state", Odometry, queue_size=1, latch=True
    )
    objects_pub = rospy.Publisher(
        "/perception/object_recognition/objects", PredictedObjects, queue_size=1
    )

    rate_hz = rospy.get_param("~hz", 10.0)
    object_spawn_time = rospy.get_param("~object_spawn_time", 2.0)
    object_x = rospy.get_param("~object_x", 30.0)
    object_speed = rospy.get_param("~object_speed", 0.0)

    trajectory = make_trajectory()
    odom = make_odometry()
    static_object = make_object(x=object_x, speed=object_speed)

    rospy.loginfo(
        "Test publisher started — publishing path, odom, then spawning object at %.2fm after %.2fs (speed=%.2f m/s)",
        object_x,
        object_spawn_time,
        object_speed,
    )

    start = rospy.Time.now()
    r = rospy.Rate(rate_hz)
    spawned = False
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        # publish static latched messages
        trajectory.header.stamp = now
        odom.header.stamp = now
        trajectory_pub.publish(trajectory)
        odom_pub.publish(odom)

        # spawn object after delay
        if not spawned and (now - start).to_sec() >= object_spawn_time:
            objects_pub.publish(make_objects_message(static_object))
            rospy.loginfo("Spawned test object at x=%.2f with speed=%.2f m/s", object_x, object_speed)
            spawned = True
        elif not spawned:
            # publish empty objects before spawning
            objects_pub.publish(make_objects_message())

        r.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
