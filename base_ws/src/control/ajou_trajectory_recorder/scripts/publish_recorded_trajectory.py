#!/usr/bin/env python3

import csv
import os

import rospy
import tf.transformations
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32


def package_root():
    return os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def resolve_package_path(path_value):
    if path_value.startswith("$(find ajou_trajectory_recorder)"):
        return path_value.replace("$(find ajou_trajectory_recorder)", package_root(), 1)
    if os.path.isabs(path_value):
        return path_value
    return os.path.join(package_root(), path_value)


class RecordedTrajectoryPublisher:
    def __init__(self):
        input_csv = rospy.get_param(
            "~input_csv",
            rospy.get_param(
                "input_csv",
                "$(find ajou_trajectory_recorder)/data/recorded_trajectory.csv",
            ),
        )
        self.input_csv = resolve_package_path(input_csv)
        self.path_topic = rospy.get_param("~path_topic", rospy.get_param("path_topic", "/recorded_trajectory_path"))
        self.target_speed_topic = rospy.get_param(
            "~target_speed_topic",
            rospy.get_param("target_speed_topic", "/recorded_trajectory/target_speed"),
        )
        self.frame_id = rospy.get_param("~frame_id", rospy.get_param("frame_id", "map"))
        self.publish_rate = float(rospy.get_param("~publish_rate", rospy.get_param("publish_rate", 10.0)))
        self.default_target_speed = float(
            rospy.get_param("~default_target_speed", rospy.get_param("default_target_speed", 5.0))
        )

        self.samples = self.load_csv()
        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=1, latch=True)
        self.speed_pub = rospy.Publisher(self.target_speed_topic, Float32, queue_size=1)
        rospy.loginfo("Loaded %d trajectory points from %s", len(self.samples), self.input_csv)

    def load_csv(self):
        if not os.path.exists(self.input_csv):
            rospy.logwarn("CSV file does not exist: %s", self.input_csv)
            return []

        samples = []
        with open(self.input_csv, "r", newline="") as csv_file:
            reader = csv.DictReader(csv_file)
            for index, row in enumerate(reader, start=2):
                try:
                    samples.append(
                        {
                            "time": float(row.get("time", 0.0) or 0.0),
                            "x": float(row["x"]),
                            "y": float(row["y"]),
                            "z": float(row.get("z", 0.0) or 0.0),
                            "yaw": float(row.get("yaw", 0.0) or 0.0),
                            "velocity": float(row.get("velocity", self.default_target_speed) or self.default_target_speed),
                        }
                    )
                except Exception as exc:
                    rospy.logwarn("Skipping invalid CSV row %d: %s", index, exc)
        return samples

    def build_path(self):
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = self.frame_id

        for sample in self.samples:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = sample["x"]
            pose.pose.position.y = sample["y"]
            pose.pose.position.z = sample["z"]

            # Recorded yaw is kept in the CSV as MORAI heading units. The current
            # MORAI message documents heading in degrees, so convert degrees to rad.
            quat = tf.transformations.quaternion_from_euler(0.0, 0.0, sample["yaw"] * 3.141592653589793 / 180.0)
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]
            path.poses.append(pose)

        return path

    def average_target_speed(self):
        speeds = [sample["velocity"] for sample in self.samples if sample["velocity"] > 0.0]
        if not speeds:
            return self.default_target_speed
        return sum(speeds) / float(len(speeds))

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            path = self.build_path()
            self.path_pub.publish(path)
            self.speed_pub.publish(Float32(data=self.average_target_speed()))
            rospy.loginfo_throttle(5.0, "Publishing %d poses on %s", len(path.poses), self.path_topic)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("publish_recorded_trajectory")
    try:
        RecordedTrajectoryPublisher().run()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr("publish_recorded_trajectory failed: %s", exc)
