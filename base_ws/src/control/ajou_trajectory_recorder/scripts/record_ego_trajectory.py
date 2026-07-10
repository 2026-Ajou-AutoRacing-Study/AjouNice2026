#!/usr/bin/env python3

import csv
import math
import os
import threading

import rospy
from morai_msgs.msg import EgoVehicleStatus


def package_root():
    return os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def resolve_package_path(path_value):
    if path_value.startswith("$(find ajou_trajectory_recorder)"):
        return path_value.replace("$(find ajou_trajectory_recorder)", package_root(), 1)
    if os.path.isabs(path_value):
        return path_value
    return os.path.join(package_root(), path_value)


def vector_norm(vec):
    return math.sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z)


class EgoTrajectoryRecorder:
    def __init__(self):
        self.ego_topic = rospy.get_param("~ego_topic", rospy.get_param("ego_topic", "/Ego_topic"))
        output_csv = rospy.get_param(
            "~output_csv",
            rospy.get_param(
                "output_csv",
                "$(find ajou_trajectory_recorder)/data/recorded_trajectory.csv",
            ),
        )
        self.output_csv = resolve_package_path(output_csv)
        self.record_rate = float(rospy.get_param("~record_rate", rospy.get_param("record_rate", 10.0)))
        self.min_distance = float(rospy.get_param("~min_distance", rospy.get_param("min_distance", 0.2)))
        self.min_speed = float(
            rospy.get_param("~min_speed_to_record", rospy.get_param("min_speed_to_record", 0.1))
        )

        self.lock = threading.Lock()
        self.latest_msg = None
        self.last_recorded_position = None
        self.point_count = 0
        self.reported_fields = False
        self.csv_file = None
        self.writer = None

        os.makedirs(os.path.dirname(self.output_csv), exist_ok=True)
        self.csv_file = open(self.output_csv, "w", newline="")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(["time", "x", "y", "z", "yaw", "velocity"])
        self.csv_file.flush()

        rospy.on_shutdown(self.close)
        self.sub = rospy.Subscriber(self.ego_topic, EgoVehicleStatus, self.ego_callback, queue_size=1)
        rospy.loginfo("Recording %s to %s", self.ego_topic, self.output_csv)

    def ego_callback(self, msg):
        with self.lock:
            self.latest_msg = msg

    def extract_sample(self, msg):
        # MORAI Noetic EgoVehicleStatus in this workspace has:
        #   geometry_msgs/Vector3 position
        #   geometry_msgs/Vector3 velocity
        #   float64 heading, in degrees
        position = getattr(msg, "position", None)
        if position is None:
            raise AttributeError("EgoVehicleStatus.position is missing")

        velocity_vec = getattr(msg, "velocity", None)
        if velocity_vec is not None:
            velocity = vector_norm(velocity_vec)
        elif hasattr(msg, "speed"):
            velocity = float(msg.speed)
        else:
            velocity = 0.0

        if hasattr(msg, "heading"):
            yaw = float(msg.heading)
        elif hasattr(msg, "yaw"):
            yaw = float(msg.yaw)
        else:
            # Some MORAI message variants may store orientation elsewhere.
            # Keep recording position/speed and expose available fields once.
            yaw = 0.0

        return rospy.Time.now().to_sec(), position.x, position.y, position.z, yaw, velocity

    def moved_enough(self, x, y, z):
        if self.last_recorded_position is None:
            return True
        lx, ly, lz = self.last_recorded_position
        distance = math.sqrt((x - lx) ** 2 + (y - ly) ** 2 + (z - lz) ** 2)
        return distance >= self.min_distance

    def record_once(self):
        with self.lock:
            msg = self.latest_msg
        if msg is None:
            rospy.logwarn_throttle(2.0, "Waiting for ego status on %s", self.ego_topic)
            return

        try:
            stamp, x, y, z, yaw, velocity = self.extract_sample(msg)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Failed to extract trajectory sample: %s", exc)
            if not self.reported_fields:
                rospy.logwarn("Available EgoVehicleStatus fields: %s", list(getattr(msg, "__slots__", [])))
                self.reported_fields = True
            return

        if velocity < self.min_speed:
            return
        if not self.moved_enough(x, y, z):
            return

        self.writer.writerow(["%.6f" % stamp, "%.6f" % x, "%.6f" % y, "%.6f" % z, "%.6f" % yaw, "%.6f" % velocity])
        self.csv_file.flush()
        self.last_recorded_position = (x, y, z)
        self.point_count += 1

    def close(self):
        if self.csv_file is not None and not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()
            rospy.loginfo("Closed %s with %d recorded points.", self.output_csv, self.point_count)

    def run(self):
        rate = rospy.Rate(self.record_rate)
        while not rospy.is_shutdown():
            self.record_once()
            rospy.loginfo_throttle(2.0, "Recorded trajectory points: %d", self.point_count)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("record_ego_trajectory")
    try:
        EgoTrajectoryRecorder().run()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr("record_ego_trajectory failed: %s", exc)
