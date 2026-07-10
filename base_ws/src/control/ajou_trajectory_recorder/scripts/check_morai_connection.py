#!/usr/bin/env python3

import math
import threading

import rospy
from morai_msgs.msg import EgoVehicleStatus


def vector_speed(vec):
    try:
        return math.sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z)
    except AttributeError:
        return None


class MoraiConnectionChecker:
    def __init__(self):
        self.ego_topic = rospy.get_param("~ego_topic", rospy.get_param("ego_topic", "/Ego_topic"))
        self.last_msg = None
        self.last_msg_time = None
        self.lock = threading.Lock()
        self.reported_fields = False

        self.sub = rospy.Subscriber(self.ego_topic, EgoVehicleStatus, self.ego_callback, queue_size=1)
        rospy.loginfo("Subscribing to %s as morai_msgs/EgoVehicleStatus", self.ego_topic)

    def ego_callback(self, msg):
        with self.lock:
            self.last_msg = msg
            self.last_msg_time = rospy.Time.now()

    def extract_status(self, msg):
        position = getattr(msg, "position", None)
        velocity_vec = getattr(msg, "velocity", None)
        speed = vector_speed(velocity_vec)

        # Current MORAI EgoVehicleStatus uses heading in degrees.
        yaw = getattr(msg, "heading", None)
        if yaw is None:
            yaw = getattr(msg, "yaw", None)

        return position, yaw, speed

    def ctrl_cmd_topic_exists(self):
        try:
            _code, _message, state = rospy.get_master().getSystemState()
            publishers, subscribers, _services = state
            topics = {name for name, _nodes in publishers}
            topics.update(name for name, _nodes in subscribers)
            return "/ctrl_cmd" in topics
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "Could not query ROS master topics: %s", exc)
            return False

    def run(self):
        rate = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            with self.lock:
                msg = self.last_msg
                msg_time = self.last_msg_time

            if msg is None or msg_time is None:
                rospy.logwarn("No ego status received on %s yet.", self.ego_topic)
            elif (now - msg_time).to_sec() > 2.0:
                rospy.logwarn("No ego status received for %.1f seconds.", (now - msg_time).to_sec())
            else:
                try:
                    position, yaw, speed = self.extract_status(msg)
                    if position is None:
                        raise AttributeError("position field is missing")
                    rospy.loginfo(
                        "Ego position=(%.3f, %.3f, %.3f), heading/yaw=%s, velocity=%.3f m/s",
                        position.x,
                        position.y,
                        position.z,
                        "n/a" if yaw is None else "%.3f" % yaw,
                        0.0 if speed is None else speed,
                    )
                except Exception as exc:
                    rospy.logwarn("Failed to extract EgoVehicleStatus fields: %s", exc)
                    if not self.reported_fields:
                        rospy.logwarn("Available message fields: %s", list(getattr(msg, "__slots__", [])))
                        self.reported_fields = True

            rospy.loginfo_throttle(
                5.0,
                "/ctrl_cmd subscriber topic exists: %s",
                "yes" if self.ctrl_cmd_topic_exists() else "no",
            )
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("check_morai_connection")
    try:
        MoraiConnectionChecker().run()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr("check_morai_connection failed: %s", exc)
