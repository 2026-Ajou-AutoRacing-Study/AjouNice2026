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
        rospy.loginfo("%s 토픽을 morai_msgs/EgoVehicleStatus 타입으로 구독합니다.", self.ego_topic)

    def ego_callback(self, msg):
        with self.lock:
            self.last_msg = msg
            self.last_msg_time = rospy.Time.now()

    def extract_status(self, msg):
        position = getattr(msg, "position", None)
        velocity_vec = getattr(msg, "velocity", None)
        speed = vector_speed(velocity_vec)

        # 현재 MORAI EgoVehicleStatus의 heading 단위는 degree입니다.
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
            rospy.logwarn_throttle(5.0, "ROS master 토픽 조회 실패: %s", exc)
            return False

    def run(self):
        rate = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            with self.lock:
                msg = self.last_msg
                msg_time = self.last_msg_time

            if msg is None or msg_time is None:
                rospy.logwarn("아직 %s 토픽에서 ego 상태를 받지 못했습니다.", self.ego_topic)
            elif (now - msg_time).to_sec() > 2.0:
                rospy.logwarn("%.1f초 동안 ego 상태를 받지 못했습니다.", (now - msg_time).to_sec())
            else:
                try:
                    position, yaw, speed = self.extract_status(msg)
                    if position is None:
                        raise AttributeError("position 필드가 없습니다")
                    rospy.loginfo(
                        "Ego 위치=(%.3f, %.3f, %.3f), heading/yaw=%s, 속도=%.3f m/s",
                        position.x,
                        position.y,
                        position.z,
                        "n/a" if yaw is None else "%.3f" % yaw,
                        0.0 if speed is None else speed,
                    )
                except Exception as exc:
                    rospy.logwarn("EgoVehicleStatus 필드 추출 실패: %s", exc)
                    if not self.reported_fields:
                        rospy.logwarn("사용 가능한 메시지 필드: %s", list(getattr(msg, "__slots__", [])))
                        self.reported_fields = True

            rospy.loginfo_throttle(
                5.0,
                "/ctrl_cmd 토픽 존재 여부: %s",
                "예" if self.ctrl_cmd_topic_exists() else "아니오",
            )
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("check_morai_connection")
    try:
        MoraiConnectionChecker().run()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr("check_morai_connection 실행 실패: %s", exc)
