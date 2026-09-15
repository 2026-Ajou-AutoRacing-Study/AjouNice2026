#!/usr/bin/env python3

"""Retimestamp MORAI Ego ground truth at ROS receipt time.

The competition LAN transport may expose a valid ``/Ego_topic`` payload with a
timestamp that is fixed, regresses, or belongs to a clock other than the ROS
host clock.  ``localization_morai_gt`` deliberately performs exact-stamp TF
lookups, so such a timestamp prevents ``map -> base_link`` from becoming a
usable Planning state.

This adapter leaves the source topic untouched and publishes a copy with a
strictly increasing ROS arrival timestamp.  It is an explicit field fallback,
not the default for recorded datasets with trustworthy measurement stamps.
"""

import copy

import rospy
from morai_msgs.msg import EgoVehicleStatus


class MoraiEgoArrivalTimeAdapter:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/Ego_topic")
        self.output_topic = rospy.get_param(
            "~output_topic", "/localization/morai_gt/ego_retimed"
        )
        self.fallback_frame_id = rospy.get_param(
            "~fallback_frame_id", "morai_map"
        )
        self.last_output_stamp = rospy.Time(0)
        self.last_source_stamp = None

        self.publisher = rospy.Publisher(
            self.output_topic, EgoVehicleStatus, queue_size=50
        )
        self.subscriber = rospy.Subscriber(
            self.input_topic,
            EgoVehicleStatus,
            self.callback,
            queue_size=100,
            tcp_nodelay=True,
        )
        rospy.loginfo(
            "MORAI Ego arrival-time adapter: %s -> %s",
            self.input_topic,
            self.output_topic,
        )

    def callback(self, message):
        source_stamp = message.header.stamp
        now = rospy.Time.now()
        if now <= self.last_output_stamp:
            now = self.last_output_stamp + rospy.Duration(nsecs=1)

        if self.last_source_stamp is not None and source_stamp <= self.last_source_stamp:
            rospy.logwarn_throttle(
                2.0,
                "MORAI Ego source timestamp is not increasing; using ROS arrival time",
            )
        self.last_source_stamp = source_stamp
        self.last_output_stamp = now

        output = copy.deepcopy(message)
        output.header.stamp = now
        if not output.header.frame_id:
            output.header.frame_id = self.fallback_frame_id
        self.publisher.publish(output)


def main():
    rospy.init_node("morai_ego_arrival_time_adapter")
    MoraiEgoArrivalTimeAdapter()
    rospy.spin()


if __name__ == "__main__":
    main()
