#!/usr/bin/env python3
"""Publish MORAI ObjectStatusList GT as canonical DetectedObjects."""

from __future__ import annotations

import rospy
from autoware_perception_msgs.msg import DetectedObjects
from morai_msgs.msg import ObjectStatusList

from perception_object_pipeline.morai_gt_detection import make_morai_gt_detections


class MoraiGtDetectionNode:
    def __init__(self):
        self.frame_id = str(rospy.get_param("~frame_id", "morai_map"))
        self.include_velocity = bool(rospy.get_param("~include_velocity", False))
        self.velocity_noise_std_mps = float(rospy.get_param("~velocity_noise_std_mps", 1.0))
        self.maximum_velocity_mps = float(rospy.get_param("~maximum_velocity_mps", 50.0))
        self.correct_vehicle_bbox_center = bool(
            rospy.get_param("~correct_vehicle_bbox_center", True)
        )
        self.passenger_maximum_length_m = float(
            rospy.get_param("~passenger_maximum_length_m", 5.5)
        )
        self.passenger_longitudinal_offset_m = float(
            rospy.get_param("~passenger_longitudinal_offset_m", 1.34)
        )
        input_topic = rospy.get_param("~input/objects", "/Object_topic")
        output_topic = rospy.get_param(
            "~output/objects",
            "/perception/object_recognition/detection/objects",
        )

        self.publisher = rospy.Publisher(
            output_topic, DetectedObjects, queue_size=1
        )
        self.subscriber = rospy.Subscriber(
            input_topic,
            ObjectStatusList,
            self.callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        rospy.loginfo(
            "[morai_gt_detection] %s -> %s, frame=%s, bbox_center=%s, sim_velocity=%s",
            input_topic,
            output_topic,
            self.frame_id,
            self.correct_vehicle_bbox_center,
            self.include_velocity,
        )

    def callback(self, message):
        if message.header.stamp == rospy.Time():
            message.header.stamp = rospy.Time.now()
        try:
            output = make_morai_gt_detections(
                message,
                frame_id=self.frame_id,
                include_velocity=self.include_velocity,
                velocity_noise_std_mps=self.velocity_noise_std_mps,
                maximum_velocity_mps=self.maximum_velocity_mps,
                correct_vehicle_bbox_center=self.correct_vehicle_bbox_center,
                passenger_maximum_length_m=self.passenger_maximum_length_m,
                passenger_longitudinal_offset_m=(
                    self.passenger_longitudinal_offset_m
                ),
            )
        except (TypeError, ValueError) as error:
            rospy.logwarn_throttle(
                2.0, "[morai_gt_detection] invalid ObjectStatusList: %s", error
            )
            return
        self.publisher.publish(output)


def main():
    rospy.init_node("morai_gt_detection")
    MoraiGtDetectionNode()
    rospy.spin()


if __name__ == "__main__":
    main()
