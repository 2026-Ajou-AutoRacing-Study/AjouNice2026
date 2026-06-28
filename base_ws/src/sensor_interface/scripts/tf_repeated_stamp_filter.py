#!/usr/bin/env python3
import copy

import rospy
from tf2_msgs.msg import TFMessage


class TfRepeatedStampFilter:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/morai/tf_raw")
        self.output_topic = rospy.get_param("~output_topic", "/tf")
        self.strip_leading_slash = rospy.get_param("~strip_leading_slash", True)
        self.last_stamp_by_child = {}
        self.dropped_count = 0

        self.publisher = rospy.Publisher(self.output_topic, TFMessage, queue_size=100)
        self.subscriber = rospy.Subscriber(
            self.input_topic, TFMessage, self.callback, queue_size=100
        )

        rospy.loginfo(
            "tf_repeated_stamp_filter: %s -> %s",
            self.input_topic,
            self.output_topic,
        )

    def callback(self, message):
        filtered_transforms = []

        for transform in message.transforms:
            transform = copy.deepcopy(transform)
            if self.strip_leading_slash:
                transform.header.frame_id = transform.header.frame_id.lstrip("/")
                transform.child_frame_id = transform.child_frame_id.lstrip("/")

            child_frame = transform.child_frame_id
            stamp = transform.header.stamp
            if self.last_stamp_by_child.get(child_frame) == stamp:
                self.dropped_count += 1
                rospy.logdebug_throttle(
                    5.0,
                    "Dropped %d repeated MORAI TF samples; latest repeated child frame: %s",
                    self.dropped_count,
                    child_frame,
                )
                continue

            self.last_stamp_by_child[child_frame] = stamp
            filtered_transforms.append(transform)

        if filtered_transforms:
            self.publisher.publish(TFMessage(filtered_transforms))


def main():
    rospy.init_node("tf_repeated_stamp_filter")
    TfRepeatedStampFilter()
    rospy.spin()


if __name__ == "__main__":
    main()
