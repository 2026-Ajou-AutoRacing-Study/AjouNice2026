#!/usr/bin/env python3
import rospy

from morai_pointpainting_ros.msg import PointPaintingDetectionArray


def callback(msg):
    rospy.loginfo(
        "stamp=%.6f frame=%s boxes=%d total=%.1fms "
        "seg=%.1fms paint=%.1fms det=%.1fms "
        "camera_age=%.1fms dropped=%d skipped=%d",
        msg.header.stamp.to_sec(),
        msg.header.frame_id,
        len(msg.detections),
        msg.total_ms,
        msg.segmentation_ms,
        msg.painting_ms,
        msg.detection_ms,
        msg.camera_age_max_ms,
        msg.dropped_lidar_frames,
        msg.skipped_lidar_frames,
    )
    for det in msg.detections[:5]:
        rospy.loginfo(
            "  id=%d class=%s score=%.3f xyz=(%.2f, %.2f, %.2f) "
            "size=(%.2f, %.2f, %.2f) yaw_quat_z=%.3f distance=%.2f",
            det.detection_id,
            det.class_name,
            det.score,
            det.pose.position.x,
            det.pose.position.y,
            det.pose.position.z,
            det.size.x,
            det.size.y,
            det.size.z,
            det.pose.orientation.z,
            det.distance,
        )


def main():
    rospy.init_node("check_pointpainting_output")
    topic = rospy.get_param("~topic", "/pointpainting/detections")
    rospy.Subscriber(
        topic,
        PointPaintingDetectionArray,
        callback,
        queue_size=1,
    )
    rospy.loginfo("Watching %s", topic)
    rospy.spin()


if __name__ == "__main__":
    main()
