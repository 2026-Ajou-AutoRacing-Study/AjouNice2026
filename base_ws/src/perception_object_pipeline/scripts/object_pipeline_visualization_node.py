#!/usr/bin/env python3
"""Render the standardized object pipeline without consuming legacy markers."""

from __future__ import annotations

import copy

import rospy
from autoware_perception_msgs.msg import (
    DetectedObjects,
    ObjectClassification,
    PredictedObjects,
    TrackedObjects,
)
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


CLASS_NAMES = {
    ObjectClassification.UNKNOWN: "unknown",
    ObjectClassification.CAR: "car",
    ObjectClassification.TRUCK: "truck",
    ObjectClassification.BUS: "bus",
    ObjectClassification.TRAILER: "trailer",
    ObjectClassification.MOTORCYCLE: "motorcycle",
    ObjectClassification.BICYCLE: "bicycle",
    ObjectClassification.PEDESTRIAN: "pedestrian",
}

CLASS_COLORS = {
    ObjectClassification.CAR: (0.15, 0.55, 1.00),
    ObjectClassification.TRUCK: (0.05, 0.80, 0.90),
    ObjectClassification.BUS: (0.05, 0.80, 0.90),
    ObjectClassification.TRAILER: (0.05, 0.80, 0.90),
    ObjectClassification.MOTORCYCLE: (0.75, 0.25, 1.00),
    ObjectClassification.BICYCLE: (0.75, 0.25, 1.00),
    ObjectClassification.PEDESTRIAN: (1.00, 0.45, 0.05),
    ObjectClassification.UNKNOWN: (0.75, 0.75, 0.75),
}


def dominant_classification(obj):
    if not obj.classification:
        return ObjectClassification.UNKNOWN, 0.0
    item = max(obj.classification, key=lambda candidate: candidate.probability)
    return item.label, item.probability


def color_for(label, alpha=1.0):
    red, green, blue = CLASS_COLORS.get(
        label, CLASS_COLORS[ObjectClassification.UNKNOWN]
    )
    return ColorRGBA(red, green, blue, alpha)


def uuid_bytes(object_id):
    return bytes(bytearray(object_id.uuid))


def uuid_marker_id(object_id):
    raw = uuid_bytes(object_id)
    return int.from_bytes(raw[:4], byteorder="big", signed=False) & 0x7FFFFFFF


def short_uuid(object_id):
    return uuid_bytes(object_id).hex()[:8]


def clear_marker(header):
    marker = Marker()
    marker.header = copy.deepcopy(header)
    marker.action = Marker.DELETEALL
    return marker


def cube_marker(header, namespace, marker_id, pose, dimensions, color, lifetime):
    marker = Marker()
    marker.header = copy.deepcopy(header)
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose = copy.deepcopy(pose)
    marker.scale = copy.deepcopy(dimensions)
    marker.color = color
    marker.lifetime = lifetime
    return marker


class ObjectPipelineVisualization:
    def __init__(self):
        self.lifetime = rospy.Duration(
            float(rospy.get_param("~marker_lifetime_sec", 0.5))
        )
        self.path_point_stride = max(
            1, int(rospy.get_param("~prediction_point_stride", 5))
        )

        detection_topic = rospy.get_param(
            "~input/detections",
            "/perception/object_recognition/detection/objects",
        )
        tracking_topic = rospy.get_param(
            "~input/tracks",
            "/perception/object_recognition/tracking/objects",
        )
        prediction_topic = rospy.get_param(
            "~input/predictions",
            "/perception/object_recognition/objects",
        )

        self.detection_pub = rospy.Publisher(
            "~output/detections", MarkerArray, queue_size=1
        )
        self.tracking_pub = rospy.Publisher(
            "~output/tracks", MarkerArray, queue_size=1
        )
        self.prediction_pub = rospy.Publisher(
            "~output/predictions", MarkerArray, queue_size=1
        )

        self.detection_sub = rospy.Subscriber(
            detection_topic, DetectedObjects, self.on_detections, queue_size=1
        )
        self.tracking_sub = rospy.Subscriber(
            tracking_topic, TrackedObjects, self.on_tracks, queue_size=1
        )
        self.prediction_sub = rospy.Subscriber(
            prediction_topic, PredictedObjects, self.on_predictions, queue_size=1
        )

        rospy.loginfo(
            "[object_pipeline_visualization] detections=%s, tracks=%s, predictions=%s",
            detection_topic,
            tracking_topic,
            prediction_topic,
        )

    def on_detections(self, message):
        markers = MarkerArray()
        markers.markers.append(clear_marker(message.header))
        for index, obj in enumerate(message.objects):
            label, probability = dominant_classification(obj)
            marker = cube_marker(
                message.header,
                "detected_boxes",
                index,
                obj.kinematics.pose_with_covariance.pose,
                obj.shape.dimensions,
                ColorRGBA(0.15, 1.0, 0.25, max(0.12, 0.30 * probability)),
                self.lifetime,
            )
            markers.markers.append(marker)
        self.detection_pub.publish(markers)

    def on_tracks(self, message):
        markers = MarkerArray()
        markers.markers.append(clear_marker(message.header))
        for obj in message.objects:
            label, probability = dominant_classification(obj)
            marker_id = uuid_marker_id(obj.object_id)
            pose = obj.kinematics.pose_with_covariance.pose
            dimensions = obj.shape.dimensions
            markers.markers.append(
                cube_marker(
                    message.header,
                    "tracked_boxes",
                    marker_id,
                    pose,
                    dimensions,
                    color_for(label, max(0.30, 0.60 * probability)),
                    self.lifetime,
                )
            )

            text = Marker()
            text.header = copy.deepcopy(message.header)
            text.ns = "tracked_labels"
            text.id = marker_id
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = pose.position.x
            text.pose.position.y = pose.position.y
            text.pose.position.z = pose.position.z + 0.5 * dimensions.z + 0.7
            text.pose.orientation.w = 1.0
            text.scale.z = 0.55
            text.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
            text.text = "%s %s v=%.1fm/s" % (
                short_uuid(obj.object_id),
                CLASS_NAMES.get(label, "unknown"),
                obj.kinematics.twist_with_covariance.twist.linear.x,
            )
            text.lifetime = self.lifetime
            markers.markers.append(text)
        self.tracking_pub.publish(markers)

    def on_predictions(self, message):
        markers = MarkerArray()
        markers.markers.append(clear_marker(message.header))
        for obj in message.objects:
            if not obj.kinematics.predicted_paths:
                continue
            label, _ = dominant_classification(obj)
            marker_id = uuid_marker_id(obj.object_id)
            path = max(
                obj.kinematics.predicted_paths,
                key=lambda candidate: candidate.confidence,
            )
            if not path.path:
                continue

            line = Marker()
            line.header = copy.deepcopy(message.header)
            line.ns = "predicted_paths"
            line.id = marker_id
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = 0.18
            line.color = color_for(label, 0.95)
            line.points = [copy.deepcopy(pose.position) for pose in path.path]
            line.lifetime = self.lifetime
            markers.markers.append(line)

            samples = Marker()
            samples.header = copy.deepcopy(message.header)
            samples.ns = "predicted_samples"
            samples.id = marker_id
            samples.type = Marker.SPHERE_LIST
            samples.action = Marker.ADD
            samples.pose.orientation.w = 1.0
            samples.scale.x = 0.28
            samples.scale.y = 0.28
            samples.scale.z = 0.28
            samples.color = color_for(label, 0.70)
            samples.points = [
                copy.deepcopy(path.path[index].position)
                for index in range(0, len(path.path), self.path_point_stride)
            ]
            if (len(path.path) - 1) % self.path_point_stride:
                samples.points.append(copy.deepcopy(path.path[-1].position))
            samples.lifetime = self.lifetime
            markers.markers.append(samples)

            end = Marker()
            end.header = copy.deepcopy(message.header)
            end.ns = "predicted_ends"
            end.id = marker_id
            end.type = Marker.SPHERE
            end.action = Marker.ADD
            end.pose = copy.deepcopy(path.path[-1])
            end.scale.x = 0.55
            end.scale.y = 0.55
            end.scale.z = 0.55
            end.color = color_for(label, 1.0)
            end.lifetime = self.lifetime
            markers.markers.append(end)

        self.prediction_pub.publish(markers)


def main():
    rospy.init_node("object_pipeline_visualization")
    ObjectPipelineVisualization()
    rospy.spin()


if __name__ == "__main__":
    main()
