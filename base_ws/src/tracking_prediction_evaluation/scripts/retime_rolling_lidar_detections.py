#!/usr/bin/env python3
"""Retimestamp recorded detections using the rolling LiDAR point-time field.

This is an offline A/B utility.  It preserves rosbag record time and changes
only the DetectedObjects header to the robust representative acquisition time
used by the online PointPainting node.
"""

import argparse
import copy
import json
from pathlib import Path

import numpy as np

from perception_object_pipeline.kinematic_prediction import quaternion_to_yaw
from perception_object_pipeline.rolling_lidar import (
    representative_detection_time_offset,
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("raw_bag", type=Path)
    parser.add_argument("pipeline_bag", type=Path)
    parser.add_argument("output_bag", type=Path)
    parser.add_argument("--report", type=Path)
    parser.add_argument("--lidar-topic", default="/sensor/lidar1")
    parser.add_argument(
        "--detection-topic",
        default="/perception/object_recognition/detection/objects",
    )
    parser.add_argument("--box-extent-scale", type=float, default=0.65)
    parser.add_argument("--minimum-box-points", type=int, default=3)
    return parser.parse_args()


def cloud_xyzi_time(message):
    fields = {field.name: field for field in message.fields}
    names = ("x", "y", "z", "intensity", "time")
    missing = [name for name in names if name not in fields]
    if missing:
        raise RuntimeError("PointCloud2 missing fields {}".format(missing))
    endian = ">" if message.is_bigendian else "<"
    formats = []
    offsets = []
    for name in names:
        field = fields[name]
        if field.datatype != field.FLOAT32:
            raise RuntimeError("{} must be FLOAT32".format(name))
        formats.append(endian + "f4")
        offsets.append(int(field.offset))
    dtype = np.dtype(
        {
            "names": list(names),
            "formats": formats,
            "offsets": offsets,
            "itemsize": int(message.point_step),
        }
    )
    raw = np.frombuffer(
        message.data,
        dtype=dtype,
        count=int(message.width) * int(message.height),
    )
    points = np.column_stack([raw[name] for name in names[:4]]).astype(
        np.float32, copy=False
    )
    return points, raw["time"].astype(np.float32, copy=False)


def detection_boxes(message):
    rows = []
    for obj in message.objects:
        pose = obj.kinematics.pose_with_covariance.pose
        dimensions = obj.shape.dimensions
        rows.append(
            [
                pose.position.x,
                pose.position.y,
                pose.position.z,
                dimensions.x,
                dimensions.y,
                dimensions.z,
                quaternion_to_yaw(pose.orientation),
            ]
        )
    return np.asarray(rows, dtype=np.float32).reshape((-1, 7))


def percentile(values, quantile):
    return float(np.percentile(np.asarray(values, dtype=np.float64), quantile))


def main():
    import rosbag
    import rospy

    args = parse_args()
    clouds = {}
    with rosbag.Bag(str(args.raw_bag), "r") as bag:
        for _, message, _ in bag.read_messages(topics=[args.lidar_topic]):
            clouds[message.header.stamp.to_nsec()] = cloud_xyzi_time(message)

    args.output_bag.parent.mkdir(parents=True, exist_ok=True)
    offsets = []
    input_count = 0
    output_count = 0
    missing_cloud_count = 0
    with rosbag.Bag(str(args.pipeline_bag), "r") as source, rosbag.Bag(
        str(args.output_bag), "w", compression="lz4"
    ) as target:
        for topic, message, record_time in source.read_messages(
            topics=[args.detection_topic]
        ):
            input_count += 1
            cloud = clouds.get(message.header.stamp.to_nsec())
            if cloud is None:
                missing_cloud_count += 1
                continue
            points, point_offsets = cloud
            offset = representative_detection_time_offset(
                points,
                point_offsets,
                detection_boxes(message),
                extent_scale=args.box_extent_scale,
                minimum_points_per_box=args.minimum_box_points,
            )
            retimed = copy.deepcopy(message)
            retimed.header.stamp = rospy.Time.from_sec(
                max(0.0, message.header.stamp.to_sec() + offset)
            )
            target.write(topic, retimed, record_time)
            offsets.append(offset)
            output_count += 1

    report = {
        "schema_version": 1,
        "input_detection_count": input_count,
        "output_detection_count": output_count,
        "missing_cloud_count": missing_cloud_count,
        "offset_sec": {
            "mean": float(np.mean(offsets)) if offsets else None,
            "p05": percentile(offsets, 5) if offsets else None,
            "p50": percentile(offsets, 50) if offsets else None,
            "p95": percentile(offsets, 95) if offsets else None,
            "minimum": float(min(offsets)) if offsets else None,
            "maximum": float(max(offsets)) if offsets else None,
        },
    }
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
