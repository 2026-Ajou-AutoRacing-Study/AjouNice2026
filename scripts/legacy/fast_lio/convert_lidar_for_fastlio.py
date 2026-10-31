#!/usr/bin/env python3
import argparse
import copy
import math
import os
import struct

import rosbag
from sensor_msgs.msg import PointCloud2, PointField


def make_cloud_xyzirt(header, points):
    """
    Create PointCloud2 with fields:
      x         float32 offset 0
      y         float32 offset 4
      z         float32 offset 8
      intensity float32 offset 12
      ring      uint16  offset 16
      time      float32 offset 18
      padding           offset 22~23

    point_step = 24

    We keep field names x/y/z/intensity/ring/time, but use 24 bytes per point
    to avoid alignment/layout issues in downstream PCL/FAST-LIO readers.
    """
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
        PointField(name="time", offset=18, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 24
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True

    buff = bytearray(msg.row_step)
    for i, (x, y, z, intensity, ring, rel_time) in enumerate(points):
        base = i * msg.point_step
        struct.pack_into("<ffffHf", buff, base, x, y, z, intensity, int(ring), rel_time)
        # last 2 bytes remain zero padding

    msg.data = bytes(buff)
    return msg


def field_map(msg):
    return {f.name: f for f in msg.fields}


def read_float32(msg, f, base):
    return struct.unpack_from("<f", msg.data, base + f.offset)[0]


def read_uint16(msg, f, base):
    return struct.unpack_from("<H", msg.data, base + f.offset)[0]


def convert_cloud(msg, stamp, normalize_time=True):
    fields = field_map(msg)

    for name in ["x", "y", "z", "intensity"]:
        if name not in fields:
            raise RuntimeError(f"missing field {name}")

    ring_field = fields.get("ring")
    time_field = fields.get("time")

    raw_points = []
    point_count = msg.width * msg.height

    min_time = None

    for i in range(point_count):
        base = i * msg.point_step

        x = read_float32(msg, fields["x"], base)
        y = read_float32(msg, fields["y"], base)
        z = read_float32(msg, fields["z"], base)
        intensity = read_float32(msg, fields["intensity"], base)

        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue

        if ring_field is not None and ring_field.datatype == PointField.UINT16:
            ring = read_uint16(msg, ring_field, base)
        else:
            ring = 0

        if time_field is not None and time_field.datatype == PointField.FLOAT32:
            rel_time = read_float32(msg, time_field, base)
        else:
            rel_time = 0.0

        if math.isfinite(rel_time):
            min_time = rel_time if min_time is None else min(min_time, rel_time)
        else:
            rel_time = 0.0

        raw_points.append((x, y, z, intensity, ring, rel_time))

    if normalize_time and min_time is not None:
        points = [
            (x, y, z, intensity, ring, rel_time - min_time)
            for x, y, z, intensity, ring, rel_time in raw_points
        ]
    else:
        points = raw_points

    new_header = copy.deepcopy(msg.header)
    new_header.stamp = stamp
    return make_cloud_xyzirt(new_header, points)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input_bag")
    parser.add_argument("output_bag")
    parser.add_argument("--input-topic", default="/sensor/lidar1")
    parser.add_argument("--output-topic", default="/sensor/lidar1_fastlio")
    parser.add_argument("--no-normalize-time", action="store_true")
    args = parser.parse_args()

    if os.path.abspath(args.input_bag) == os.path.abspath(args.output_bag):
        raise RuntimeError("input and output bag must be different")

    total = 0
    converted = 0

    with rosbag.Bag(args.output_bag, "w", compression="lz4") as outbag:
        with rosbag.Bag(args.input_bag, "r") as inbag:
            for topic, msg, t in inbag.read_messages():
                total += 1

                if hasattr(msg, "header"):
                    msg = copy.deepcopy(msg)
                    msg.header.stamp = t

                if topic == args.input_topic:
                    new_msg = convert_cloud(
                        msg,
                        t,
                        normalize_time=not args.no_normalize_time,
                    )
                    outbag.write(args.output_topic, new_msg, t)
                    converted += 1
                else:
                    outbag.write(topic, msg, t)

    print(f"input: {args.input_bag}")
    print(f"output: {args.output_bag}")
    print(f"total messages: {total}")
    print(f"converted {args.input_topic} -> {args.output_topic}: {converted}")


if __name__ == "__main__":
    main()