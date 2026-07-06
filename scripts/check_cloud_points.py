#!/usr/bin/env python3
import math
import sys

import rosbag
import sensor_msgs.point_cloud2 as pc2


def check(bag_path, topic):
    with rosbag.Bag(bag_path, "r") as bag:
        for _, msg, t in bag.read_messages(topics=[topic]):
            print("bag time:", t.to_sec())
            print("header stamp:", msg.header.stamp.to_sec())
            print("frame:", msg.header.frame_id)
            print("width:", msg.width)
            print("height:", msg.height)
            print("point_step:", msg.point_step)
            print("row_step:", msg.row_step)
            print("is_dense:", msg.is_dense)
            print("fields:", [(f.name, f.offset, f.datatype, f.count) for f in msg.fields])

            total = 0
            finite = 0
            sample = []

            for p in pc2.read_points(
                msg,
                field_names=("x", "y", "z", "intensity", "ring", "time"),
                skip_nans=False,
            ):
                total += 1
                x, y, z, intensity, ring, time = p
                if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                    finite += 1
                    if len(sample) < 10:
                        sample.append((x, y, z, intensity, ring, time))
                if total >= 30000:
                    break

            print("checked:", total)
            print("finite xyz:", finite)
            print("sample:")
            for s in sample:
                print(" ", s)
            return

    print("no message found")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("usage: check_cloud_points.py BAG TOPIC")
        sys.exit(1)
    check(sys.argv[1], sys.argv[2])