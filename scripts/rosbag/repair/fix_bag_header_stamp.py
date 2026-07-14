#!/usr/bin/env python3
import argparse
import copy
import os

import rosbag


TOPICS_TO_FIX = {
    "/sensor/imu",
    "/sensor/lidar1",
    "/sensor/lidar2",
    "/Ego_topic",
    "/Object_topic",
    "/sensor/gps",
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input_bag")
    parser.add_argument("output_bag")
    parser.add_argument(
        "--all-header-topics",
        action="store_true",
        help="Fix header.stamp for every message that has a header.",
    )
    args = parser.parse_args()

    if os.path.abspath(args.input_bag) == os.path.abspath(args.output_bag):
        raise RuntimeError("input_bag and output_bag must be different")

    count_total = 0
    count_fixed = 0

    with rosbag.Bag(args.output_bag, "w", compression="lz4") as outbag:
        with rosbag.Bag(args.input_bag, "r") as inbag:
            for topic, msg, t in inbag.read_messages():
                count_total += 1

                should_fix = topic in TOPICS_TO_FIX
                if args.all_header_topics and hasattr(msg, "header"):
                    should_fix = True

                if should_fix and hasattr(msg, "header"):
                    msg = copy.deepcopy(msg)
                    msg.header.stamp = t
                    count_fixed += 1

                outbag.write(topic, msg, t)

    print(f"input:  {args.input_bag}")
    print(f"output: {args.output_bag}")
    print(f"total messages: {count_total}")
    print(f"fixed header.stamp: {count_fixed}")


if __name__ == "__main__":
    main()