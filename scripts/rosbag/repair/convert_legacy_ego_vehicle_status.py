#!/usr/bin/env python3
"""Convert legacy MORAI EgoVehicleStatus messages to the beta_drive schema.

The input bag is never modified. All topics are copied to a new bag, while
messages on the selected Ego topic are reconstructed with the currently
installed morai_msgs/EgoVehicleStatus class. Legacy-only fields remain in the
original bag and are intentionally omitted from the converted message.
"""

import argparse
import copy
import os

import rosbag
from morai_msgs.msg import EgoVehicleStatus


LEGACY_EGO_MD5 = "861828353f677b0c61ddd2cdf47e398c"
VECTOR_FIELDS = ("acceleration", "position", "velocity")
SCALAR_FIELDS = (
    "unique_id",
    "heading",
    "accel",
    "brake",
    "wheel_angle",
    "lateral_offset",
    "tire_lateral_force_fl",
    "tire_lateral_force_fr",
    "tire_lateral_force_rl",
    "tire_lateral_force_rr",
    "side_slip_angle_fl",
    "side_slip_angle_fr",
    "side_slip_angle_rl",
    "side_slip_angle_rr",
    "tire_cornering_stiffness_fl",
    "tire_cornering_stiffness_fr",
    "tire_cornering_stiffness_rl",
    "tire_cornering_stiffness_rr",
)
LEGACY_ONLY_FIELDS = (
    "angular_velocity",
    "distance_left_lane_boundary",
    "distance_right_lane_boundary",
    "cross_track_error",
)


def convert_ego(message):
    if getattr(message, "_type", "") != EgoVehicleStatus._type:
        raise RuntimeError(
            "selected Ego topic has unexpected type: {}".format(
                getattr(message, "_type", "<unknown>")
            )
        )

    source_md5 = getattr(message, "_md5sum", "")
    supported_md5 = {LEGACY_EGO_MD5, EgoVehicleStatus._md5sum}
    if source_md5 not in supported_md5:
        raise RuntimeError(
            "unsupported EgoVehicleStatus MD5 {} (expected legacy {} or current {})".format(
                source_md5 or "<unknown>",
                LEGACY_EGO_MD5,
                EgoVehicleStatus._md5sum,
            )
        )

    output = EgoVehicleStatus()
    output.header = copy.deepcopy(message.header)

    for field in VECTOR_FIELDS:
        if not hasattr(message, field):
            raise RuntimeError("source EgoVehicleStatus is missing field '{}'".format(field))
        setattr(output, field, copy.deepcopy(getattr(message, field)))

    for field in SCALAR_FIELDS:
        if not hasattr(message, field):
            raise RuntimeError("source EgoVehicleStatus is missing field '{}'".format(field))
        setattr(output, field, getattr(message, field))

    return output, source_md5


def convert_bag(input_path, output_path, ego_topic):
    input_path = os.path.abspath(input_path)
    output_path = os.path.abspath(output_path)
    if input_path == output_path:
        raise RuntimeError("input and output bag paths must be different")
    if not os.path.isfile(input_path):
        raise RuntimeError("input bag does not exist: {}".format(input_path))
    if os.path.exists(output_path):
        raise RuntimeError("output path already exists: {}".format(output_path))

    output_directory = os.path.dirname(output_path)
    os.makedirs(output_directory, exist_ok=True)
    temporary_path = output_path + ".active"
    if os.path.exists(temporary_path):
        raise RuntimeError("temporary output already exists: {}".format(temporary_path))

    total_count = 0
    ego_count = 0
    source_md5s = set()
    try:
        with rosbag.Bag(temporary_path, "w", compression="lz4") as output_bag:
            with rosbag.Bag(input_path, "r") as input_bag:
                for topic, message, bag_time, connection_header in input_bag.read_messages(
                    return_connection_header=True
                ):
                    total_count += 1
                    if topic == ego_topic:
                        message, source_md5 = convert_ego(message)
                        source_md5s.add(source_md5)
                        ego_count += 1
                        # Do not reuse the legacy connection header: it contains
                        # the old message MD5 and definition.
                        output_bag.write(topic, message, bag_time)
                    else:
                        # Preserve connection metadata such as latching for
                        # every topic whose schema is not being converted.
                        output_bag.write(
                            topic,
                            message,
                            bag_time,
                            connection_header=connection_header,
                        )

                    if total_count % 5000 == 0:
                        print(
                            "copied {} messages (converted {} Ego messages)".format(
                                total_count, ego_count
                            )
                        )

        if ego_count == 0:
            raise RuntimeError("no messages found on Ego topic '{}'".format(ego_topic))
        os.replace(temporary_path, output_path)
    except BaseException:
        if os.path.exists(temporary_path):
            os.remove(temporary_path)
        raise

    print("conversion complete")
    print("input:  {}".format(input_path))
    print("output: {}".format(output_path))
    print("total messages copied: {}".format(total_count))
    print("Ego messages converted: {}".format(ego_count))
    print("source Ego MD5: {}".format(", ".join(sorted(source_md5s))))
    print("output Ego MD5: {}".format(EgoVehicleStatus._md5sum))
    if LEGACY_EGO_MD5 in source_md5s:
        print("legacy-only fields omitted: {}".format(", ".join(LEGACY_ONLY_FIELDS)))


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Copy a rosbag while converting legacy /Ego_topic messages to "
            "the installed beta_drive EgoVehicleStatus schema."
        )
    )
    parser.add_argument("input_bag")
    parser.add_argument("output_bag")
    parser.add_argument("--ego-topic", default="/Ego_topic")
    args = parser.parse_args()

    convert_bag(args.input_bag, args.output_bag, args.ego_topic)


if __name__ == "__main__":
    main()
