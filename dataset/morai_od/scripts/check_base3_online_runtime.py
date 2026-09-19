#!/usr/bin/env python3
"""Validate Base3-Online sensor and oracle-state publisher contracts."""

import argparse
import sys
import xmlrpc.client

import rosgraph
import rospy
import rostopic


SENSOR_REQUIRED = {
    "/sensor/lidar1": ("lidar_nodelet_manager",),
    "/sensor/camera/front/compressed": ("morai_sensor_udp_receiver",),
    "/sensor/camera/front_left/compressed": ("morai_sensor_udp_receiver",),
    "/sensor/camera/front_right/compressed": ("morai_sensor_udp_receiver",),
    "/sensor/camera/back/compressed": ("morai_sensor_udp_receiver",),
}
STATE_TOPICS = ("/Ego_topic", "/Object_topic")


def system_publishers(master_uri=None):
    master_uri = master_uri or rosgraph.get_master_uri()
    master = xmlrpc.client.ServerProxy(master_uri)
    code, message, state = master.getSystemState("/base3_online_contract")
    if code != 1:
        raise RuntimeError(message)
    publishers, _, _ = state
    return {topic: sorted(nodes) for topic, nodes in publishers}


def audit_publishers(publishers, gt_transport):
    errors = []
    lines = []
    required = dict(SENSOR_REQUIRED)
    if gt_transport == "udp":
        required.update(
            {topic: ("morai_ego_udp_receiver",) for topic in STATE_TOPICS}
        )
    elif gt_transport == "ros":
        required.update({topic: ("rosbridge_websocket",) for topic in STATE_TOPICS})
    else:
        required.update(
            {
                topic: ("morai_ego_udp_receiver", "rosbridge_websocket")
                for topic in STATE_TOPICS
            }
        )
    for topic, expected_fragments in required.items():
        nodes = publishers.get(topic, [])
        lines.append("{} <- {}".format(topic, ", ".join(nodes) or "<none>"))
        if len(nodes) != 1:
            errors.append("{} must have exactly one publisher, got {}".format(topic, nodes))
            continue
        if not any(fragment in nodes[0] for fragment in expected_fragments):
            errors.append(
                "{} publisher does not match {} transport: {}".format(
                    topic, gt_transport, nodes[0]
                )
            )
    return lines, errors


def audit_live_messages(topics, timeout_sec):
    errors = []
    lines = []
    rospy.init_node("base3_online_contract", anonymous=True)
    for topic in topics:
        topic_class, _real_topic, _eval_fn = rostopic.get_topic_class(
            topic, blocking=False
        )
        if topic_class is None:
            errors.append("{} message type is unavailable".format(topic))
            continue
        try:
            message = rospy.wait_for_message(
                topic, topic_class, timeout=timeout_sec
            )
        except Exception as error:
            errors.append(
                "{} has a publisher but no live message within {:.1f}s: {}".format(
                    topic, timeout_sec, error
                )
            )
            continue
        header = getattr(message, "header", None)
        stamp = getattr(header, "stamp", None)
        lines.append(
            "{} live (stamp={})".format(
                topic,
                stamp.to_sec() if stamp is not None else "n/a",
            )
        )
    return lines, errors


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--gt-transport",
        choices=("udp", "ros", "either"),
        default="udp",
        help="Expected publisher path for Ego/Object oracle state.",
    )
    parser.add_argument("--message-timeout-sec", type=float, default=3.0)
    return parser.parse_args()


def main():
    args = parse_args()
    try:
        publishers = system_publishers()
        lines, errors = audit_publishers(publishers, args.gt_transport)
        live_lines, live_errors = audit_live_messages(
            tuple(SENSOR_REQUIRED) + STATE_TOPICS,
            args.message_timeout_sec,
        )
        lines.extend(live_lines)
        errors.extend(live_errors)
    except Exception as error:
        raise SystemExit("ROS master/runtime check failed: {}".format(error))
    print("Base3-Online publisher contract:")
    print("\n".join("  " + line for line in lines))
    if errors:
        print("ERROR:", file=sys.stderr)
        print("\n".join("  - " + error for error in errors), file=sys.stderr)
        raise SystemExit(2)
    print(
        "PASS: sensors are UDP-derived and Ego/Object match {} transport"
        .format(args.gt_transport)
    )


if __name__ == "__main__":
    main()
