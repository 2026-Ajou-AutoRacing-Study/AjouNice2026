#!/usr/bin/env python3
"""Publish fixed traffic-light tokens in place of traffic_light_node (planner-only check).

Kill traffic_light_node first. Watch /planning/planning_factors/traffic_light:
published -> STOP, silent -> GO.

  inject_signal_tokens.py D-2 RC+GLA                # hold one phase
  inject_signal_tokens.py A GC:10,YC:3,RC:20        # phase sequence, seconds each (repeats)
  inject_signal_tokens.py C RC --group-id 0         # wrong/zero group id (fail-open path 3)
  inject_signal_tokens.py B RC --stamp-age 1.5      # stale stamp (fail-open path 2)
  inject_signal_tokens.py D-2 UNKNOWN               # all-UNKNOWN lamp (fail-open path 3)
  inject_signal_tokens.py A RC --stop-after 5       # stop publishing after 5 s (path 2)

Tokens: RC YC GC GLA GRA UNKNOWN, joined with '+'. OFF publishes no lamps.
"""
import argparse

import rospy
from autoware_perception_msgs.msg import (
    TrafficLightElement,
    TrafficLightGroup,
    TrafficLightGroupArray,
)
from builtin_interfaces.msg import Time

SECTIONS = {"A": 201351, "B": 201365, "C": 201290, "D-2": 201326, "D-1": 201397}
E = TrafficLightElement
TOKENS = {
    "RC": (E.RED, E.CIRCLE),
    "YC": (E.AMBER, E.CIRCLE),
    "GC": (E.GREEN, E.CIRCLE),
    "GLA": (E.GREEN, E.LEFT_ARROW),
    "GRA": (E.GREEN, E.RIGHT_ARROW),
    "UNKNOWN": (E.UNKNOWN, E.UNKNOWN),
}


def parse_phases(text):
    phases = []
    for part in text.split(","):
        lamps, _, seconds = part.partition(":")
        tokens = [] if lamps == "OFF" else lamps.split("+")
        unknown = set(tokens) - set(TOKENS)
        if unknown:
            raise SystemExit(f"unknown tokens: {sorted(unknown)}")
        phases.append((tokens, float(seconds) if seconds else float("inf")))
    return phases


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("section", help="A, B, C, D-2, D-1 or a regulatory element id")
    parser.add_argument("phases", help="TOKENS or TOKENS:SEC,TOKENS:SEC,...")
    parser.add_argument("--group-id", type=int, help="override the published group id")
    parser.add_argument("--stamp-age", type=float, default=0.0, help="seconds subtracted from stamp")
    parser.add_argument("--stop-after", type=float, help="stop publishing after this many seconds")
    parser.add_argument("--rate", type=float, default=10.0)
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("inject_signal_tokens", anonymous=True)
    group_id = args.group_id
    if group_id is None:
        group_id = SECTIONS.get(args.section) or int(args.section)
    phases = parse_phases(args.phases)
    pub = rospy.Publisher(
        "/perception/traffic_light_recognition/traffic_signals",
        TrafficLightGroupArray,
        queue_size=1,
    )
    rate = rospy.Rate(args.rate)
    start = rospy.Time.now()
    index, phase_start, last_label = 0, start, None
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        if args.stop_after is not None and (now - start).to_sec() > args.stop_after:
            rospy.logwarn_once("[inject] stopped publishing")
            rate.sleep()
            continue
        tokens, seconds = phases[index]
        if (now - phase_start).to_sec() >= seconds:
            index, phase_start = (index + 1) % len(phases), now
            tokens, _ = phases[index]
        label = "+".join(tokens) or "OFF"
        if label != last_label:
            rospy.loginfo(f"[inject] t={(now - start).to_sec():.1f}s group={group_id} {label}")
            last_label = label
        stamp = now - rospy.Duration(args.stamp_age)
        group = TrafficLightGroup(traffic_light_group_id=group_id)
        for token in tokens:
            color, shape = TOKENS[token]
            group.elements.append(
                TrafficLightElement(color=color, shape=shape, status=E.SOLID_ON, confidence=1.0)
            )
        pub.publish(
            TrafficLightGroupArray(
                stamp=Time(sec=stamp.secs, nanosec=stamp.nsecs), traffic_light_groups=[group]
            )
        )
        rate.sleep()


if __name__ == "__main__":
    main()
