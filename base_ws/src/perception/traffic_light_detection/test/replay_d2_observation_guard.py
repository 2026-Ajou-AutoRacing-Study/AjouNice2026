"""Offline replay of recorded normalized labels + provenance, not camera inference.

Usage: python3 replay_d2_observation_guard.py BAG
Never publishes to ROS. Prints output transitions for audit.
"""
import os
import sys
import rosbag

PACKAGE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PACKAGE, "scripts"))
from signal_phase_policy import SignalPhasePolicy
from signal_smoother import SignalSmoother


def replay(path):
    streams = {"debug_raw_label": [], "debug_policy_reason": []}
    with rosbag.Bag(path) as bag:
        start = bag.get_start_time()
        for topic, msg, stamp in bag.read_messages(
            topics=["/traffic_light_node/" + name for name in streams]
        ):
            streams[topic.rsplit("/", 1)[1]].append((stamp.to_sec(), msg.data))
    raw, reasons = streams.values()
    assert len(raw) == len(reasons) and raw, "Missing per-frame debug evidence"
    assert max(abs(a[0] - b[0]) for a, b in zip(raw, reasons)) < .05
    policy = SignalPhasePolicy.from_yaml(os.path.join(PACKAGE, "config", "traffic_signal_profiles.yaml"))
    smoother = SignalSmoother(7, 4)
    lookup = {"red_circle": ("red", "circle", None),
              "yellow_circle": ("yellow", "circle", None),
              "green_circle": ("green", "circle", None),
              "green_arrow_left": ("green", "arrow", True)}
    previous = None
    for (stamp, label), (_, reason) in zip(raw, reasons):
        lamps = [lookup[token] for token in label.split("+") if token in lookup]
        output, _, decision = policy.process_normalized(
            lamps, reason.split("|")[0], 201326, stamp, 1.0, smoother)
        if output != previous:
            print(round(stamp - start, 3), output, decision)
            previous = output


if __name__ == "__main__":
    replay(sys.argv[1])
