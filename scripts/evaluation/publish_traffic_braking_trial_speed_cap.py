#!/usr/bin/env python3
"""Owned evaluation-only speed ceiling; never commands throttle/brake/gear.

Uses a unique internal velocity-limit sender and clears only that sender on
normal/SIGINT/SIGTERM teardown. SIGKILL cannot guarantee cleanup; the trial
owner must stop its complete planner stack on helper failure.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import threading
import time


def validate_cap(value):
    if type(value) not in (int,float) or not math.isfinite(value) or not 1 <= value <= 20:
        raise ValueError('Evaluation speed cap must be within 1..20 m/s')
    return float(value)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--max-velocity-mps', type=float, required=True)
    parser.add_argument('--max-seconds', type=float, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    cap = validate_cap(args.max_velocity_mps)
    if not math.isfinite(args.max_seconds) or not 1 <= args.max_seconds <= 1800:
        raise ValueError('Bounded lifetime required')
    args.output.resolve().relative_to(Path(__file__).resolve().parents[2]/'artifacts/evaluation')
    args.output.mkdir(parents=True, exist_ok=False)
    import rospy
    from autoware_internal_planning_msgs.msg import VelocityLimit, VelocityLimitClearCommand
    rospy.init_node('traffic_braking_trial_speed_cap', anonymous=True, disable_signals=True)
    sender = 'traffic_braking_trial_%d' % os.getpid()
    pub = rospy.Publisher('/planning/scenario_planning/max_velocity_candidates', VelocityLimit, queue_size=1)
    clear = rospy.Publisher('/planning/scenario_planning/clear_velocity_limit', VelocityLimitClearCommand, queue_size=1)
    stop = threading.Event()
    for sig in (signal.SIGINT,signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    (args.output/'metadata.json').write_text(json.dumps(dict(development_only=True,
        sender=sender, max_velocity_mps=cap, direct_vehicle_commands=False),indent=2))
    deadline = time.monotonic()+args.max_seconds
    try:
        while not stop.is_set() and not rospy.is_shutdown():
            if time.monotonic() >= deadline:
                raise RuntimeError('Evaluation cap lifetime exceeded; owning trial must stop')
            message = VelocityLimit()
            now = rospy.Time.now()
            message.stamp.sec, message.stamp.nanosec = now.secs, now.nsecs
            message.sender, message.max_velocity = sender, cap
            message.use_constraints = False
            pub.publish(message)
            stop.wait(.1)
    finally:
        for _ in range(5):
            message = VelocityLimitClearCommand()
            now = rospy.Time.now()
            message.stamp.sec, message.stamp.nanosec = now.secs, now.nsecs
            message.sender, message.command = sender, True
            clear.publish(message)
            time.sleep(.08)
        (args.output/'clear_requested.json').write_text(json.dumps(dict(sender=sender,
            own_sender_only=True, selector_acknowledgment=False)))


if __name__ == '__main__':
    main()
