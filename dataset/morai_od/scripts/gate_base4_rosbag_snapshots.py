#!/usr/bin/env python3
"""Flush rosbag snapshot buffers only for completed dataset capture windows."""

import argparse
import json
import threading
import time
from pathlib import Path

import rospy
from std_msgs.msg import Empty, String


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--marker-topic", default="/dataset/scenario_marker",
        help="JSON String topic emitted by ros_scenario_autocapture_poc.py",
    )
    parser.add_argument(
        "--snapshot-topic", default="/snapshot_trigger",
        help="std_msgs/Empty topic consumed by rosbag record --snapshot",
    )
    parser.add_argument("--event-log", type=Path, required=True)
    parser.add_argument(
        "--bag-prefix", type=Path, required=True,
        help="output prefix passed to rosbag record --output-prefix",
    )
    parser.add_argument(
        "--post-roll-sec", type=float, default=0.25,
        help="wall-time delay after capture_end before flushing the ring buffer",
    )
    parser.add_argument(
        "--max-window-sec", type=float, default=3.40,
        help="warn when a marker window may exceed the configured snapshot buffer",
    )
    parser.add_argument(
        "--save-failed", action="store_true",
        help="also flush scene_failed/scene_interrupted windows for diagnostics",
    )
    return parser.parse_args()


class SnapshotGate:
    def __init__(self, args):
        self.args = args
        self.active = {}
        self.lock = threading.Lock()
        self.snapshot_lock = threading.Lock()
        self.completed = 0
        self.skipped = 0
        self.fatal_error = None
        self.trigger_pub = rospy.Publisher(
            args.snapshot_topic, Empty, queue_size=4
        )
        self.marker_sub = rospy.Subscriber(
            args.marker_topic, String, self.marker_callback, queue_size=50
        )
        args.event_log.parent.mkdir(parents=True, exist_ok=True)

    def append_event(self, payload):
        record = {
            "schema_version": 1,
            "wall_time_sec": time.time(),
            **payload,
        }
        with self.args.event_log.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, sort_keys=True) + "\n")

    def marker_callback(self, message):
        try:
            payload = json.loads(message.data)
        except Exception as error:
            rospy.logerr("Invalid dataset marker JSON: %s", error)
            return

        event = payload.get("event")
        scene_id = payload.get("scene_id")
        if not scene_id:
            rospy.logwarn("Ignoring marker without scene_id: %s", payload)
            return

        if event == "capture_start":
            with self.lock:
                if self.active:
                    rospy.logwarn(
                        "capture_start overlaps active scene(s): %s",
                        sorted(self.active),
                    )
                self.active[scene_id] = payload
            self.append_event(
                {"event": "gate_open", "scene_id": scene_id,
                 "source_marker": payload}
            )
            return

        if event not in ("capture_end", "scene_failed", "scene_interrupted"):
            return

        with self.lock:
            start = self.active.pop(scene_id, None)
        if start is None:
            rospy.logwarn("Terminal marker without active gate: %s %s", event, scene_id)
            return

        start_stamp = float(start.get("ros_time_sec", 0.0))
        end_stamp = float(payload.get("ros_time_sec", 0.0))
        duration = end_stamp - start_stamp if end_stamp and start_stamp else None
        should_save = event == "capture_end" or self.args.save_failed
        if not should_save:
            self.skipped += 1
            self.append_event(
                {"event": "snapshot_skipped", "scene_id": scene_id,
                 "terminal_event": event, "duration_sec": duration}
            )
            rospy.logwarn("Skipping failed scene snapshot: %s (%s)", scene_id, event)
            return

        if duration is not None and duration > self.args.max_window_sec:
            rospy.logwarn(
                "Scene %s marker window %.3fs exceeds configured safe window %.3fs; "
                "capture_start may have fallen out of the rosbag ring buffer",
                scene_id, duration, self.args.max_window_sec,
            )

        timer = threading.Timer(
            self.args.post_roll_sec,
            self.flush_snapshot,
            args=(scene_id, event, duration, payload),
        )
        timer.daemon = True
        timer.start()

    def flush_snapshot(self, scene_id, terminal_event, duration, marker):
        with self.snapshot_lock:
            if self.trigger_pub.get_num_connections() < 1:
                rospy.logerr(
                    "No rosbag snapshot subscriber; scene %s was NOT saved", scene_id
                )
                self.append_event(
                    {"event": "snapshot_trigger_failed", "scene_id": scene_id,
                     "terminal_event": terminal_event, "duration_sec": duration,
                     "reason": "no_snapshot_subscriber"}
                )
                self.fatal_error = "no_snapshot_subscriber"
                rospy.signal_shutdown(self.fatal_error)
                return

            parent = self.args.bag_prefix.parent
            pattern = self.args.bag_prefix.name + "_*.bag"
            before = set(parent.glob(pattern))
            self.trigger_pub.publish(Empty())
            self.completed += 1
            trigger_index = self.completed

            written = None
            deadline = time.monotonic() + 15.0
            while time.monotonic() < deadline and not rospy.is_shutdown():
                created = sorted(
                    set(parent.glob(pattern)) - before,
                    key=lambda path: path.stat().st_mtime,
                )
                if created:
                    written = created[-1]
                    break
                time.sleep(0.05)

            event = {
                "event": "snapshot_written" if written else "snapshot_trigger_failed",
                "scene_id": scene_id,
                "terminal_event": terminal_event,
                "duration_sec": duration,
                "post_roll_sec": self.args.post_roll_sec,
                "trigger_index": trigger_index,
                "source_marker": marker,
            }
            if written:
                event.update(
                    {"bag_path": str(written.resolve()),
                     "bag_size_bytes": written.stat().st_size}
                )
            else:
                event["reason"] = "bag_close_timeout"
            self.append_event(event)

            if written:
                rospy.loginfo(
                    "Saved gated snapshot #%d scene=%s window=%s sec bag=%s",
                    trigger_index, scene_id,
                    "unknown" if duration is None else "{:.3f}".format(duration),
                    written.name,
                )
            else:
                rospy.logerr(
                    "Snapshot trigger #%d for scene=%s did not produce a closed bag",
                    trigger_index, scene_id,
                )
                self.fatal_error = "bag_close_timeout"
                rospy.signal_shutdown(self.fatal_error)


def main():
    args = parse_args()
    if args.post_roll_sec < 0.0:
        raise SystemExit("--post-roll-sec must be >= 0")
    if args.max_window_sec <= 0.0:
        raise SystemExit("--max-window-sec must be > 0")
    rospy.init_node("base4_rosbag_snapshot_gate", anonymous=False)
    gate = SnapshotGate(args)
    rospy.loginfo(
        "Base4 snapshot gate ready: marker=%s trigger=%s post_roll=%.3fs",
        args.marker_topic, args.snapshot_topic, args.post_roll_sec,
    )
    rospy.spin()
    rospy.loginfo(
        "Base4 snapshot gate stopped: triggered=%d skipped=%d active=%s",
        gate.completed, gate.skipped, sorted(gate.active),
    )
    if gate.fatal_error:
        raise SystemExit("fatal snapshot gate error: {}".format(gate.fatal_error))


if __name__ == "__main__":
    main()
