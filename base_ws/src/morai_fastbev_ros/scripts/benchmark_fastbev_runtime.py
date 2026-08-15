#!/usr/bin/env python3
"""Measure synchronized camera input and Fast-BEV ROS output rates/timings."""

import argparse
import json
import statistics
import time
from pathlib import Path

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import CompressedImage


def _summary(values):
    if not values:
        return {"count": 0}
    ordered = sorted(values)
    return {
        "count": len(values),
        "mean": statistics.fmean(values),
        "p50": ordered[int(0.50 * (len(ordered) - 1))],
        "p95": ordered[int(0.95 * (len(ordered) - 1))],
        "max": ordered[-1],
    }


class Monitor:
    def __init__(self):
        self.input_wall = []
        self.input_stamp = []
        self.output_wall = []
        self.output_stamp = []
        self.timings = {
            "camera_span_ms": [],
            "queue_delay_ms": [],
            "preprocess_ms": [],
            "inference_ms": [],
            "total_ms": [],
        }
        rospy.Subscriber(
            "/sensor/camera/front/compressed", CompressedImage,
            self._input, queue_size=200)
        rospy.Subscriber(
            "/fastbev/diagnostics", DiagnosticArray,
            self._output, queue_size=200)

    def _input(self, message):
        self.input_wall.append(time.perf_counter())
        self.input_stamp.append(message.header.stamp.to_sec())

    def _output(self, message):
        if not message.status:
            return
        values = {item.key: item.value for item in message.status[0].values}
        if not all(key in values for key in self.timings):
            rospy.logwarn_throttle(
                1.0, "Fast-BEV diagnostics are missing runtime timing keys")
            return
        self.output_wall.append(time.perf_counter())
        self.output_stamp.append(message.header.stamp.to_sec())
        for key in self.timings:
            self.timings[key].append(float(values[key]))

    @staticmethod
    def _rate(wall, stamps):
        if len(wall) < 2:
            return {"count": len(wall), "wall_hz": 0.0, "stamp_hz": 0.0}
        wall_span = wall[-1] - wall[0]
        stamp_span = stamps[-1] - stamps[0]
        return {
            "count": len(wall),
            "wall_hz": (len(wall) - 1) / wall_span if wall_span > 0 else 0.0,
            "stamp_hz": (len(stamps) - 1) / stamp_span if stamp_span > 0 else 0.0,
            "wall_span_sec": wall_span,
            "stamp_span_sec": stamp_span,
        }

    def report(self):
        return {
            "input": self._rate(self.input_wall, self.input_stamp),
            "output": self._rate(self.output_wall, self.output_stamp),
            "timings_ms": {
                key: _summary(values) for key, values in self.timings.items()
            },
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("fastbev_runtime_benchmark", anonymous=True)
    monitor = Monitor()
    deadline = time.monotonic() + args.duration
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        time.sleep(0.1)
    report = monitor.report()
    text = json.dumps(report, indent=2, sort_keys=True)
    print(text)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
