#!/usr/bin/env python3
"""Measure synchronized camera input and Fast-BEV ROS output rates/timings."""

import argparse
import json
import math
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

    def percentile(percent):
        # Match NumPy's default linear percentile interpolation, which is also
        # used by the PointPainting benchmark report.
        rank = (len(ordered) - 1) * (percent / 100.0)
        lower = int(math.floor(rank))
        upper = int(math.ceil(rank))
        if lower == upper:
            return ordered[lower]
        weight = rank - lower
        return ordered[lower] * (1.0 - weight) + ordered[upper] * weight

    return {
        "count": len(values),
        "mean": statistics.fmean(values),
        "p50": percentile(50),
        "p90": percentile(90),
        "p95": percentile(95),
        "p99": percentile(99),
        "max": ordered[-1],
    }


class Monitor:
    def __init__(self, exclude_outputs=0):
        self.exclude_outputs = max(0, int(exclude_outputs))
        self.input_wall = []
        self.input_stamp = []
        self.output_wall = []
        self.output_stamp = []
        self.timings = {
            "camera_span_ms": [],
            "queue_delay_ms": [],
            "decode_ms": [],
            "transform_ms": [],
            "inference_queue_ms": [],
            "h2d_ms": [],
            "preprocess_ms": [],
            "inference_ms": [],
            "publish_ms": [],
            "total_ms": [],
            "latency_ms": [],
        }
        self.counter_start = None
        self.counter_end = None
        self.counter_history = []
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
        counters = {
            key: int(values[key])
            for key in (
                "received_sets",
                "processed_sets",
                "dropped_raw_sets",
                "dropped_prepared_sets",
                "skipped_sets",
            )
            if key in values
        }
        if counters:
            if self.counter_start is None:
                self.counter_start = counters
            self.counter_end = counters
            self.counter_history.append(counters)

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
        excluded = min(self.exclude_outputs, len(self.output_wall))
        timing_values = {
            key: values[excluded:] for key, values in self.timings.items()
        }
        steady_counter_start = self.counter_start
        if self.counter_history:
            steady_counter_start = self.counter_history[
                min(excluded, len(self.counter_history) - 1)
            ]
        counter_delta = {}
        if steady_counter_start is not None and self.counter_end is not None:
            counter_delta = {
                key: self.counter_end.get(key, 0)
                - steady_counter_start.get(key, 0)
                for key in self.counter_end
            }
        return {
            "schema_version": 2,
            "input": self._rate(self.input_wall, self.input_stamp),
            "output": self._rate(self.output_wall, self.output_stamp),
            "counters": {
                "first": self.counter_start,
                "steady_first": steady_counter_start,
                "last": self.counter_end,
                "steady_state_delta": counter_delta,
            },
            "excluded_output_frames": excluded,
            "analyzed_output_frames": len(self.output_wall) - excluded,
            "timings_ms": {
                key: _summary(values) for key, values in timing_values.items()
            },
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--exclude-output-frames", type=int, default=10)
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("fastbev_runtime_benchmark", anonymous=True)
    monitor = Monitor(exclude_outputs=args.exclude_output_frames)
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
