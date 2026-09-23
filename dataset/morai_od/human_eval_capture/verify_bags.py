#!/usr/bin/env python3
"""Audit raw human-driving rosbag parts without running a detector."""

import argparse
import json
from pathlib import Path

import rosbag


RATE_GATES = {
    "/sensor/lidar1": 8.0,
    # Historical MR bags nominally configured at 20 Hz measure just below
    # 20.0 over a finite record window.  Use 19 Hz as the integrity floor;
    # the current competition-local profile should normally deliver ~30 Hz.
    "/sensor/camera/front/compressed": 19.0,
    "/sensor/camera/front_left/compressed": 19.0,
    "/sensor/camera/front_right/compressed": 19.0,
    "/sensor/camera/back/compressed": 19.0,
    "/Ego_topic": 15.0,
    "/Object_topic": 15.0,
    "/sensor/gps": 10.0,
    "/sensor/imu": 20.0,
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bags", nargs="+", type=Path)
    parser.add_argument("--json-output", type=Path)
    parser.add_argument("--minimum-duration-sec", type=float, default=20.0)
    args = parser.parse_args()

    counts = {}
    start = None
    end = None
    errors = []
    parts = []
    for path in sorted({item.resolve() for item in args.bags}):
        if not path.is_file() or path.suffix != ".bag":
            errors.append("missing bag: {}".format(path))
            continue
        with rosbag.Bag(str(path), "r") as bag:
            part_start = bag.get_start_time()
            part_end = bag.get_end_time()
            start = part_start if start is None else min(start, part_start)
            end = part_end if end is None else max(end, part_end)
            info = bag.get_type_and_topic_info().topics
            for topic, topic_info in info.items():
                counts[topic] = counts.get(topic, 0) + int(topic_info.message_count)
            parts.append({
                "path": str(path),
                "size_bytes": path.stat().st_size,
                "start": part_start,
                "end": part_end,
            })

    duration = 0.0 if start is None or end is None else max(0.0, end - start)
    if duration < args.minimum_duration_sec:
        errors.append(
            "duration {:.2f}s is below {:.2f}s".format(
                duration, args.minimum_duration_sec
            )
        )
    rates = {}
    for topic, minimum_rate in RATE_GATES.items():
        count = counts.get(topic, 0)
        rate = count / duration if duration > 0.0 else 0.0
        rates[topic] = rate
        if count == 0:
            errors.append("required topic missing: {}".format(topic))
        elif rate < minimum_rate:
            errors.append(
                "{} rate {:.2f} Hz < {:.2f} Hz".format(topic, rate, minimum_rate)
            )

    report = {
        "schema_version": 1,
        "passed": not errors,
        "duration_sec": duration,
        "parts": parts,
        "topic_counts": counts,
        "topic_rates_hz": rates,
        "errors": errors,
    }
    print("duration: {:.2f}s, parts: {}".format(duration, len(parts)))
    for topic in RATE_GATES:
        print("{:<48} {:>8d} {:>7.2f} Hz".format(
            topic, counts.get(topic, 0), rates.get(topic, 0.0)
        ))
    if args.json_output:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print("report: {}".format(args.json_output))
    if errors:
        print("FAIL:")
        for error in errors:
            print("  - {}".format(error))
        raise SystemExit(2)
    print("PASS: raw capture contract is complete")


if __name__ == "__main__":
    main()
