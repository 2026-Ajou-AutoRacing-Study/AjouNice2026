#!/usr/bin/env python3
"""Audit Base3-Online rosbag topic, marker, and camera/LiDAR timing contracts."""

import argparse
import bisect
import json
import math
from collections import Counter, defaultdict
from pathlib import Path

import rosbag


MARKER_TOPIC = "/dataset/scenario_marker"
LIDAR_TOPIC = "/sensor/lidar1"
CAMERA_TOPICS = (
    "/sensor/camera/front/compressed",
    "/sensor/camera/front_left/compressed",
    "/sensor/camera/front_right/compressed",
    "/sensor/camera/back/compressed",
)
STATE_TOPICS = ("/Ego_topic", "/Object_topic")
REQUIRED_TOPICS = (LIDAR_TOPIC,) + CAMERA_TOPICS + STATE_TOPICS + (MARKER_TOPIC,)


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    lower = int(math.floor(index))
    upper = int(math.ceil(index))
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (index - lower)


def message_stamp(message, bag_time):
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    value = stamp.to_sec() if stamp is not None else 0.0
    return value if value > 0.0 else bag_time.to_sec()


def build_capture_windows(markers):
    active = {}
    windows = []
    errors = []
    for marker in sorted(markers, key=lambda item: item[0]):
        stamp, payload = marker
        scene = payload.get("scene_id")
        event = payload.get("event")
        if not scene:
            errors.append("marker without scene_id at {:.6f}".format(stamp))
            continue
        if event == "capture_start":
            if active:
                errors.append("capture_start overlaps active scenes {}".format(sorted(active)))
            if scene in active:
                errors.append("duplicate capture_start for {}".format(scene))
            active[scene] = (stamp, payload)
        elif event in ("capture_end", "scene_failed", "scene_interrupted"):
            if scene not in active:
                if event == "capture_end":
                    errors.append(
                        "capture_end without capture_start for {}".format(scene)
                    )
                continue
            start, start_payload = active.pop(scene)
            if stamp <= start:
                errors.append("non-positive capture window for {}".format(scene))
            else:
                windows.append(
                    {
                        "scene_id": scene,
                        "start": start,
                        "end": stamp,
                        "mission_family": start_payload.get("mission_family"),
                        "terminal_event": event,
                        "terminal_status": payload.get("status"),
                        "terminal_error": payload.get("error"),
                    }
                )
    for scene in sorted(active):
        errors.append("capture_start without terminal marker for {}".format(scene))
    windows.sort(key=lambda item: item["start"])
    for previous, current in zip(windows, windows[1:]):
        if current["start"] < previous["end"]:
            errors.append(
                "capture windows overlap: {} and {}".format(
                    previous["scene_id"], current["scene_id"]
                )
            )
    return windows, errors


def inside_windows(stamp, windows):
    return any(window["start"] <= stamp <= window["end"] for window in windows)


def nearest_delta(stamp, ordered_stamps):
    if not ordered_stamps:
        return None
    index = bisect.bisect_left(ordered_stamps, stamp)
    candidates = []
    if index < len(ordered_stamps):
        candidates.append(abs(ordered_stamps[index] - stamp))
    if index:
        candidates.append(abs(ordered_stamps[index - 1] - stamp))
    return min(candidates)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bags", nargs="+", type=Path)
    parser.add_argument("--max-camera-age-sec", type=float, default=0.10)
    parser.add_argument(
        "--max-backward-jitter-sec", type=float, default=0.005,
        help="small rosbridge delivery reordering at or below this value is a warning",
    )
    parser.add_argument(
        "--max-camera-over-limit-fraction", type=float, default=0.001,
        help="raw-frame outlier fraction allowed as a warning; selected samples are checked strictly by the converter",
    )
    parser.add_argument("--json-output", type=Path)
    return parser.parse_args()


def main():
    args = parse_args()
    counts = Counter()
    topic_types = {}
    stamps = defaultdict(list)
    record_times = defaultdict(list)
    markers = []
    nonmonotonic_global = Counter()
    last_header_stamp = {}
    bag_start = None
    bag_end = None

    for path in args.bags:
        if not path.is_file():
            raise SystemExit("bag not found: {}".format(path))
        with rosbag.Bag(str(path), "r") as bag:
            info = bag.get_type_and_topic_info().topics
            for topic, value in info.items():
                topic_types.setdefault(topic, value.msg_type)
            for topic, message, bag_time in bag.read_messages(
                topics=list(REQUIRED_TOPICS)
            ):
                current_bag_time = bag_time.to_sec()
                bag_start = current_bag_time if bag_start is None else min(bag_start, current_bag_time)
                bag_end = current_bag_time if bag_end is None else max(bag_end, current_bag_time)
                counts[topic] += 1
                record_times[topic].append(current_bag_time)
                if topic == MARKER_TOPIC:
                    try:
                        payload = json.loads(message.data)
                    except Exception as error:
                        raise SystemExit("invalid marker JSON at {:.6f}: {}".format(current_bag_time, error))
                    markers.append((current_bag_time, payload))
                    continue
                stamp = message_stamp(message, bag_time)
                stamps[topic].append(stamp)
                previous = last_header_stamp.get(topic)
                if previous is not None and stamp < previous:
                    nonmonotonic_global[topic] += 1
                last_header_stamp[topic] = stamp

    windows, marker_errors = build_capture_windows(markers)
    per_window = []
    camera_ages = defaultdict(list)
    nonmonotonic_capture = Counter()
    nonmonotonic_capture_details = []
    for window in windows:
        row = dict(window)
        row["duration_sec"] = window["end"] - window["start"]
        row["counts"] = {}
        row["rates_hz"] = {}
        camera_window_stamps = {
            topic: [
                stamp
                for index, stamp in enumerate(stamps[topic])
                if window["start"]
                <= record_times[topic][index]
                <= window["end"]
            ]
            for topic in CAMERA_TOPICS
        }
        for topic in (LIDAR_TOPIC,) + CAMERA_TOPICS + STATE_TOPICS:
            selected = [
                index
                for index, record_time in enumerate(record_times[topic])
                if window["start"] <= record_time <= window["end"]
            ]
            row["counts"][topic] = len(selected)
            row["rates_hz"][topic] = (
                len(selected) / row["duration_sec"]
                if row["duration_sec"] > 0.0
                else 0.0
            )
            previous = None
            for index in selected:
                stamp = stamps[topic][index]
                if previous is not None and stamp < previous:
                    nonmonotonic_capture[topic] += 1
                    nonmonotonic_capture_details.append(
                        {
                            "scene_id": window["scene_id"],
                            "topic": topic,
                            "previous_stamp": previous,
                            "current_stamp": stamp,
                            "backward_ms": (previous - stamp) * 1000.0,
                        }
                    )
                previous = stamp
        for lidar_index, lidar_stamp in enumerate(stamps[LIDAR_TOPIC]):
            lidar_record_time = record_times[LIDAR_TOPIC][lidar_index]
            if window["start"] <= lidar_record_time <= window["end"]:
                for camera in CAMERA_TOPICS:
                    delta = nearest_delta(lidar_stamp, camera_window_stamps[camera])
                    if delta is not None:
                        camera_ages[camera].append(delta)
        per_window.append(row)

    errors = list(marker_errors)
    for topic in REQUIRED_TOPICS:
        if counts[topic] == 0:
            errors.append("required topic has zero messages: {}".format(topic))
    for window in per_window:
        for topic, count in window["counts"].items():
            if count == 0:
                errors.append("{} has no {} messages".format(window["scene_id"], topic))
    for topic, value in nonmonotonic_capture.items():
        serious = [
            item for item in nonmonotonic_capture_details
            if item["topic"] == topic
            and item["backward_ms"] > args.max_backward_jitter_sec * 1000.0
        ]
        if serious:
            errors.append(
                "{} has {} header-stamp reversals above {:.1f} ms inside capture windows".format(
                    topic, len(serious), args.max_backward_jitter_sec * 1000.0
                )
            )

    warnings = []
    for window in per_window:
        if window["terminal_event"] != "capture_end":
            warnings.append(
                "{} closed by {} after online capture started: {}".format(
                    window["scene_id"],
                    window["terminal_event"],
                    window.get("terminal_error"),
                )
            )
    for topic, value in nonmonotonic_global.items():
        if value and not nonmonotonic_capture[topic]:
            warnings.append(
                "{} has {} non-monotonic header stamps outside capture windows".format(
                    topic, value
                )
            )
    for topic, value in nonmonotonic_capture.items():
        if value and not any(
            item["topic"] == topic
            and item["backward_ms"] > args.max_backward_jitter_sec * 1000.0
            for item in nonmonotonic_capture_details
        ):
            warnings.append(
                "{} has {} small (<= {:.1f} ms) header-stamp reversals; extraction sorts by stamp".format(
                    topic, value, args.max_backward_jitter_sec * 1000.0
                )
            )

    timing = {}
    for topic in CAMERA_TOPICS:
        values = camera_ages[topic]
        timing[topic] = {
            "samples": len(values),
            "p50_ms": None if not values else percentile(values, 0.50) * 1000.0,
            "p95_ms": None if not values else percentile(values, 0.95) * 1000.0,
            "max_ms": None if not values else max(values) * 1000.0,
            "over_limit": sum(value > args.max_camera_age_sec for value in values),
        }
        if values and max(values) > args.max_camera_age_sec:
            fraction = timing[topic]["over_limit"] / len(values)
            message = "{} nearest-frame age exceeds {:.0f} ms for {}/{} frames ({:.3%}, max {:.1f} ms)".format(
                topic, args.max_camera_age_sec * 1000.0,
                timing[topic]["over_limit"], len(values), fraction,
                max(values) * 1000.0,
            )
            if fraction <= args.max_camera_over_limit_fraction:
                warnings.append(message + "; selected samples remain strict")
            else:
                errors.append(message)

    report = {
        "schema_version": 2,
        "bags": [str(path.resolve()) for path in args.bags],
        "duration_sec": None if bag_start is None else bag_end - bag_start,
        "topic_counts": dict(sorted(counts.items())),
        "topic_types": dict(sorted(topic_types.items())),
        "marker_count": len(markers),
        "capture_windows": per_window,
        "camera_nearest_lidar_age": timing,
        "nonmonotonic_capture_details": nonmonotonic_capture_details,
        "warnings": warnings,
        "errors": errors,
        "passed": not errors,
    }
    rendered = json.dumps(report, ensure_ascii=False, indent=2)
    print(rendered)
    if args.json_output:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(rendered + "\n", encoding="utf-8")
    raise SystemExit(0 if report["passed"] else 2)


if __name__ == "__main__":
    main()
