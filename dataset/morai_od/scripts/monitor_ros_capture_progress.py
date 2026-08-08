#!/usr/bin/env python3
"""Display live progress for a long MORAI ROS auto-capture run."""

import argparse
import json
import time
from datetime import datetime
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", type=Path)
    parser.add_argument(
        "--capture-root",
        type=Path,
        default=Path.home() / "morai_cap",
    )
    parser.add_argument(
        "--name-contains",
        default="",
        help="Used to select the newest run when --run-dir is omitted.",
    )
    parser.add_argument("--interval-sec", type=float, default=2.0)
    parser.add_argument("--once", action="store_true")
    return parser.parse_args()


def newest_run(root, name_contains):
    candidates = [
        path
        for path in root.iterdir()
        if path.is_dir() and name_contains in path.name
    ]
    if not candidates:
        raise FileNotFoundError(
            "No capture directory containing {!r} below {}".format(
                name_contains, root
            )
        )
    return max(candidates, key=lambda path: path.stat().st_mtime)


def read_json(path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError):
        return None


def last_log_line(path):
    try:
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except FileNotFoundError:
        return "console.log not created yet"
    for line in reversed(lines):
        if line.strip():
            return line.strip()
    return "console.log is empty"


def format_duration(seconds):
    if seconds is None:
        return "-"
    seconds = max(0, int(round(seconds)))
    hours, remainder = divmod(seconds, 3600)
    minutes, seconds = divmod(remainder, 60)
    return "{:02d}:{:02d}:{:02d}".format(hours, minutes, seconds)


def render(run_dir):
    final_path = run_dir / "ros_capture_poc_report.json"
    partial_path = run_dir / "ros_capture_poc_report.partial.json"
    report = read_json(final_path) or read_json(partial_path)
    now = time.time()

    print("\033[2J\033[H", end="")
    print("MORAI capture monitor", datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
    print("run:", run_dir)
    if report is None:
        print("report: waiting for first completed/failed scene")
        print("last log:", last_log_line(run_dir / "console.log"))
        return False

    scenes = report.get("scenes", [])
    summary = report.get("summary", {})
    requested = summary.get("requested_scenes", len(scenes))
    attempted = summary.get("attempted_scenes", len(scenes))
    completed = summary.get(
        "completed_scenes",
        sum(item.get("status") == "completed" for item in scenes),
    )
    failed = summary.get(
        "failed_scenes",
        sum(item.get("status") == "failed" for item in scenes),
    )
    frames = summary.get(
        "completed_captures",
        sum(len(item.get("captures", [])) for item in scenes),
    )
    started = report.get("started_unix")
    elapsed = now - started if started else None
    eta = None
    if elapsed is not None and attempted and requested > attempted:
        eta = elapsed / attempted * (requested - attempted)
    percent = 100.0 * attempted / requested if requested else 0.0

    print(
        "scenes: {}/{} attempted ({:.1f}%) | completed={} | failed={}".format(
            attempted, requested, percent, completed, failed
        )
    )
    print("verified frames:", frames)
    print("elapsed:", format_duration(elapsed), "| ETA:", format_duration(eta))
    if scenes:
        latest = scenes[-1]
        print(
            "latest scene: {} [{}]".format(
                latest.get("id", "?"), latest.get("status", "?")
            )
        )
    failed_ids = [
        item.get("id", "?")
        for item in scenes
        if item.get("status") == "failed"
    ]
    if failed_ids:
        print("recent failures:", ", ".join(failed_ids[-8:]))
    print("last log:", last_log_line(run_dir / "console.log"))
    return final_path.is_file()


def main():
    args = parse_args()
    run_dir = (
        args.run_dir.expanduser().resolve()
        if args.run_dir
        else newest_run(
            args.capture_root.expanduser().resolve(), args.name_contains
        )
    )
    while True:
        finished = render(run_dir)
        if args.once or finished:
            break
        time.sleep(max(0.2, args.interval_sec))


if __name__ == "__main__":
    main()
