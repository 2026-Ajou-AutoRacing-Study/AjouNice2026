#!/usr/bin/env python3
"""Read-only terminal dashboard for long MORAI SaveSensorData captures.

The monitor is deliberately a sidecar.  It never imports the capture runner,
subscribes to ROS topics, scans sensor directories, or writes into the run
directory.  It polls the small atomic summary and only consumes bytes appended
to JSONL/console files after the previous refresh.
"""

import argparse
import json
import os
import re
import shutil
import sys
import time
from collections import Counter, deque
from datetime import datetime
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_CAPTURE_ROOT = Path.home() / "morai_cap"
MAX_JSON_BYTES = 1024 * 1024
MISSION_DYNAMIC = {
    "roundabout_circulation",
    "highway_merge_cruise",
    "highway_cruise_overtake",
    "highway_lane_change_overtake",
    "crosswalk_pedestrian",
}
STATIC_TEMPORAL = {
    "static_obstacle",
    "static_highway_obstacle",
    "stopped_vehicle",
    "standing_pedestrian",
    "realistic_mixed_static",
    "sparse_background",
}
HARD_TRANSITION = {
    "roundabout_sample_spawn",
    "highway_merge_sample_spawn",
    "gps_shadow_obstacle",
}


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path)
    parser.add_argument("--manifest", type=Path)
    parser.add_argument(
        "--scenario",
        action="append",
        default=[],
        help="Selected scene ID/file_name. Repeat for subset runs.",
    )
    parser.add_argument("--capture-root", type=Path, default=DEFAULT_CAPTURE_ROOT)
    parser.add_argument("--name-contains", default="")
    parser.add_argument("--interval-sec", type=float, default=2.0)
    parser.add_argument("--stall-warning-sec", type=float, default=60.0)
    parser.add_argument("--disk-warning-gb", type=float, default=100.0)
    parser.add_argument("--failure-rate-warning", type=float, default=0.10)
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--no-clear", action="store_true")
    parser.add_argument("--no-color", action="store_true")
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print one compact JSON snapshot; implies --once and --no-clear.",
    )
    return parser.parse_args()


def newest_run(root, name_contains):
    candidates = [
        path
        for path in root.expanduser().resolve().iterdir()
        if path.is_dir() and name_contains in path.name
    ]
    if not candidates:
        raise FileNotFoundError(
            "No capture directory containing {!r} below {}".format(
                name_contains, root
            )
        )
    return max(candidates, key=lambda path: path.stat().st_mtime)


def read_small_json(path):
    try:
        if path.stat().st_size > MAX_JSON_BYTES:
            return None
        with path.open("r", encoding="utf-8") as stream:
            return json.load(stream)
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return None


class AppendedLineReader:
    """Read a growing text file once, then only consume appended bytes."""

    def __init__(
        self,
        path,
        max_rows=4096,
        max_line_bytes=1024 * 1024,
        initial_tail_bytes=None,
    ):
        self.path = Path(path)
        self.offset = 0
        self.identity = None
        self.rows = deque(maxlen=max_rows)
        self.max_line_bytes = max_line_bytes
        self.initial_tail_bytes = initial_tail_bytes
        self.discard_partial_line = False

    def update(self, decoder=None):
        try:
            stat = self.path.stat()
        except OSError:
            return []
        identity = (stat.st_dev, stat.st_ino)
        if identity != self.identity or stat.st_size < self.offset:
            self.identity = identity
            self.offset = (
                max(0, stat.st_size - self.initial_tail_bytes)
                if self.initial_tail_bytes is not None
                else 0
            )
            self.discard_partial_line = self.offset > 0
            self.rows.clear()
        appended = []
        try:
            with self.path.open("rb") as stream:
                stream.seek(self.offset)
                if self.discard_partial_line:
                    stream.readline(self.max_line_bytes + 1)
                    self.discard_partial_line = False
                while True:
                    raw = stream.readline(self.max_line_bytes + 1)
                    if not raw:
                        break
                    if len(raw) > self.max_line_bytes and not raw.endswith(b"\n"):
                        while raw and not raw.endswith(b"\n"):
                            raw = stream.readline(self.max_line_bytes + 1)
                        continue
                    text = raw.decode("utf-8", errors="replace").strip()
                    if not text:
                        continue
                    try:
                        value = decoder(text) if decoder else text
                    except (ValueError, TypeError, json.JSONDecodeError):
                        continue
                    self.rows.append(value)
                    appended.append(value)
                self.offset = stream.tell()
        except OSError:
            return []
        return appended


def load_manifest(path, requested):
    if path is None:
        return None
    document = read_small_json(path.expanduser().resolve())
    if not document or "scenes" not in document:
        raise ValueError("invalid or oversized manifest: {}".format(path))
    scenes = document["scenes"]
    if requested:
        wanted = set(requested)
        scenes = [
            scene
            for scene in scenes
            if scene.get("id") in wanted or scene.get("file_name") in wanted
        ]
        found = {
            value
            for scene in scenes
            for value in (scene.get("id"), scene.get("file_name"))
            if value in wanted
        }
        missing = wanted - found
        if missing:
            raise ValueError(
                "unknown manifest scenarios: {}".format(", ".join(sorted(missing)))
            )
    else:
        scenes = [scene for scene in scenes if scene.get("run_by_default", True)]
    return {
        "path": str(path.expanduser().resolve()),
        "dataset_id": document.get("dataset_id"),
        "scenes": scenes,
        "by_id": {scene["id"]: scene for scene in scenes},
        "by_file": {scene["file_name"]: scene for scene in scenes},
    }


def family_group(family):
    if family in MISSION_DYNAMIC:
        return "mission_dynamic"
    if family in STATIC_TEMPORAL:
        return "static_temporal"
    if family in HARD_TRANSITION:
        return "hard_transition"
    return "other"


def scene_family(scene):
    if not scene:
        return "unknown"
    return scene.get("mission_family") or scene.get("scenario_type") or "other"


def classify_failure(error):
    text = str(error or "").lower()
    if "collision" in text:
        return "collision"
    if any(token in text for token in ("did not respond", "ros connection", "rosbridge", "service is stale")):
        return "ros_network"
    if any(token in text for token in ("missing sensor", "artifact", "sensordata", "files=")):
        return "sensor_artifact"
    if any(token in text for token in ("route deviation", "route error", "heading error", "route alignment")):
        return "route_safety"
    if any(token in text for token in ("did not arm", "motion stall", "actor ready", "did not move")):
        return "motion_readiness"
    if "object counts" in text or "expected objects" in text:
        return "actor_missing"
    if "did not become ready" in text:
        return "scenario_readiness"
    return "other"


def format_duration(seconds):
    if seconds is None:
        return "-"
    seconds = max(0, int(round(seconds)))
    hours, remainder = divmod(seconds, 3600)
    minutes, seconds = divmod(remainder, 60)
    return "{:02d}:{:02d}:{:02d}".format(hours, minutes, seconds)


def percentile(values, fraction):
    values = sorted(float(value) for value in values)
    if not values:
        return None
    index = int(round((len(values) - 1) * fraction))
    return values[max(0, min(index, len(values) - 1))]


def progress_bar(value, total, width=28):
    ratio = 0.0 if not total else max(0.0, min(1.0, value / total))
    filled = int(round(width * ratio))
    return "[{}{}]".format("#" * filled, "-" * (width - filled))


def parse_console_state(lines, manifest):
    state = {
        "stage": "waiting_for_first_checkpoint",
        "scene_id": None,
        "capture_index": None,
        "capture_total": None,
        "last_line": lines[-1] if lines else None,
        "last_warning": None,
    }
    file_lookup = manifest["by_file"] if manifest else {}
    for line in lines:
        if "[WARN]" in line or "[ERROR]" in line:
            state["last_warning"] = line
        match = re.search(r"ScenarioLoad ROS published:\s+(\S+)", line)
        if match:
            file_name = match.group(1)
            scene = file_lookup.get(file_name)
            state.update(
                stage="scenario_load",
                scene_id=scene.get("id") if scene else file_name,
                capture_index=None,
                capture_total=None,
            )
        match = re.search(r"Scenario verified:\s+(\S+)", line)
        if match:
            file_name = match.group(1)
            scene = file_lookup.get(file_name)
            state.update(
                stage="scenario_verified",
                scene_id=scene.get("id") if scene else file_name,
            )
        match = re.search(
            r"(?:Timed-grouped|Rolling-grouped) trigger\s+(\d+)/(\d+)",
            line,
            re.IGNORECASE,
        )
        if match:
            state.update(
                stage="sensor_capture",
                capture_index=int(match.group(1)),
                capture_total=int(match.group(2)),
            )
        if "complete:" in line.lower():
            state["stage"] = "finalizing_scene"
        match = re.search(r"Scene\s+(\S+)\s+failed", line)
        if match:
            state.update(stage="scene_failed", scene_id=match.group(1))
    return state


class CaptureMonitor:
    def __init__(self, run_dir, manifest=None, args=None):
        self.run_dir = Path(run_dir).expanduser().resolve()
        self.manifest = manifest
        self.args = args
        self.scenes_reader = AppendedLineReader(self.run_dir / "scenes.jsonl")
        self.failures_reader = AppendedLineReader(self.run_dir / "failures.jsonl")
        self.console_reader = AppendedLineReader(
            self.run_dir / "console.log",
            max_rows=256,
            max_line_bytes=256 * 1024,
            initial_tail_bytes=256 * 1024,
        )

    def refresh(self):
        summary = read_small_json(self.run_dir / "summary.json")
        self.scenes_reader.update(json.loads)
        self.failures_reader.update(json.loads)
        self.console_reader.update()
        return build_snapshot(
            self.run_dir,
            summary,
            list(self.scenes_reader.rows),
            list(self.failures_reader.rows),
            list(self.console_reader.rows),
            self.manifest,
            self.args,
        )


def build_snapshot(run_dir, summary, scene_rows, failure_rows, console_lines, manifest, args):
    now = time.time()
    summary = summary or {}
    requested = int(summary.get("requested_scenes", len(manifest["scenes"]) if manifest else 0))
    attempted = int(summary.get("attempted_scenes", len(scene_rows)))
    completed = int(summary.get("completed_scenes", sum(row.get("status") == "completed" for row in scene_rows)))
    failed = int(summary.get("failed_scenes", sum(row.get("status") == "failed" for row in scene_rows)))
    frames = int(summary.get("completed_captures", sum(int(row.get("capture_count", 0)) for row in scene_rows if row.get("status") == "completed")))
    started = summary.get("started_unix")
    elapsed_end = summary.get("finished_unix", now)
    elapsed = elapsed_end - started if started else None
    eta = None
    if elapsed is not None and attempted > 0 and requested > attempted:
        eta = elapsed / attempted * (requested - attempted)

    by_id = manifest["by_id"] if manifest else {}
    selected_scenes = manifest["scenes"] if manifest else []
    target_frames = None
    if selected_scenes and (not requested or requested == len(selected_scenes)):
        target_frames = sum(
            int(scene.get("sequence_contract", {}).get("clip_length", 1))
            for scene in selected_scenes
        )

    targets = Counter()
    family_targets = Counter()
    split_targets = Counter()
    for scene in selected_scenes:
        family = scene_family(scene)
        targets[family_group(family)] += 1
        family_targets[family] += 1
        split_targets[scene.get("split") or "unspecified"] += 1
    attempted_groups = Counter()
    completed_groups = Counter()
    failed_groups = Counter()
    split_completed = Counter()
    family_completed = Counter()
    family_attempted = Counter()
    family_failed = Counter()
    for row in scene_rows:
        scene = by_id.get(row.get("id"))
        family = scene_family(scene)
        group = family_group(family)
        attempted_groups[group] += 1
        family_attempted[family] += 1
        if row.get("status") == "completed":
            completed_groups[group] += 1
            family_completed[family] += 1
            split_completed[(scene or {}).get("split") or row.get("split") or "unspecified"] += 1
        elif row.get("status") == "failed":
            failed_groups[group] += 1
            family_failed[family] += 1

    durations = [
        row.get("metrics", {}).get("scene_duration_sec")
        for row in scene_rows
        if row.get("metrics", {}).get("scene_duration_sec") is not None
    ]
    trigger_rates = [
        row.get("metrics", {}).get("trigger_rate_hz")
        for row in scene_rows
        if row.get("status") == "completed"
        and row.get("metrics", {}).get("trigger_rate_hz") is not None
    ]
    consecutive_failures = 0
    for row in reversed(scene_rows):
        if row.get("status") != "failed":
            break
        consecutive_failures += 1

    failure_types = Counter(
        classify_failure(row.get("error")) for row in failure_rows
    )
    recent_failures = [
        {
            "id": row.get("id", "?"),
            "type": classify_failure(row.get("error")),
            "error": str(row.get("error", ""))[:180],
        }
        for row in failure_rows[-6:]
    ]
    console_state = parse_console_state(console_lines, manifest)
    latest = summary.get("latest_scene") or (scene_rows[-1] if scene_rows else None)
    if not console_lines and summary.get("finished"):
        console_state["stage"] = "finished"
    elif not console_lines:
        attempted_ids = {row.get("id") for row in scene_rows}
        next_scene = next(
            (
                scene for scene in selected_scenes
                if scene.get("id") not in attempted_ids
            ),
            None,
        )
        if next_scene:
            console_state.update(
                stage="next_or_running_unobserved",
                scene_id=next_scene.get("id"),
            )
        elif latest:
            console_state["stage"] = "between_scene_checkpoints"
    checkpoint = summary.get("checkpoint_unix")
    checkpoint_age = now - checkpoint if checkpoint else None
    try:
        disk = shutil.disk_usage(run_dir)
        disk_free_gb = disk.free / 1_000_000_000.0
    except OSError:
        disk_free_gb = None

    warnings = []
    finished = bool(summary.get("finished"))
    stall_limit = float(getattr(args, "stall_warning_sec", 60.0))
    if not finished and checkpoint_age is not None and checkpoint_age > stall_limit:
        warnings.append("checkpoint_stale")
    disk_limit = float(getattr(args, "disk_warning_gb", 100.0))
    if disk_free_gb is not None and disk_free_gb < disk_limit:
        warnings.append("disk_low")
    failure_limit = float(getattr(args, "failure_rate_warning", 0.10))
    if attempted >= 10 and failed / attempted > failure_limit:
        warnings.append("failure_rate_high")
    if consecutive_failures >= 3:
        warnings.append("consecutive_failures")
    if manifest and summary.get("dataset_id") and manifest.get("dataset_id") != summary.get("dataset_id"):
        warnings.append("manifest_dataset_mismatch")
    if manifest and requested and requested != len(selected_scenes):
        warnings.append("manifest_selection_mismatch")

    return {
        "timestamp_unix": now,
        "run_dir": str(run_dir),
        "dataset_id": summary.get("dataset_id") or (manifest and manifest.get("dataset_id")),
        "finished": finished,
        "interrupted": bool(summary.get("interrupted")),
        "progress": {
            "requested_scenes": requested,
            "attempted_scenes": attempted,
            "completed_scenes": completed,
            "failed_scenes": failed,
            "verified_frames": frames,
            "target_frames": target_frames,
            "elapsed_sec": elapsed,
            "eta_sec": eta,
        },
        "latest_scene": latest,
        "current": console_state,
        "groups": {
            group: {
                "target": targets.get(group, 0),
                "attempted": attempted_groups.get(group, 0),
                "completed": completed_groups.get(group, 0),
                "failed": failed_groups.get(group, 0),
            }
            for group in sorted(set(targets) | set(attempted_groups))
        },
        "families": {
            family: {
                "target": family_targets.get(family, 0),
                "attempted": family_attempted.get(family, 0),
                "completed": family_completed.get(family, 0),
                "failed": family_failed.get(family, 0),
            }
            for family in sorted(set(family_targets) | set(family_attempted))
        },
        "splits": {
            split: {
                "target": split_targets.get(split, 0),
                "completed": split_completed.get(split, 0),
            }
            for split in sorted(set(split_targets) | set(split_completed))
        },
        "performance": {
            "scene_duration_p50_sec": percentile(durations, 0.50),
            "scene_duration_p95_sec": percentile(durations, 0.95),
            "sequence_rate_p50_hz": percentile(trigger_rates, 0.50),
            "sequence_rate_p05_hz": percentile(trigger_rates, 0.05),
        },
        "health": {
            "checkpoint_age_sec": checkpoint_age,
            "disk_free_gb": disk_free_gb,
            "consecutive_failures": consecutive_failures,
            "failure_types": dict(sorted(failure_types.items())),
            "warnings": warnings,
        },
        "recent_failures": recent_failures,
    }


def render(snapshot, use_color=True):
    def color(text, code):
        return "\033[{}m{}\033[0m".format(code, text) if use_color else text

    progress = snapshot["progress"]
    health = snapshot["health"]
    attempted = progress["attempted_scenes"]
    requested = progress["requested_scenes"]
    percent = 100.0 * attempted / requested if requested else 0.0
    status = "FINISHED" if snapshot["finished"] else "RUNNING"
    if snapshot["interrupted"]:
        status = "INTERRUPTED"
    if health["warnings"]:
        status = "WARNING"
    status_code = "32" if status == "FINISHED" else "33" if status == "WARNING" else "36"

    print("Capture Monitor V2", datetime.now().strftime("%Y-%m-%d %H:%M:%S"), color(status, status_code))
    print("run:", snapshot["run_dir"])
    print("dataset:", snapshot.get("dataset_id") or "-")
    print("mode: read-only sidecar; no ROS subscriptions or sensor scans.")

    if snapshot["groups"]:
        print("\nQuota groups")
        for group, row in snapshot["groups"].items():
            print(
                "  {:20s} {:>3}/{:<3} complete | failed={}".format(
                    group,
                    row["completed"],
                    row["target"],
                    row["failed"],
                )
            )
    if snapshot["families"]:
        print("\nFamilies")
        for family, row in snapshot["families"].items():
            print(
                "  {:32s} {:>3}/{:<3} complete | failed={}".format(
                    family,
                    row["completed"],
                    row["target"],
                    row["failed"],
                )
            )
    if snapshot["splits"]:
        print("\nSplits")
        print(
            "  " + " | ".join(
                "{} {}/{}".format(name, row["completed"], row["target"])
                for name, row in snapshot["splits"].items()
            )
        )

    if snapshot["recent_failures"]:
        print("\nRecent failures")
        for row in snapshot["recent_failures"]:
            print("  {} [{}] {}".format(row["id"], row["type"], row["error"]))

    current = snapshot["current"]
    latest = snapshot.get("latest_scene") or {}
    current_id = current.get("scene_id") or latest.get("id") or "-"

    # Keep the operational block at the bottom.  On a short terminal the
    # verbose quota/family/failure history can scroll above the viewport while
    # the information needed to decide whether capture is healthy remains next
    # to the cursor.
    print("\n" + "=" * 72)
    print("LIVE STATUS", color(status, status_code))
    print("\nCurrent")
    print("  scene: {} | stage: {}".format(current_id, current.get("stage") or "-"))
    if current.get("capture_total"):
        print(
            "  clip frames: {}/{}".format(
                current.get("capture_index", 0), current["capture_total"]
            )
        )
    if latest:
        duration = latest.get("metrics", {}).get("scene_duration_sec")
        print(
            "  latest result: {} [{}]{}".format(
                latest.get("id", "?"),
                latest.get("status", "?"),
                " {:.2f}s".format(duration) if duration is not None else "",
            )
        )

    performance = snapshot["performance"]
    print("\nPerformance")
    print(
        "  scene duration p50/p95: {} / {} sec".format(
            "{:.2f}".format(performance["scene_duration_p50_sec"])
            if performance["scene_duration_p50_sec"] is not None else "-",
            "{:.2f}".format(performance["scene_duration_p95_sec"])
            if performance["scene_duration_p95_sec"] is not None else "-",
        )
    )
    print(
        "  sequence rate p50/p05: {} / {} Hz".format(
            "{:.2f}".format(performance["sequence_rate_p50_hz"])
            if performance["sequence_rate_p50_hz"] is not None else "-",
            "{:.2f}".format(performance["sequence_rate_p05_hz"])
            if performance["sequence_rate_p05_hz"] is not None else "-",
        )
    )

    print("\nHealth (passive, no ROS subscriptions)")
    print(
        "  checkpoint age: {} sec | disk free: {} GB".format(
            "{:.1f}".format(health["checkpoint_age_sec"])
            if health["checkpoint_age_sec"] is not None else "-",
            "{:.1f}".format(health["disk_free_gb"])
            if health["disk_free_gb"] is not None else "-",
        )
    )
    print(
        "  consecutive failures: {} | types: {}".format(
            health["consecutive_failures"],
            ", ".join("{}={}".format(k, v) for k, v in health["failure_types"].items()) or "none",
        )
    )
    if health["warnings"]:
        print("  alerts:", color(", ".join(health["warnings"]), "31"))
    else:
        print("  alerts:", color("none", "32"))

    if current.get("last_warning"):
        print("\nLast WARN/ERROR")
        print("  " + current["last_warning"][-220:])

    print("\nProgress")
    print(
        "  Scenes {} {:>4}/{:<4} {:5.1f}% | completed={} failed={}".format(
            progress_bar(attempted, requested),
            attempted,
            requested,
            percent,
            progress["completed_scenes"],
            progress["failed_scenes"],
        )
    )
    target_frames = progress.get("target_frames")
    if target_frames:
        print(
            "  Frames {} {:>4}/{:<4} verified".format(
                progress_bar(progress["verified_frames"], target_frames),
                progress["verified_frames"],
                target_frames,
            )
        )
    else:
        print("  Frames {} verified".format(progress["verified_frames"]))
    print(
        "  Time   elapsed={} | ETA={}".format(
            format_duration(progress.get("elapsed_sec")),
            format_duration(progress.get("eta_sec")),
        )
    )

    rate = performance["sequence_rate_p50_hz"]
    rate_text = "{:.2f}Hz".format(rate) if rate is not None else "-"
    frame_text = (
        "{}/{}".format(progress["verified_frames"], target_frames)
        if target_frames else str(progress["verified_frames"])
    )
    disk_text = (
        "{:.1f}GB".format(health["disk_free_gb"])
        if health["disk_free_gb"] is not None else "-"
    )
    print(
        "\nNOW {} | scene={} stage={} | frames={} | ETA={} | rate={} | "
        "failed={} consecutive={} | disk={}".format(
            color(status, status_code),
            current_id,
            current.get("stage") or "-",
            frame_text,
            format_duration(progress.get("eta_sec")),
            rate_text,
            progress["failed_scenes"],
            health["consecutive_failures"],
            disk_text,
        )
    )


def main():
    args = parse_args()
    if args.interval_sec < 1.0:
        raise ValueError("--interval-sec must be >= 1.0 to limit monitoring I/O")
    if args.stall_warning_sec <= 0.0:
        raise ValueError("--stall-warning-sec must be positive")
    run_dir = (
        args.run_dir.expanduser().resolve()
        if args.run_dir
        else newest_run(args.capture_root, args.name_contains)
    )
    manifest = load_manifest(args.manifest, args.scenario)
    monitor = CaptureMonitor(run_dir, manifest, args)
    while True:
        snapshot = monitor.refresh()
        if args.json:
            print(json.dumps(snapshot, ensure_ascii=False, separators=(",", ":")))
        else:
            if not args.no_clear and sys.stdout.isatty():
                print("\033[2J\033[H", end="")
            render(
                snapshot,
                use_color=(not args.no_color and sys.stdout.isatty()),
            )
        if args.once or args.json or snapshot["finished"]:
            break
        time.sleep(max(1.0, args.interval_sec))


if __name__ == "__main__":
    main()
