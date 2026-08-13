#!/usr/bin/env python3
"""Stream a MORAI console log and emit only a compact failure aggregate."""

import argparse
import ast
import json
import re
from collections import Counter, defaultdict
from pathlib import Path


FAILURE_RE = re.compile(
    r"Scene\s+(?P<scene>\S+)\s+failed and will be skipped:\s+(?P<reason>.*)$"
)
OBJECT_COUNTS_RE = re.compile(
    r"object counts\s+(?P<actual>\{.*?\})\s+!=\s+(?P<expected>\{.*?\})"
)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--console-log", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--sample-limit", type=int, default=8)
    return parser.parse_args()


def classify(reason):
    lowered = reason.lower()
    if "object counts" in lowered and "!=" in lowered:
        return "actor_count_mismatch"
    if "collision reported" in lowered or "collision" in lowered:
        return "collision"
    if "complete timestamp groups" in lowered or "missing_artifacts" in lowered:
        return "sensor_artifact_incomplete"
    if "route deviation" in lowered:
        return "route_deviation"
    if "heading error" in lowered:
        return "heading_error"
    if "ended with 0/" in lowered:
        return "ego_no_motion"
    if "ended with" in lowered and "trigger" in lowered:
        return "incomplete_triggers"
    if "subscriber connection missing" in lowered:
        return "ros_connection_missing"
    if "control-mode" in lowered or "morai event" in lowered:
        return "control_mode_failure"
    if "did not become ready" in lowered:
        return "scenario_not_ready_other"
    return "other"


def bounded_sample(scene, reason, limit=300):
    return {
        "scene_id": scene,
        "reason_prefix": reason[:limit],
        "truncated": len(reason) > limit,
    }


def main():
    args = parse_args()
    source = args.console_log.expanduser().resolve()
    categories = Counter()
    category_scenes = defaultdict(list)
    category_samples = defaultdict(list)
    missing_actor_total = Counter()
    missing_actor_scenes = Counter()
    warning_counts = Counter()
    failed_scene_ids = []
    line_count = 0
    completed_scene_events = 0

    with source.open("r", encoding="utf-8", errors="replace") as stream:
        for line in stream:
            line_count += 1
            if "Rolling-grouped complete:" in line:
                completed_scene_events += 1
            if "Waiting for MORAI ROS connections" in line:
                warning_counts["network_reconnect_wait"] += 1
            if "Control-mode attempt" in line:
                warning_counts["control_mode_retry"] += 1

            match = FAILURE_RE.search(line)
            if not match:
                continue
            scene = match.group("scene")
            reason = match.group("reason").strip()
            category = classify(reason)
            failed_scene_ids.append(scene)
            categories[category] += 1
            category_scenes[category].append(scene)
            if len(category_samples[category]) < args.sample_limit:
                category_samples[category].append(
                    bounded_sample(scene, reason)
                )

            counts = OBJECT_COUNTS_RE.search(reason)
            if counts:
                try:
                    actual = ast.literal_eval(counts.group("actual"))
                    expected = ast.literal_eval(counts.group("expected"))
                except (SyntaxError, ValueError):
                    continue
                for actor_type in ("npc", "pedestrian", "obstacle"):
                    missing = max(
                        0,
                        int(expected.get(actor_type, 0))
                        - int(actual.get(actor_type, 0)),
                    )
                    if missing:
                        missing_actor_total[actor_type] += missing
                        missing_actor_scenes[actor_type] += 1

    result = {
        "schema_version": 1,
        "source": {
            "path": str(source),
            "size_bytes": source.stat().st_size,
            "line_count": line_count,
            "processing": "streamed_line_by_line_not_loaded_into_model_context",
        },
        "events": {
            "completed_scene_events": completed_scene_events,
            "failure_events": len(failed_scene_ids),
            "unique_failed_scenes": len(set(failed_scene_ids)),
        },
        "failure_categories": {
            category: {
                "count": count,
                "unique_scene_count": len(set(category_scenes[category])),
                "sample_scene_ids": list(
                    dict.fromkeys(category_scenes[category])
                )[: args.sample_limit],
                "samples": category_samples[category],
            }
            for category, count in categories.most_common()
        },
        "actor_count_mismatch": {
            "missing_actor_total": dict(missing_actor_total),
            "affected_scene_count": dict(missing_actor_scenes),
        },
        "warning_counts": dict(warning_counts),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print("analysis:", args.output)
    print("source lines:", line_count)
    print("completed events:", completed_scene_events)
    print("failure events:", len(failed_scene_ids))
    print("categories:", dict(categories))


if __name__ == "__main__":
    main()
