#!/usr/bin/env python3
"""Select complete scenario clips into an exact 1,500-frame OD batch."""

import argparse
import json
from pathlib import Path


FRAMES_PER_SCENE = {"mission": 12, "static": 3, "hard_dynamic": 12, "hard_static": 3}
TARGETS = {"mission": 744, "static": 525, "hard": 231}


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mission-run", type=Path, required=True)
    parser.add_argument("--static-run", type=Path, required=True)
    parser.add_argument("--hard-dynamic-run", type=Path, required=True)
    parser.add_argument("--hard-static-run", type=Path, required=True)
    parser.add_argument("--static-reserve-run", type=Path, required=True)
    parser.add_argument("--static-recovery-run", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def summary(run):
    path = run / "summary.json"
    value = json.loads(path.read_text(encoding="utf-8"))
    if not value.get("finished") or value.get("interrupted"):
        raise RuntimeError("capture run is not cleanly finished: {}".format(run))
    return value


def completed(run):
    return list(summary(run).get("completed_scene_ids", []))


def component(role, run, scene_ids, purpose=None):
    frame_role = "hard_static" if role == "hard_static_recovery" else role
    frames_per_scene = FRAMES_PER_SCENE[frame_role]
    return {
        "role": role,
        "purpose": purpose or role,
        "capture_dir": str(run.resolve()),
        "scene_ids": list(scene_ids),
        "scene_count": len(scene_ids),
        "frames_per_scene": frames_per_scene,
        "frame_count": len(scene_ids) * frames_per_scene,
    }


def main():
    args = parse_args()
    mission_ids = completed(args.mission_run)
    static_ids = completed(args.static_run)
    hard_dynamic_ids = completed(args.hard_dynamic_run)
    hard_static_ids = completed(args.hard_static_run)
    reserve_ids = completed(args.static_reserve_run)
    recovery_ids = completed(args.static_recovery_run)

    if len(mission_ids) * 12 != TARGETS["mission"]:
        raise RuntimeError("mission quota is not complete")
    if len(hard_dynamic_ids) * 12 > TARGETS["hard"]:
        raise RuntimeError("hard dynamic exceeds hard quota")

    static_needed = TARGETS["static"] // 3 - len(static_ids) - len(reserve_ids)
    hard_static_needed = (
        TARGETS["hard"] - len(hard_dynamic_ids) * 12 - len(hard_static_ids) * 3
    ) // 3
    if static_needed < 0 or hard_static_needed < 0:
        raise RuntimeError("primary and reserve clips exceed a target quota")
    needed_recovery = static_needed + hard_static_needed
    if len(recovery_ids) < needed_recovery:
        raise RuntimeError(
            "static recovery needs {} complete clips but has {}".format(
                needed_recovery, len(recovery_ids)
            )
        )
    static_recovery_ids = recovery_ids[:static_needed]
    hard_recovery_ids = recovery_ids[
        static_needed : static_needed + hard_static_needed
    ]
    unused_recovery_ids = recovery_ids[needed_recovery:]

    components = [
        component("mission", args.mission_run, mission_ids),
        component("static", args.static_run, static_ids, "static_primary"),
        component("static", args.static_reserve_run, reserve_ids, "static_reserve"),
        component("static", args.static_recovery_run, static_recovery_ids, "static_recovery"),
        component("hard_dynamic", args.hard_dynamic_run, hard_dynamic_ids),
        component("hard_static", args.hard_static_run, hard_static_ids),
        component(
            "hard_static_recovery",
            args.static_recovery_run,
            hard_recovery_ids,
            "hard_static_replacement",
        ),
    ]
    frame_totals = {
        "mission": sum(item["frame_count"] for item in components if item["role"] == "mission"),
        "static": sum(item["frame_count"] for item in components if item["role"] == "static"),
        "hard": sum(item["frame_count"] for item in components if item["role"].startswith("hard")),
    }
    all_ids = [scene_id for item in components for scene_id in item["scene_ids"]]
    if len(all_ids) != len(set(all_ids)):
        raise RuntimeError("accepted scene ids are not unique")
    if frame_totals != TARGETS:
        raise RuntimeError("frame quotas do not match: {}".format(frame_totals))

    output = {
        "schema_version": 1,
        "dataset_id": "morai_dynamic_dataset_batch001_accepted1500",
        "selection_unit": "scenario_clip",
        "frame_quotas": frame_totals,
        "total_frames": sum(frame_totals.values()),
        "components": components,
        "unused_complete_recovery_scene_ids": unused_recovery_ids,
        "notes": [
            "All frames from an accepted clip stay together.",
            "Failed or partial scenes are excluded.",
            "Unused complete recovery clips remain reserve data and are not part of the 1,500 frames.",
        ],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(output, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print("accepted scenes:", len(all_ids))
    print("frame quotas:", frame_totals)
    print("total frames:", output["total_frames"])
    print("unused recovery scenes:", len(unused_recovery_ids))
    print("selection:", args.output)


if __name__ == "__main__":
    main()
