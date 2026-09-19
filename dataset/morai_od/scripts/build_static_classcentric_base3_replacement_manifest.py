#!/usr/bin/env python3
"""Build a small Base3 replacement manifest from campaign audit reports.

An exact-quota reserve scene is preferred.  When the reserve pool does not
contain the failed scene's spatial quota, the original failed scene is retried
instead of silently substituting a different geometry and corrupting the final
campaign distribution.
"""

import argparse
import copy
import json
from collections import defaultdict
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
QUOTA_FIELDS = (
    "family",
    "split",
    "spatial_stratum",
    "highway_phase_variant",
    "ordinary_two_lane_side",
    "roundabout_phase",
)


def load_json(path):
    with Path(path).open("r", encoding="utf-8") as stream:
        return json.load(stream)


def resolve(path):
    path = Path(path)
    return path if path.is_absolute() else PROJECT_ROOT / path


def metadata_quota(metadata):
    spatial = metadata.get("spatial_sampling") or {}
    return {
        "family": metadata.get("mission_family"),
        "split": metadata.get("split"),
        "spatial_stratum": spatial.get("spatial_stratum") or "unspecified",
        "highway_phase_variant": spatial.get("highway_phase_variant"),
        "ordinary_two_lane_side": spatial.get("ordinary_two_lane_side"),
        "roundabout_phase": spatial.get("roundabout_phase"),
    }


def quota_tuple(quota):
    return tuple(quota.get(field) for field in QUOTA_FIELDS)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--audit-report", action="append", type=Path, required=True)
    parser.add_argument("--reserve-manifest", action="append", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--dataset-id",
        default="morai_static_classcentric_base3_10k_replacement_v1",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    requests = []
    for path in args.audit_report:
        requests.extend(load_json(path).get("replacement_requests", []))
    if not requests:
        raise SystemExit("audit reports contain no replacement requests")

    reserves = defaultdict(list)
    for manifest_path in args.reserve_manifest:
        manifest = load_json(manifest_path)
        for scene in manifest.get("scenes", []):
            metadata = load_json(resolve(scene["metadata"]))
            reserves[quota_tuple(metadata_quota(metadata))].append(
                (str(manifest_path), copy.deepcopy(scene))
            )

    source_manifests = {}
    source_scenes = {}
    for request in requests:
        path = request["source_manifest"]
        if path not in source_manifests:
            source_manifests[path] = load_json(path)
            source_scenes[path] = {
                scene["id"]: scene for scene in source_manifests[path]["scenes"]
            }

    used_scene_ids = set()
    selected = []
    mapping = []
    for request in requests:
        key = quota_tuple(request["quota"])
        candidate = None
        while reserves[key]:
            reserve_manifest, reserve_scene = reserves[key].pop(0)
            if reserve_scene["id"] not in used_scene_ids:
                candidate = ("exact_reserve", reserve_manifest, reserve_scene)
                break
        if candidate is None:
            source_manifest = request["source_manifest"]
            source_scene = copy.deepcopy(
                source_scenes[source_manifest][request["source_scene_id"]]
            )
            candidate = ("retry_source", source_manifest, source_scene)

        selection_mode, selected_manifest, scene = candidate
        if scene["id"] in used_scene_ids:
            raise RuntimeError("duplicate replacement scene id: {}".format(scene["id"]))
        used_scene_ids.add(scene["id"])
        selected.append(scene)
        mapping.append(
            {
                "failed_scene_id": request["source_scene_id"],
                "replacement_scene_id": scene["id"],
                "selection_mode": selection_mode,
                "selected_from_manifest": str(Path(selected_manifest).resolve()),
                "quota": request["quota"],
            }
        )

    template = copy.deepcopy(next(iter(source_manifests.values())))
    template["dataset_id"] = args.dataset_id
    template["scenario_group"] = "static_classcentric_base3_replacement_v1"
    template["scenes"] = selected
    template["generation"] = {
        "purpose": "fill_failed_base3_primary_clips",
        "replacement_clip_count": len(selected),
        "replacement_frame_count": sum(
            int(scene.get("capture_policy_overrides", {}).get("captures_per_scene", 3))
            for scene in selected
        ),
        "selection": mapping,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(template, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print("replacement clips: {}".format(len(selected)))
    print("exact reserve: {}".format(
        sum(row["selection_mode"] == "exact_reserve" for row in mapping)
    ))
    print("source retry: {}".format(
        sum(row["selection_mode"] == "retry_source" for row in mapping)
    ))
    print("output: {}".format(args.output))


if __name__ == "__main__":
    main()
