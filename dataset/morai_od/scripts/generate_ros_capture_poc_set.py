#!/usr/bin/env python3
"""Generate a small deterministic scenario set for ROS load/capture testing."""

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path


EGO_SIDECAR_REQUIRED_ROS_TOPICS = (
    "/ScenarioLoad",
    "/SaveSensorData",
    "/Ego_topic",
    "/Object_topic",
)


SCENE_SPECS = (
    {
        "id": "p01",
        "stem": "od_poc_01",
        "anchor_index": 500,
        "seed": 26080601,
        "npc_count": 4,
        "pedestrian_count": 3,
        "obstacle_count": 2,
        "run_by_default": True,
    },
    {
        "id": "p02",
        "stem": "od_poc_02",
        "anchor_index": 2516,
        "seed": 26080602,
        "npc_count": 5,
        "pedestrian_count": 4,
        "obstacle_count": 2,
        "run_by_default": True,
    },
    {
        "id": "p03",
        "stem": "od_poc_03",
        "anchor_index": 3500,
        "seed": 26080603,
        "npc_count": 4,
        "pedestrian_count": 5,
        "obstacle_count": 3,
        "run_by_default": True,
    },
    {
        "id": "speed01",
        "stem": "od_speed_clear_01",
        "anchor_index": 500,
        "seed": 26080607,
        "npc_count": 0,
        "pedestrian_count": 0,
        "obstacle_count": 0,
        "run_by_default": False,
    },
)


def parse_args():
    project_root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(
        description=(
            "Generate MORAI ROS auto-capture proof-of-concept and speed "
            "benchmark scenes."
        )
    )
    parser.add_argument(
        "--project-root",
        type=Path,
        default=project_root,
    )
    parser.add_argument(
        "--install-dir",
        type=Path,
        required=True,
        help=(
            "Current-map MORAI scenario directory. Generated JSON files and "
            "ROS EgoNetwork sidecars are installed together."
        ),
    )
    parser.add_argument(
        "--network-template",
        type=Path,
        default=None,
        help=(
            "ROS bridge profile saved by the same MORAI version. If a full "
            "profile is supplied, only Ego UNIQUEID=0 is written to scenario "
            "sidecars; Simulator UNIQUEID=-100 remains a separate one-time "
            "bootstrap profile. Defaults to "
            "<install-dir>/EgoNetwork/ros_path_probe_MN.json."
        ),
    )
    parser.add_argument(
        "--stress-count",
        type=int,
        default=0,
        help=(
            "Generate this many short batch-stress scenes instead of the "
            "four normal PoC/benchmark scenes. The stress scenes reuse one "
            "known-good anchor and vary only their seed."
        ),
    )
    return parser.parse_args()


def selected_scene_specs(stress_count):
    if stress_count < 0:
        raise ValueError("--stress-count must be non-negative")
    if stress_count == 0:
        return SCENE_SPECS, "ros_capture_poc", "ros_capture_poc.json"
    if stress_count > 999:
        raise ValueError("--stress-count must be <= 999")

    specs = []
    for index in range(1, stress_count + 1):
        specs.append(
            {
                "id": "stress{:03d}".format(index),
                "stem": "od_stress_{:03d}".format(index),
                # Keep route geometry constant so this test isolates repeated
                # ScenarioLoad/network recovery/capture behavior.
                "anchor_index": 500,
                "seed": 26080700 + index,
                "npc_count": 4,
                "pedestrian_count": 3,
                "obstacle_count": 2,
                "run_by_default": True,
            }
        )
    return tuple(specs), "ros_capture_stress", "ros_capture_stress.json"


def copy_file(source, destination):
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_bytes(source.read_bytes())


def network_unique_id(entry):
    for key in ("uniqueId", "UNIQUEID", "uniqueID"):
        if key in entry:
            return int(entry[key])
    return None


def build_ego_only_network_template(path):
    if not path.is_file():
        raise FileNotFoundError(
            "ROS network template does not exist: {}".format(path)
        )
    document = json.loads(path.read_text(encoding="utf-8"))
    ego_network_data = document.get("egoNetworkData")
    if not isinstance(ego_network_data, list) or not ego_network_data:
        raise ValueError(
            "network template has no egoNetworkData: {}".format(path)
        )

    ego_only_entries = [
        entry
        for entry in ego_network_data
        if network_unique_id(entry) != -100
    ]
    if not ego_only_entries:
        raise ValueError(
            "network template has no Ego network entry: {}".format(path)
        )
    unique_ids = {network_unique_id(entry) for entry in ego_only_entries}
    if 0 not in unique_ids:
        raise ValueError(
            "network template has no UNIQUEID=0 Ego entry: {}".format(path)
        )

    configured_topics = set()
    ros_bridge_entries = 0
    for ego in ego_only_entries:
        if ego.get("connectedType") == 3:
            ros_bridge_entries += 1
        for entry in ego.get("listEgoNetworkInfo", []):
            if entry.get("netType") == 3:
                ros_bridge_entries += 1
            topic = (
                entry.get("config", {})
                .get("rosConfig", {})
                .get("Topic")
            )
            if topic:
                configured_topics.add(topic)

    missing = sorted(
        set(EGO_SIDECAR_REQUIRED_ROS_TOPICS) - configured_topics
    )
    if ros_bridge_entries == 0 or missing:
        raise ValueError(
            "Ego network template is not the required ROS bridge profile; "
            "missing topics={}: {}".format(missing, path)
        )
    document = {"egoNetworkData": ego_only_entries}
    payload = (
        json.dumps(document, ensure_ascii=False, indent=2) + "\n"
    ).encode("utf-8")
    return document, payload


def resolve_network_template(args, install_dir):
    if args.network_template is not None:
        path = args.network_template.expanduser().resolve()
    else:
        path = install_dir / "EgoNetwork/ros_path_probe_MN.json"
    _document, payload = build_ego_only_network_template(path)
    digest = hashlib.sha256(payload).hexdigest()
    return path, payload, digest


def main():
    args = parse_args()
    project_root = args.project_root.resolve()
    workspace = project_root / "dataset/morai_od"
    generator = workspace / "scripts/generate_morai_od_prototype.py"
    scene_specs, scenario_group, manifest_name = selected_scene_specs(
        args.stress_count
    )
    manifest_path = workspace / "manifests" / manifest_name
    install_dir = args.install_dir.expanduser().resolve()
    network_template, ego_sidecar_payload, network_sha256 = (
        resolve_network_template(args, install_dir)
    )
    scenes = []

    for spec in scene_specs:
        maximum_attempts = 100 if args.stress_count else 1
        generation = None
        used_seed = spec["seed"]
        for attempt in range(maximum_attempts):
            used_seed = spec["seed"] + attempt * 1000
            command = [
                sys.executable,
                str(generator),
                "--output-root",
                str(workspace),
                "--scenario-group",
                scenario_group,
                "--output-stem",
                spec["stem"],
                "--anchor-index",
                str(spec["anchor_index"]),
                "--seed",
                str(used_seed),
                "--npc-count",
                str(spec["npc_count"]),
                "--pedestrian-count",
                str(spec["pedestrian_count"]),
                "--obstacle-count",
                str(spec["obstacle_count"]),
                "--weather",
                "sunny",
                "--time-hour",
                "13",
            ]
            generation = subprocess.run(
                command,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
            )
            if generation.returncode == 0:
                print(generation.stdout, end="")
                if attempt:
                    print(
                        "stress scene {} accepted retry seed {} "
                        "(attempt {})".format(
                            spec["id"], used_seed, attempt + 1
                        )
                    )
                break
        if generation is None or generation.returncode != 0:
            raise RuntimeError(
                "scenario generation failed after {} attempts for {}:\n{}"
                .format(
                    maximum_attempts,
                    spec["id"],
                    generation.stdout if generation is not None else "",
                )
            )

        scenario_rel = Path(
            "dataset/morai_od/scenarios"
        ) / scenario_group / f"{spec['stem']}.json"
        metadata_rel = Path(
            "dataset/morai_od/metadata"
        ) / scenario_group / f"{spec['stem']}.metadata.json"
        metadata = json.loads(
            (project_root / metadata_rel).read_text(encoding="utf-8")
        )
        copy_file(
            project_root / scenario_rel,
            install_dir / scenario_rel.name,
        )
        network_sidecar = (
            install_dir / "EgoNetwork" / f"{spec['stem']}_MN.json"
        )
        network_sidecar.parent.mkdir(parents=True, exist_ok=True)
        network_sidecar.write_bytes(ego_sidecar_payload)
        if network_sidecar.read_bytes() != ego_sidecar_payload:
            raise RuntimeError(
                "installed Ego-only sidecar differs from template: {}".format(
                    network_sidecar
                )
            )

        scenes.append(
            {
                "id": spec["id"],
                "file_name": spec["stem"],
                "scenario": str(scenario_rel),
                "metadata": str(metadata_rel),
                "network_sidecar": "EgoNetwork/{}_MN.json".format(
                    spec["stem"]
                ),
                "anchor_index": spec["anchor_index"],
                "generation_seed": used_seed,
                "expected_ego": metadata["ego"],
                "expected_counts": {
                    "npc": spec["npc_count"],
                    "pedestrian": spec["pedestrian_count"],
                    "obstacle": spec["obstacle_count"],
                },
                "run_by_default": spec["run_by_default"],
            }
        )

    manifest = {
        "schema_version": 2,
        "purpose": (
            "MORAI ROS repeated ScenarioLoad and SaveSensorData stress test"
            if args.stress_count
            else "MORAI ROS ScenarioLoad and SaveSensorData proof of concept"
        ),
        "map_name": "R_KR_PR_K-city_2025",
        "network_policy": {
            "transport": "rosbridge",
            "load_network_connection_data": True,
            "sidecar_scope": "ego_only",
            "required_unique_ids": [0],
            "forbidden_unique_ids": [-100],
            "required_topics": list(EGO_SIDECAR_REQUIRED_ROS_TOPICS),
            "template_sha256": network_sha256,
            "note": (
                "Each scenario must have EgoNetwork/<stem>_MN.json copied "
                "from only the UNIQUEID=0 part of a ROS profile. Keep "
                "UNIQUEID=-100 in the separately loaded Simulator Network "
                "bootstrap profile so ScenarioLoad does not disconnect it."
            ),
        },
        "environment": {
            "weather": "sunny",
            "time_hour": 13,
            "note": "Set once in the MORAI UI before running this PoC.",
        },
        "waypoints": "configs/global_path/2026_molit_comp_global_path.txt",
        "capture_policy": {
            "captures_per_scene": 1 if args.stress_count else 6,
            "capture_spacing_m": 0.6,
            "ego_target_speed_kph": 3.6,
            "maximum_motion_m": 4.0,
        },
        "scenes": scenes,
    }
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    print("MORAI ROS capture set generated")
    print(f"manifest: {manifest_path}")
    print(f"scenes: {len(scenes)}")
    print(f"MORAI install: {install_dir}")
    print(f"ROS source profile: {network_template}")
    print(f"Ego-only sidecar SHA256: {network_sha256}")
    print(f"Ego-only ROS sidecars: {len(scenes)}")


if __name__ == "__main__":
    main()
