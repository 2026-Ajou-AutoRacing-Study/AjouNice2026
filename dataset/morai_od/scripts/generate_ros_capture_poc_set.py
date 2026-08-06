#!/usr/bin/env python3
"""Generate a small deterministic scenario set for ROS load/capture testing."""

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path


REQUIRED_ROS_TOPICS = (
    "/ScenarioLoad",
    "/SaveSensorData",
    "/Ego_topic",
    "/Object_topic",
    "/ego_setting",
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
    },
    {
        "id": "p02",
        "stem": "od_poc_02",
        "anchor_index": 2516,
        "seed": 26080602,
        "npc_count": 5,
        "pedestrian_count": 4,
        "obstacle_count": 2,
    },
    {
        "id": "p03",
        "stem": "od_poc_03",
        "anchor_index": 3500,
        "seed": 26080603,
        "npc_count": 4,
        "pedestrian_count": 5,
        "obstacle_count": 3,
    },
)


def parse_args():
    project_root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(
        description="Generate three MORAI ROS auto-capture proof-of-concept scenes."
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
            "Complete ROS bridge network profile saved by the same MORAI "
            "version. It must include the system UNIQUEID=-100 handlers such "
            "as /ego_setting. Defaults to "
            "<install-dir>/EgoNetwork/ros_path_probe_MN.json for backwards "
            "compatibility, but teleport mode normally requires the full "
            "SaveFile/Network/.../ros.json profile."
        ),
    )
    return parser.parse_args()


def copy_file(source, destination):
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_bytes(source.read_bytes())


def load_ros_network_template(path):
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

    configured_topics = set()
    ros_bridge_entries = 0
    for ego in ego_network_data:
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

    missing = sorted(set(REQUIRED_ROS_TOPICS) - configured_topics)
    if ros_bridge_entries == 0 or missing:
        raise ValueError(
            "network template is not the required ROS bridge profile; "
            "missing topics={}: {}".format(missing, path)
        )
    return document


def resolve_network_template(args, install_dir):
    if args.network_template is not None:
        path = args.network_template.expanduser().resolve()
    else:
        path = install_dir / "EgoNetwork/ros_path_probe_MN.json"
    document = load_ros_network_template(path)
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    return path, digest


def main():
    args = parse_args()
    project_root = args.project_root.resolve()
    workspace = project_root / "dataset/morai_od"
    generator = workspace / "scripts/generate_morai_od_prototype.py"
    manifest_path = workspace / "manifests/ros_capture_poc.json"
    install_dir = args.install_dir.expanduser().resolve()
    network_template, network_sha256 = resolve_network_template(
        args, install_dir
    )
    scenes = []

    for spec in SCENE_SPECS:
        command = [
            sys.executable,
            str(generator),
            "--output-root",
            str(workspace),
            "--scenario-group",
            "ros_capture_poc",
            "--output-stem",
            spec["stem"],
            "--anchor-index",
            str(spec["anchor_index"]),
            "--seed",
            str(spec["seed"]),
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
        subprocess.run(command, check=True)

        scenario_rel = Path(
            "dataset/morai_od/scenarios/ros_capture_poc"
        ) / f"{spec['stem']}.json"
        metadata_rel = Path(
            "dataset/morai_od/metadata/ros_capture_poc"
        ) / f"{spec['stem']}.metadata.json"
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
        copy_file(network_template, network_sidecar)
        if network_sidecar.read_bytes() != network_template.read_bytes():
            raise RuntimeError(
                "installed network sidecar differs from template: {}".format(
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
                "expected_ego": metadata["ego"],
                "expected_counts": {
                    "npc": spec["npc_count"],
                    "pedestrian": spec["pedestrian_count"],
                    "obstacle": spec["obstacle_count"],
                },
            }
        )

    manifest = {
        "schema_version": 2,
        "purpose": "MORAI ROS ScenarioLoad and SaveSensorData proof of concept",
        "map_name": "R_KR_PR_K-city_2025",
        "network_policy": {
            "transport": "rosbridge",
            "load_network_connection_data": True,
            "required_topics": list(REQUIRED_ROS_TOPICS),
            "template_sha256": network_sha256,
            "note": (
                "Each scenario must have EgoNetwork/<stem>_MN.json copied "
                "from a ROS bridge profile saved by the target MORAI version."
            ),
        },
        "environment": {
            "weather": "sunny",
            "time_hour": 13,
            "note": "Set once in the MORAI UI before running this PoC.",
        },
        "waypoints": "configs/global_path/2026_molit_comp_global_path.txt",
        "capture_policy": {
            "captures_per_scene": 6,
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

    print("MORAI ROS capture PoC set generated")
    print(f"manifest: {manifest_path}")
    print(f"scenes: {len(scenes)}")
    print(f"MORAI install: {install_dir}")
    print(f"ROS network template: {network_template}")
    print(f"ROS network SHA256: {network_sha256}")
    print(f"ROS network sidecars: {len(scenes)}")


if __name__ == "__main__":
    main()
