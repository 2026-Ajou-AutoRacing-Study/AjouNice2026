#!/usr/bin/env python3
"""Install dynamic scenarios and one verified Ego ROS sidecar into MORAI."""

import argparse
import copy
import hashlib
import json
import shutil
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_INSTALL = Path.home() / (
    "MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/Scenario/"
    "R_KR_PR_K-city_2025"
)
REQUIRED_TOPICS = {
    "/ScenarioLoad",
    "/SaveSensorData",
    "/Ego_topic",
    "/Object_topic",
}


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--network-template", type=Path, required=True)
    parser.add_argument("--install-dir", type=Path, default=DEFAULT_INSTALL)
    parser.add_argument(
        "--runtime-manifest",
        type=Path,
        help=(
            "SaveSensorData runner manifest. Defaults to "
            "<manifest_stem>_runtime.json beside the source manifest."
        ),
    )
    return parser.parse_args()


def load_json(path):
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def collect_strings(value):
    if isinstance(value, str):
        yield value
    elif isinstance(value, dict):
        for item in value.values():
            yield from collect_strings(item)
    elif isinstance(value, list):
        for item in value:
            yield from collect_strings(item)


def network_unique_ids(document):
    result = set()
    for entry in document.get("egoNetworkData", []):
        for key in ("uniqueId", "UNIQUEID", "uniqueID"):
            if key in entry:
                result.add(int(entry[key]))
                break
    return result


def resolve(value):
    path = Path(value)
    return path if path.is_absolute() else PROJECT_ROOT / path


def main():
    args = parse_args()
    manifest = load_json(args.manifest)
    network = load_json(args.network_template)
    unique_ids = network_unique_ids(network)
    strings = set(collect_strings(network))
    missing_topics = sorted(REQUIRED_TOPICS - strings)
    if unique_ids != {0}:
        raise SystemExit(
            f"network template must contain only Ego UNIQUEID=0, got {sorted(unique_ids)}"
        )
    if missing_topics:
        raise SystemExit(f"network template misses required topics: {missing_topics}")

    args.install_dir.mkdir(parents=True, exist_ok=True)
    network_dir = args.install_dir / "EgoNetwork"
    network_dir.mkdir(parents=True, exist_ok=True)
    installed = []
    for scene in manifest["scenes"]:
        source = resolve(scene["scenario"])
        scenario_target = args.install_dir / f"{scene['file']}.json"
        sidecar_target = network_dir / f"{scene['file']}_MN.json"
        shutil.copy2(source, scenario_target)
        shutil.copy2(args.network_template, sidecar_target)
        if load_json(source) != load_json(scenario_target):
            raise SystemExit(f"installed scenario differs from source: {scenario_target}")
        installed.append(
            {
                "id": scene["id"],
                "scenario": str(scenario_target),
                "network_sidecar": str(sidecar_target),
            }
        )

    report = {
        "manifest": str(args.manifest.resolve()),
        "install_dir": str(args.install_dir.resolve()),
        "network_template": str(args.network_template.resolve()),
        "network_template_sha256": sha256(args.network_template),
        "scenario_count": len(installed),
        "installed": installed,
    }
    runtime_manifest = copy.deepcopy(manifest)
    runtime_manifest["network_policy"] = {
        "transport": "rosbridge",
        "load_network_connection_data": True,
        "template_sha256": sha256(args.network_template),
        "required_topics": sorted(REQUIRED_TOPICS),
        "required_unique_ids": [0],
        "forbidden_unique_ids": [],
    }
    for scene in runtime_manifest["scenes"]:
        scene["network_sidecar"] = "EgoNetwork/{}_MN.json".format(
            scene["file_name"]
        )
    runtime_path = args.runtime_manifest or args.manifest.with_name(
        args.manifest.stem + "_runtime.json"
    )
    runtime_path.parent.mkdir(parents=True, exist_ok=True)
    with runtime_path.open("w", encoding="utf-8") as stream:
        json.dump(runtime_manifest, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    report["runtime_manifest"] = str(runtime_path.resolve())
    report_path = (
        PROJECT_ROOT
        / "dataset/morai_od/reports"
        / f"{args.manifest.stem}_installation.json"
    )
    report_path.parent.mkdir(parents=True, exist_ok=True)
    with report_path.open("w", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print(f"installed scenarios: {len(installed)}")
    print(f"install dir: {args.install_dir}")
    print(f"report: {report_path}")
    print(f"runtime manifest: {runtime_path}")


if __name__ == "__main__":
    main()
