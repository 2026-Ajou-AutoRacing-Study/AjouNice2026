#!/usr/bin/env python3
"""Remove a known-bad actor from failed Base3 scenes and emit safe retries."""

import argparse
import copy
import json
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
ACTOR_LISTS = {
    "vehicle": ("vehicleList", "npc", "static_npc"),
    "pedestrian": ("pedestrianList", "pedestrian", "pedestrian"),
    "obstacle": ("objectList", "obstacle", "obstacle"),
}


def load_json(path):
    with Path(path).open("r", encoding="utf-8") as stream:
        return json.load(stream)


def resolve(path):
    path = Path(path)
    return path if path.is_absolute() else PROJECT_ROOT / path


def unique_id(actor):
    for key in ("UNIQUEID", "uniqueId", "uniqueID", "unique_id"):
        if key in actor:
            return int(actor[key])
    raise KeyError("actor has no unique id")


def parse_remove(value):
    scene_id, class_name, actor_id = value.rsplit(":", 2)
    if class_name not in ACTOR_LISTS:
        raise argparse.ArgumentTypeError("unknown actor class: {}".format(class_name))
    return scene_id, class_name, int(actor_id)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-manifest", type=Path, required=True)
    parser.add_argument("--remove", type=parse_remove, action="append", required=True)
    parser.add_argument("--output-manifest", type=Path, required=True)
    parser.add_argument("--output-scenario-dir", type=Path, required=True)
    parser.add_argument("--output-metadata-dir", type=Path, required=True)
    parser.add_argument("--suffix", default="safe1")
    parser.add_argument("--name-prefix", default="b3srepv2")
    parser.add_argument(
        "--dataset-id",
        default="morai_static_classcentric_base3_10k_replacement_v2",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    args.output_manifest = resolve(args.output_manifest)
    args.output_scenario_dir = resolve(args.output_scenario_dir)
    args.output_metadata_dir = resolve(args.output_metadata_dir)
    source = load_json(args.source_manifest)
    by_id = {scene["id"]: scene for scene in source["scenes"]}
    args.output_scenario_dir.mkdir(parents=True, exist_ok=True)
    args.output_metadata_dir.mkdir(parents=True, exist_ok=True)
    selected = []
    operations = []

    family_codes = {"vehicle": "sv", "pedestrian": "sp", "obstacle": "so"}
    for operation_index, (scene_id, class_name, actor_id) in enumerate(args.remove, 1):
        source_scene = by_id[scene_id]
        scene = copy.deepcopy(source_scene)
        scenario = load_json(resolve(scene["scenario"]))
        metadata = load_json(resolve(scene["metadata"]))
        list_name, expected_count, expected_detail = ACTOR_LISTS[class_name]

        before = len(scenario[list_name])
        scenario[list_name] = [
            actor for actor in scenario[list_name] if unique_id(actor) != actor_id
        ]
        if len(scenario[list_name]) != before - 1:
            raise RuntimeError(
                "{} {} did not contain actor {}".format(scene_id, class_name, actor_id)
            )
        metadata["runtime_actors"] = [
            actor
            for actor in metadata.get("runtime_actors", [])
            if not (
                actor.get("class") == class_name
                and int(actor.get("unique_id")) == actor_id
            )
        ]

        code = family_codes[class_name]
        new_id = "{}_d{}{:03d}".format(args.name_prefix, code, operation_index)
        new_file = "{}_{}{:03d}".format(args.name_prefix, code, operation_index)
        scenario_path = args.output_scenario_dir / "{}.json".format(new_file)
        metadata_path = args.output_metadata_dir / "{}.metadata.json".format(new_file)
        scenario_path.write_text(
            json.dumps(scenario, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )

        metadata["scene_id"] = new_id
        metadata["scenario_stem"] = new_file
        if "validation" in metadata and "unique_id_count" in metadata["validation"]:
            metadata["validation"]["unique_id_count"] -= 1
        note = "safe retry removed {} actor {} after deterministic runtime failure".format(
            class_name, actor_id
        )
        notes = metadata.setdefault("notes", {})
        if isinstance(notes, dict):
            notes["safe_retry"] = note
        else:
            notes.append(note)
        metadata_path.write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )

        scene["id"] = new_id
        scene["file"] = new_file
        scene["file_name"] = new_file
        scene["scenario"] = str(scenario_path.relative_to(PROJECT_ROOT))
        scene["metadata"] = str(metadata_path.relative_to(PROJECT_ROOT))
        scene["sequence_contract"]["split_group"] = new_id
        scene["expected_counts"][expected_count] -= 1
        scene["expected"][expected_detail] -= 1
        selected.append(scene)
        operations.append(
            {
                "source_scene_id": scene_id,
                "safe_scene_id": new_id,
                "removed_class": class_name,
                "removed_unique_id": actor_id,
            }
        )

    output = copy.deepcopy(source)
    output["dataset_id"] = args.dataset_id
    output["scenario_group"] = args.name_prefix
    output["scenes"] = selected
    output["generation"] = {
        "purpose": "safe_retry_after_deterministic_actor_failure",
        "replacement_clip_count": len(selected),
        "replacement_frame_count": 3 * len(selected),
        "operations": operations,
    }
    args.output_manifest.parent.mkdir(parents=True, exist_ok=True)
    args.output_manifest.write_text(
        json.dumps(output, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print("safe retry clips: {}".format(len(selected)))
    print("output: {}".format(args.output_manifest))


if __name__ == "__main__":
    main()
