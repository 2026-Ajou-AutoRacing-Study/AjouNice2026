#!/usr/bin/env python3
"""Extract a reproducible MORAI asset catalog from user-built scenarios."""

import argparse
import csv
import hashlib
import json
from collections import Counter
from pathlib import Path


def parse_args():
    project_root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario-dir", type=Path, required=True)
    parser.add_argument("--option-list", type=Path, required=True)
    parser.add_argument(
        "--vehicle-file",
        default="vehicle_list.json",
        help="MORAI scenario containing one instance of each vehicleList asset.",
    )
    parser.add_argument(
        "--pedestrian-file",
        default="pedestrian_list.json",
        help=(
            "MORAI scenario containing one instance of each pedestrianList "
            "asset."
        ),
    )
    parser.add_argument(
        "--obstacle-file",
        default="obstacle_list_without_vehicle_and_ped_like_objects.json",
        help="Curated MORAI scenario containing the allowed objectList assets.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=(
            project_root
            / "dataset/morai_od/configs/morai_asset_catalog_260807.json"
        ),
    )
    return parser.parse_args()


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_names(path):
    names = {}
    with path.open(newline="", encoding="utf-8", errors="replace") as stream:
        reader = csv.reader(stream)
        next(reader, None)
        for row in reader:
            if len(row) >= 4 and row[0].isdigit():
                names[int(row[0])] = row[3]
    return names


def vehicle_policy(data_id, name):
    prefix = data_id // 100000
    subtype = {
        301: "passenger_car",
        302: "suv",
        303: "bus_van",
        304: "truck",
        305: "wagon",
        307: "electric_passenger_car",
        308: "mpv",
    }.get(prefix, "vehicle_other")
    return {
        "ontology_class": "Vehicle",
        "subtype": subtype,
        "pointpainting_pedestrian_role": "context_negative",
        "future_bev_role": "positive",
        "generation_policy": "enabled",
        "label_audit_required": False,
    }


def pedestrian_policy(data_id, name, source_list):
    lowered = name.lower()
    if any(token in lowered for token in ("dog", "deer")):
        ontology = "Animal"
        subtype = "animal"
        policy = "hard_negative_only"
        audit = True
    elif "pet" in lowered:
        ontology = "CompositePersonAnimal"
        subtype = "person_with_pet"
        policy = "quarantine"
        audit = True
    elif "bicycle" in lowered:
        ontology = "Cyclist"
        subtype = "bicycle_rider"
        policy = "quarantine"
        audit = True
    elif "ncap" in lowered:
        ontology = "Pedestrian"
        subtype = "pedestrian_dummy"
        policy = "limited_weight"
        audit = True
    else:
        ontology = "Pedestrian"
        subtype = "ai_pedestrian" if source_list == "aiPedstrianList" else "person"
        policy = "enabled"
        audit = source_list == "aiPedstrianList"
    return {
        "ontology_class": ontology,
        "subtype": subtype,
        "pointpainting_pedestrian_role": (
            "positive"
            if ontology == "Pedestrian"
            else "hard_negative_or_excluded"
        ),
        "future_bev_role": (
            "positive" if ontology in ("Pedestrian", "Cyclist") else "separate_class"
        ),
        "generation_policy": policy,
        "label_audit_required": audit,
    }


def obstacle_policy(data_id, name):
    lowered = name.lower()
    if any(token in lowered for token in ("bike", "scooter", "bicycle")):
        ontology = "Obstacle"
        subtype = "unoccupied_bicycle_or_scooter"
        policy = "enabled_realistic_zone_only"
        audit = True
    elif "stroller" in lowered:
        ontology = "Stroller"
        subtype = "stroller"
        policy = "hard_negative_only"
        audit = True
    elif any(token in lowered for token in ("sign",)):
        ontology = "RoadFurniture"
        subtype = "road_sign"
        policy = "enabled_semantic_zone_only"
        audit = False
    elif any(token in lowered for token in ("tree", "streetlight")):
        ontology = "RoadsideFurniture"
        subtype = "fixed_roadside_object"
        policy = "enabled_roadside_only"
        audit = False
    elif any(token in lowered for token in ("speed_bump", "hole-cover")):
        ontology = "RoadSurfaceObject"
        subtype = "road_surface"
        policy = "enabled_surface_aligned_only"
        audit = True
    elif any(token in lowered for token in ("box", "cargo")):
        ontology = "Obstacle"
        subtype = "box"
        policy = "enabled"
        audit = False
    elif any(token in lowered for token in ("barrel", "drum")):
        ontology = "Obstacle"
        subtype = "barrel_or_drum"
        policy = "enabled"
        audit = False
    elif any(
        token in lowered
        for token in (
            "barrier",
            "barricade",
            "firewall",
            "separation",
            "bollard",
        )
    ):
        ontology = "Obstacle"
        subtype = "barrier"
        policy = "enabled"
        audit = False
    else:
        ontology = "Obstacle"
        subtype = "other"
        policy = "enabled_after_visual_review"
        audit = True
    return {
        "ontology_class": ontology,
        "subtype": subtype,
        "pointpainting_pedestrian_role": (
            "context_background"
            if subtype == "unoccupied_bicycle_or_scooter"
            else "hard_negative_or_context"
        ),
        "future_bev_role": (
            "candidate_two_wheeler"
            if subtype == "unoccupied_bicycle_or_scooter"
            else (
                "positive"
                if ontology == "Obstacle"
                else "context_or_separate_class"
            )
        ),
        "generation_policy": policy,
        "label_audit_required": audit,
    }


def extract_entries(document, source_file, source_list, names, policy_fn):
    rows = document.get(source_list, [])
    entries = []
    occurrences = Counter(row["DataID"] for row in rows)
    seen = set()
    for source_index, row in enumerate(rows):
        data_id = int(row["DataID"])
        if data_id in seen:
            continue
        seen.add(data_id)
        name = names.get(data_id, "UNKNOWN_{}".format(data_id))
        policy = policy_fn(data_id, name)
        entries.append(
            {
                "data_id": data_id,
                "name": name,
                "source_file": source_file,
                "source_list": source_list,
                "source_index": source_index,
                "source_occurrences": occurrences[data_id],
                "morai_object_type": row.get("m_eobstacleObjType"),
                **policy,
            }
        )
    return entries


def main():
    args = parse_args()
    scenario_dir = args.scenario_dir.expanduser().resolve()
    option_list = args.option_list.expanduser().resolve()
    names = load_names(option_list)
    sources = {
        "vehicle": (args.vehicle_file, "vehicleList"),
        "pedestrian": (args.pedestrian_file, "pedestrianList"),
        "obstacle": (
            args.obstacle_file,
            "objectList",
        ),
    }
    documents = {}
    source_metadata = {}
    for key, (file_name, _source_list) in sources.items():
        path = scenario_dir / file_name
        documents[key] = json.loads(path.read_text(encoding="utf-8"))
        source_metadata[key] = {
            "file_name": file_name,
            "sha256": sha256(path),
        }

    vehicles = extract_entries(
        documents["vehicle"],
        sources["vehicle"][0],
        sources["vehicle"][1],
        names,
        vehicle_policy,
    )
    pedestrians = extract_entries(
        documents["pedestrian"],
        sources["pedestrian"][0],
        sources["pedestrian"][1],
        names,
        lambda data_id, name: pedestrian_policy(
            data_id, name, "pedestrianList"
        ),
    )
    pedestrians.extend(
        extract_entries(
            documents["pedestrian"],
            sources["pedestrian"][0],
            "aiPedstrianList",
            names,
            lambda data_id, name: pedestrian_policy(
                data_id, name, "aiPedstrianList"
            ),
        )
    )
    obstacles = extract_entries(
        documents["obstacle"],
        sources["obstacle"][0],
        sources["obstacle"][1],
        names,
        obstacle_policy,
    )

    catalog = {
        "schema_version": 1,
        "generated_from": source_metadata,
        "option_list": {
            "file_name": option_list.name,
            "sha256": sha256(option_list),
        },
        "policy": {
            "vehicle_shaped_object_entries": (
                "excluded by user before catalog extraction"
            ),
            "human_integrated_obstacle_assets": (
                "excluded by user before catalog extraction; only unoccupied "
                "bicycle/scooter assets remain eligible as hard negatives"
            ),
            "deduplicate_by_data_id": True,
        },
        "counts": {
            "vehicles": len(vehicles),
            "pedestrian_container_assets": len(pedestrians),
            "obstacles": len(obstacles),
            "total_unique_entries": (
                len(vehicles) + len(pedestrians) + len(obstacles)
            ),
        },
        "vehicles": vehicles,
        "pedestrians_and_related": pedestrians,
        "obstacles_and_related": obstacles,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(catalog, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print("catalog:", args.output)
    print("counts:", catalog["counts"])


if __name__ == "__main__":
    main()
