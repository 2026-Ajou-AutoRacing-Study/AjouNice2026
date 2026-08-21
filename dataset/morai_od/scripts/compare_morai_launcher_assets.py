#!/usr/bin/env python3
"""Compare competition and personal MORAI asset catalogs and templates."""

import argparse
import json
from pathlib import Path


GROUPS = (
    ("vehicle", "vehicles", "vehicleList"),
    ("pedestrian", "pedestrians_and_related", "pedestrianList"),
    ("obstacle", "obstacles_and_related", "objectList"),
)

# Trees are static map context, not target OD assets for this dataset.  Keep
# this policy separate from launcher compatibility so that a tree present in
# both launchers is still never selected automatically.
POLICY_EXCLUDED_NAME_PREFIXES = ("tree",)
POLICY_EXCLUDED_DATA_IDS = {
    "pedestrian": {},
    "obstacle": {
        40100032: (
            "Speed_Bump is a thin road-surface infrastructure asset, not a "
            "target object or mission element in the competition. Excluded "
            "from both-launcher automatic OD generation to avoid distorting "
            "3D box geometry and class balance."
        ),
    },
}

IGNORED_TEMPLATE_KEYS = {
    "UNIQUEID",
    "pos",
    "rot",
    "initPos",
    "initRot",
    "standardPos",
    "waypointEventList",
    "listObjIDForBehavior",
    "stateNormalizedTime",
}


def policy_exclusion_reason(group, row):
    name = str(row.get("name", "")).strip().lower()
    if any(name.startswith(prefix) for prefix in POLICY_EXCLUDED_NAME_PREFIXES):
        return (
            "Tree asset excluded from the OD actor dataset by project policy."
        )
    data_id = int(row.get("data_id", row.get("DataID", -1)))
    return POLICY_EXCLUDED_DATA_IDS.get(group, {}).get(data_id)


def parse_args():
    project_root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--competition-catalog",
        type=Path,
        default=project_root
        / "dataset/morai_od/configs/morai_asset_catalog_260807.json",
    )
    parser.add_argument(
        "--personal-catalog",
        type=Path,
        default=project_root
        / (
            "dataset/morai_od/configs/"
            "morai_asset_catalog_personal_260807.json"
        ),
    )
    parser.add_argument(
        "--competition-scenario-dir",
        type=Path,
        default=Path.home()
        / (
            "MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/"
            "Scenario/R_KR_PR_K-city_2025"
        ),
    )
    parser.add_argument(
        "--personal-scenario-dir",
        type=Path,
        default=Path.home()
        / (
            "MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/"
            "Scenario/R_KR_PR_K-city_2025"
        ),
    )
    parser.add_argument(
        "--capture-report",
        type=Path,
        default=Path.home()
        / (
            "morai_cap/sunny13_180_smoke_260807_125533/"
            "ros_capture_poc_report.json"
        ),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=project_root
        / (
            "dataset/morai_od/configs/"
            "morai_asset_launcher_compatibility_260807.json"
        ),
    )
    return parser.parse_args()


def load_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def source_documents(catalog, scenario_dir):
    documents = {}
    for key, _catalog_key, source_list in GROUPS:
        source = catalog["generated_from"][key]["file_name"]
        document = load_json(scenario_dir / source)
        lists = (source_list,)
        if key == "pedestrian":
            lists = ("pedestrianList", "aiPedstrianList")
        documents[key] = {}
        for list_name in lists:
            documents[key].update(
                {
                    int(row["DataID"]): row
                    for row in document.get(list_name, [])
                }
            )
    return documents


def normalize_template(value):
    if isinstance(value, dict):
        return {
            key: normalize_template(child)
            for key, child in sorted(value.items())
            if key not in IGNORED_TEMPLATE_KEYS
        }
    if isinstance(value, list):
        return [normalize_template(child) for child in value]
    return value


def changed_paths(left, right, prefix=""):
    if isinstance(left, dict) and isinstance(right, dict):
        changes = []
        for key in sorted(set(left) | set(right)):
            path = "{}.{}".format(prefix, key) if prefix else key
            if key not in left:
                changes.append({"path": path, "change": "personal_only_key"})
            elif key not in right:
                changes.append(
                    {"path": path, "change": "competition_only_key"}
                )
            else:
                changes.extend(changed_paths(left[key], right[key], path))
        return changes
    if left != right:
        return [
            {
                "path": prefix,
                "change": "value_changed",
                "competition": left,
                "personal": right,
            }
        ]
    return []


def flatten_changes(value):
    result = []
    for item in value:
        if isinstance(item, tuple):
            result.extend(item)
        else:
            result.append(item)
    return result


def empirical_outcomes(report_path, metadata_dir):
    if not report_path.is_file():
        return {}
    report = load_json(report_path)
    outcomes = {}
    for scene in report.get("scenes", []):
        number = int(scene["id"].split("_")[-1])
        metadata_path = metadata_dir / "od_s13_{:03d}.metadata.json".format(
            number
        )
        if not metadata_path.is_file():
            continue
        metadata = load_json(metadata_path)
        for asset in metadata["assets"]:
            key = (asset["group"], int(asset["data_id"]))
            row = outcomes.setdefault(
                key,
                {"appearances": 0, "completed_scenes": 0, "failed_scenes": 0},
            )
            row["appearances"] += 1
            if scene.get("status") == "completed":
                row["completed_scenes"] += 1
            else:
                row["failed_scenes"] += 1
    return outcomes


def main():
    args = parse_args()
    competition_catalog = load_json(args.competition_catalog.resolve())
    personal_catalog = load_json(args.personal_catalog.resolve())
    competition_documents = source_documents(
        competition_catalog,
        args.competition_scenario_dir.expanduser().resolve(),
    )
    personal_documents = source_documents(
        personal_catalog,
        args.personal_scenario_dir.expanduser().resolve(),
    )
    metadata_dir = (
        Path(__file__).resolve().parents[1]
        / "metadata/sunny13_180_smoke"
    )
    outcomes = empirical_outcomes(
        args.capture_report.expanduser().resolve(), metadata_dir
    )

    result = {
        "schema_version": 2,
        "policy": {
            "target_domain": "competition_launcher",
            "capture_runtime": "personal_launcher",
            "automatic_asset_rule": (
                "Competition catalog AND personal catalog; instantiate using "
                "the personal launcher template."
            ),
            "competition_only_rule": (
                "Exclude from personal automatic capture and list for "
                "separate competition-launcher manual capture."
            ),
            "personal_only_rule": (
                "Exclude because the target competition launcher does not "
                "provide the asset."
            ),
            "dataset_scope_exclusion_rule": (
                "Exclude Tree-prefixed assets regardless of launcher "
                "availability because trees are static map context and are "
                "outside the target OD actor catalog."
            ),
            "dataset_scope_excluded_name_prefixes": list(
                POLICY_EXCLUDED_NAME_PREFIXES
            ),
            "runtime_observed_excluded_data_ids": {
                group: sorted(rows)
                for group, rows in POLICY_EXCLUDED_DATA_IDS.items()
            },
            "geometry_fields_from_catalog_scenarios": False,
        },
        "competition": {
            "catalog": str(args.competition_catalog),
            "generated_from": competition_catalog["generated_from"],
            "counts": competition_catalog["counts"],
        },
        "personal": {
            "catalog": str(args.personal_catalog),
            "generated_from": personal_catalog["generated_from"],
            "counts": personal_catalog["counts"],
        },
        "automatic_eligible": {},
        "competition_only_manual": {},
        "personal_only_excluded": {},
        "policy_excluded": {},
        "common_template_differences": {},
    }

    for group, catalog_key, source_list in GROUPS:
        competition_rows = {
            int(row["data_id"]): row
            for row in competition_catalog[catalog_key]
        }
        personal_rows = {
            int(row["data_id"]): row
            for row in personal_catalog[catalog_key]
        }
        competition_ids = set(competition_rows)
        personal_ids = set(personal_rows)
        all_ids = competition_ids | personal_ids
        excluded_ids = {
            data_id
            for data_id in all_ids
            if policy_exclusion_reason(
                group,
                competition_rows.get(data_id, personal_rows.get(data_id))
            )
        }
        common = sorted((competition_ids & personal_ids) - excluded_ids)
        result["automatic_eligible"][group] = common
        result["competition_only_manual"][group] = [
            {
                "data_id": data_id,
                "name": competition_rows[data_id]["name"],
                "empirical_personal_capture": outcomes.get(
                    (source_list, data_id),
                    {"appearances": 0, "completed_scenes": 0, "failed_scenes": 0},
                ),
                "note": (
                    "Absence from the user-curated personal list is not proof "
                    "that the runtime cannot render it; manually audit if "
                    "empirical completed_scenes is nonzero."
                ),
            }
            for data_id in sorted(
                (competition_ids - personal_ids) - excluded_ids
            )
        ]
        result["personal_only_excluded"][group] = [
            {
                "data_id": data_id,
                "name": personal_rows[data_id]["name"],
            }
            for data_id in sorted(
                (personal_ids - competition_ids) - excluded_ids
            )
        ]
        result["policy_excluded"][group] = [
            {
                "data_id": data_id,
                "name": (
                    competition_rows.get(data_id)
                    or personal_rows[data_id]
                )["name"],
                "present_in_competition": data_id in competition_ids,
                "present_in_personal": data_id in personal_ids,
                "reason": policy_exclusion_reason(
                    group,
                    competition_rows.get(data_id, personal_rows.get(data_id)),
                ),
            }
            for data_id in sorted(excluded_ids)
        ]
        differences = []
        for data_id in common:
            left = normalize_template(competition_documents[group][data_id])
            right = normalize_template(personal_documents[group][data_id])
            changes = flatten_changes(changed_paths(left, right))
            if changes:
                differences.append(
                    {
                        "data_id": data_id,
                        "competition_name": competition_rows[data_id]["name"],
                        "personal_name": personal_rows[data_id]["name"],
                        "changed_fields": changes,
                    }
                )
        result["common_template_differences"][group] = differences

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print("compatibility:", args.output)
    for key in (
        "automatic_eligible",
        "competition_only_manual",
        "personal_only_excluded",
        "policy_excluded",
        "common_template_differences",
    ):
        print(
            key,
            {
                group: len(rows)
                for group, rows in result[key].items()
            },
        )


if __name__ == "__main__":
    main()
