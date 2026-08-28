#!/usr/bin/env python3
"""Audit one generated Base3-Static manifest without launching MORAI."""

import argparse
import csv
import json
import math
from collections import Counter, defaultdict
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
EXPECTED_CLASSES = {
    "stopped_vehicle": {"vehicle"},
    "static_obstacle": {"obstacle"},
    "standing_pedestrian": {"pedestrian"},
    "realistic_mixed_static": {"vehicle", "pedestrian", "obstacle"},
    "sparse_background": set(),
}


def load_json(path):
    with Path(path).open("r", encoding="utf-8") as stream:
        return json.load(stream)


def resolve(value):
    path = Path(value)
    return path if path.is_absolute() else PROJECT_ROOT / path


def local_sector(ego, position):
    yaw = math.radians(float(ego.get("yaw", 0.0)))
    dx = float(position["x"]) - float(ego["x"])
    dy = float(position["y"]) - float(ego["y"])
    longitudinal = math.cos(yaw) * dx + math.sin(yaw) * dy
    lateral = -math.sin(yaw) * dx + math.cos(yaw) * dy
    if longitudinal >= 5.0:
        sector = "front"
    elif longitudinal <= -5.0:
        sector = "rear"
    elif lateral >= 0.0:
        sector = "left"
    else:
        sector = "right"
    return longitudinal, lateral, sector


def circular_gap(values, circumference):
    if not values:
        return None
    ordered = sorted(float(value) % circumference for value in values)
    gaps = [after - before for before, after in zip(ordered, ordered[1:])]
    gaps.append(circumference - ordered[-1] + ordered[0])
    return max(gaps)


def write_csv(path, rows):
    fields = sorted({key for row in rows for key in row})
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def exact_weight_quotas(total, rows):
    weights = {str(row["name"]): float(row.get("weight", 1.0)) for row in rows}
    weight_sum = sum(weights.values())
    exact = {name: int(total) * weight / weight_sum for name, weight in weights.items()}
    quotas = {name: int(math.floor(value)) for name, value in exact.items()}
    remaining = int(total) - sum(quotas.values())
    for name in sorted(weights, key=lambda item: (-(exact[item] - quotas[item]), item))[:remaining]:
        quotas[name] += 1
    return quotas


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--placement-pilot",
        action="store_true",
        help=(
            "Audit placement safety and route geometry without requiring the "
            "full 10K campaign's class/asset coverage quotas."
        ),
    )
    args = parser.parse_args()

    manifest = load_json(args.manifest)
    config = load_json(resolve(manifest["generation"]["config"]))
    expected_family_counts = {
        row["name"]: int(row["count"]) for row in config["families"]
    }
    config_family_rows = {row["name"]: row for row in config["families"]}
    route_length = float(
        manifest["generation"]["route_reconstruction"]["length_m"]
    )

    scene_rows = []
    actor_rows = []
    family_counts = Counter()
    split_counts = Counter()
    family_split_counts = defaultdict(Counter)
    family_bins = defaultdict(set)
    family_arcs = defaultdict(list)
    family_crosswalks = defaultdict(set)
    family_strata = defaultdict(Counter)
    family_roundabout_phases = defaultdict(Counter)
    family_highway_lanes = defaultdict(Counter)
    class_assets = defaultdict(set)
    class_sectors = defaultdict(Counter)
    class_capture_sectors = defaultdict(Counter)
    class_capture_distance_bins = defaultdict(Counter)
    geometry_roles = Counter()
    vehicle_geometry_roles = Counter()
    roundabout_vehicle_geometry_roles = Counter()
    highway_vehicle_geometry_roles = Counter()
    actor_counts = defaultdict(list)
    opposite_heading_vehicle_count = 0
    same_lane_final_separation_violations = []
    same_lane_front_count_distribution = Counter()
    same_lane_rear_count_distribution = Counter()
    diagonal_vehicle_quadrants = Counter()
    errors = []

    for entry in manifest["scenes"]:
        metadata = load_json(resolve(entry["metadata"]))
        scenario = load_json(resolve(entry["scenario"]))
        family = metadata["mission_family"]
        split = metadata["split"]
        actors = metadata.get("runtime_actors", [])
        ego = metadata["ego"]["pose"]
        spatial = metadata.get("spatial_sampling", {})
        family_counts[family] += 1
        split_counts[split] += 1
        family_split_counts[family][split] += 1
        actor_counts[family].append(len(actors))
        if family in {"stopped_vehicle", "realistic_mixed_static"}:
            same_lane_front_count_distribution[sum(
                actor.get("geometry_role") == "same_lane_primary_front"
                for actor in actors
            )] += 1
            same_lane_rear_count_distribution[sum(
                actor.get("geometry_role") == "same_lane_optional_rear"
                for actor in actors
            )] += 1
        if spatial.get("route_wide_bin") is not None:
            family_bins[family].add(int(spatial["route_wide_bin"]))
        if spatial.get("route_arc_m") is not None:
            family_arcs[family].append(float(spatial["route_arc_m"]))
        if spatial.get("crosswalk_id"):
            family_crosswalks[family].add(spatial["crosswalk_id"])
        if spatial.get("spatial_stratum"):
            family_strata[family][spatial["spatial_stratum"]] += 1
        if spatial.get("roundabout_phase"):
            family_roundabout_phases[family][spatial["roundabout_phase"]] += 1
        if spatial.get("highway_lane_id") is not None:
            family_highway_lanes[family][str(spatial["highway_lane_id"])] += 1

        classes = {actor.get("class") for actor in actors}
        if classes != EXPECTED_CLASSES.get(family):
            errors.append(
                "{} has classes {}, expected {}".format(
                    entry["id"], sorted(classes), sorted(EXPECTED_CLASSES[family])
                )
            )
        if any(actor.get("motion") != "static" for actor in actors):
            errors.append("{} contains a non-static actor".format(entry["id"]))
        if scenario.get("spawnPointList") or scenario.get("pedspawnPointList"):
            errors.append("{} retains a dynamic spawn point".format(entry["id"]))
        if scenario.get("waypointDataList"):
            errors.append("{} retains an actor waypoint path".format(entry["id"]))
        for actor in scenario.get("vehicleList", []):
            moving_fields = (
                "linkVelocity",
                "constantVelocity",
                "desiredVelocity",
                "currentVelocity",
                "viewCurrentVelocity",
            )
            if any(float(actor.get(field, 0.0)) != 0.0 for field in moving_fields):
                errors.append("{} contains a moving Vehicle".format(entry["id"]))
        if any(
            float(actor.get("speed", 0.0)) != 0.0
            for actor in scenario.get("pedestrianList", [])
        ):
            errors.append("{} contains a moving Pedestrian".format(entry["id"]))
        if any(
            float(actor.get("moveSpeed", 0.0)) != 0.0
            for actor in scenario.get("objectList", [])
        ):
            errors.append("{} contains a moving Obstacle".format(entry["id"]))

        scene_rows.append(
            {
                "scene_id": entry["id"],
                "family": family,
                "split": split,
                "route_arc_m": spatial.get("route_arc_m"),
                "route_wide_bin": spatial.get("route_wide_bin"),
                "crosswalk_id": spatial.get("crosswalk_id"),
                "spatial_stratum": spatial.get("spatial_stratum"),
                "roundabout_phase": spatial.get("roundabout_phase"),
                "highway_lane_id": spatial.get("highway_lane_id"),
                "actor_count": len(actors),
                "ego_x_m": ego["x"],
                "ego_y_m": ego["y"],
                "ego_yaw_deg": ego.get("yaw"),
            }
        )
        for actor in actors:
            position = actor.get("position")
            if not position:
                errors.append("{} actor lacks position".format(entry["id"]))
                continue
            longitudinal, lateral, sector = local_sector(ego, position)
            actor_class = actor["class"]
            geometry_role = actor.get("geometry_role", "unspecified")
            if actor_class == "vehicle":
                if actor.get("opposite_heading_applied", False):
                    opposite_heading_vehicle_count += 1
                    errors.append(
                        "{} contains a same-lane 180-degree Vehicle flip".format(
                            entry["id"]
                        )
                    )
                planned = actor.get(
                    "planned_capture_relative_longitudinal_m"
                )
                if planned and geometry_role in {
                    "same_lane_safe_fallback",
                    "same_lane_queue",
                    "same_lane_primary_front",
                    "same_lane_optional_rear",
                }:
                    policy = config_family_rows[family].get(
                        "minimum_final_capture_separation_m_by_subtype", {}
                    )
                    minimum = float(policy.get(actor.get("subtype"), 8.0))
                    final = float(planned[-1])
                    if float(planned[0]) > 0.0 and final < minimum:
                        same_lane_final_separation_violations.append(
                            {
                                "scene_id": entry["id"],
                                "name": actor.get("name"),
                                "subtype": actor.get("subtype"),
                                "final_separation_m": final,
                                "minimum_separation_m": minimum,
                            }
                        )
                        errors.append(
                            "{} Vehicle final separation {:.2f}m < {:.2f}m".format(
                                entry["id"], final, minimum
                            )
                        )
                    physical_minimum = actor.get(
                        "planned_ego_minimum_physical_distance_m"
                    )
                    if (
                        geometry_role in {
                            "same_lane_safe_fallback",
                            "same_lane_primary_front",
                            "same_lane_optional_rear",
                        }
                        and physical_minimum is not None
                        and float(physical_minimum) < minimum
                    ):
                        same_lane_final_separation_violations.append(
                            {
                                "scene_id": entry["id"],
                                "name": actor.get("name"),
                                "subtype": actor.get("subtype"),
                                "physical_minimum_m": float(physical_minimum),
                                "minimum_separation_m": minimum,
                            }
                        )
                        errors.append(
                            "{} Vehicle physical clearance {:.2f}m < {:.2f}m".format(
                                entry["id"], float(physical_minimum), minimum
                            )
                        )
            geometry_roles[geometry_role] += 1
            if actor_class == "vehicle":
                vehicle_geometry_roles[geometry_role] += 1
            if actor_class == "vehicle" and spatial.get("roundabout_phase"):
                roundabout_vehicle_geometry_roles[geometry_role] += 1
            if actor_class == "vehicle" and spatial.get("highway_lane_id") is not None:
                highway_vehicle_geometry_roles[geometry_role] += 1
            class_assets[actor_class].add(
                (actor.get("data_id"), actor.get("name"), actor.get("subtype"))
            )
            class_sectors[actor_class][sector] += 1
            if actor_class == "vehicle" and (
                geometry_role.startswith("highway_parallel_lane_")
                or geometry_role.startswith("route_general_")
                or geometry_role == "roundabout_left_legal_path"
            ):
                longitudinal_side = "front" if longitudinal >= 0.0 else "rear"
                lateral_side = "left" if lateral >= 0.0 else "right"
                diagonal_vehicle_quadrants[
                    "{}_{}".format(longitudinal_side, lateral_side)
                ] += 1
            arm = float(
                metadata.get("capture_policy_overrides", {}).get(
                    "capture_motion_arm_m",
                    metadata["capture_policy"].get("capture_motion_arm_m", 0.0),
                )
            )
            spacing = float(metadata["capture_policy"].get("capture_spacing_m", 3.0))
            for frame_index in range(3):
                capture_longitudinal = longitudinal - arm - frame_index * spacing
                if capture_longitudinal >= 5.0:
                    capture_sector = "front"
                elif capture_longitudinal <= -5.0:
                    capture_sector = "rear"
                elif lateral >= 0.0:
                    capture_sector = "left"
                else:
                    capture_sector = "right"
                class_capture_sectors[actor_class][capture_sector] += 1
                capture_distance = math.hypot(capture_longitudinal, lateral)
                if capture_distance < 10.0:
                    distance_bin = "0-10m"
                elif capture_distance < 20.0:
                    distance_bin = "10-20m"
                elif capture_distance < 30.0:
                    distance_bin = "20-30m"
                else:
                    distance_bin = "30m+"
                class_capture_distance_bins[actor_class][distance_bin] += 1
            actor_rows.append(
                {
                    "scene_id": entry["id"],
                    "family": family,
                    "split": split,
                    "class": actor_class,
                    "data_id": actor.get("data_id"),
                    "name": actor.get("name"),
                    "subtype": actor.get("subtype"),
                    "longitudinal_m": longitudinal,
                    "lateral_m": lateral,
                    "sector": sector,
                    "geometry_role": geometry_role,
                }
            )

    if dict(family_counts) != expected_family_counts:
        errors.append(
            "family counts {} != config {}".format(
                dict(family_counts), expected_family_counts
            )
        )
    policy = manifest["capture_policy"]
    if int(policy.get("captures_per_scene", 0)) != 3:
        errors.append("capture policy is not 3 frames per clip")
    if not math.isclose(float(policy.get("capture_spacing_m", 0.0)), 3.0):
        errors.append("capture policy is not 3 m spacing")
    for family, expected in expected_family_counts.items():
        splits = family_split_counts[family]
        if sum(splits.values()) != expected:
            errors.append("{} split count does not sum to family quota".format(family))
        if (
            family != "standing_pedestrian"
            and expected >= 24
            and not config_family_rows[family].get("spatial_strata")
            and len(family_bins[family]) != 24
        ):
            errors.append("{} does not cover all 24 route bins".format(family))
        config_row = config_family_rows[family]
        if config_row.get("spatial_strata"):
            global_schedule = config_row.get("global_spatial_stratum_schedule")
            offset = int(config_row.get("family_global_variant_offset", 0))
            expected_strata = Counter(
                global_schedule[offset:offset + expected]
                if global_schedule else []
            )
            if (
                not args.placement_pilot
                and dict(family_strata[family]) != dict(expected_strata)
            ):
                errors.append(
                    "{} spatial strata {} != expected {}".format(
                        family, dict(family_strata[family]), dict(expected_strata)
                    )
                )
            lanes = family_highway_lanes[family]
            if lanes and max(lanes.values()) - min(lanes.values()) > 1:
                errors.append("{} highway lane counts are not balanced: {}".format(family, dict(lanes)))

    checks = {
        "family_counts_match_config": dict(family_counts) == expected_family_counts,
        "three_frames_per_clip": int(policy.get("captures_per_scene", 0)) == 3,
        "three_meter_spacing": math.isclose(
            float(policy.get("capture_spacing_m", 0.0)), 3.0
        ),
        "all_positive_families_cover_24_bins": all(
            len(family_bins[name]) == 24
            for name in EXPECTED_CLASSES
            if name != "standing_pedestrian"
            and expected_family_counts.get(name, 0) >= 24
            and not config_family_rows[name].get("spatial_strata")
        ),
        "pedestrian_general_route_uses_at_least_18_bins": len(
            family_bins["standing_pedestrian"]
        ) >= 18,
        "pedestrian_crosswalk_uses_at_least_5_locations": len(
            family_crosswalks["standing_pedestrian"]
        ) >= 5,
        "vehicle_asset_diversity_at_least_30": len(class_assets["vehicle"]) >= 30,
        "pedestrian_asset_diversity_at_least_8": len(class_assets["pedestrian"]) >= 8,
        "obstacle_asset_diversity_at_least_20": len(class_assets["obstacle"]) >= 20,
        "each_positive_class_has_front_side_and_rear_capture_views": all(
            class_capture_sectors[name]["front"] > 0
            and class_capture_sectors[name]["rear"] > 0
            and (
                class_capture_sectors[name]["left"]
                + class_capture_sectors[name]["right"]
            ) > 0
            for name in ("vehicle", "pedestrian", "obstacle")
        ),
        "each_positive_class_covers_all_distance_bands": all(
            all(
                class_capture_distance_bins[name][distance_bin] > 0
                for distance_bin in ("0-10m", "10-20m", "20-30m", "30m+")
            )
            for name in ("vehicle", "pedestrian", "obstacle")
        ),
        "all_actors_static": not any(
            "non-static actor" in error for error in errors
        ),
        "no_same_lane_180_degree_vehicle_flips": (
            opposite_heading_vehicle_count == 0
        ),
        "same_lane_vehicle_final_separation_safe": (
            not same_lane_final_separation_violations
        ),
        "vehicle_scenes_have_exactly_one_same_lane_front": (
            same_lane_front_count_distribution == Counter({1: sum(
                expected_family_counts.get(name, 0)
                for name in ("stopped_vehicle", "realistic_mixed_static")
            )})
        ),
        "vehicle_scenes_have_at_most_one_same_lane_rear": (
            not any(
                count > 1 and scenes > 0
                for count, scenes in same_lane_rear_count_distribution.items()
            )
        ),
        "roundabout_has_legal_left_vehicle_snapshots": (
            roundabout_vehicle_geometry_roles["roundabout_left_legal_path"] > 0
        ),
        "highway_has_legal_parallel_lane_snapshots": any(
            name.startswith("highway_parallel_lane_") and count > 0
            for name, count in highway_vehicle_geometry_roles.items()
        ),
        "highway_has_near_and_far_lane_snapshots": (
            any(
                name.startswith("highway_parallel_lane_")
                and name.endswith("_gap_1")
                and count > 0
                for name, count in highway_vehicle_geometry_roles.items()
            )
            and any(
                name.startswith("highway_parallel_lane_")
                and name.endswith("_gap_2")
                and count > 0
                for name, count in highway_vehicle_geometry_roles.items()
            )
        ),
    }
    if args.placement_pilot:
        checks = {
            name: passed
            for name, passed in checks.items()
            if name
            not in {
                "all_positive_families_cover_24_bins",
                "pedestrian_general_route_uses_at_least_18_bins",
                "pedestrian_crosswalk_uses_at_least_5_locations",
                "vehicle_asset_diversity_at_least_30",
                "pedestrian_asset_diversity_at_least_8",
                "obstacle_asset_diversity_at_least_20",
                "each_positive_class_has_front_side_and_rear_capture_views",
                "each_positive_class_covers_all_distance_bands",
            }
        }
    if not all(checks.values()):
        errors.extend(
            "check failed: {}".format(name)
            for name, passed in checks.items()
            if not passed
        )

    report = {
        "schema_version": 1,
        "dataset_id": manifest["dataset_id"],
        "manifest": str(args.manifest.resolve()),
        "clip_count": len(scene_rows),
        "frame_count": len(scene_rows) * 3,
        "family_clip_counts": dict(family_counts),
        "split_clip_counts": dict(split_counts),
        "family_split_clip_counts": {
            name: dict(counts) for name, counts in family_split_counts.items()
        },
        "family_route_bin_counts": {
            name: len(values) for name, values in family_bins.items()
        },
        "family_crosswalk_counts": {
            name: len(values) for name, values in family_crosswalks.items()
        },
        "family_spatial_strata": {
            name: dict(values) for name, values in family_strata.items()
        },
        "family_roundabout_phases": {
            name: dict(values) for name, values in family_roundabout_phases.items()
        },
        "family_highway_lanes": {
            name: dict(values) for name, values in family_highway_lanes.items()
        },
        "family_maximum_circular_arc_gap_m": {
            name: circular_gap(values, route_length)
            for name, values in family_arcs.items()
        },
        "family_actor_count_min_mean_max": {
            name: {
                "min": min(values),
                "mean": sum(values) / len(values),
                "max": max(values),
            }
            for name, values in actor_counts.items()
        },
        "class_unique_asset_counts": {
            name: len(values) for name, values in class_assets.items()
        },
        "class_view_sectors": {
            name: dict(counts) for name, counts in class_sectors.items()
        },
        "class_estimated_capture_view_sectors": {
            name: dict(counts) for name, counts in class_capture_sectors.items()
        },
        "class_estimated_capture_distance_bins": {
            name: dict(counts)
            for name, counts in class_capture_distance_bins.items()
        },
        "geometry_roles": dict(geometry_roles),
        "vehicle_geometry_roles": dict(vehicle_geometry_roles),
        "opposite_heading_vehicle_count": opposite_heading_vehicle_count,
        "same_lane_final_separation_violations": (
            same_lane_final_separation_violations
        ),
        "same_lane_front_count_distribution": dict(
            same_lane_front_count_distribution
        ),
        "same_lane_rear_count_distribution": dict(
            same_lane_rear_count_distribution
        ),
        "diagonal_vehicle_quadrants": dict(diagonal_vehicle_quadrants),
        "roundabout_vehicle_geometry_roles": dict(
            roundabout_vehicle_geometry_roles
        ),
        "highway_vehicle_geometry_roles": dict(highway_vehicle_geometry_roles),
        "checks": checks,
        "audit_mode": "placement_pilot" if args.placement_pilot else "campaign",
        "errors": errors,
        "status": "pass" if not errors else "fail",
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.output_dir / "generated_scenes.csv", scene_rows)
    write_csv(args.output_dir / "generated_actors.csv", actor_rows)
    with (args.output_dir / "audit.json").open("w", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print("status: {}".format(report["status"]))
    print("clips/frames: {}/{}".format(report["clip_count"], report["frame_count"]))
    print("families: {}".format(report["family_clip_counts"]))
    print("assets: {}".format(report["class_unique_asset_counts"]))
    print("sectors: {}".format(report["class_view_sectors"]))
    print(
        "capture sectors: {}".format(
            report["class_estimated_capture_view_sectors"]
        )
    )
    print("report: {}".format(args.output_dir / "audit.json"))
    if errors:
        for error in errors[:20]:
            print("ERROR {}".format(error))
        raise SystemExit(1)


if __name__ == "__main__":
    main()
