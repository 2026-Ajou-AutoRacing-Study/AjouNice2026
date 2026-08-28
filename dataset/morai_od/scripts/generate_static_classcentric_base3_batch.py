#!/usr/bin/env python3
"""Generate one Base3-Static class-centric batch and its failure reserve.

The campaign is a controlled single-frame OD experiment, not a velocity or
temporal-model dataset.  It preserves Base3's continuous MGeo Ego coverage but
uses dense, static class-centric layouts inspired by the strong E65 baseline.
"""

import argparse
import copy
import subprocess
import sys
from pathlib import Path

import generate_temporal_sequence_base3_batch as base3
import generate_temporal_sequence_v2_batch as v2
import generate_dynamic_object_capture_set as dynamic_generator


DATASET_ID = "morai_static_classcentric_base3_10k"
FRAMES_PER_CLIP = 3
TOTAL_CLIPS = 3334
TOTAL_FRAMES = TOTAL_CLIPS * FRAMES_PER_CLIP
INSTALLER = Path(__file__).resolve().with_name("install_dynamic_capture_set.py")

GLOBAL_PRIMARY_COUNTS = {
    "stopped_vehicle": 1167,
    "static_obstacle": 1167,
    "standing_pedestrian": 500,
    "realistic_mixed_static": 167,
    "sparse_background": 333,
}

# Distribute the indivisible global quota into seven nearly equal capture
# batches.  Batch totals are 477, 477, then 476 clips, giving exactly 10,002
# frames over the complete campaign.
PRIMARY_COUNTS_BY_BATCH = (
    {"stopped_vehicle": 167, "static_obstacle": 167, "standing_pedestrian": 72, "realistic_mixed_static": 24, "sparse_background": 47},
    {"stopped_vehicle": 167, "static_obstacle": 167, "standing_pedestrian": 72, "realistic_mixed_static": 23, "sparse_background": 48},
    {"stopped_vehicle": 167, "static_obstacle": 167, "standing_pedestrian": 71, "realistic_mixed_static": 24, "sparse_background": 47},
    {"stopped_vehicle": 167, "static_obstacle": 166, "standing_pedestrian": 72, "realistic_mixed_static": 24, "sparse_background": 47},
    {"stopped_vehicle": 166, "static_obstacle": 167, "standing_pedestrian": 71, "realistic_mixed_static": 24, "sparse_background": 48},
    {"stopped_vehicle": 167, "static_obstacle": 166, "standing_pedestrian": 71, "realistic_mixed_static": 24, "sparse_background": 48},
    {"stopped_vehicle": 166, "static_obstacle": 167, "standing_pedestrian": 71, "realistic_mixed_static": 24, "sparse_background": 48},
)

RESERVE_COUNTS = {
    "stopped_vehicle": 9,
    "static_obstacle": 9,
    "standing_pedestrian": 4,
    "realistic_mixed_static": 1,
    "sparse_background": 2,
}

HIGHWAY_ROUTE_SHARE = 0.217
ROUNDABOUT_PHASES = [
    {"name": "approach", "weight": 0.27, "arc_m": [820.0, 880.0]},
    {"name": "entry_merge", "weight": 0.30, "arc_m": [880.0, 910.0]},
    {"name": "circulation", "weight": 0.35, "arc_m": [910.0, 950.0]},
    # Keep the low-share exit stratum on the last stable roundabout link.
    # Static roadside vehicles placed farther down A2256W000846 can fall
    # through non-drivable shoulder terrain and disappear before capture.
    {"name": "exit_egress", "weight": 0.08, "arc_m": [955.0, 982.0]},
]
ROUNDABOUT_SHARE = {
    "stopped_vehicle": 0.20,
    "realistic_mixed_static": 0.15,
    "sparse_background": 0.15,
}


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--batch-index", type=int, default=1)
    parser.add_argument("--campaign-tag", default="b3s")
    parser.add_argument("--seed-base", type=int, default=260826000)
    parser.add_argument("--output-root", type=Path, default=v2.DATASET_ROOT)
    parser.add_argument("--install-dir", type=Path)
    parser.add_argument("--network-template", type=Path)
    parser.add_argument("--no-reserve", action="store_true")
    return parser.parse_args()


def install_runtime_manifest(manifest, install_dir, network_template=None):
    if network_template is None:
        network_template = install_dir / "EgoNetwork/ros_path_probe_MN.json"
    runtime_manifest = manifest.with_name(manifest.stem + "_runtime.json")
    subprocess.run(
        [
            sys.executable,
            str(INSTALLER),
            str(manifest),
            "--network-template",
            str(network_template),
            "--install-dir",
            str(install_dir),
            "--runtime-manifest",
            str(runtime_manifest),
        ],
        check=True,
    )
    return runtime_manifest


def validate_global_quota():
    summed = {name: 0 for name in GLOBAL_PRIMARY_COUNTS}
    for counts in PRIMARY_COUNTS_BY_BATCH:
        for name, count in counts.items():
            summed[name] += int(count)
    if summed != GLOBAL_PRIMARY_COUNTS:
        raise ValueError("Base3-Static global quota drift: {}".format(summed))
    if sum(summed.values()) != TOTAL_CLIPS:
        raise ValueError("Base3-Static clip total drift")


def static_defaults():
    base, rows = base3.base3_defaults()
    base["dataset_id"] = DATASET_ID
    base["use_runtime_vehicle_pool"] = True
    base["use_runtime_pedestrian_pool"] = True
    base["use_runtime_obstacle_pool"] = True
    base["weighted_obstacle_quota_deck"] = True
    base["capture"].update(
        {
            "format": "morai_save_sensor_data_rolling_grouped",
            "captures_per_scene": FRAMES_PER_CLIP,
            # This remains a positive metadata fallback; rolling-grouped is
            # distance-triggered and records the actual inter-frame times.
            "capture_period_sec": 0.35,
            "capture_spacing_m": 3.0,
            "capture_motion_arm_m": 3.0,
            "minimum_capture_progress_m": 0.3,
            "clip_duration_sec": 0.0,
            "maximum_motion_m": 75.0,
            "split_unit": "static_layout_clip",
        }
    )

    common = {
        "spatial_policy": "route_wide_competition_route",
        "route_wide_bin_count": 24,
        "route_wide_margin_m": 0.0,
        "route_wide_closed_loop": True,
        "stable_ego_gate": True,
        "minimum_ego_speed_ratio": 0.65,
        "maximum_motion_m": 75.0,
        "motion_stall_timeout_sec": 15.0,
        "ego_lead_distance_m": [10.0, 22.0],
        "ego_target_speed_kph": [15.0, 35.0],
        "ego_initial_speed_kph": [10.0, 28.0],
        "motion_phase": "static_classcentric_singleframe",
        "capture_motion_arm_m": 3.0,
        "capture_spacing_m": 3.0,
        "captures_per_scene": FRAMES_PER_CLIP,
    }
    for name in GLOBAL_PRIMARY_COUNTS:
        rows[name].update(copy.deepcopy(common))

    # Preserve route-wide background coverage while explicitly stratifying
    # the two spatially important road structures.  Highway clips use all
    # three legal lanes equally.  Roundabout oversampling is limited to the
    # families that directly diagnose Vehicle recall and hard negatives.
    for name in (
        "stopped_vehicle",
        "static_obstacle",
        "realistic_mixed_static",
        "sparse_background",
    ):
        roundabout_share = float(ROUNDABOUT_SHARE.get(name, 0.0))
        strata = []
        if roundabout_share:
            strata.append({"name": "roundabout", "weight": roundabout_share})
            rows[name]["roundabout_phases"] = copy.deepcopy(ROUNDABOUT_PHASES)
            general_intervals = [
                [0.0, 820.0],
                [1045.0, 1119.0],
                [1594.0, 2184.6117233360674],
            ]
        else:
            general_intervals = [
                [0.0, 1119.0],
                [1594.0, 2184.6117233360674],
            ]
        strata.extend(
            [
                {"name": "highway_three_lane", "weight": HIGHWAY_ROUTE_SHARE},
                {
                    "name": "route_general",
                    "weight": 1.0 - roundabout_share - HIGHWAY_ROUTE_SHARE,
                },
            ]
        )
        rows[name]["spatial_strata"] = strata
        rows[name]["route_general_arc_intervals_m"] = general_intervals

    rows["stopped_vehicle"].update(
        {
            # One learnable same-lane lead is the invariant.  Additional
            # Vehicles may occupy verified neighbouring paths, while an
            # optional rear observation is interleaved across half the clips.
            # Never fill a failed side placement with a second front queue.
            "actors_per_scene": [1, 4],
            "controlled_same_lane_front_rear": True,
            "same_lane_rear_count_weights": [
                {"name": "0", "weight": 0.50},
                {"name": "1", "weight": 0.50},
            ],
            "actor_longitudinal_slots_m": [0.0],
            "shuffle_actor_longitudinal_slots": False,
            # Preserve small pose diversity without creating wrong-way cars
            # on the same physical lane. Long vehicles receive much less
            # jitter because a few degrees moves their corners materially.
            "actor_heading_jitter_deg": 5.0,
            "actor_heading_jitter_deg_by_subtype": {
                "passenger_car": 5.0,
                "electric_passenger_car": 5.0,
                "suv": 3.0,
                "mpv": 3.0,
                "bus_van": 1.0,
                "truck": 1.0,
            },
            "legal_path_heading_jitter_deg": 1.0,
            "opposite_heading_probability": 0.0,
            # A geometric 3.5--5 m offset is not necessarily paved road.  Snap
            # lateral candidates onto an actual MGeo lane centreline and use
            # a safe same-lane queue when no neighbouring lane exists.
            "snap_static_vehicle_to_mgeo": True,
            "maximum_vehicle_lane_snap_distance_m": 2.75,
            # If no legal side lane exists, distribute the static queue around
            # the first capture pose rather than pushing every actor far past
            # 30 m. Negative slots are already-passed rear observations.
            # Route-general side lanes are not guaranteed to remain parallel
            # through intersections and merges. Keep ordinary actors in a
            # class-aware same-lane queue; legal side views are injected only
            # by the explicit roundabout/highway path helpers below.
            "same_lane_fallback_only": True,
            "same_lane_front_capture_slots_m": [18.0, 21.0, 24.0],
            "same_lane_front_capture_slots_by_subtype": {
                "passenger_car": [18.0, 21.0, 24.0],
                "electric_passenger_car": [18.0, 21.0, 24.0],
                "suv": [22.0, 25.0, 28.0],
                "mpv": [22.0, 25.0, 28.0],
                "bus_van": [26.0, 28.0, 30.0],
                "truck": [26.0, 28.0, 30.0],
            },
            "same_lane_rear_capture_slots_m": [-18.0, -21.0, -24.0],
            "same_lane_rear_capture_slots_by_subtype": {
                "passenger_car": [-18.0, -21.0, -24.0],
                "electric_passenger_car": [-18.0, -21.0, -24.0],
                "suv": [-20.0, -24.0, -28.0],
                "mpv": [-20.0, -24.0, -28.0],
                "bus_van": [-28.0, -30.0, -32.0],
                "truck": [-28.0, -30.0, -32.0],
            },
            "minimum_final_capture_separation_m_by_subtype": {
                "passenger_car": 8.0,
                "electric_passenger_car": 8.0,
                "suv": 11.0,
                "mpv": 11.0,
                "bus_van": 16.0,
                "truck": 16.0,
            },
            # Legal-path side-view anchors are added separately where MGeo
            # topology is authoritative. Roundabout scenes retain 30% clean
            # no-left-actor layouts; 30% of the active layouts use two
            # well-separated circulating actors. Highway scenes cover every
            # non-Ego lane instead of the old left-only helper.
            "roundabout_left_vehicle_probability": 0.70,
            "roundabout_second_left_vehicle_probability": 0.30,
            "roundabout_left_actor_count_weights": [
                {"name": "0", "weight": 0.30},
                {"name": "1", "weight": 0.49},
                {"name": "2", "weight": 0.21},
            ],
            "highway_side_vehicle_probability": 1.0,
            "highway_second_side_vehicle_probability": 0.35,
            # Ordinary roads use side actors only where MGeo explicitly
            # exposes a compatible lane-change destination. Most route-wide
            # anchors remain conservative same-lane queues.
            "route_general_parallel_vehicle_probability": 1.0,
            "sampling_role": "base3_static_vehicle_centric",
        }
    )
    rows["static_obstacle"].update(
        {
            "objects_per_scene": [5, 8],
            "scale_factor": [0.85, 1.15],
            "sampling_role": "base3_static_obstacle_centric",
        }
    )
    rows["standing_pedestrian"].update(
        {
            "actors_per_scene": [4, 7],
            "actor_longitudinal_slots_m": [-12.0, -7.0, -2.0, 4.0, 10.0, 16.0, 22.0],
            "shuffle_actor_longitudinal_slots": True,
            # Crosswalks remain the majority, while the rest use route-wide
            # roadside placements so Man1/the official mission is not the
            # only appearance context.  build_config converts this ratio to
            # an exact, interleaved scene count.
            "crosswalk_scene_ratio": 0.65,
            "maximum_crosswalk_route_distance_m": 18.0,
            "sampling_role": "base3_static_pedestrian_centric",
        }
    )
    rows["realistic_mixed_static"].update(
        {
            "objects_per_scene": [1, 3],
            "vehicle_actors_per_scene": 1,
            "pedestrian_actors_per_scene": [1, 2],
            "actor_longitudinal_slots_m": [-10.0, -3.0, 5.0, 13.0],
            "shuffle_actor_longitudinal_slots": True,
            "actor_heading_jitter_deg": 4.0,
            "actor_heading_jitter_deg_by_subtype": {
                "passenger_car": 4.0,
                "electric_passenger_car": 4.0,
                "suv": 3.0,
                "mpv": 3.0,
                "bus_van": 1.0,
                "truck": 1.0,
            },
            "legal_path_heading_jitter_deg": 1.0,
            "opposite_heading_probability": 0.0,
            "same_lane_fallback_only": True,
            "controlled_same_lane_front_rear": True,
            "same_lane_front_capture_slots_m": [18.0, 21.0, 24.0],
            "same_lane_front_capture_slots_by_subtype": {
                "passenger_car": [18.0, 21.0, 24.0],
                "electric_passenger_car": [18.0, 21.0, 24.0],
                "suv": [22.0, 25.0, 28.0],
                "mpv": [22.0, 25.0, 28.0],
                "bus_van": [26.0, 28.0, 30.0],
                "truck": [26.0, 28.0, 30.0],
            },
            "minimum_final_capture_separation_m_by_subtype": {
                "passenger_car": 8.0,
                "electric_passenger_car": 8.0,
                "suv": 11.0,
                "mpv": 11.0,
                "bus_van": 16.0,
                "truck": 16.0,
            },
            "sampling_role": "base3_static_mixed_classcentric",
        }
    )
    rows["sparse_background"].update(
        {
            "objects_per_scene": 0,
            "empty_scene": True,
            "sampling_role": "base3_static_route_wide_hard_negative",
        }
    )
    return base, rows


def build_config(batch_index, role, counts, campaign_tag, seed_base):
    base, defaults = static_defaults()
    config = copy.deepcopy(base)
    config["dataset_id"] = "{}_batch{:03d}_{}".format(
        DATASET_ID, batch_index, role
    )
    config["seed"] = int(seed_base) + batch_index * 10000 + (
        0 if role == "primary" else 5000
    )
    config["scenario_prefix"] = "b3s{}b{:03d}{}".format(
        campaign_tag, batch_index, "p" if role == "primary" else "r"
    )
    config["families"] = []
    for family_index, (name, count) in enumerate(counts.items()):
        if int(count) <= 0:
            continue
        row = copy.deepcopy(defaults[name])
        row["count"] = int(count)
        row["route_wide_slot_count"] = int(count)
        row["route_wide_phase_offset"] = (
            (batch_index - 1) * 37 + family_index * 11
            + (5 if role == "reserve" else 0)
        )
        if name == "standing_pedestrian":
            row["crosswalk_scene_count"] = int(
                round(int(count) * float(row.pop("crosswalk_scene_ratio")))
            )
        if name == "sparse_background":
            row["empty_scene_count"] = int(count)
        if row.get("spatial_strata"):
            if role == "primary":
                global_count = int(GLOBAL_PRIMARY_COUNTS[name])
                global_row = {**row, "count": global_count}
                global_schedule = dynamic_generator.weighted_zone_schedule(
                    global_row, key="spatial_strata"
                )
                offset = sum(
                    int(PRIMARY_COUNTS_BY_BATCH[index].get(name, 0))
                    for index in range(batch_index - 1)
                )
                row["family_global_variant_offset"] = offset
                row["global_spatial_stratum_schedule"] = global_schedule
            else:
                row["family_global_variant_offset"] = 0
                row["global_spatial_stratum_schedule"] = (
                    dynamic_generator.weighted_zone_schedule(
                        row, key="spatial_strata"
                    )
                )
        config["families"].append(row)
    return config


def main():
    args = parse_args()
    validate_global_quota()
    if not 1 <= args.batch_index <= len(PRIMARY_COUNTS_BY_BATCH):
        raise ValueError("--batch-index must be in [1, 7]")
    campaign_tag = "".join(
        char for char in args.campaign_tag.lower() if char.isalnum()
    )
    if not campaign_tag:
        raise ValueError("--campaign-tag must contain an alphanumeric character")

    primary_counts = PRIMARY_COUNTS_BY_BATCH[args.batch_index - 1]
    roles = [("primary", primary_counts)]
    if not args.no_reserve:
        roles.append(("reserve", RESERVE_COUNTS))

    config_root = (
        args.output_root
        / "metadata/static_classcentric_base3_batches"
        / "batch{:03d}".format(args.batch_index)
    )
    plan = {
        "schema_version": 1,
        "dataset_id": DATASET_ID,
        "human_name": "Base3-Static 10K",
        "batch_index": args.batch_index,
        "campaign_tag": campaign_tag,
        "seed_base": args.seed_base,
        "canonical_frames_per_clip": FRAMES_PER_CLIP,
        "capture_mode": "rolling-grouped",
        "capture_spacing_m": 3.0,
        "campaign_primary_clip_count": TOTAL_CLIPS,
        "campaign_primary_frame_count": TOTAL_FRAMES,
        "campaign_family_clip_counts": GLOBAL_PRIMARY_COUNTS,
        "roles": [],
    }
    for role, counts in roles:
        config = build_config(
            args.batch_index, role, counts, campaign_tag, args.seed_base
        )
        config_path = config_root / "{}_config.json".format(role)
        v2.write_json(config_path, config)
        group = "static_classcentric_base3_batch{:03d}_{}".format(
            args.batch_index, role
        )
        id_prefix = "b3s{}b{:03d}{}_".format(
            campaign_tag,
            args.batch_index,
            "p" if role == "primary" else "r",
        )
        v2.run_generator(
            config_path,
            args.output_root,
            group,
            id_prefix,
            None,
        )
        manifest_path = args.output_root / "manifests/{}.json".format(group)
        runtime_manifest = None
        if args.install_dir is not None:
            runtime_manifest = install_runtime_manifest(
                manifest_path,
                args.install_dir,
                args.network_template,
            )
        clip_count = sum(counts.values())
        plan["roles"].append(
            {
                "role": role,
                "group": group,
                "config": str(config_path),
                "scene_id_prefix": id_prefix,
                "clip_count": clip_count,
                "frame_count": clip_count * FRAMES_PER_CLIP,
                "family_clip_counts": counts,
                "runtime_manifest": (
                    str(runtime_manifest) if runtime_manifest is not None else None
                ),
            }
        )
    plan_path = config_root / "batch_plan.json"
    v2.write_json(plan_path, plan)
    print("Base3-Static primary clips: {}".format(sum(primary_counts.values())))
    print(
        "Base3-Static primary frames: {}".format(
            sum(primary_counts.values()) * FRAMES_PER_CLIP
        )
    )
    print("plan: {}".format(plan_path))


if __name__ == "__main__":
    main()
