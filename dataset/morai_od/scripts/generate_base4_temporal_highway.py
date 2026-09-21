#!/usr/bin/env python3
"""Generate Base4-Temporal highway merge and high-speed overtake scenes."""

import argparse
import copy
import json
import subprocess
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DATASET_ROOT = PROJECT_ROOT / "dataset/morai_od"
GENERATOR = DATASET_ROOT / "scripts/generate_dynamic_object_capture_set.py"
INSTALLER = DATASET_ROOT / "scripts/install_dynamic_capture_set.py"
DEFAULT_NETWORK_TEMPLATE = (
    PROJECT_ROOT / "configs/network/ego_network_mr_minimal_hybrid.json"
)

sys.path.insert(0, str(DATASET_ROOT / "scripts"))
from generate_temporal_sequence_v2_batch import family_defaults  # noqa: E402


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene-count", type=int, default=30)
    parser.add_argument("--seed", type=int, default=260920401)
    parser.add_argument("--campaign-tag", default="pilot30")
    parser.add_argument(
        "--profile",
        choices=("mission", "velocity-diagnostic"),
        default="mission",
        help=(
            "mission preserves the existing highway pilot distribution; "
            "velocity-diagnostic cycles explicit NPC speed anchors for a "
            "small velocity-head sanity dataset"
        ),
    )
    parser.add_argument("--output-root", type=Path, default=DATASET_ROOT)
    parser.add_argument("--install-dir", type=Path)
    parser.add_argument(
        "--network-template", type=Path, default=DEFAULT_NETWORK_TEMPLATE
    )
    return parser.parse_args()


def write_json(path, document):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(document, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def build_config(scene_count, seed, campaign_tag, profile="mission"):
    if scene_count < 8:
        raise ValueError("Base4 highway pilot needs at least 8 scenes")
    base, defaults = family_defaults()
    merge_count = max(2, int(round(scene_count * 0.25)))
    cruise_count = scene_count - merge_count

    capture = copy.deepcopy(base["capture"])
    capture.pop("scenario_load_initial_speed_kph", None)
    capture.update(
        {
            "format": "morai_online_rosbag_temporal_gated",
            "captures_per_scene": 16,
            "capture_period_sec": 1.0 / 12.0,
            "capture_spacing_m": 1.5,
            "clip_duration_sec": 1.4,
            "ego_target_speed_kph": 58.0,
            "warmup_sec": 0.0,
            "prime_capture_before_clip": False,
            "maximum_motion_m": 180.0,
            "motion_stall_timeout_sec": 15.0,
            "actor_ready_timeout_sec": 18.0,
            "route_deviation_limit_m": 2.0,
            "heading_error_limit_deg": 35.0,
            "lookahead_m": 8.0,
            "maximum_steering_rad": 0.22,
            # A loaded scenario must remain parked until MORAI has restored
            # the network and external-control mode. A non-zero saved speed
            # previously let Ego travel straight through the curved merge
            # lane before the first steering command was published.
            "maximum_precontrol_drift_m": 0.75,
            "adaptive_curve_control": False,
            "minimum_curve_speed_kph": 45.0,
            "minimum_curve_lookahead_m": 4.0,
            "curve_maximum_steering_rad": 0.30,
        }
    )

    merge = copy.deepcopy(defaults["highway_merge_cruise"])
    merge.update(
        {
            "count": merge_count,
            # Competition rule: the ramp/merge approach must remain strictly
            # below 59 km/h. These are choices, not a continuous range.
            "ego_target_speed_kph": [50.0, 54.0, 58.0],
            "ego_initial_speed_kph": 0.0,
            "start_ego_at_target_speed": False,
            "minimum_ego_speed_ratio": 0.88,
            "source_spawn_ids": [6, 8, 10],
            "target_speed_scale": [1.0, 1.0],
            "spawn_velocity_equals_target": True,
            "preserve_template_npc_target_speed": True,
            "maximum_spawn_vehicles": 1,
            "spawn_wait_sec": 0.0,
            "spawn_period_sec": [10.0, 12.0],
            "ego_capture_zones": [
                {
                    "name": "merge_conflict",
                    # Indices below 2030 are still on the unstable approach
                    # geometry: live runs either never brought the spawned
                    # mainline actor into the gate or stopped Ego midway.
                    # Start at the actual conflict region instead of turning
                    # those pre-merge failures into a biased temporal clip.
                    "route_index_range": [2030, 2060],
                    "capture_preroll_distance_m": [14.0, 24.0],
                    "weight": 1.0,
                },
            ],
            "capture_preroll_distance_m": [14.0, 24.0],
            "captures_per_scene": 16,
            "desired_clip_duration_sec": 1.4,
            "maximum_motion_m": 180.0,
            "motion_stall_timeout_sec": 15.0,
            "lookahead_m": 7.0,
            "load_settle_sec": 0.20,
            "route_deviation_limit_m": 2.0,
            "heading_error_limit_deg": 40.0,
            "interaction_roles": [
                {
                    "name": "merge_interacting_traffic",
                    "weight": 1.0,
                    "npc_longitudinal_offset_m": [-50.0, -35.0],
                    "gate_longitudinal_m": [-40.0, 65.0],
                    "maximum_abs_lateral_m": 22.0,
                    "minimum_distance_m": 5.0,
                    "maximum_distance_m": 72.0,
                }
            ],
            "motion_phase": "highway_merge_below_59kph",
            "sampling_role": "base4_merge_dynamic_vehicle",
        }
    )

    cruise = copy.deepcopy(defaults["highway_cruise_overtake"])
    cruise.update(
        {
            "count": cruise_count,
            "source_spawn_ids": [6, 8, 10],
            # Keep all legal mainline lanes represented.  The schedule varies
            # lanes and relative roles, avoiding a fixed-coordinate shortcut.
            "ego_lane_weights": {"6": 0.25, "8": 0.40, "10": 0.35},
            "ego_lane_speed_kph": {
                "6": [70.0, 100.0],
                "8": [70.0, 100.0],
                "10": [85.0, 100.0],
            },
            # Prove the zero-speed control handoff at competition-relevant
            # speeds before restoring 115/120 km/h stress cases.
            "ego_speed_schedule_kph": [
                70.0,
                85.0,
                85.0,
                100.0,
                100.0,
                100.0,
                85.0,
                100.0,
            ],
            "npc_target_speed_scale": [1.0, 1.0],
            "preserve_template_npc_target_speed": True,
            "spawn_velocity_equals_target": True,
            "maximum_spawn_vehicles": 1,
            # Decouple the exact lane and role decks. Without this offset the
            # two periodic schedules bind every lane-6 slot to front_overtake,
            # whose collision guard moves Ego to lane 8.
            "interaction_role_schedule_offset": 1,
            "ego_initial_speed_kph": 0.0,
            "start_ego_at_target_speed": False,
            "minimum_ego_speed_ratio": 0.85,
            "stable_ego_gate": True,
            "maximum_stable_acceleration_mps2": 1.5,
            "expected_ego_acceleration_mps2": 1.8,
            "compensate_npc_motion_until_capture_gate": True,
            "ego_start_arc_m": [70.0, 285.0],
            "ego_speed_advantage_kph": [20.0, 65.0],
            "ego_speed_limit_kph": 100.0,
            "captures_per_scene": 16,
            "desired_clip_duration_sec": 1.4,
            # At 100 km/h the stable-speed gate opens around 175 m. Leave
            # enough route for all 16 LiDAR-paced triggers after that point.
            "maximum_motion_m": 260.0,
            "motion_stall_timeout_sec": 14.0,
            "capture_motion_arm_m": 80.0,
            "load_settle_sec": 0.10,
            "interaction_roles": [
                {
                    "name": "front_overtake",
                    "weight": 3.0,
                    "npc_longitudinal_offset_m": [20.0, 42.0],
                    "gate_longitudinal_m": [5.0, 50.0],
                    "maximum_abs_lateral_m": 6.5,
                },
                {
                    "name": "side_parallel",
                    "weight": 1.0,
                    "npc_longitudinal_offset_m": [-10.0, 14.0],
                    "gate_longitudinal_m": [-22.0, 25.0],
                    "maximum_abs_lateral_m": 6.5,
                },
            ],
            "motion_phase": "post_merge_high_speed_overtake",
            "sampling_role": "base4_high_speed_relative_vehicle",
        }
    )

    if profile == "velocity-diagnostic":
        # This is deliberately a small diagnostic distribution, not the
        # competition prior.  Explicit anchors prevent a repeat of the first
        # Base4 pilot, whose completed frames were dominated by 40/50 km/h.
        # The runtime/audit must still use achieved ObjectStatus motion rather
        # than trusting these requested values.
        speed_schedule = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0]
        merge.update(
            {
                "preserve_template_npc_target_speed": False,
                "npc_target_speed_schedule_kph": speed_schedule,
                "spawn_velocity_equals_target": True,
            }
        )
        cruise.update(
            {
                "preserve_template_npc_target_speed": False,
                "npc_target_speed_schedule_kph": speed_schedule,
                "spawn_velocity_equals_target": True,
                # Keep the first diagnostic easy to interpret and collision
                # safe: the NPC is in the adjacent right lane while Ego
                # passes it.  Lateral/parallel diversity belongs to the next
                # campaign after the speed head proves it can separate bins.
                "interaction_roles": [
                    {
                        "name": "front_overtake",
                        "weight": 1.0,
                        "npc_longitudinal_offset_m": [20.0, 42.0],
                        "gate_longitudinal_m": [5.0, 50.0],
                        "maximum_abs_lateral_m": 6.5,
                    }
                ],
                "interaction_role_schedule_offset": 0,
                "sampling_role": "base4_velocity_diagnostic_vehicle",
            }
        )

    clean_tag = "".join(ch for ch in campaign_tag.lower() if ch.isalnum())
    if not clean_tag:
        raise ValueError("campaign tag must contain an alphanumeric character")
    return {
        "dataset_id": "morai_base4_temporal_highway_{}".format(clean_tag),
        "seed": int(seed),
        "scenario_prefix": "b4th{}".format(clean_tag),
        "use_runtime_obstacle_pool": False,
        "split_ratios": {"train": 0.8, "val": 0.2, "test": 0.0},
        "environment": copy.deepcopy(base["environment"]),
        "capture": capture,
        "families": [merge, cruise],
        "base4_design": {
            "profile": profile,
            "scene_count": scene_count,
            "merge_scene_count": merge_count,
            "post_merge_scene_count": cruise_count,
            "npc_goal_speed_contract_kph": [40.0, 50.0, 60.0],
            "premerge_ego_speed_contract_kph": [50.0, 54.0, 58.0],
            "postmerge_ego_speed_contract_kph": [70.0, 85.0, 100.0],
            "internal_test_split": False,
            "diagnostic_requested_npc_speeds_kph": (
                [10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0]
                if profile == "velocity-diagnostic"
                else None
            ),
        },
    }


def main():
    args = parse_args()
    config = build_config(
        args.scene_count, args.seed, args.campaign_tag, profile=args.profile
    )
    tag = "".join(ch for ch in args.campaign_tag.lower() if ch.isalnum())
    config_path = (
        args.output_root
        / "metadata/base4_temporal_highway"
        / "{}_config.json".format(tag)
    )
    write_json(config_path, config)
    command = [
        sys.executable,
        str(GENERATOR),
        "--config",
        str(config_path),
        "--output-root",
        str(args.output_root),
        "--group",
        "base4_temporal_highway_{}".format(tag),
        "--id-prefix",
        "b4{}d".format(tag),
        "--stem-prefix",
        "b4{}_".format(tag),
    ]
    subprocess.run(command, check=True)
    manifest_path = (
        args.output_root
        / "manifests/base4_temporal_highway_{}.json".format(tag)
    )
    if args.install_dir is not None:
        runtime_manifest = manifest_path.with_name(
            manifest_path.stem + "_runtime.json"
        )
        subprocess.run(
            [
                sys.executable,
                str(INSTALLER),
                str(manifest_path),
                "--network-template",
                str(args.network_template),
                "--install-dir",
                str(args.install_dir),
                "--runtime-manifest",
                str(runtime_manifest),
                "--online-bag-only",
            ],
            check=True,
        )
        print("runtime manifest: {}".format(runtime_manifest))
    print("config: {}".format(config_path))


if __name__ == "__main__":
    main()
