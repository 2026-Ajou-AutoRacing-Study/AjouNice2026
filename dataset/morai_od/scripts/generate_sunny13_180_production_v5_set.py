#!/usr/bin/env python3
"""Generate the mission-aware, camera-sector-balanced Sunny 13:00 v5 set."""

import sys
from pathlib import Path

import generate_sunny13_180_production_v4_set as production_v4


generator = production_v4.generator
generator.DATASET_ID = "morai_sunny13_180_production_v5"
generator.SCENARIO_GROUP = "sunny13_180_production_v5"
generator.MANIFEST_STEM = "sunny13_180_production_v5"
generator.SCENE_ID_PREFIX = "s13p5_"
generator.SCENARIO_FILE_PREFIX = "od_s13p5_"
generator.MASTER_SEED = 260809180

# These two assets loaded in earlier scenes but were reproducibly omitted in
# the v5 smoke scene across multiple positions and two unrelated anchors.
# Preserve them in the catalog; replace them only in unattended v5 capture.
generator.RUNTIME_EXCLUDED_ASSET_IDS = {
    key: set(values)
    for key, values in generator.RUNTIME_EXCLUDED_ASSET_IDS.items()
}
generator.RUNTIME_EXCLUDED_ASSET_IDS["vehicle"].update(
    {
        30100009,  # 2020_Kia_Stinger
        30800004,  # DEFAULT_MPV
    }
)

# This tight intersection anchor reproducibly omitted two otherwise proven
# vehicle assets even after both were moved to different valid MGeo links.
# Replace the scene anchor instead of accumulating asset-specific exceptions.
generator.EXCLUDED_ANCHOR_INDICES = set(
    generator.EXCLUDED_ANCHOR_INDICES
) | {154}

# Six captures cover about 9 m, but collision clearance must also include the
# Ego front overhang and motion while the final trigger is handled. The old
# 10.5 m check accepted Man2 at route-ahead 13.50 m and caused a live collision
# after trigger 5/6. Protect 15 m for actor and static-geometry validation.
generator.EGO_ROUTE_AHEAD_M = 15.0

# The weights are targets, not hard quotas. MGeo feasibility, the Ego swept
# corridor, crosswalk placement and collision checks always take priority.
# Left is intentionally a little more common than right for K-City's
# right-hand traffic, while rear pedestrians/obstacles are kept uncommon.
generator.DIRECTIONAL_SECTOR_WEIGHTS = {
    "vehicle": {"front": 0.50, "left": 0.17, "right": 0.13, "rear": 0.20},
    "pedestrian": {
        "front": 0.65,
        "left": 0.17,
        "right": 0.13,
        "rear": 0.05,
    },
    "obstacle": {
        "front": 0.70,
        "left": 0.16,
        "right": 0.12,
        "rear": 0.02,
    },
}

# Opposing traffic remains useful and realistic, but v4's measured 34.5% was
# excessive for the competition mission. Same-direction and merge/crossing
# vehicles carry most of the useful avoidance and lane-change supervision.
generator.VEHICLE_TRAFFIC_RELATION_WEIGHTS = {
    "same_direction": 0.62,
    "opposing_direction": 0.18,
    "crossing_or_merging": 0.20,
}

# Preserve overlap between adjacent camera views without deliberately putting
# half of seam-scene pedestrians on the rear-camera boundary.
generator.CAMERA_SEAM_ANGLES_DEG = (45.0, -45.0, 100.0, -100.0)

# Keep empirically bad positions documented even though the two newly
# unreliable assets are also excluded from unattended v5 generation.
generator.VEHICLE_LINK_EXCLUSION_ZONES = (
    {
        "link_idx": "A2256W000131",
        "x": -42.4204343,
        "y": -531.3443126,
        "radius_m": 6.0,
        "reason": "repeated MORAI actor omission beside GPS-shadow dome",
    },
    {
        "link_idx": "A2256W000075",
        "x": -86.2105990,
        "y": -358.2504731,
        "radius_m": 6.0,
        "reason": "Stinger omitted in three consecutive s13p5_005 loads",
    },
    {
        "link_idx": "A2256W000324",
        "x": -80.5328075,
        "y": -361.7210144,
        "radius_m": 6.0,
        "reason": "DEFAULT_MPV omitted in three consecutive s13p5_005 loads",
    },
)


if __name__ == "__main__":
    if "--config" not in sys.argv:
        project_root = Path(__file__).resolve().parents[3]
        sys.argv.extend(
            [
                "--config",
                str(
                    project_root
                    / (
                        "dataset/morai_od/configs/"
                        "od_sunny13_180_production_v5.yaml"
                    )
                ),
            ]
        )
    generator.main()
