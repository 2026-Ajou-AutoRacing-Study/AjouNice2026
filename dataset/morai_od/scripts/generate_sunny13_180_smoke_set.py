#!/usr/bin/env python3
"""Generate 180 unique MORAI layouts for a Sunny 13:00 smoke dataset."""

import sys
from pathlib import Path

import generate_od180_pilot_set as generator


generator.DATASET_ID = "morai_sunny13_180_smoke_v2"
generator.SCENARIO_GROUP = "sunny13_180_smoke_v2"
generator.MANIFEST_STEM = "sunny13_180_smoke_v2"
generator.SCENE_ID_PREFIX = "s13v2_"
generator.SCENARIO_FILE_PREFIX = "od_s13v2_"
generator.BASE_SCENARIO_COUNT = 180
generator.SPLIT_COUNTS = {"train": 126, "val": 27, "test": 27}
generator.MINIMUM_ANCHOR_ARC_DISTANCE_M = 8.0
generator.ANCHOR_CANDIDATE_SPACING_M = 2.0
generator.MASTER_SEED = 260807180
generator.ENVIRONMENTS = (
    {"id": "sunny_t13", "weather": "sunny", "time_hour": 13},
)

# Keep the same scene mixture used by the reusable multi-sensor design:
# 25% pedestrian, 20% vehicle, 30% mixed, 10% obstacle hard-negative,
# 10% camera-seam/occlusion, and 5% sparse background.
generator.SCENE_TYPE_SPECS = (
    {
        "name": "pedestrian_centric",
        "count": 45,
        "splits": ("train",) * 32 + ("val",) * 7 + ("test",) * 6,
        "npc_range": (0, 2),
        "pedestrian_range": (5, 8),
        "obstacle_range": (0, 2),
        "topology_preference": (
            "crosswalk",
            "roadside_parking_clutter",
            "intersection",
        ),
    },
    {
        "name": "vehicle_centric",
        "count": 36,
        "splits": ("train",) * 25 + ("val",) * 5 + ("test",) * 6,
        "npc_range": (5, 8),
        "pedestrian_range": (0, 1),
        "obstacle_range": (0, 2),
        "topology_preference": ("straight", "intersection", "merge_diverge"),
    },
    {
        "name": "mixed_vehicle_pedestrian",
        "count": 54,
        "splits": ("train",) * 38 + ("val",) * 8 + ("test",) * 8,
        "npc_range": (3, 5),
        "pedestrian_range": (3, 6),
        "obstacle_range": (1, 3),
        "topology_preference": ("intersection", "merge_diverge", "curve"),
    },
    {
        "name": "obstacle_context",
        "count": 18,
        "splits": ("train",) * 13 + ("val",) * 3 + ("test",) * 2,
        "npc_range": (0, 2),
        "pedestrian_range": (0, 0),
        "obstacle_range": (3, 4),
        "topology_preference": ("roadside_parking_clutter", "straight"),
    },
    {
        "name": "camera_seam_occlusion",
        "count": 18,
        "splits": ("train",) * 12 + ("val",) * 3 + ("test",) * 3,
        "npc_range": (2, 4),
        "pedestrian_range": (3, 5),
        "obstacle_range": (0, 2),
        "topology_preference": ("curve", "intersection"),
    },
    {
        "name": "sparse_empty_background",
        "count": 9,
        "splits": ("train",) * 6 + ("val",) + ("test",) * 2,
        "npc_range": (0, 0),
        "pedestrian_range": (0, 0),
        "obstacle_range": (0, 0),
        "topology_preference": ("straight",),
    },
)

generator.TOPOLOGY_TARGETS = (
    ("crosswalk", 30),
    ("intersection", 30),
    ("merge_diverge", 24),
    ("roadside_parking_clutter", 24),
    ("curve", 30),
    ("straight", 42),
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
                        "od_sunny13_180_smoke.yaml"
                    )
                ),
            ]
        )
    generator.main()
