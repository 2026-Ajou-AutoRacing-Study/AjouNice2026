#!/usr/bin/env python3
"""Generate a 20-scene geometry and capture-spacing trial set."""

import sys
from pathlib import Path

import generate_od180_pilot_set as generator


generator.DATASET_ID = "morai_geometry_trial20_v1"
generator.SCENARIO_GROUP = "geometry_trial20_v1"
generator.MANIFEST_STEM = "geometry_trial20_v1"
generator.SCENE_ID_PREFIX = "geo"
generator.SCENARIO_FILE_PREFIX = "od_geo_"
generator.BASE_SCENARIO_COUNT = 20
generator.SPLIT_COUNTS = {"train": 14, "val": 3, "test": 3}
generator.MINIMUM_ANCHOR_ARC_DISTANCE_M = 8.0
generator.ANCHOR_CANDIDATE_SPACING_M = 2.0
generator.MASTER_SEED = 260807520
generator.ENVIRONMENTS = (
    {"id": "sunny_t13", "weather": "sunny", "time_hour": 13},
)

generator.SCENE_TYPE_SPECS = (
    {
        "name": "pedestrian_centric",
        "count": 5,
        "splits": ("train",) * 3 + ("val",) + ("test",),
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
        "count": 4,
        "splits": ("train",) * 3 + ("test",),
        "npc_range": (5, 8),
        "pedestrian_range": (0, 1),
        "obstacle_range": (0, 2),
        "topology_preference": ("straight", "intersection", "merge_diverge"),
    },
    {
        "name": "mixed_vehicle_pedestrian",
        "count": 6,
        "splits": ("train",) * 4 + ("val",) + ("test",),
        "npc_range": (3, 5),
        "pedestrian_range": (3, 6),
        "obstacle_range": (1, 3),
        "topology_preference": ("intersection", "merge_diverge", "curve"),
    },
    {
        "name": "obstacle_context",
        "count": 2,
        "splits": ("train", "val"),
        "npc_range": (0, 2),
        "pedestrian_range": (0, 0),
        "obstacle_range": (3, 4),
        "topology_preference": ("roadside_parking_clutter", "straight"),
    },
    {
        "name": "camera_seam_occlusion",
        "count": 2,
        "splits": ("train", "train"),
        "npc_range": (2, 4),
        "pedestrian_range": (3, 5),
        "obstacle_range": (0, 2),
        "topology_preference": ("curve", "intersection"),
    },
    {
        "name": "sparse_empty_background",
        "count": 1,
        "splits": ("train",),
        "npc_range": (0, 0),
        "pedestrian_range": (0, 0),
        "obstacle_range": (0, 0),
        "topology_preference": ("straight",),
    },
)

generator.TOPOLOGY_TARGETS = (
    ("crosswalk", 3),
    ("intersection", 4),
    ("merge_diverge", 3),
    ("roadside_parking_clutter", 3),
    ("curve", 3),
    ("straight", 4),
)


if __name__ == "__main__":
    if "--config" not in sys.argv:
        project_root = Path(__file__).resolve().parents[3]
        sys.argv.extend(
            [
                "--config",
                str(
                    project_root
                    / "dataset/morai_od/configs/od_geometry_trial_20.yaml"
                ),
            ]
        )
    generator.main()
