#!/usr/bin/env python3
"""Generate 666 independent static scenes for 1,998 keyframes."""

import sys
from pathlib import Path

import generate_sunny13_600_production_v6_set as production_v6


generator = production_v6.generator
generator.DATASET_ID = "morai_static_random_1998_v1"
generator.SCENARIO_GROUP = "static_random_1998_v1"
generator.MANIFEST_STEM = "static_random_1998_v1"
generator.SCENE_ID_PREFIX = "sr1_"
generator.SCENARIO_FILE_PREFIX = "od_sr1_"
generator.BASE_SCENARIO_COUNT = 666
generator.SPLIT_COUNTS = {"train": 466, "val": 100, "test": 100}
generator.MASTER_SEED = 2608201998
generator.ALLOW_ANCHOR_REUSE = True
generator.ENVIRONMENTS = (
    {"id": "sunny_t13", "weather": "sunny", "time_hour": 13},
)

# Three triggers at 3, 6 and 9 m preserve the old spatial coverage while
# replacing highly correlated 1.5 m neighbours with independent layouts.
generator.SCENE_TYPE_SPECS = (
    {
        "name": "realistic_mixed",
        "count": 266,
        "splits": ("train",) * 186 + ("val",) * 40 + ("test",) * 40,
        "npc_range": (2, 5),
        "pedestrian_range": (1, 4),
        "obstacle_range": (1, 3),
        "topology_preference": ("intersection", "merge_diverge", "curve"),
    },
    {
        "name": "vehicle_centric",
        "count": 133,
        "splits": ("train",) * 93 + ("val",) * 20 + ("test",) * 20,
        "npc_range": (5, 8),
        "pedestrian_range": (0, 2),
        "obstacle_range": (0, 2),
        "topology_preference": ("straight", "intersection", "merge_diverge"),
    },
    {
        "name": "pedestrian_centric",
        "count": 100,
        "splits": ("train",) * 70 + ("val",) * 15 + ("test",) * 15,
        "npc_range": (0, 2),
        "pedestrian_range": (6, 10),
        "obstacle_range": (0, 2),
        "topology_preference": ("crosswalk", "intersection", "roadside_parking_clutter"),
    },
    {
        "name": "obstacle_centric",
        "count": 134,
        "splits": ("train",) * 94 + ("val",) * 20 + ("test",) * 20,
        "npc_range": (0, 2),
        "pedestrian_range": (0, 2),
        "obstacle_range": (5, 7),
        "topology_preference": ("roadside_parking_clutter", "straight", "intersection"),
    },
    {
        "name": "sparse_empty_background",
        "count": 33,
        "splits": ("train",) * 23 + ("val",) * 5 + ("test",) * 5,
        "npc_range": (0, 1),
        "pedestrian_range": (0, 0),
        "obstacle_range": (0, 0),
        "topology_preference": ("straight",),
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
                    / "dataset/morai_od/configs/od_static_random_1998_v1.yaml"
                ),
            ]
        )
    generator.main()
