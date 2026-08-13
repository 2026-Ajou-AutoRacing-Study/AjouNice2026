#!/usr/bin/env python3
"""Generate 600 Sunny 13:00 scenes (up to 3,600 multisensor frames)."""

import sys
from pathlib import Path

import generate_sunny13_180_production_v5_set as production_v5


generator = production_v5.generator
generator.DATASET_ID = "morai_sunny13_600_production_v6"
generator.SCENARIO_GROUP = "sunny13_600_production_v6"
generator.MANIFEST_STEM = "sunny13_600_production_v6"
generator.SCENE_ID_PREFIX = "s13p6_"
generator.SCENARIO_FILE_PREFIX = "od_s13p6_"
generator.BASE_SCENARIO_COUNT = 600
generator.SPLIT_COUNTS = {"train": 420, "val": 90, "test": 90}
generator.MASTER_SEED = 260809600
generator.ALLOW_ANCHOR_REUSE = True

# Reuse the 180 validated topology-balanced anchors from v5. Every scene still
# receives an independent seed and actor layout. This avoids inventing unsafe
# anchors merely to make 600 starting coordinates unique.
generator.SCENE_TYPE_SPECS = (
    {
        "name": "pedestrian_centric",
        "count": 150,
        "splits": ("train",) * 105 + ("val",) * 23 + ("test",) * 22,
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
        "count": 120,
        "splits": ("train",) * 84 + ("val",) * 18 + ("test",) * 18,
        "npc_range": (5, 8),
        "pedestrian_range": (0, 1),
        "obstacle_range": (0, 2),
        "topology_preference": ("straight", "intersection", "merge_diverge"),
    },
    {
        "name": "mixed_vehicle_pedestrian",
        "count": 180,
        "splits": ("train",) * 126 + ("val",) * 27 + ("test",) * 27,
        "npc_range": (3, 5),
        "pedestrian_range": (3, 6),
        "obstacle_range": (1, 3),
        "topology_preference": ("intersection", "merge_diverge", "curve"),
    },
    {
        "name": "obstacle_context",
        "count": 60,
        "splits": ("train",) * 42 + ("val",) * 9 + ("test",) * 9,
        "npc_range": (0, 2),
        "pedestrian_range": (0, 0),
        "obstacle_range": (3, 4),
        "topology_preference": ("roadside_parking_clutter", "straight"),
    },
    {
        "name": "camera_seam_occlusion",
        "count": 60,
        "splits": ("train",) * 42 + ("val",) * 9 + ("test",) * 9,
        "npc_range": (2, 4),
        "pedestrian_range": (3, 5),
        "obstacle_range": (0, 2),
        "topology_preference": ("curve", "intersection"),
    },
    {
        "name": "sparse_empty_background",
        "count": 30,
        "splits": ("train",) * 21 + ("val",) * 4 + ("test",) * 5,
        "npc_range": (0, 0),
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
                    / "dataset/morai_od/configs/od_sunny13_600_production_v6.yaml"
                ),
            ]
        )
    generator.main()
