#!/usr/bin/env python3
"""Generate the reliability- and trajectory-aware Sunny 13:00 v4 set."""

import sys
from pathlib import Path

import generate_sunny13_180_production_v3_set as production_v3


generator = production_v3.generator
generator.DATASET_ID = "morai_sunny13_180_production_v4"
generator.SCENARIO_GROUP = "sunny13_180_production_v4"
generator.MANIFEST_STEM = "sunny13_180_production_v4"
generator.SCENE_ID_PREFIX = "s13p4_"
generator.SCENARIO_FILE_PREFIX = "od_s13p4_"
generator.MASTER_SEED = 260808181

# The 2026-08-22 interleaved control/probe revalidation showed that every
# formerly quarantined common vehicle loaded in both placements. The earlier
# scene failures were therefore not asset-specific and must not shrink the
# automatic asset catalog.
generator.RUNTIME_EXCLUDED_ASSET_IDS = {
    "vehicle": set(),
    "pedestrian": set(),
    "obstacle": set(),
}

# Six captures span approximately 0--9 m. Place actors around 4.5 m, not only
# around the initial pose. Mission-heavy scenes receive a moderate front bias;
# vehicle-only scenes retain natural MGeo lane/direction availability.
generator.ACTOR_LAYOUT_REFERENCE_AHEAD_M = 4.5
generator.FRONT_BIASED_SCENE_TYPES = {
    "pedestrian_centric",
    "mixed_vehicle_pedestrian",
    "obstacle_context",
}
generator.FRONT_BIASED_PROBABILITY = 0.45


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
                        "od_sunny13_180_production_v4.yaml"
                    )
                ),
            ]
        )
    generator.main()
