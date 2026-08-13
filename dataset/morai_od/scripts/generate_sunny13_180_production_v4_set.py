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

# These assets existed in both launchers but repeatedly vanished after scene
# load across different links in the v3 180-scene run. Keep them in the
# catalog for manual investigation, but not in unattended production sets.
generator.RUNTIME_EXCLUDED_ASSET_IDS = {
    "vehicle": {
        30100001,  # 2017_Hyundai_Grandeur
        30200011,  # DEFAULT_SUV
        30300003,  # DEFAULT_BUS
        30400001,  # 2013_Hyundai_Porter2
        30400005,  # 2014_Hyundai_Xcient(Mixer)
        30700004,  # 2020_Kia_Niro(EV)
    },
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
