#!/usr/bin/env python3
"""Generate the corrected 20-scene geometry/controller trial set."""

import sys
from pathlib import Path

import generate_geometry_trial_20_set as trial_v1


generator = trial_v1.generator
generator.DATASET_ID = "morai_geometry_trial20_v2"
generator.SCENARIO_GROUP = "geometry_trial20_v2"
generator.MANIFEST_STEM = "geometry_trial20_v2"
generator.SCENE_ID_PREFIX = "geo2_"
generator.SCENARIO_FILE_PREFIX = "od_geo2_"
generator.MASTER_SEED = 260807521


if __name__ == "__main__":
    if "--config" not in sys.argv:
        project_root = Path(__file__).resolve().parents[3]
        sys.argv.extend(
            [
                "--config",
                str(
                    project_root
                    / "dataset/morai_od/configs/od_geometry_trial_20_v2.yaml"
                ),
            ]
        )
    generator.main()
