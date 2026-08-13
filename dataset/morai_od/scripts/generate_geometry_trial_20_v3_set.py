#!/usr/bin/env python3
"""Generate the personal-MORAI steering-sign correction trial set."""

import sys
from pathlib import Path

import generate_geometry_trial_20_set as trial_v1


generator = trial_v1.generator
generator.DATASET_ID = "morai_geometry_trial20_v3"
generator.SCENARIO_GROUP = "geometry_trial20_v3"
generator.MANIFEST_STEM = "geometry_trial20_v3"
generator.SCENE_ID_PREFIX = "geo3_"
generator.SCENARIO_FILE_PREFIX = "od_geo3_"
# Keep the v2 seed so geometry stays comparable across the controller change.
generator.MASTER_SEED = 260807521


if __name__ == "__main__":
    if "--config" not in sys.argv:
        project_root = Path(__file__).resolve().parents[3]
        sys.argv.extend(
            [
                "--config",
                str(
                    project_root
                    / "dataset/morai_od/configs/od_geometry_trial_20_v3.yaml"
                ),
            ]
        )
    generator.main()
