#!/usr/bin/env python3
"""Generate the validated Sunny 13:00 production dataset scenarios."""

import sys
from pathlib import Path

import generate_sunny13_180_smoke_set as smoke_v2


generator = smoke_v2.generator
generator.DATASET_ID = "morai_sunny13_180_production_v3"
generator.SCENARIO_GROUP = "sunny13_180_production_v3"
generator.MANIFEST_STEM = "sunny13_180_production_v3"
generator.SCENE_ID_PREFIX = "s13p3_"
generator.SCENARIO_FILE_PREFIX = "od_s13p3_"
generator.MASTER_SEED = 260808180
# Latest MGeo clearance leaves 362 safe candidate points.  Three metres keeps
# all 180 Ego anchors unique while allowing the requested topology mix; the
# randomized actor layouts remain independent even where capture corridors
# partially overlap.
generator.MINIMUM_ANCHOR_ARC_DISTANCE_M = 3.0
# Anchor 3936 is a tight curve surrounded by MGeo static geometry.  It cannot
# satisfy the camera-seam pedestrian layout even at the scene type's minimum
# actor count without weakening the validated pedestrian collider clearance.
generator.EXCLUDED_ANCHOR_INDICES = {3936}


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
                        "od_sunny13_180_production_v3.yaml"
                    )
                ),
            ]
        )
    generator.main()
