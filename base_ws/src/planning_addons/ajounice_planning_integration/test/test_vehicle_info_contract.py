#!/usr/bin/env python3

import json
import math
from pathlib import Path
import unittest

import yaml


REPO_ROOT = Path(__file__).resolve().parents[5]
PLANNING_CONFIG = REPO_ROOT / "configs" / "planning" / "vehicle_info.yaml"
CONTROL_CONFIG = REPO_ROOT / "configs" / "vehicle" / "ioniq5.yaml"
MEASUREMENT_RECORD = (
    REPO_ROOT
    / "base_ws"
    / "src"
    / "vehicle"
    / "meshes"
    / "ioniq5"
    / "ioniq5_mesh_measurements.json"
)


class VehicleInfoContractTest(unittest.TestCase):
    def test_flat_planning_values_match_physical_sources(self):
        planning = yaml.safe_load(PLANNING_CONFIG.read_text())
        control = yaml.safe_load(CONTROL_CONFIG.read_text())["vehicle"]
        target = json.loads(MEASUREMENT_RECORD.read_text())["target_dimensions_m"]

        self.assertEqual(
            set(planning),
            {
                "wheel_radius",
                "wheel_width",
                "wheel_base",
                "wheel_tread",
                "front_overhang",
                "rear_overhang",
                "left_overhang",
                "right_overhang",
                "vehicle_height",
                "max_steer_angle",
            },
        )
        self.assertAlmostEqual(planning["wheel_radius"], target["tire_outer_diameter_m"] / 2.0)
        self.assertAlmostEqual(planning["wheel_width"], target["tire_width_m"])
        self.assertAlmostEqual(planning["wheel_base"], target["wheelbase_m"])
        self.assertAlmostEqual(planning["wheel_tread"], target["rear_track_m"])
        self.assertAlmostEqual(planning["front_overhang"], target["front_overhang_m"])
        self.assertAlmostEqual(planning["rear_overhang"], target["rear_overhang_m"])
        self.assertAlmostEqual(planning["vehicle_height"], target["height_m"])
        self.assertAlmostEqual(
            planning["wheel_tread"]
            + planning["left_overhang"]
            + planning["right_overhang"],
            target["body_width_m"],
        )
        self.assertAlmostEqual(
            planning["rear_overhang"]
            + planning["wheel_base"]
            + planning["front_overhang"],
            target["length_m"],
        )
        self.assertAlmostEqual(
            planning["max_steer_angle"],
            math.radians(control["max_wheel_angle_deg"]),
        )


if __name__ == "__main__":
    unittest.main()
