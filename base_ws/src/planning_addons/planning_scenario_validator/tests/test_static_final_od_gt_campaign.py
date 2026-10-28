"""static_final OD-GT importer의 지도·파생 입력 계약을 검증한다."""

import copy
import json
import sys
import unittest
from pathlib import Path

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "scripts"))

import build_static_final_od_gt_campaign as builder


def sample_scene():
    return {
        "version": 1.0,
        "mapInfo": {"mapName": builder.MAP_NAME},
        "egoVehicle": {
            "currentVelocity": 0.0,
            "initPosition": {
                "pos": {"x": builder.EXPECTED_EGO_XY[0],
                        "y": builder.EXPECTED_EGO_XY[1], "z": 28.387},
                "rot": {"yaw": str(builder.EXPECTED_EGO_YAW)},
                "initLink": "",
                "initLinkRatio": 0,
                "gear": 1,
            },
        },
        "objectList": [{
            "DataID": 40100001,
            "UNIQUEID": 410001,
            "scale": {"x": 0.7, "y": 1.05, "z": 0.7},
            "moveSpeed": 0.0,
            "waypointEventList": [],
        }],
        "vehicleList": [],
        "pedestrianList": [],
        "aiPedstrianList": [],
    }


class StaticFinalOdGtCampaignTest(unittest.TestCase):
    def test_expected_ego_projects_to_current_mgeo_link(self):
        projection = builder.project_to_link(builder.EXPECTED_EGO_XY)
        self.assertEqual(projection["link"], "A2256W000213")
        self.assertLess(projection["distance_m"], 0.001)
        self.assertAlmostEqual(projection["ratio"], 0.025441178348301677, places=12)

    def test_validation_changes_only_the_missing_link_fields(self):
        original = sample_scene()
        source = copy.deepcopy(original)
        name = "static_final/scenario_001_cloudy_intersection_1obj.json"
        match = builder.NAME_PATTERN.search(name)
        projection = builder.project_to_link(builder.EXPECTED_EGO_XY)
        derived = builder.validate_scene(name, json.dumps(source).encode(), match, projection)
        initial = derived["egoVehicle"]["initPosition"]
        self.assertEqual(initial["initLink"], projection["link"])
        self.assertEqual(initial["initLinkRatio"], projection["ratio"])
        initial["initLink"] = ""
        initial["initLinkRatio"] = 0
        self.assertEqual(derived, original)

    def test_campaign_is_complete_od_gt_schema_v2(self):
        text = builder.campaign_text(
            "test_static_final", builder.DEFAULT_OUTPUT,
            Path("/tmp/morai-scenario-install"), ":1")
        document = yaml.safe_load(text)
        self.assertEqual(document["schema_version"], 2)
        self.assertEqual(document["stack"]["detector"], "od_gt")
        self.assertEqual(document["stack"]["pose_source"], "localization")
        self.assertFalse(document["stack"]["traffic_light"])
        self.assertEqual(document["default_case"]["timeout_sec"], 90)
        self.assertFalse(document["upload"]["enabled"])


if __name__ == "__main__":
    unittest.main()
