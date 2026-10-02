"""Offline contract tests: not proof of MORAI actor motion or driving success."""
import json
import sys
import unittest
from collections import Counter
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "src"))
sys.path.insert(0, str(PACKAGE / "scripts"))
from build_mission_starters import build, OUTPUT, SAMPLE, read
from planning_scenario_validator.config import load_campaign
from planning_scenario_validator.gui import campaign_examples, default_name
from planning_scenario_validator.runner import check_free_disk
from planning_scenario_validator.runtime import RuntimeFailure


class StarterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.expected = build()
        cls.sample = read(SAMPLE)

    def test_checked_in_files_match_generator(self):
        for path, content in self.expected.items():
            self.assertEqual(path.read_text(encoding="utf-8"), content, str(path))

    def test_all_campaigns_load_and_keep_low_volume_recording(self):
        for family in OUTPUT.iterdir():
            if not family.is_dir():
                continue
            cfg = load_campaign(family / "example.yaml", check_install=False)
            self.assertEqual(len(cfg["cases"]), 24)
            self.assertEqual(cfg["stack"]["detector"], "bevfusion")
            self.assertFalse(cfg["recording"]["include_sensors"])
            self.assertTrue(cfg["recording"]["video"]["enabled"])
            self.assertFalse(cfg["upload"]["enabled"])
            self.assertFalse(any(t.startswith('/sensor/camera/') or t == '/sensor/lidar1'
                                 for t in cfg["recording"]["topics"]))
            meta = read(family / "manifest.json")
            self.assertEqual(Counter(x["tier"] for x in meta["cases"]),
                             {"baseline": 2, "normal": 14, "stress": 8})
            self.assertFalse(meta["live_verified"])

    def test_spawns_keep_official_anchors_and_middle_size_filter(self):
        by_id = {p["UNIQUEID"]: p for p in self.sample["spawnPointList"]}
        for family in ("highway", "roundabout"):
            for path in (OUTPUT / family / "scenarios").glob('*.json'):
                for p in read(path)["spawnPointList"]:
                    source = by_id[p["UNIQUEID"]]
                    for key in ("pos", "rot", "startLinkInfo", "endLinkInfo", "destinationObjUniqueID"):
                        self.assertEqual(p[key], source[key])
                    self.assertFalse(p["isLaneChange"])
                    if family == "highway":
                        self.assertEqual(p["spawnVehicleTypeID"], -1)
                        self.assertEqual(p["listVehicleLengthType"], [2])

    def test_shadow_has_no_dynamic_cut_in_and_dynamic_stays_outside_shadow(self):
        for path in (OUTPUT / 'gps_shadow/scenarios').glob('*.json'):
            scene = read(path)
            self.assertTrue(scene['shadedAreaList'])
            self.assertFalse(scene['spawnPointList'])
            self.assertFalse(scene['waypointDataList'])
            for actor in scene['vehicleList']:
                self.assertEqual(actor['currentVelocity'], 0)
            for actor in scene['pedestrianList']:
                self.assertEqual(actor['speed'], 0)
        for path in (OUTPUT / 'dynamic_obstacle/scenarios').glob('*.json'):
            scene = read(path)
            self.assertFalse(scene['shadedAreaList'])
            for way in scene['waypointDataList']:
                uid = int(way['pathName'].split(' - ')[1])
                actor = next(x for key in ('objectList', 'pedestrianList') for x in scene[key] if x['UNIQUEID'] == uid)
                p = way['waypointData'][0]['pos']
                self.assertAlmostEqual(actor['pos']['x'], -p['x'])
                self.assertAlmostEqual(actor['pos']['y'], -p['z'])

    def test_gui_discovers_campaigns_but_not_case_sidecars(self):
        choices = campaign_examples()
        self.assertEqual(len([p for p in choices if p.parent.parent == OUTPUT]), 5)
        self.assertFalse(any(p.name.endswith('.case.yaml') for p in choices))
        self.assertTrue(default_name(OUTPUT / 'highway/example.yaml').startswith('highway_starter_'))

    def test_space_guard_does_not_require_ros(self):
        cfg = {'output': OUTPUT, 'recording': {'minimum_free_disk_gib': 10}}
        with patch('planning_scenario_validator.runner.shutil.disk_usage',
                   return_value=SimpleNamespace(free=9 * 1024 ** 3)):
            with self.assertRaisesRegex(RuntimeFailure, '저장 공간 부족'):
                check_free_disk(cfg)
        with patch('planning_scenario_validator.runner.shutil.disk_usage',
                   return_value=SimpleNamespace(free=11 * 1024 ** 3)):
            check_free_disk(cfg)
        check_free_disk({'recording': {}})


if __name__ == '__main__':
    unittest.main()
