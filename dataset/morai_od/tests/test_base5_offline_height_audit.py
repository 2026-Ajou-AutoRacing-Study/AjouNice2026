import sys
import unittest
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from audit_base5_scenarios import bounded_vehicle_pose_matches


class OfflineHeightAuditTest(unittest.TestCase):
    def test_candidate_replaces_legacy_offset_without_doubling(self):
        road=dict(x=1.,y=2.,z=28.)
        init=dict(usable=True,pivot_height_candidate_m=.532)
        self.assertTrue(bounded_vehicle_pose_matches(dict(road,z=28.532),road,init))
        self.assertFalse(bounded_vehicle_pose_matches(dict(road,z=28.892),road,init))
        self.assertFalse(bounded_vehicle_pose_matches(dict(road,x=1.01,z=28.532),road,init))

    def test_unavailable_or_legacy_candidate_retains_original_offset(self):
        road=dict(x=1.,y=2.,z=28.)
        for init in (None,dict(usable=False)):
            self.assertTrue(bounded_vehicle_pose_matches(dict(road,z=28.36),road,init))
            self.assertFalse(bounded_vehicle_pose_matches(road,road,init))
