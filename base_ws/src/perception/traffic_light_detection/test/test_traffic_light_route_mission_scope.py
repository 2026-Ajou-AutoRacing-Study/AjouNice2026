#!/usr/bin/env python3
"""Pure opt-in route scope; no ROS, map loading, model or process startup."""
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch


def load():
    name='traffic_light_route_mission_scope_fake'
    path=Path(__file__).resolve().parents[1]/'scripts/traffic_light_route_group_id_node.py'
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules,{'rospy':types.ModuleType('rospy'),
        'std_msgs.msg':types.SimpleNamespace(UInt64=object),
        'lanelet_route_matcher':types.SimpleNamespace(LaneletRouteGroupIdMatcher=object)}):
        spec.loader.exec_module(module)
    return module


class MissionScopeTest(unittest.TestCase):
    def test_map_candidate_skips_old_D1_without_hiding_next_D2(self):
        module=load();original={1:201290,2:201397,3:201326,4:201398}
        scoped=module.restrict_route_groups(original,sorted(module.MISSION_GROUP_IDS))
        self.assertEqual(scoped,{1:201290,3:201326})
        self.assertEqual(original[2],201397)
        self.assertEqual(next(g for lane,g in scoped.items() if lane>1),201326)

    def test_legacy_empty_scope_preserves_D1_and_does_not_mutate_map(self):
        module=load();original={1:201397,2:201326}
        scoped=module.restrict_route_groups(original,[])
        self.assertEqual(scoped,original);self.assertIsNot(scoped,original)

    def test_nonmission_bool_and_string_ids_never_silently_enable_all(self):
        module=load()
        for value in ([201397],[True],['201351'],None,{},201351):
            with self.assertRaises(ValueError):module.restrict_route_groups({1:201351},value)


if __name__=='__main__':unittest.main()
