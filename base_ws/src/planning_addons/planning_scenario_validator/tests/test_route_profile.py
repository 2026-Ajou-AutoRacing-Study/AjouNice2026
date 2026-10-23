import copy
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import yaml

from planning_scenario_validator.config import ROOT, load_campaign, resolve_path, ConfigError
from planning_scenario_validator.executions import prepare_execution, load_execution
from planning_scenario_validator.route_profile import normalize, ProfileGeometry, preferred_primitives, apply_and_verify

DEFINITION = ROOT / 'base_ws/src/planning_addons/competition_route_requester/config/kcity_v85_competition_route.yaml'
SOURCE = ROOT / 'base_ws/src/planning_addons/planning_scenario_validator/config/missions/highway/base/example.yaml'


class RouteProfileTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.document = yaml.safe_load(SOURCE.read_text())
        cls.document['route'] = {'name':'route_b','definition_file':str(DEFINITION)}
        cls.document['default_case']['goal'] = dict(x=201.86260986328125,y=126.93470764160156,
                                                   z=0.0,yaw_deg=-98.6938087895289)
        cls.profile = normalize(cls.document, resolve_path)
        cls.geometry = ProfileGeometry(cls.profile, resolve_path)

    def plan(self):
        return self.geometry.plan(dict(x=59.62204096,y=887.64435555,yaw_deg=-0.5),
                                  self.document['default_case']['goal'])

    def test_cropped_b_route_preserves_highway_goal_and_uses_left_column(self):
        plan = self.plan()
        ids = plan['expected_preferred_lanelet_ids']
        self.assertEqual(plan['goal'], self.document['default_case']['goal'])
        self.assertEqual(ids[:6], [201096,201087,201086,201459,201436,201465])
        self.assertEqual(ids[-1], 200183)
        self.assertNotIn(200210,ids)
        self.assertEqual(len(plan['waypoints']),len(ids)-2)

    def test_every_waypoint_is_on_its_configured_b_lane(self):
        plan = self.plan()
        for pose,lane in zip(plan['waypoints'],plan['expected_preferred_lanelet_ids'][1:-1]):
            i,s = self.geometry.locate(pose)
            self.assertEqual(self.geometry.sequence[i],lane)
            self.assertAlmostEqual(s,self.geometry.lengths[i]/2,places=4)
            self.assertGreater(pose['z'],20)

    def test_off_route_and_backwards_goals_are_rejected(self):
        with self.assertRaises(ValueError):
            self.geometry.plan(dict(x=1e6,y=1e6,yaw_deg=0),self.document['default_case']['goal'])
        with self.assertRaises(ValueError):
            self.geometry.plan(self.document['default_case']['goal'],dict(x=59.622,y=887.644,yaw_deg=-0.5))

    def test_preferred_selection_never_falls_back_to_observed_right_lane(self):
        left=SimpleNamespace(id=201459,primitive_type='lane')
        right=SimpleNamespace(id=201456,primitive_type='lane')
        segment=SimpleNamespace(preferred_primitive=right,primitives=[left,right])
        route=SimpleNamespace(segments=[segment])
        self.assertEqual([p.id for p in preferred_primitives(route,[201459])],[201459])
        segment.primitives=[right]
        with self.assertRaisesRegex(ValueError,'후보'):
            preferred_primitives(route,[201459])
        with self.assertRaisesRegex(ValueError,'구간 수'):
            preferred_primitives(route,[201459,201436])

    def test_custom_waypoints_cannot_silently_override_b_profile(self):
        doc=copy.deepcopy(self.document)
        doc['default_case']['waypoints']=[dict(x=1,y=2)]
        with self.assertRaises(ValueError):normalize(doc,resolve_path)
        doc=copy.deepcopy(self.document);doc['route']['name']='typo'
        with self.assertRaises(ValueError):normalize(doc,resolve_path)

    def test_route_definition_is_frozen_for_resume_and_tampering_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            doc=copy.deepcopy(self.document)
            doc['scenarios']['include']=[p.name for p in sorted(
                resolve_path(doc['scenarios']['directory']).glob('*.json'))][:1]
            doc['stack']['detector']='od_gt'
            doc['output_root']=str(root/'output')
            definition=root/'definition.yaml';definition.write_bytes(DEFINITION.read_bytes())
            doc['route']['definition_file']=str(definition)
            source=root/'campaign.yaml';source.write_text(yaml.safe_dump(doc))
            cfg=prepare_execution(source,check_install=False)
            frozen=cfg['route']['definition_file']
            self.assertEqual(frozen,cfg['output']/'routes/definition.yaml')
            definition.write_text('changed: true')
            resumed=load_execution(cfg['output'],check_install=False)
            self.assertEqual(resumed['route']['definition_sha256'],self.profile['definition_sha256'])
            frozen.write_text(frozen.read_text()+'\n')
            with self.assertRaisesRegex(ValueError,'사본'):
                load_execution(cfg['output'],check_install=False)

    def test_invalid_profile_config_reports_clear_config_error(self):
        with tempfile.TemporaryDirectory() as directory:
            doc=copy.deepcopy(self.document);doc['route']['name']='typo'
            p=Path(directory)/'campaign.yaml';p.write_text(yaml.safe_dump(doc))
            with self.assertRaisesRegex(ConfigError,'route 설정'):
                load_campaign(p,check_install=False)

    def test_empty_route_definition_is_rejected_before_ros(self):
        with tempfile.TemporaryDirectory() as directory:
            definition=Path(directory)/'empty.yaml';definition.write_text('')
            doc=copy.deepcopy(self.document)
            doc['route']['definition_file']=str(definition)
            with self.assertRaisesRegex(ValueError,'mapping'):
                normalize(doc,resolve_path)

    def make_monitor(self, preferred=201459):
        from planning_scenario_validator.runtime import LiveMonitor
        from autoware_planning_msgs.msg import LaneletRoute,LaneletSegment,LaneletPrimitive
        route=LaneletRoute()
        route.uuid.uuid=[1]*16
        segment=LaneletSegment()
        segment.preferred_primitive=LaneletPrimitive(id=preferred,primitive_type='lane')
        segment.primitives=[LaneletPrimitive(id=i,primitive_type='lane') for i in (201459,201456)]
        route.segments=[segment]
        event_monitor=LiveMonitor.__new__(LiveMonitor)
        event_monitor.events_path=None
        monitor=SimpleNamespace(route=route,stamps={'route':0},event=event_monitor.event,
                                wait_route_and_trajectory=Mock())
        def wait(predicate,*args):
            self.assertTrue(predicate())
        monitor.wait=wait
        plan={'name':'route_b','expected_preferred_lanelet_ids':[201459],'verified':False}
        return monitor,plan

    def test_exact_initial_b_route_is_verified_without_another_rpc(self):
        monitor,plan=self.make_monitor()
        with patch('rospy.wait_for_service') as wait,patch('rospy.ServiceProxy') as proxy:
            apply_and_verify(plan,monitor,bytes([1]*16),timeout=1)
        wait.assert_not_called();proxy.assert_not_called()
        self.assertTrue(plan['verified'])
        self.assertFalse(plan['preferred_update_requested'])

    def test_verified_event_uses_the_real_monitor_logger(self):
        import json
        from planning_scenario_validator.runtime import LiveMonitor
        monitor,plan=self.make_monitor()
        with tempfile.TemporaryDirectory() as directory:
            logger=LiveMonitor.__new__(LiveMonitor)
            logger.events_path=Path(directory)/'events.jsonl'
            monitor.event=logger.event
            apply_and_verify(plan,monitor,bytes([1]*16),timeout=1)
            row=json.loads(logger.events_path.read_text())
            self.assertEqual(row['event'],'ROUTE_PROFILE_VERIFIED')
            self.assertEqual(row['profile'],'route_b')
            self.assertEqual(row['preferred_lanelet_ids'],[201459])

    def test_changed_preference_waits_for_new_b_route_and_trajectory(self):
        import time
        monitor,plan=self.make_monitor(preferred=201456)
        def invoke(request):
            monitor.route.segments[0].preferred_primitive=request.preferred_primitives[0]
            monitor.stamps['route']=time.monotonic()
            return SimpleNamespace(status=SimpleNamespace(success=True))
        with patch('rospy.wait_for_service'),patch('rospy.ServiceProxy',return_value=invoke):
            apply_and_verify(plan,monitor,bytes([1]*16),timeout=1)
        self.assertTrue(plan['verified'])
        self.assertEqual(plan['verified_preferred_lanelet_ids'],[201459])
        monitor.wait_route_and_trajectory.assert_called_once()

    def test_rejected_preference_does_not_mark_route_verified(self):
        from planning_scenario_validator.runtime import RuntimeFailure
        monitor,plan=self.make_monitor(preferred=201456)
        response=SimpleNamespace(status=SimpleNamespace(success=False))
        with patch('rospy.wait_for_service'),patch('rospy.ServiceProxy',return_value=lambda _:response):
            with self.assertRaises(RuntimeFailure):
                apply_and_verify(plan,monitor,bytes([1]*16),timeout=1)
        self.assertFalse(plan['verified'])
        monitor.wait_route_and_trajectory.assert_not_called()


if __name__ == '__main__':
    unittest.main()
