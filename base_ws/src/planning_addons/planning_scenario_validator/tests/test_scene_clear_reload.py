"""A clear-only UDP request must finish before loading unchanged actors."""
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from planning_scenario_validator.runner import _load_scene
from planning_scenario_validator.runtime import LiveMonitor, RuntimeFailure


class SceneClearReloadTest(unittest.TestCase):
    def setUp(self):
        self.sent = []
        self.socket = Mock()
        self.socket.__enter__ = Mock(return_value=self.socket)
        self.socket.__exit__ = Mock(return_value=False)
        self.socket.sendto.side_effect = lambda payload, address: self.sent.append((payload, address)) or len(payload)
        self.monitor = SimpleNamespace(ego=None, event=Mock(), wait_empty_scene=Mock(), wait_scenario=Mock())
        self.cfg = {'document': {'morai': {'scenario_load_host': '10.42.0.252',
                   'scenario_load_port': 9095, 'clear_before_load': True}}}

    def load(self):
        with patch('planning_scenario_validator.runner.socket.socket', return_value=self.socket), \
                patch('planning_scenario_validator.runner.time.sleep'):
            _load_scene(self.cfg, Path('/unused/installed_case.json'), self.monitor, (1., 2.))

    def test_clear_is_separate_and_does_not_reload_ego_until_normal_request(self):
        self.load()
        self.assertEqual(len(self.sent), 2)
        clear, normal = [p for p, address in self.sent]
        self.assertEqual(clear[:60], normal[:60])
        self.assertEqual(clear[60:67], bytes([1, 1, 0, 1, 1, 1, 0]))
        self.assertEqual(normal[60:67], bytes([0, 1, 1, 1, 1, 1, 0]))
        self.monitor.wait_empty_scene.assert_called_once()
        self.monitor.wait_scenario.assert_called_once()
        self.assertEqual(self.sent[0][1], ('10.42.0.252', 9095))

    def test_failed_clear_confirmation_prevents_normal_load(self):
        self.monitor.wait_empty_scene.side_effect = RuntimeFailure('not empty')
        with self.assertRaises(RuntimeFailure):
            self.load()
        self.assertEqual(len(self.sent), 1)
        self.monitor.wait_scenario.assert_not_called()

    def test_existing_missions_keep_single_load_unless_enabled(self):
        self.cfg['document']['morai']['clear_before_load'] = False
        self.load()
        self.assertEqual(len(self.sent), 1)
        self.monitor.wait_empty_scene.assert_not_called()

    def test_empty_scene_requires_fresh_gt_and_all_actor_classes_empty(self):
        monitor = LiveMonitor.__new__(LiveMonitor)
        monitor.gt_objects = SimpleNamespace(npc_list=[], pedestrian_list=[], obstacle_list=[])
        monitor.stamps = {'gt_objects': 11.}
        monitor.wait = lambda predicate, timeout, description: self.assertTrue(predicate())
        monitor.wait_empty_scene(after=10., timeout=1.)
        monitor.wait = lambda predicate, timeout, description: self.assertFalse(predicate())
        monitor.wait_empty_scene(after=12., timeout=1.)
        monitor.gt_objects.obstacle_list = [object()]
        monitor.wait_empty_scene(after=10., timeout=1.)


if __name__ == '__main__':
    unittest.main()
