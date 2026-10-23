import argparse
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location('viewer', Path(__file__).with_name('view_e12_postprocessing.py'))
viewer = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(viewer)


class ViewerTest(unittest.TestCase):
    def scene(self):
        return dict(schema='e12_postprocessing_rviz_scene_v1', complete=True,
            raw_bag='/raw.bag', marker_bag='/markers.bag',
            fixed_frame='lidar1', topics=list(viewer.MARKER_TOPICS),
            start_offset_sec=2.5, duration_sec=60.)

    def test_paused_real_time_and_strict_whitelist(self):
        command = viewer.play_command(self.scene(), 1.)
        self.assertIn('--pause', command)
        self.assertIn('--keep-alive', command)
        self.assertEqual(command[command.index('--rate') + 1], '1.0')
        topics = command[command.index('--topics') + 1:]
        self.assertEqual(set(topics), set(viewer.SENSOR_TOPICS + viewer.MARKER_TOPICS))
        self.assertTrue(all(topic.startswith(('/sensor/', '/e12_view/')) for topic in topics))
        self.assertNotIn('/tf', topics)
        self.assertNotIn('/Ego_topic', topics)
        self.assertEqual(command[command.index('--start') + 1], '2.25')
        self.assertEqual(command[command.index('--duration') + 1], '60.25')

    def test_short_smoke_exits(self):
        command = viewer.play_command(self.scene(), 1., True, 1.)
        self.assertNotIn('--pause', command)
        self.assertNotIn('--keep-alive', command)
        self.assertEqual(command[command.index('--duration') + 1], '1.0')

    def test_rate_validation(self):
        for value in ('0', '-1', 'nan', 'inf'):
            with self.assertRaises(argparse.ArgumentTypeError):
                viewer.positive(value)
        self.assertEqual(viewer.positive('1.0'), 1.)

    def test_private_environment(self):
        with patch.dict(viewer.os.environ, {'ROS_MASTER_URI': 'http://host:11311', 'ROS_HOSTNAME': 'host'}):
            env = viewer.private_environment(Path('/new'), 43321)
        self.assertEqual(env['ROS_MASTER_URI'], 'http://127.0.0.1:43321')
        self.assertNotIn('ROS_HOSTNAME', env)
        self.assertEqual(env['ROS_LOG_DIR'], '/new/ros_logs')

    def test_displays_have_no_tf_robot_or_live_topics(self):
        config = viewer.rviz_configuration('lidar1')
        manager = config['Visualization Manager']
        self.assertEqual(manager['Global Options']['Fixed Frame'], 'lidar1')
        self.assertFalse(any(item['Class'] in ('rviz/TF', 'rviz/RobotModel') for item in manager['Displays']))
        prefix = next(item for item in manager['Displays'] if item.get('Marker Topic') == '/e12_view/prefix5')
        self.assertFalse(prefix['Enabled'])

    def test_scene_rejects_non_audited_contract(self):
        with tempfile.TemporaryDirectory() as root:
            root = Path(root)
            bag = root / 'raw.bag'
            bag.touch()
            scene = self.scene()
            scene.update(raw_bag=str(bag), marker_bag=str(bag))
            path = root / 'scene.json'
            path.write_text(json.dumps(scene))
            self.assertEqual(viewer.read_scene(path)['fixed_frame'], 'lidar1')
            scene['topics'].append('/vehicle_cmd')
            path.write_text(json.dumps(scene))
            with self.assertRaises(ValueError):
                viewer.read_scene(path)

    def test_incomplete_and_changed_export_rejected(self):
        with tempfile.TemporaryDirectory() as root:
            root = Path(root)
            bag = root / 'markers.bag'
            bag.touch()
            scene = self.scene()
            scene.update(raw_bag=str(bag), marker_bag=str(bag), complete=False)
            path = root / 'scene.json'
            path.write_text(json.dumps(scene))
            with self.assertRaises(ValueError):
                viewer.read_scene(path)
            scene.update(complete=True, marker_bag_sha256='not-the-bag-digest')
            path.write_text(json.dumps(scene))
            with self.assertRaises(ValueError):
                viewer.read_scene(path)


if __name__ == '__main__':
    unittest.main()
