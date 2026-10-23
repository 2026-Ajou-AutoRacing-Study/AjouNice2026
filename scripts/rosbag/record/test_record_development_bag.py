import contextlib
import importlib.util
import io
from pathlib import Path
import re
import sys
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location('recorder', Path(__file__).with_name('record_development_bag.py'))
recorder = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(recorder)


class RecordingTests(unittest.TestCase):
    def test_core_and_diagnostics_are_selected(self):
        for topic in recorder.REQUIRED + recorder.DIAGNOSTICS + [
                '/sensor/lidar1/packets', '/sensor/camera/back_left/compressed',
                '/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory',
                '/planning/planning_factors/roundabout', '/control/command/control_cmd']:
            with self.subTest(topic=topic):
                self.assertTrue(any(re.match(p, topic) for p in recorder.TOPICS))

    def test_no_duplicate_decoded_images_by_default(self):
        self.assertFalse(any(re.match(p, '/sensor/camera/front') for p in recorder.TOPICS))
        self.assertFalse(any(re.match(p, '/sensor/camera/front/image_raw') for p in recorder.TOPICS))

    def test_placeholder_cruise_output_is_not_required(self):
        self.assertFalse(any(n.endswith('/obstacle_cruise/trajectory') for n in recorder.DIAGNOSTICS))

    def test_list_needs_neither_ros_nor_files(self):
        with patch.object(sys, 'argv', ['recorder', '--list']), contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertEqual(recorder.main(), 0)
        self.assertIn('obstacle_cruise', output.getvalue())

    def test_secret_parameters_redacted_recursively(self):
        self.assertEqual(recorder.scrub({'safe': [{'api_key': 'private', 'n': 2}], 'TOKEN': 'secret'}),
                         {'safe': [{'api_key': '[REDACTED]', 'n': 2}], 'TOKEN': '[REDACTED]'})

    def test_bad_options_fail_without_ros(self):
        for args in (['--duration', '-1'], ['--minimum-free-gib', 'nan'], ['--minimum-free-gib', '1']):
            with self.subTest(args=args), patch.object(sys, 'argv', ['recorder']+args), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    recorder.main()


if __name__ == '__main__':
    unittest.main()
