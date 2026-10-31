#!/usr/bin/env python3
"""Resolve real launch XML without starting a master, sensor or controller."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[5]
BRINGUP = ROOT / 'base_ws/src/bringup/launch/localization_gnss_imu_gt_ctra.launch'
PREFIX = '/morai_gps_to_local_odometry/'


class GnssDelayLaunchTest(unittest.TestCase):
    def parameters(self, *arguments, installed=False):
        command = ['roslaunch', '--dump-params']
        command += (['bringup', 'localization_gnss_imu_gt_ctra.launch'] if installed
                    else [str(BRINGUP)])
        with tempfile.TemporaryDirectory(prefix='gnss-delay-launch-') as directory:
            env = dict(os.environ, AJOUNICE_ROOT=str(ROOT), ROS_HOME=directory,
                       ROS_LOG_DIR=directory)
            result = subprocess.run(command + list(arguments), env=env,
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                    text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stderr)
        return yaml.safe_load(result.stdout)

    def test_omitted_ms_keeps_zero_legacy_default(self):
        params = self.parameters()
        self.assertEqual(params[PREFIX + 'gnss_delay_ms'], '')
        self.assertEqual(params[PREFIX + 'motion_compensation_sec'], 0.)

    def test_ms_forwarded_as_string_to_converter(self):
        for value in ['0', '10', '20', '30', '22.5']:
            with self.subTest(value=value):
                params = self.parameters('gnss_delay_ms:=' + value)
                self.assertEqual(params[PREFIX + 'gnss_delay_ms'], value)
                self.assertEqual(params[PREFIX + 'motion_compensation_sec'], 0.)

    def test_legacy_seconds_still_forwarded(self):
        params = self.parameters('gnss_motion_compensation_sec:=0.02')
        self.assertEqual(params[PREFIX + 'gnss_delay_ms'], '')
        self.assertEqual(params[PREFIX + 'motion_compensation_sec'], .02)

    def test_installed_entrypoint_forwards_ms_and_preserves_official_start(self):
        params = self.parameters('gnss_delay_ms:=20', 'gt_source:=competition',
                                 'initial_position_source:=official_start', installed=True)
        self.assertEqual(params[PREFIX + 'gnss_delay_ms'], '20')
        self.assertEqual(params['/gnss_imu_initial_pose_publisher/initial_position_source'],
                         'official_start')
        self.assertEqual(params['/morai_gt_motion_adapter/source'], 'competition')
        self.assertFalse(params['/use_sim_time'])


if __name__ == '__main__':
    unittest.main()
