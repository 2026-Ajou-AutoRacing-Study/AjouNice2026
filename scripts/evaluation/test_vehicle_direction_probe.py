#!/usr/bin/env python3
"""Compile pure causal-policy boundaries without ROS, a GPU, or a master."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


class VehicleDirectionGuardBoundaryTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which('g++'), 'C++ compiler unavailable')
    def test_actual_cpp_boundary_contract(self):
        source = Path(__file__).with_name('test_vehicle_motion_direction_guard.cpp')
        with tempfile.TemporaryDirectory(prefix='ajou-direction-test-') as directory:
            binary = Path(directory)/'test_guard'
            subprocess.run(['g++','-std=c++14','-O2',str(source),'-o',str(binary)],
                           check=True,capture_output=True,text=True,timeout=30)
            result = subprocess.run([str(binary)],check=True,capture_output=True,text=True,timeout=10)
            self.assertIn('boundary checks passed',result.stdout)


if __name__=='__main__': unittest.main()
