"""Pure roslaunch parameter evaluation; no master, drivers, UDP, or nodes.

Compare source defaults to frozen git HEAD, including the included lidar launch.
The source package is intentionally first in ROS_PACKAGE_PATH: old installed
launch copies are not rewritten or deployed by this qualification.
"""

import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[4]
PACKAGE = ROOT / "base_ws/src/sensor_interface"
DRIVER = "/lidar/lidar_driver/"


class LidarLaunchTrialTests(unittest.TestCase):
    def test_driver_restart_clears_only_its_private_parameter_namespace(self):
        document = ET.parse(PACKAGE/'launch/lidar.launch')
        nodes = {node.attrib['name']: node for node in document.findall('.//node')}
        self.assertEqual(nodes['lidar_driver'].attrib.get('clear_params'), 'true')
        self.assertNotIn('clear_params', nodes['lidar_transform'].attrib)
        self.assertNotIn('clear_params', nodes['lidar_nodelet_manager'].attrib)

    @classmethod
    def setUpClass(cls):
        if shutil.which("roslaunch") is None:
            raise unittest.SkipTest("Source the ROS workspace before pure launch tests")
        cls.reference = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
        cls.temporary = tempfile.TemporaryDirectory(prefix="lidar-launch-reference-")
        cls.reference_package = Path(cls.temporary.name) / "sensor_interface"
        (cls.reference_package / "launch").mkdir(parents=True)
        shutil.copyfile(PACKAGE / "package.xml", cls.reference_package / "package.xml")
        for filename in ("lidar.launch", "morai_udp_interface.launch"):
            relative = f"base_ws/src/sensor_interface/launch/{filename}"
            content = subprocess.check_output(["git", "show", f"{cls.reference}:{relative}"], cwd=ROOT)
            (cls.reference_package / "launch" / filename).write_bytes(content)

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def dump(self, filename="lidar.launch", arguments=(), reference=False, expect_success=True):
        package = self.reference_package if reference else PACKAGE
        environment = dict(os.environ, AJOUNICE_ROOT=str(ROOT))
        environment["ROS_PACKAGE_PATH"] = str(package) + ":" + environment.get("ROS_PACKAGE_PATH", "")
        # $(find pkg)/resource prefers catkin installed share even over a source
        # rospack match. Keep only the distro prefix for this isolated source test.
        environment["CMAKE_PREFIX_PATH"] = "/opt/ros/noetic"
        # Even an accidental master-dependent path must not contact the user's master.
        environment["ROS_MASTER_URI"] = "http://127.0.0.1:1"
        result = subprocess.run(["roslaunch", "--dump-params", str(package / "launch" / filename),
                                 *arguments], env=environment, capture_output=True, text=True, timeout=15)
        # Noetic's --dump-params prints evaluation errors but can return exit status 0.
        error = "Unable to load file" in result.stdout + result.stderr
        if expect_success:
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertFalse(error, result.stdout + result.stderr)
            values = yaml.safe_load(result.stdout)
            self.assertIsInstance(values, dict, result.stdout + result.stderr)
            return values
        self.assertTrue(result.returncode != 0 or error, result.stdout + result.stderr)
        return result.stdout + result.stderr

    def test_defaults_exactly_preserve_all_head_driver_and_gateway_parameters(self):
        for filename in ("lidar.launch", "morai_udp_interface.launch"):
            with self.subTest(filename=filename):
                before = self.dump(filename, reference=True)
                after = self.dump(filename)
                self.assertEqual(after, before)
                self.assertNotIn(DRIVER + "npackets", after)
                self.assertEqual(after[DRIVER + "cut_angle"], -0.01)
                self.assertIs(after[DRIVER + "gps_time"], False)
                self.assertIs(after[DRIVER + "timestamp_first_packet"], True)

    def test_explicit_empty_packet_override_preserves_default_parameter_absence(self):
        self.assertEqual(self.dump(arguments=("npackets:=",)), self.dump())
        self.assertEqual(self.dump("morai_udp_interface.launch", ("lidar_npackets:=",)),
                         self.dump("morai_udp_interface.launch"))
        # Noetic CLI drops empty mappings, so an empty cut CLI argument also
        # means the default, not an empty radian number passed to the driver.
        self.assertEqual(self.dump(arguments=("cut_angle:=",)), self.dump())

    def test_fixed50_changes_only_packet_override(self):
        for filename, argument in (("lidar.launch", "npackets:=50"),
                                   ("morai_udp_interface.launch", "lidar_npackets:=50")):
            with self.subTest(filename=filename):
                expected = self.dump(filename)
                expected[DRIVER + "npackets"] = 50
                candidate = self.dump(filename, (argument,))
                self.assertEqual(candidate, expected)
                self.assertIs(type(candidate[DRIVER + "npackets"]), int)

    def test_cut_zero_changes_only_azimuth_cut(self):
        for filename, argument in (("lidar.launch", "cut_angle:=0"),
                                   ("morai_udp_interface.launch", "lidar_cut_angle:=0")):
            with self.subTest(filename=filename):
                expected = self.dump(filename)
                expected[DRIVER + "cut_angle"] = 0.0
                self.assertEqual(self.dump(filename, (argument,)), expected)

    def test_valid_positive_packet_counts_and_radian_cuts(self):
        for value in ("1", "51", "100", "2147483647"):
            with self.subTest(npackets=value):
                self.assertEqual(self.dump(arguments=(f"npackets:={value}",))[DRIVER + "npackets"], int(value))
        for value in ("-0.01", "0.0", "3.141592653589793", "6.283185307179585"):
            with self.subTest(cut_angle=value):
                self.assertEqual(self.dump(arguments=(f"cut_angle:={value}",))[DRIVER + "cut_angle"], float(value))

    def test_bad_packet_counts_fail_during_launch_evaluation(self):
        for value in ("0", "-1", "-50", "50.5", "50.0", "true", "false", "none", "nan", "inf", "2147483648"):
            with self.subTest(npackets=value):
                self.dump(arguments=(f"npackets:={value}",), expect_success=False)
        self.dump("morai_udp_interface.launch", ("lidar_npackets:=0",), expect_success=False)

    def test_bad_cuts_fail_independently_of_packet_override(self):
        for value in ("-0.02", "-1", "6.283185307179586", "7", "nan", "inf", "-inf", "true", "false", "none"):
            with self.subTest(cut_angle=value):
                self.dump(arguments=("npackets:=50", f"cut_angle:={value}"), expect_success=False)
        self.dump("morai_udp_interface.launch", ("lidar_cut_angle:=7",), expect_success=False)

    def test_constant_validation_arguments_cannot_be_overridden(self):
        self.dump(arguments=("validated_npackets:=50",), expect_success=False)
        self.dump(arguments=("validated_cut_angle:=0",), expect_success=False)


if __name__ == "__main__":
    unittest.main()
