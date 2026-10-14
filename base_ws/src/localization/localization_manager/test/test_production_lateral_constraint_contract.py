#!/usr/bin/env python3
"""Offline production/opt-out wiring checks; no ROS master or vehicle required.

The launch arguments are the authority, including when old personal noise/EKF
profiles are supplied.  The separate model-prior topic must never become a
measurement of the withheld Competition velocity.y or a Local EKF input.
"""

import importlib.util
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[5]
MANAGER = ROOT / "base_ws/src/localization/localization_manager"
PROVIDER = MANAGER / "launch/gnss_imu_gt_ctra.launch"
BRINGUP = ROOT / "base_ws/src/bringup/launch/localization_gnss_imu_gt_ctra.launch"
CONTRACT_PATH = (
    ROOT
    / "base_ws/src/planning_addons/planning_scenario_validator/src"
    / "planning_scenario_validator/localization_contract.py"
)
PRIOR_TOPIC = "/localization/constraints/lateral_velocity"
NOISE_NAMESPACE = "$(arg noise_namespace)/lateral_model/"
ENABLE_ARG = "use_lateral_velocity_constraint"
SIGMA_ARG = "lateral_velocity_stddev_mps"


def yaml_config(name):
    return yaml.safe_load((ROOT / "configs/localization" / name).read_text())


def launch_arg(root, name):
    return next(item for item in root.findall("arg") if item.get("name") == name)


def named_parameter(parent, name):
    return [
        element
        for element in parent
        if element.tag in ("param", "rosparam")
        and (element.get("name") or element.get("param")) == name
    ]


def global_node(root):
    return next(
        item
        for item in root.findall("node")
        if item.get("name") == "gnss_imu_gt_global_ekf"
    )


class ProductionLateralConstraintContractTest(unittest.TestCase):
    def setUp(self):
        self.provider = ET.parse(PROVIDER).getroot()
        self.bringup = ET.parse(BRINGUP).getroot()

    def test_default_on_and_sigma_are_forwarded_through_bringup(self):
        include = next(
            item
            for item in self.bringup.findall("include")
            if item.get("file")
            == "$(find localization_manager)/launch/gnss_imu_gt_ctra.launch"
        )
        for name, expected in ((ENABLE_ARG, "true"), (SIGMA_ARG, "0.20")):
            with self.subTest(argument=name):
                self.assertEqual(launch_arg(self.provider, name).get("default"), expected)
                self.assertEqual(launch_arg(self.bringup, name).get("default"), expected)
                forwarded = next(arg for arg in include.findall("arg") if arg.get("name") == name)
                self.assertEqual(forwarded.get("value"), "$(arg {})".format(name))

    def test_explicit_prior_overrides_legacy_profile_and_stale_parameters(self):
        children = list(self.provider)
        profile_index = next(
            index
            for index, element in enumerate(children)
            if element.tag == "rosparam"
            and element.get("file") == "$(arg measurement_noise_config)"
        )
        adapter_index = next(
            index
            for index, element in enumerate(children)
            if element.tag == "node" and element.get("name") == "morai_gt_motion_adapter"
        )
        expected = {
            "enabled": ("bool", "$(arg {})".format(ENABLE_ARG)),
            "stddev_mps": ("double", "$(arg {})".format(SIGMA_ARG)),
            "topic": ("str", PRIOR_TOPIC),
        }
        for suffix, (parameter_type, value) in expected.items():
            with self.subTest(parameter=suffix):
                matches = named_parameter(self.provider, NOISE_NAMESPACE + suffix)
                self.assertEqual(len(matches), 1)
                element = matches[0]
                self.assertGreater(children.index(element), profile_index)
                self.assertLess(children.index(element), adapter_index)
                # roslaunch's omitted type is string, matching the canonical topic.
                self.assertEqual(element.get("type", "str"), parameter_type)
                self.assertEqual(element.get("value"), value)

    def test_both_ekf_namespaces_are_cleared_between_runs(self):
        for name in ("gnss_imu_gt_local_ekf", "gnss_imu_gt_global_ekf"):
            with self.subTest(ekf=name):
                node = next(item for item in self.provider.findall("node") if item.get("name") == name)
                self.assertEqual(node.get("clear_params"), "true")

    def test_global_only_mask_and_opt_out_override_legacy_overlay(self):
        node = global_node(self.provider)
        children = list(node)
        overlay_index = next(
            index
            for index, element in enumerate(children)
            if element.tag == "rosparam"
            and element.get("file") == "$(arg global_ekf_gt_config)"
        )
        masks = named_parameter(node, "twist1_config")
        self.assertEqual(len(masks), 1)
        mask = masks[0]
        self.assertGreater(children.index(mask), overlay_index)
        self.assertEqual(mask.get("subst_value"), "true")
        for enabled in (True, False):
            with self.subTest(enabled=enabled):
                resolved = (mask.text or "").replace(
                    "$(arg {})".format(ENABLE_ARG), "true" if enabled else "false"
                )
                values = yaml.safe_load(resolved)
                self.assertEqual(len(values), 15)
                self.assertTrue(all(type(value) is bool for value in values))
                self.assertEqual(
                    [index for index, value in enumerate(values) if value],
                    [7] if enabled else [],
                )
        fixed_parameters = {
            "twist1": PRIOR_TOPIC,
            "twist1_queue_size": "100",
            "twist1_nodelay": "true",
            "twist1_differential": "false",
            "twist1_relative": "false",
        }
        for name, expected in fixed_parameters.items():
            with self.subTest(parameter=name):
                matches = named_parameter(node, name)
                self.assertEqual(len(matches), 1)
                self.assertGreater(children.index(matches[0]), overlay_index)
                self.assertEqual(matches[0].get("value"), expected)

    def test_local_ekf_and_existing_motion_masks_are_unchanged(self):
        local = next(
            item
            for item in self.provider.findall("node")
            if item.get("name") == "gnss_imu_gt_local_ekf"
        )
        self.assertFalse(named_parameter(local, "twist1"))
        self.assertFalse(named_parameter(local, "twist1_config"))
        for filename in (
            "robot_localization_gnss_imu_gt_ctra_local.yaml",
            "robot_localization_gnss_imu_gt_ctra_global.yaml",
        ):
            with self.subTest(profile=filename):
                config = yaml_config(filename)
                self.assertEqual([i for i, v in enumerate(config["imu0_config"]) if v], [11, 12])
                self.assertEqual([i for i, v in enumerate(config["twist0_config"]) if v], [6])
        self.assertNotIn("twist1", yaml_config("robot_localization_gnss_imu_gt_ctra_local.yaml"))

    def test_tracked_profiles_document_global_only_soft_model(self):
        noise = yaml_config("gnss_imu_gt_noise_sensor6_ideal.yaml")
        self.assertIs(noise["lateral_model"]["enabled"], True)
        self.assertEqual(noise["lateral_model"]["stddev_mps"], 0.20)
        self.assertEqual(noise["lateral_model"]["topic"], PRIOR_TOPIC)
        self.assertEqual(noise["gnss"]["horizontal_stddev_m"], 0.10)
        global_config = yaml_config("robot_localization_gnss_imu_gt_ctra_global.yaml")
        self.assertEqual(global_config["twist1"], PRIOR_TOPIC)
        self.assertEqual([i for i, v in enumerate(global_config["twist1_config"]) if v], [7])

    def test_validator_defaults_and_explicit_opt_out_are_recordable(self):
        spec = importlib.util.spec_from_file_location("production_localization_contract", str(CONTRACT_PATH))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.assertIn(ENABLE_ARG, module.BOOL_ARGS)
        self.assertIn(SIGMA_ARG, module.POSITIVE_ARGS)
        self.assertIs(module.LOCALIZATION_DEFAULTS[ENABLE_ARG], True)
        self.assertEqual(module.LOCALIZATION_DEFAULTS[SIGMA_ARG], 0.20)
        explicit = {ENABLE_ARG: False, SIGMA_ARG: 0.25}
        stack = {"pose_source": "localization", "localization_args": explicit}
        result = module.normalize_localization_launch_args(stack, check_paths=False)
        self.assertEqual(result, explicit)
        config = {"network": {"sensor_json": "/test/sensor.json"}, "localization_launch_args": result}
        self.assertEqual(module.effective_localization_args(config, ROOT)[ENABLE_ARG], False)
        for key, value in ((ENABLE_ARG, "false"), (SIGMA_ARG, 0), (SIGMA_ARG, True), (SIGMA_ARG, float("nan"))):
            with self.subTest(argument=key, value=value):
                with self.assertRaises(ValueError):
                    module.normalize_localization_launch_args(
                        {"pose_source": "localization", "localization_args": {key: value}},
                        check_paths=False,
                    )


if __name__ == "__main__":
    unittest.main()
