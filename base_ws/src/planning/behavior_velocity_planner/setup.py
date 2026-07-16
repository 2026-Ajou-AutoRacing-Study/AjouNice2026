#!/usr/bin/env python3

from setuptools import setup

from catkin_pkg.python_setup import generate_distutils_setup


setup_args = generate_distutils_setup(
    packages=[
        "behavior_velocity_planner_common",
        "behavior_velocity_blind_spot_module",
        "behavior_velocity_crosswalk_module",
        "behavior_velocity_detection_area_module",
        "behavior_velocity_intersection_module",
        "behavior_velocity_merge_from_private_module",
        "behavior_velocity_no_drivable_lane_module",
        "behavior_velocity_no_stopping_area_module",
        "behavior_velocity_occlusion_spot_module",
        "behavior_velocity_run_out_module",
        "behavior_velocity_speed_bump_module",
        "behavior_velocity_stop_line_module",
        "behavior_velocity_traffic_light_module",
        "behavior_velocity_virtual_traffic_light_module",
        "behavior_velocity_walkway_module",
    ],
    package_dir={"": "src"},
)

setup(**setup_args)
