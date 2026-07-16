#!/usr/bin/env python3

from catkin_pkg.python_setup import generate_distutils_setup
from distutils.core import setup


setup(
    **generate_distutils_setup(
        packages=["autoware_path_optimizer"],
        package_dir={"": "src"},
    )
)
