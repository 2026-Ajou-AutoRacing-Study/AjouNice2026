#!/usr/bin/env python3
# 목적: trajectory_batch_runner 내부 Python 모듈을 catkin 환경에 설치한다.
# 역할: scripts 진입점이 src/trajectory_batch_runner package를 import할 수 있게 한다.

"""trajectory_batch_runner Python package setup."""

from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup


setup(
    **generate_distutils_setup(
        packages=["trajectory_batch_runner"],
        package_dir={"": "src"},
    )
)
