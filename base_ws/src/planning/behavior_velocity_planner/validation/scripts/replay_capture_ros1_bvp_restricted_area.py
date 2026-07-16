#!/usr/bin/env python3

import os
import runpy
import sys


TOOL = os.path.expanduser(
    "~/AJOUNICE2026/local_autoware_planning_sim/tools/replay_capture_ros1_bvp_restricted_area.py"
)

if not os.path.isfile(TOOL):
    raise RuntimeError("restricted-area replay tool is missing: {}".format(TOOL))

sys.path.insert(0, os.path.dirname(TOOL))
runpy.run_path(TOOL, run_name="__main__")
