#!/usr/bin/env python3
"""Load the source lane-change ROS2 parameter document into a ROS1 namespace.

This is a candidate-local configuration adapter.  It does not manufacture
defaults: the exact mapping below `/**.ros__parameters` is uploaded unchanged
to the supplied ROS1 private namespace, where the ROS1 manager reads it.
"""

import argparse
from pathlib import Path

import rosparam
import yaml


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("namespace", help="ROS1 private namespace, e.g. /behavior_path_planner")
    parser.add_argument(
        "--source", action="append", type=Path,
        help="source ROS2 parameter YAML; repeat to merge the original test configuration set")
    parser.add_argument(
        "--enable", action="store_true", help="also enable BPP's candidate static lane-change registry")
    args = parser.parse_args()

    parameters = {}
    # In the source tree config/ is one level above scripts/.  After catkin
    # installation the executable lives under lib/<package>/ while config/ is
    # installed under share/<package>/, so resolve the installed layout
    # explicitly rather than relying on the source-tree-relative path.
    source_tree_config = Path(__file__).resolve().parents[1] / "config" / "lane_change.param.yaml"
    installed_config = (
        Path(__file__).resolve().parents[2]
        / "share"
        / "autoware_behavior_path_lane_change_module"
        / "config"
        / "lane_change.param.yaml"
    )
    default_source = source_tree_config if source_tree_config.is_file() else installed_config
    sources = args.source or [default_source]
    for source in sources:
        with source.open(encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
        try:
            current = document["/**"]["ros__parameters"]
        except (KeyError, TypeError) as error:
            raise SystemExit(f"source YAML lacks /**.ros__parameters: {source}") from error
        if not isinstance(current, dict):
            raise SystemExit(f"source parameters must be a mapping: {source}")
        parameters.update(current)
    if args.enable:
        parameters = dict(parameters)
        parameters["enable_lane_change_modules"] = True
    def project(mapping):
        result = {}
        for key, value in mapping.items():
            cursor = result
            names = key.split('.')
            for name in names[:-1]:
                cursor = cursor.setdefault(name, {})
            cursor[names[-1]] = project(value) if isinstance(value, dict) else value
        return result
    rosparam.upload_params(args.namespace.rstrip("/") or "/", project(parameters))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
