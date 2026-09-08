#!/usr/bin/env python3
"""Render the exact ROS2 parameter leaves as nested ROS1 YAML.

The renderer is deliberately lossless: it never invents a missing Static
Obstacle parameter.  The three manager-interface values are explicit ROS1
provider requirements, not substitutes for module policy values.
"""
import argparse
from pathlib import Path

import yaml


def merge_leaf(output, dotted_path, value):
    """Write ``a.b.c`` as the ROS1 parameter path ``/a/b/c``.

    The ROS1 compatibility facade deliberately turns a ROS2 dotted lookup
    into a slash-separated NodeHandle lookup.  YAML must therefore be nested;
    a literal dotted YAML key would publish ``/a.b.c`` and silently fall back
    to the facade's zero/default value.
    """
    current = output
    parts = dotted_path.split(".")
    for part in parts[:-1]:
        current = current.setdefault(part, {})
    current[parts[-1]] = value


def flatten(value, prefix, output):
    if isinstance(value, dict):
        for key, child in value.items():
            flatten(child, f"{prefix}.{key}" if prefix else str(key), output)
        return
    merge_leaf(output, prefix, value)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    source = yaml.safe_load(args.input.read_text())
    try:
        parameters = source["/**"]["ros__parameters"]
    except (KeyError, TypeError) as exc:
        raise SystemExit("expected ROS2 /**/ros__parameters root") from exc
    rendered = {}
    flatten(parameters, "", rendered)
    merge_leaf(rendered, "enable_all_modules_auto_mode", False)
    merge_leaf(rendered, "static_obstacle_avoidance.enable_rtc", False)
    merge_leaf(
        rendered,
        "static_obstacle_avoidance.enable_simultaneous_execution_as_approved_module",
        False,
    )
    merge_leaf(
        rendered,
        "static_obstacle_avoidance.enable_simultaneous_execution_as_candidate_module",
        False,
    )
    args.output.write_text(yaml.safe_dump(rendered, sort_keys=True))


if __name__ == "__main__":
    main()
