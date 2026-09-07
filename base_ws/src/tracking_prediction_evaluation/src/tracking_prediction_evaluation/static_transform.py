"""Configuration helpers shared by installed evaluation entry points."""

from __future__ import annotations

import yaml


def load_morai_to_map_transform(path):
    document = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    transform = document["map_to_morai_map"]
    xyz = tuple(float(value) for value in transform["xyz"])
    rpy = tuple(float(value) for value in transform["rpy"])
    if len(xyz) != 3 or len(rpy) != 3:
        raise ValueError("map_to_morai_map xyz/rpy must each contain three values")
    if abs(rpy[0]) > 1.0e-9 or abs(rpy[1]) > 1.0e-9:
        raise ValueError("evaluator supports a yaw-only MORAI static transform")
    return xyz, rpy[2]
