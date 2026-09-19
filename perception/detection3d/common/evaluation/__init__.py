"""Common evaluation protocols for repository 3D detectors."""

from .morai_detection import morai_detection_eval
from .obstacle_shape_merge import merge_obstacle_shape_schema

__all__ = ["morai_detection_eval", "merge_obstacle_shape_schema"]
