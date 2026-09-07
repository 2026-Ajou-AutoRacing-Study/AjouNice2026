"""Shared object-pipeline helpers for ROS1 detector, tracker and predictor nodes."""

from .detected_objects import make_detected_object, make_detected_objects

__all__ = ["make_detected_object", "make_detected_objects"]
