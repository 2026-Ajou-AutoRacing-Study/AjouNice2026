#!/usr/bin/env python3
"""Normalize physically impossible or incomplete lamp combinations.

The detector currently distinguishes only left/right arrow direction.  A red
circle and a green circle cannot be simultaneously lit by the competition
signals, so that observation is treated as a missed green left-arrow shape at
every intersection rather than as a D-2-only exception.
"""


def _has(color_results, color, shape):
    return any(c == color and s == shape for c, s, _ in color_results)


def has_impossible_conflict(color_results):
    has_red_circle = _has(color_results, "red", "circle")
    has_green_circle = _has(color_results, "green", "circle")
    has_yellow_circle = _has(color_results, "yellow", "circle")
    has_green_arrow = _has(color_results, "green", "arrow")

    red_green_collision = has_red_circle and has_green_circle
    red_yellow_collision = has_red_circle and has_yellow_circle
    fully_orphan_arrow = has_green_arrow and not has_red_circle and not has_green_circle
    return red_green_collision or red_yellow_collision or fully_orphan_arrow


def resolve_conflict(color_results, gid=None):
    """Return a physically valid combination; ``gid`` is kept for API compatibility."""
    if not has_impossible_conflict(color_results):
        return color_results

    fixed = list(color_results)
    has_red_circle = _has(fixed, "red", "circle")
    has_green_circle = _has(fixed, "green", "circle")
    has_yellow_circle = _has(fixed, "yellow", "circle")
    has_green_arrow = _has(fixed, "green", "arrow")

    if has_red_circle and has_green_circle:
        fixed = [(c, s, l) for c, s, l in fixed if not (c == "green" and s == "circle")]
        if not has_green_arrow:
            fixed.append(("green", "arrow", True))
        has_green_circle = False

    if has_red_circle and has_yellow_circle:
        fixed = [(c, s, l) for c, s, l in fixed if not (c == "yellow" and s == "circle")]

    has_green_arrow = _has(fixed, "green", "arrow")
    if has_green_arrow and not has_red_circle and not has_green_circle:
        fixed.append(("green", "circle", None))

    return fixed
