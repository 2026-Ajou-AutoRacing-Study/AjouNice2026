"""Shared geometric obstacle-shape taxonomy used for diagnostics."""


SHAPE_CLASSES = (
    "compact_obstacle",
    "tall_obstacle",
    "long_obstacle",
)


def classify_obstacle_shape(size, ratio_threshold=2.0):
    """Return an exclusive shape label for a (length, width, height) box."""
    length, width, height = (float(value) for value in size)
    horizontal_max = max(length, width)
    horizontal_min = max(min(length, width), 1e-8)
    if height > ratio_threshold * horizontal_max:
        return "tall_obstacle"
    if (horizontal_max > ratio_threshold * horizontal_min or
            horizontal_max > ratio_threshold * max(height, 1e-8)):
        return "long_obstacle"
    return "compact_obstacle"
