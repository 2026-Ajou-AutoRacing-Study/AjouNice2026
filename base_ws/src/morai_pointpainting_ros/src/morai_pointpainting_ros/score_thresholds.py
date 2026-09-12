"""Class-aware score threshold helpers for PointPainting output filtering."""

from __future__ import annotations

import numpy as np


def build_score_thresholds(
    labels,
    class_names,
    default_threshold,
    class_thresholds,
):
    """Return the threshold assigned to each one-based OpenPCDet label."""
    labels = np.asarray(labels)
    thresholds = np.full(labels.shape, float(default_threshold), dtype=np.float32)
    normalized = {
        str(name).strip().lower(): float(value)
        for name, value in class_thresholds.items()
    }
    for class_label, class_name in enumerate(class_names, start=1):
        threshold = normalized.get(str(class_name).strip().lower())
        if threshold is not None:
            thresholds[labels == class_label] = threshold
    return thresholds
