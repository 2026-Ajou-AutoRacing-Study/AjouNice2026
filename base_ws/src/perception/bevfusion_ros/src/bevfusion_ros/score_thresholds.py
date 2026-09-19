"""Class-aware confidence filtering for the BEVFusion ROS bridge."""


KNOWN_CLASS_NAMES = ("vehicle", "pedestrian", "obstacle")


def normalize_class_name(class_name):
    """Return the detector class spelling used by the threshold contract."""
    return str(class_name or "unknown").strip().lower()


def build_score_thresholds(fallback, vehicle, pedestrian, obstacle):
    """Build a normalized class threshold table."""
    thresholds = {
        "vehicle": float(vehicle),
        "pedestrian": float(pedestrian),
        "obstacle": float(obstacle),
    }
    fallback = float(fallback)
    for name, value in [("fallback", fallback)] + list(thresholds.items()):
        if not 0.0 <= value <= 1.0:
            raise ValueError("{} score threshold must be in [0, 1]".format(name))
    return fallback, thresholds


def filter_detections(detections, fallback, thresholds):
    """Apply the per-class production operating point to server detections."""
    kept = []
    for detection in detections:
        class_name = normalize_class_name(detection.get("class_name"))
        threshold = thresholds.get(class_name, fallback)
        if float(detection.get("score", 0.0)) >= threshold:
            kept.append(detection)
    return kept


def server_score_floor(fallback, thresholds):
    """Lowest score the server may safely discard before host filtering."""
    return min([float(fallback)] + [float(value) for value in thresholds.values()])
