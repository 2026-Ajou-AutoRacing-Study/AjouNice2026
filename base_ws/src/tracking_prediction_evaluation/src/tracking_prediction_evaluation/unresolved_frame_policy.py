"""Conservative frame exclusion when nearby GT geometry cannot be resolved."""
from .pipeline_evaluator import active_gt, interpolate_ego


def resolved_frame_mask(raw_segments, calibrated_segments, ego_points, timestamps,
                        maximum_range_m=30.0, bracket_sec=0.15):
    supported = {s.key for s in calibrated_segments}
    unresolved = tuple(s for s in raw_segments if s.key not in supported)
    mask = {}
    for timestamp in timestamps:
        ego = interpolate_ego(ego_points, timestamp, bracket_sec)
        # Raw rear-axle/pivot may lie just outside center-based range. A 5m
        # guard is deliberately conservative; no uncertain center is invented.
        mask[timestamp] = ego is not None and not active_gt(
            unresolved, timestamp, bracket_sec, ego, maximum_range_m + 5.0)
    return mask, dict(policy='whole-frame-exclusion-near-unresolved-gt-v1',
        unresolved_segments=len(unresolved), input_frames=len(mask),
        retained_frames=sum(mask.values()), excluded_frames=sum(not v for v in mask.values()),
        raw_pivot_guard_m=5.0)
