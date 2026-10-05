"""Opt-in planar velocity reference; no production evaluator default changes."""

import math


def pedestrian_message_velocity(point, unit, frame, frame_yaw=0.0):
    """Decode an explicitly audited message contract; never infer units."""
    if unit not in ('mps', 'kmh') or frame not in ('body', 'morai-map'):
        raise ValueError('Pedestrian message velocity requires explicit unit/frame')
    factor = 1.0 if unit == 'mps' else 1.0 / 3.6
    yaw = point.yaw if frame == 'body' else frame_yaw
    return axle_to_bbox_velocity(point.longitudinal_velocity * factor,
                                 point.lateral_velocity * factor, yaw, 0., 0., 0.)


def pedestrian_message_available(segment):
    return sum(math.isfinite(p.longitudinal_velocity) and math.isfinite(p.lateral_velocity)
               and math.hypot(p.longitudinal_velocity, p.lateral_velocity) > 1e-6
               for p in segment.points) >= 3


def pedestrian_velocity_source(segment, history, position_velocity, message_available=None):
    """Segment-level availability, with sustained moving-zero fallback.

    A single zero sample cannot switch the reference. Stationary zeros remain
    valid when the segment has demonstrated message availability.
    """
    finite = lambda p: (math.isfinite(p.longitudinal_velocity)
                        and math.isfinite(p.lateral_velocity))
    nonzero = lambda p: math.hypot(p.longitudinal_velocity, p.lateral_velocity) > 1e-6
    if message_available is None:
        message_available = pedestrian_message_available(segment)
    if not message_available:
        return 'position_fallback_message_unavailable'
    if not history or not finite(history[-1]):
        return 'position_fallback_nonfinite'
    if (history[-1].timestamp - history[0].timestamp >= 0.1
            and all(finite(p) and not nonzero(p) for p in history)
            and math.hypot(*position_velocity) > 0.2):
        return 'position_fallback_moving_zero'
    return 'message'


def axle_to_bbox_velocity(vx, vy, yaw, yaw_rate, offset_x, offset_y):
    """Return map XY velocity, assuming message body velocity is at the axle.

    Offsets are axle-to-geometric-center in the actor body frame. Yaw rate
    is rad/s, velocities m/s, offsets metres. A velocity already expressed
    at the BBox center must not receive this correction again.
    """
    center_body_x = vx - yaw_rate * offset_y
    center_body_y = vy + yaw_rate * offset_x
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * center_body_x - s * center_body_y,
            s * center_body_x + c * center_body_y)
