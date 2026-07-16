#!/usr/bin/env python3
"""Speed-cap and acceleration/deceleration/jerk limited profile builder."""

import math
from competition_route_loader import RouteLoadError


KPH_TO_MPS = 1.0 / 3.6


def _positive(value, name):
    value = float(value)
    if value <= 0.0:
        raise RouteLoadError("speed_profile.%s must be positive" % name)
    return value


def build_profile(points, config):
    if len(points) < 2:
        raise RouteLoadError("profile needs at least two points")
    mode = str(config.get("mode", "constant"))
    if mode not in ("constant", "regulation_curvature"):
        raise RouteLoadError("unsupported speed_profile.mode %r" % mode)
    max_lat = _positive(config.get("max_lateral_accel_mps2", 1.5), "max_lateral_accel_mps2")
    max_accel = _positive(config.get("max_accel_mps2", 1.0), "max_accel_mps2")
    max_decel = _positive(config.get("max_decel_mps2", 1.5), "max_decel_mps2")
    max_jerk = _positive(config.get("max_jerk_mps3", 0.8), "max_jerk_mps3")
    vehicle_max = _positive(config.get("vehicle_max_speed_kph", 80.0), "vehicle_max_speed_kph") * KPH_TO_MPS
    constant = _positive(config.get("constant_kph", 20.0), "constant_kph")
    cruise = _positive(config.get("default_cruise_kph", 40.0), "default_cruise_kph")
    clamp = bool(config.get("clamp_to_legal", False))
    caps = []
    for point in points:
        legal_kph = float(point["lanelet"]["speed_limit_kph"])
        requested = constant if mode == "constant" else point["lanelet"].get("target_speed_kph") or cruise
        if mode == "constant" and point["lanelet"].get("target_speed_kph") is not None:
            raise RouteLoadError("constant profile cannot be combined with selection target_speed_kph")
        if requested > legal_kph + 1e-6 and mode == "constant" and not clamp:
            raise RouteLoadError("constant_kph %.1f exceeds lanelet %s legal %.1f kph" %
                                 (requested, point["lanelet"]["lanelet_id"], legal_kph))
        curve = math.sqrt(max_lat / max(abs(point["curvature"]), 1e-9))
        point["speed_limit_kph"] = legal_kph
        point["requested_speed_kph"] = requested
        caps.append(min(legal_kph * KPH_TO_MPS, requested * KPH_TO_MPS, curve, vehicle_max))
    velocity = list(caps)
    velocity[-1] = 0.0

    def project_acceleration_limits():
        """Apply the distance-domain acceleration envelope without raising a cap."""
        for i in range(len(velocity) - 2, -1, -1):
            ds = points[i + 1]["s_m"] - points[i]["s_m"]
            reachable = math.sqrt(max(0.0, velocity[i + 1] ** 2 + 2.0 * max_decel * ds))
            velocity[i] = min(velocity[i], reachable)
        for i in range(len(velocity) - 1):
            ds = points[i + 1]["s_m"] - points[i]["s_m"]
            reachable = math.sqrt(max(0.0, velocity[i] ** 2 + 2.0 * max_accel * ds))
            velocity[i + 1] = min(velocity[i + 1], reachable)

    def segment_dynamics():
        acceleration, duration = [], []
        for i in range(len(velocity) - 1):
            ds = max(points[i + 1]["s_m"] - points[i]["s_m"], 1e-6)
            acceleration.append((velocity[i + 1] ** 2 - velocity[i] ** 2) / (2.0 * ds))
            duration.append(2.0 * ds / max(velocity[i] + velocity[i + 1], 0.2))
        return acceleration, duration

    # Alternate acceleration projection with forward/backward jerk projection.
    # Every operation only lowers an existing cap, so legal/curvature caps remain
    # inviolable.  Backward projection starts deceleration early instead of creating
    # a large 0 -> negative-acceleration step at a lanelet/curve boundary.
    for _ in range(300):
        project_acceleration_limits()
        acceleration, duration = segment_dynamics()
        changed = False
        for i in range(1, len(acceleration)):
            allowed = acceleration[i - 1] + max_jerk * duration[i]
            if acceleration[i] > allowed + 1e-8:
                ds = points[i + 1]["s_m"] - points[i]["s_m"]
                reduced = math.sqrt(max(0.0, velocity[i] ** 2 + 2.0 * allowed * ds))
                if reduced < velocity[i + 1] - 1e-10:
                    velocity[i + 1] = reduced
                    changed = True
        acceleration, duration = segment_dynamics()
        for i in range(len(acceleration) - 2, -1, -1):
            allowed = acceleration[i + 1] + max_jerk * duration[i + 1]
            if acceleration[i] > allowed + 1e-8:
                ds = points[i + 1]["s_m"] - points[i]["s_m"]
                reduced = math.sqrt(max(0.0, velocity[i] ** 2 + 2.0 * allowed * ds))
                if reduced < velocity[i + 1] - 1e-10:
                    velocity[i + 1] = reduced
                    changed = True
        if not changed:
            break
    project_acceleration_limits()
    acceleration, duration = segment_dynamics()
    jerk = [abs((acceleration[i] - acceleration[i - 1]) / max(duration[i], 1e-6)) for i in range(1, len(acceleration))]
    if jerk and max(jerk) > max_jerk * 1.001:
        raise RouteLoadError("speed profile jerk projection did not converge: %.3f > %.3f m/s^3" % (max(jerk), max_jerk))
    time = 0.0
    for i, point in enumerate(points):
        point["target_speed_mps"] = velocity[i]
        if i == 0:
            point["acceleration_mps2"] = 0.0
            point["time_from_start_sec"] = 0.0
            continue
        ds = point["s_m"] - points[i - 1]["s_m"]
        denominator = max(velocity[i] + velocity[i - 1], 0.2)
        dt = 2.0 * ds / denominator
        time += dt
        point["acceleration_mps2"] = (velocity[i] - velocity[i - 1]) / max(dt, 1e-6)
        point["time_from_start_sec"] = time
    points[-1]["target_speed_mps"] = 0.0
    return points
