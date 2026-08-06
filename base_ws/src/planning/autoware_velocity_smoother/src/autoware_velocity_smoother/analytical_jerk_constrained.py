"""Analytical jerk-constrained speed-profile utilities.

The backward stop envelope follows the same analytical S-curve idea used by
Autoware Core's ``autoware_velocity_smoother`` (Apache-2.0). This ROS 1 port
keeps trajectory geometry and updates longitudinal velocity and acceleration.
"""

from dataclasses import dataclass
import math


EPSILON = 1.0e-6


@dataclass(frozen=True)
class SmootherParameters:
    """Limits used while generating a speed profile."""

    max_velocity_mps: float = 11.1
    max_acceleration_mps2: float = 1.0
    max_deceleration_mps2: float = 2.5
    max_jerk_mps3: float = 1.0
    max_deceleration_jerk_mps3: float = 1.5
    min_velocity_for_time_calculation_mps: float = 1.0
    stop_velocity_threshold_mps: float = 0.01


@dataclass(frozen=True)
class VelocityProfile:
    """Smoothed speed and acceleration values aligned with trajectory points."""

    velocities: tuple
    accelerations: tuple
    deceleration_target_indices: tuple
    velocity_cap_clamp_count: int


@dataclass(frozen=True)
class InitialState:
    """Initial velocity-planning state and why it was selected."""

    velocity_mps: float
    acceleration_mps2: float
    source: str


@dataclass(frozen=True)
class _DecelerationCurve:
    """A zero-initial/final-acceleration jerk-constrained deceleration."""

    initial_velocity: float
    target_velocity: float
    jerk: float
    peak_deceleration: float
    ramp_duration: float
    hold_duration: float
    total_distance: float

    @property
    def total_duration(self):
        return 2.0 * self.ramp_duration + self.hold_duration

    def state_at_time(self, time_sec):
        """Return ``(distance, velocity, acceleration)`` at profile time."""
        time_sec = clamp(float(time_sec), 0.0, self.total_duration)
        if self.ramp_duration <= EPSILON:
            return 0.0, self.target_velocity, 0.0

        if time_sec <= self.ramp_duration:
            return _integrate_constant_jerk(
                self.initial_velocity,
                0.0,
                -self.jerk,
                time_sec,
            )

        first_distance, velocity, acceleration = _integrate_constant_jerk(
            self.initial_velocity,
            0.0,
            -self.jerk,
            self.ramp_duration,
        )
        after_first = time_sec - self.ramp_duration
        if after_first <= self.hold_duration:
            distance, velocity, acceleration = _integrate_constant_jerk(
                velocity,
                acceleration,
                0.0,
                after_first,
            )
            return first_distance + distance, velocity, acceleration

        hold_distance, velocity, acceleration = _integrate_constant_jerk(
            velocity,
            acceleration,
            0.0,
            self.hold_duration,
        )
        recovery_time = after_first - self.hold_duration
        recovery_distance, velocity, acceleration = _integrate_constant_jerk(
            velocity,
            acceleration,
            self.jerk,
            recovery_time,
        )
        return first_distance + hold_distance + recovery_distance, velocity, acceleration

    def state_at_distance(self, distance_m):
        """Return the S-curve state at a travelled distance using bisection."""
        distance_m = clamp(float(distance_m), 0.0, self.total_distance)
        if distance_m <= EPSILON:
            return 0.0, self.initial_velocity, 0.0
        if distance_m >= self.total_distance - EPSILON:
            return self.total_distance, self.target_velocity, 0.0

        lower = 0.0
        upper = self.total_duration
        for _ in range(40):
            middle = 0.5 * (lower + upper)
            state = self.state_at_time(middle)
            if state[0] <= distance_m:
                lower = middle
            else:
                upper = middle
        return self.state_at_time(0.5 * (lower + upper))


def clamp(value, low, high):
    return max(low, min(high, value))


def cumulative_lengths(positions):
    """Return trajectory arc lengths for a sequence of ``(x, y, z)`` tuples."""
    if not positions:
        return []

    lengths = [0.0]
    for first, second in zip(positions[:-1], positions[1:]):
        dx = float(second[0]) - float(first[0])
        dy = float(second[1]) - float(first[1])
        dz = float(second[2]) - float(first[2])
        lengths.append(lengths[-1] + math.sqrt(dx * dx + dy * dy + dz * dz))
    return lengths


def nearest_index_2d(positions, target_position):
    """Return ``(index, distance)`` for the closest 2D point."""
    if not positions:
        return -1, float("inf")

    target_x = float(target_position[0])
    target_y = float(target_position[1])
    best_index = -1
    best_distance_squared = float("inf")
    for index, position in enumerate(positions):
        dx = float(position[0]) - target_x
        dy = float(position[1]) - target_y
        distance_squared = dx * dx + dy * dy
        if distance_squared < best_distance_squared:
            best_index = index
            best_distance_squared = distance_squared
    return best_index, math.sqrt(best_distance_squared)


def select_initial_state(
    current_velocity_mps,
    previous_velocity_mps=None,
    previous_acceleration_mps2=None,
    replan_velocity_deviation_mps=5.53,
    default_velocity_mps=0.0,
):
    """Select the Autoware-style initial state for velocity planning."""
    current_velocity = None
    if current_velocity_mps is not None:
        current_velocity = float(current_velocity_mps)
        if not math.isfinite(current_velocity):
            current_velocity = None

    previous_velocity = None
    if previous_velocity_mps is not None:
        previous_velocity = float(previous_velocity_mps)
        if not math.isfinite(previous_velocity):
            previous_velocity = None

    previous_acceleration = 0.0
    if previous_acceleration_mps2 is not None:
        previous_acceleration = float(previous_acceleration_mps2)
        if not math.isfinite(previous_acceleration):
            previous_acceleration = 0.0

    if previous_velocity is None:
        velocity = current_velocity
        source = "current"
        if velocity is None:
            velocity = float(default_velocity_mps)
            source = "default"
        return InitialState(max(velocity, 0.0), 0.0, source)

    if (
        current_velocity is not None
        and abs(current_velocity - previous_velocity)
        > max(float(replan_velocity_deviation_mps), 0.0)
    ):
        return InitialState(
            max(current_velocity, 0.0),
            previous_acceleration,
            "current_replan",
        )

    return InitialState(
        max(previous_velocity, 0.0),
        previous_acceleration,
        "previous_plan",
    )


def _curvature_from_triplet(first, middle, last):
    ax = float(middle[0]) - float(first[0])
    ay = float(middle[1]) - float(first[1])
    bx = float(last[0]) - float(first[0])
    by = float(last[1]) - float(first[1])
    side_ab = math.hypot(ax, ay)
    side_bc = math.hypot(
        float(last[0]) - float(middle[0]),
        float(last[1]) - float(middle[1]),
    )
    side_ac = math.hypot(bx, by)
    denominator = side_ab * side_bc * side_ac
    if denominator <= EPSILON:
        return 0.0
    twice_area = abs(ax * by - ay * bx)
    return 2.0 * twice_area / denominator


def apply_lateral_acceleration_limits(
    positions,
    velocity_caps,
    max_lateral_accel_mps2,
    min_curve_velocity_mps,
    decel_distance_before_curve_m,
    decel_distance_after_curve_m,
    curvature_window=1,
    min_curve_radius_m=1.0,
    stop_velocity_threshold_mps=0.01,
    max_deceleration_mps2=None,
    max_deceleration_jerk_mps3=None,
    initial_velocity_mps=None,
):
    """Reduce speed caps on curves using ``v <= sqrt(a_lat / curvature)``."""
    caps = []
    for value in velocity_caps:
        value = float(value)
        caps.append(max(value, 0.0) if math.isfinite(value) else 0.0)
    if len(caps) != len(positions):
        raise ValueError("positions and velocity_caps must have the same length")
    if len(caps) < 3:
        return tuple(caps), 0

    max_lateral_accel = max(float(max_lateral_accel_mps2), 0.0)
    if max_lateral_accel <= EPSILON:
        return tuple(caps), 0

    window = max(int(curvature_window), 1)
    if len(caps) <= 2 * window:
        return tuple(caps), 0

    min_curve_velocity = max(float(min_curve_velocity_mps), 0.0)
    decel_distance_before = max(float(decel_distance_before_curve_m), 0.0)
    decel_distance_after = max(float(decel_distance_after_curve_m), 0.0)
    stop_threshold = max(float(stop_velocity_threshold_mps), 0.0)
    max_curvature = 1.0 / max(float(min_curve_radius_m), EPSILON)
    has_deceleration_feasibility = (
        max_deceleration_mps2 is not None and max_deceleration_jerk_mps3 is not None
    )
    initial_velocity = None
    if initial_velocity_mps is not None:
        initial_velocity = float(initial_velocity_mps)
        if not math.isfinite(initial_velocity):
            initial_velocity = None

    arc_lengths = cumulative_lengths(positions)
    capped = list(caps)
    for index in range(window, len(positions) - window):
        curvature = _curvature_from_triplet(
            positions[index - window],
            positions[index],
            positions[index + window],
        )
        if curvature <= EPSILON:
            continue
        curvature = min(curvature, max_curvature)
        curve_velocity = max(
            math.sqrt(max_lateral_accel / curvature),
            min_curve_velocity,
        )
        start_station = arc_lengths[index] - decel_distance_before
        end_station = arc_lengths[index] + decel_distance_after
        if has_deceleration_feasibility:
            upstream_index = next(
                (
                    cap_index
                    for cap_index, station in enumerate(arc_lengths)
                    if station >= start_station - EPSILON
                ),
                0,
            )
            reference_velocity = caps[upstream_index]
            if upstream_index == 0 and initial_velocity is not None:
                reference_velocity = min(max(initial_velocity, 0.0), reference_velocity)
            available_distance = max(
                arc_lengths[index] - arc_lengths[upstream_index],
                0.0,
            )
            required_distance = minimum_deceleration_distance(
                reference_velocity,
                curve_velocity,
                max_deceleration_mps2,
                max_deceleration_jerk_mps3,
            )
            if required_distance > available_distance + EPSILON:
                continue
        for cap_index, station in enumerate(arc_lengths):
            if station < start_station - EPSILON or station > end_station + EPSILON:
                continue
            if caps[cap_index] <= stop_threshold:
                continue
            capped[cap_index] = min(capped[cap_index], curve_velocity)

    clamp_count = sum(
        1 for before, after in zip(caps, capped) if after + EPSILON < before
    )
    return tuple(capped), clamp_count


def _integrate_constant_jerk(velocity, acceleration, jerk, duration):
    distance = (
        velocity * duration
        + 0.5 * acceleration * duration * duration
        + jerk * duration * duration * duration / 6.0
    )
    next_velocity = velocity + acceleration * duration + 0.5 * jerk * duration * duration
    next_acceleration = acceleration + jerk * duration
    return distance, next_velocity, next_acceleration


def _make_deceleration_curve(
    initial_velocity_mps,
    target_velocity_mps,
    max_deceleration_mps2,
    max_jerk_mps3,
):
    """Build a zero-initial/final-acceleration S-curve deceleration."""
    initial_velocity = max(float(initial_velocity_mps), 0.0)
    target_velocity = max(float(target_velocity_mps), 0.0)
    velocity_delta = initial_velocity - target_velocity
    if velocity_delta <= EPSILON:
        return _DecelerationCurve(
            initial_velocity,
            target_velocity,
            max(abs(float(max_jerk_mps3)), EPSILON),
            0.0,
            0.0,
            0.0,
            0.0,
        )

    maximum_deceleration = max(abs(float(max_deceleration_mps2)), EPSILON)
    maximum_jerk = max(abs(float(max_jerk_mps3)), EPSILON)
    peak_deceleration = min(
        maximum_deceleration,
        math.sqrt(velocity_delta * maximum_jerk),
    )
    ramp_duration = peak_deceleration / maximum_jerk
    ramp_velocity_delta = peak_deceleration * peak_deceleration / maximum_jerk
    hold_duration = max(
        (velocity_delta - ramp_velocity_delta) / peak_deceleration,
        0.0,
    )

    distance, velocity, acceleration = _integrate_constant_jerk(
        initial_velocity,
        0.0,
        -maximum_jerk,
        ramp_duration,
    )
    hold_distance, velocity, acceleration = _integrate_constant_jerk(
        velocity,
        acceleration,
        0.0,
        hold_duration,
    )
    recovery_distance, _velocity, _acceleration = _integrate_constant_jerk(
        velocity,
        acceleration,
        maximum_jerk,
        ramp_duration,
    )
    return _DecelerationCurve(
        initial_velocity,
        target_velocity,
        maximum_jerk,
        peak_deceleration,
        ramp_duration,
        hold_duration,
        max(distance + hold_distance + recovery_distance, 0.0),
    )


def minimum_deceleration_distance(
    initial_velocity_mps,
    target_velocity_mps,
    max_deceleration_mps2,
    max_jerk_mps3,
):
    """Distance needed for a zero-acceleration S-curve deceleration."""
    return _make_deceleration_curve(
        initial_velocity_mps,
        target_velocity_mps,
        max_deceleration_mps2,
        max_jerk_mps3,
    ).total_distance


def _make_deceleration_curve_for_distance(
    initial_velocity_mps,
    target_velocity_mps,
    available_distance_m,
    max_deceleration_mps2,
    max_jerk_mps3,
):
    """Build the slowest triangular S-curve that ends at the target station."""
    base_curve = _make_deceleration_curve(
        initial_velocity_mps,
        target_velocity_mps,
        max_deceleration_mps2,
        max_jerk_mps3,
    )
    available_distance = max(float(available_distance_m), 0.0)
    velocity_delta = max(
        float(initial_velocity_mps) - float(target_velocity_mps),
        0.0,
    )
    if (
        velocity_delta <= EPSILON
        or available_distance <= base_curve.total_distance + EPSILON
    ):
        return base_curve

    fitted_jerk = velocity_delta * (
        float(initial_velocity_mps) + float(target_velocity_mps)
    ) ** 2 / (available_distance * available_distance)
    fitted_jerk = min(fitted_jerk, abs(float(max_jerk_mps3)))
    return _make_deceleration_curve(
        initial_velocity_mps,
        target_velocity_mps,
        max_deceleration_mps2,
        max(fitted_jerk, EPSILON),
    )


def maximum_velocity_for_distance(
    available_distance_m,
    target_velocity_mps,
    upper_velocity_mps,
    max_deceleration_mps2,
    max_jerk_mps3,
):
    """Find the greatest speed that can reach a target speed in the distance."""
    target_velocity = max(float(target_velocity_mps), 0.0)
    upper_velocity = max(float(upper_velocity_mps), target_velocity)
    available_distance = max(float(available_distance_m), 0.0)
    if available_distance <= EPSILON:
        return target_velocity
    if minimum_deceleration_distance(
        upper_velocity,
        target_velocity,
        max_deceleration_mps2,
        max_jerk_mps3,
    ) <= available_distance:
        return upper_velocity

    lower = target_velocity
    for _ in range(40):
        middle = 0.5 * (lower + upper_velocity)
        required_distance = minimum_deceleration_distance(
            middle,
            target_velocity,
            max_deceleration_mps2,
            max_jerk_mps3,
        )
        if required_distance <= available_distance:
            lower = middle
        else:
            upper_velocity = middle
    return lower


def _sanitize_velocity_caps(velocity_caps, parameters):
    max_velocity = max(float(parameters.max_velocity_mps), 0.0)
    values = []
    for value in velocity_caps:
        value = float(value)
        if not math.isfinite(value):
            value = 0.0
        values.append(clamp(value, 0.0, max_velocity))
    return values


def _deceleration_target_indices(velocity_caps):
    return tuple(
        index
        for index in range(1, len(velocity_caps))
        if velocity_caps[index] + EPSILON < velocity_caps[index - 1]
    )


def apply_backward_deceleration_limits(velocity_caps, arc_lengths, parameters):
    """Sample a physical S-curve before every lower speed cap."""
    filtered = list(velocity_caps)
    target_indices = _deceleration_target_indices(velocity_caps)
    for target_index in reversed(target_indices):
        target_velocity = filtered[target_index]
        start_index = None
        curve = None

        for index in range(target_index - 1, -1, -1):
            initial_velocity = filtered[index]
            if initial_velocity <= target_velocity + EPSILON:
                continue
            candidate_curve = _make_deceleration_curve(
                initial_velocity,
                target_velocity,
                parameters.max_deceleration_mps2,
                parameters.max_deceleration_jerk_mps3,
            )
            available_distance = arc_lengths[target_index] - arc_lengths[index]
            if candidate_curve.total_distance <= available_distance + EPSILON:
                start_index = index
                curve = candidate_curve
                break

        if start_index is None:
            # Preserve an infeasibly close stop cap and brake from the first point.
            start_index = 0
            curve = _make_deceleration_curve(
                filtered[start_index],
                target_velocity,
                parameters.max_deceleration_mps2,
                parameters.max_deceleration_jerk_mps3,
            )

        available_distance = arc_lengths[target_index] - arc_lengths[start_index]
        if curve.total_distance <= available_distance + EPSILON:
            curve = _make_deceleration_curve_for_distance(
                filtered[start_index],
                target_velocity,
                available_distance,
                parameters.max_deceleration_mps2,
                parameters.max_deceleration_jerk_mps3,
            )

        start_station = arc_lengths[start_index]
        for index in range(start_index + 1, target_index + 1):
            travelled_distance = arc_lengths[index] - start_station
            if travelled_distance >= curve.total_distance - EPSILON:
                planned_velocity = target_velocity
            else:
                _distance, planned_velocity, _acceleration = curve.state_at_distance(
                    travelled_distance
                )
            filtered[index] = min(filtered[index], planned_velocity)
    return filtered, target_indices


def _segment_time(distance_m, velocity_mps, parameters):
    reference_velocity = max(
        abs(float(velocity_mps)),
        float(parameters.min_velocity_for_time_calculation_mps),
        EPSILON,
    )
    return max(float(distance_m), 0.0) / reference_velocity


def accelerations_from_velocity_profile(velocities, arc_lengths):
    """Calculate spatial acceleration values aligned with a speed profile."""
    accelerations = [0.0] * len(velocities)
    for index in range(len(velocities) - 1):
        distance = arc_lengths[index + 1] - arc_lengths[index]
        if distance <= EPSILON:
            continue
        accelerations[index] = (
            velocities[index + 1] * velocities[index + 1]
            - velocities[index] * velocities[index]
        ) / (2.0 * distance)
    if len(accelerations) > 1:
        accelerations[-1] = accelerations[-2]
    return accelerations


def apply_engage_velocity_floor(
    positions,
    velocities,
    velocity_caps,
    current_velocity_mps,
    engage_velocity_mps,
    engage_distance_m,
    stop_distance_to_prohibit_engage_m,
    stop_velocity_threshold_mps,
):
    """Keep a stopped vehicle's near trajectory above controller hold speed.

    The controller uses the lowest speed in a 5 m preview window. The engage
    floor is applied across that initial window, but a planned low-speed or
    zero-speed cap is never overridden.
    """
    output = list(velocities)
    if (
        not output
        or len(output) != len(positions)
        or len(output) != len(velocity_caps)
    ):
        return tuple(output), False

    engage_velocity = max(float(engage_velocity_mps), 0.0)
    current_velocity = (
        float(current_velocity_mps)
        if current_velocity_mps is not None
        and math.isfinite(float(current_velocity_mps))
        else None
    )
    if (
        engage_velocity <= EPSILON
        or current_velocity is None
        or current_velocity > engage_velocity + EPSILON
    ):
        return tuple(output), False

    stop_threshold = max(float(stop_velocity_threshold_mps), 0.0)
    arc_lengths = cumulative_lengths(positions)
    first_stop_distance = next(
        (
            distance
            for distance, cap in zip(arc_lengths, velocity_caps)
            if math.isfinite(float(cap)) and float(cap) <= stop_threshold
        ),
        None,
    )
    if (
        first_stop_distance is not None
        and first_stop_distance
        <= max(float(stop_distance_to_prohibit_engage_m), 0.0)
    ):
        return tuple(output), False

    engaged = False
    for index, distance in enumerate(arc_lengths):
        if distance > max(float(engage_distance_m), 0.0) + EPSILON:
            break
        cap = float(velocity_caps[index])
        if not math.isfinite(cap) or cap <= engage_velocity + EPSILON:
            break
        output[index] = max(output[index], engage_velocity)
        engaged = True
    return tuple(output), engaged


def smooth_velocity_profile(
    positions,
    velocity_caps,
    initial_velocity_mps=None,
    initial_acceleration_mps2=0.0,
    parameters=None,
):
    """Create a jerk-constrained velocity profile under supplied speed caps."""
    if parameters is None:
        parameters = SmootherParameters()

    caps = _sanitize_velocity_caps(velocity_caps, parameters)
    if len(caps) != len(positions):
        raise ValueError("positions and velocity_caps must have the same length")
    if not caps:
        return VelocityProfile((), (), (), 0)
    if len(caps) == 1:
        return VelocityProfile((caps[0],), (0.0,), (), 0)

    arc_lengths = cumulative_lengths(positions)
    backward_caps, target_indices = apply_backward_deceleration_limits(
        caps,
        arc_lengths,
        parameters,
    )

    if initial_velocity_mps is None or not math.isfinite(float(initial_velocity_mps)):
        initial_velocity = backward_caps[0]
    else:
        initial_velocity = clamp(
            float(initial_velocity_mps),
            0.0,
            backward_caps[0],
        )

    max_acceleration = max(float(parameters.max_acceleration_mps2), EPSILON)
    max_deceleration = max(float(parameters.max_deceleration_mps2), EPSILON)
    max_jerk = max(float(parameters.max_jerk_mps3), EPSILON)
    max_deceleration_jerk = max(
        float(parameters.max_deceleration_jerk_mps3),
        EPSILON,
    )
    velocity = [0.0] * len(caps)
    velocity[0] = initial_velocity
    acceleration = clamp(
        float(initial_acceleration_mps2)
        if math.isfinite(float(initial_acceleration_mps2))
        else 0.0,
        -max_deceleration,
        max_acceleration,
    )
    velocity_cap_clamp_count = 0

    for index in range(len(caps) - 1):
        distance = arc_lengths[index + 1] - arc_lengths[index]
        cap = backward_caps[index + 1]
        if distance <= EPSILON:
            velocity[index + 1] = min(velocity[index], cap)
            continue

        duration = _segment_time(distance, velocity[index], parameters)
        desired_acceleration = (
            cap * cap - velocity[index] * velocity[index]
        ) / (2.0 * distance)
        desired_acceleration = clamp(
            desired_acceleration,
            -max_deceleration,
            max_acceleration,
        )
        desired_jerk = (desired_acceleration - acceleration) / max(duration, EPSILON)
        jerk = clamp(desired_jerk, -max_deceleration_jerk, max_jerk)
        next_acceleration = clamp(
            acceleration + jerk * duration,
            -max_deceleration,
            max_acceleration,
        )
        predicted_velocity = max(
            velocity[index]
            + acceleration * duration
            + 0.5 * jerk * duration * duration,
            0.0,
        )

        if predicted_velocity > cap + EPSILON:
            velocity[index + 1] = cap
            velocity_cap_clamp_count += 1
            acceleration = (
                cap * cap - velocity[index] * velocity[index]
            ) / (2.0 * distance)
        else:
            velocity[index + 1] = predicted_velocity
            acceleration = next_acceleration

    stop_threshold = max(float(parameters.stop_velocity_threshold_mps), 0.0)
    for index, cap in enumerate(caps):
        if cap <= stop_threshold:
            velocity[index] = 0.0

    accelerations = accelerations_from_velocity_profile(velocity, arc_lengths)
    return VelocityProfile(
        tuple(velocity),
        tuple(accelerations),
        target_indices,
        velocity_cap_clamp_count,
    )
