"""Causal kinematic prediction models used by the oracle evaluator."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Protocol, Sequence


@dataclass(frozen=True)
class MotionState:
    x: float
    y: float
    yaw: float
    speed: float
    yaw_rate: float
    acceleration: float
    z: float = 0.0


class Predictor(Protocol):
    name: str

    def predict(self, state: MotionState, horizon: float) -> tuple[float, float, float]:
        ...


class CTRVPredictor:
    name = "ctrv"

    def __init__(self, omega_epsilon: float = 1.0e-3):
        self.omega_epsilon = omega_epsilon

    def predict(self, state: MotionState, horizon: float) -> tuple[float, float, float]:
        if abs(state.yaw_rate) < self.omega_epsilon:
            distance = state.speed * horizon
            return (
                state.x + distance * math.cos(state.yaw),
                state.y + distance * math.sin(state.yaw),
                state.yaw,
            )

        next_yaw = state.yaw + state.yaw_rate * horizon
        return (
            state.x
            + state.speed
            / state.yaw_rate
            * (math.sin(next_yaw) - math.sin(state.yaw)),
            state.y
            + state.speed
            / state.yaw_rate
            * (math.cos(state.yaw) - math.cos(next_yaw)),
            next_yaw,
        )

    def predict_path(
        self,
        state: MotionState,
        horizons: Sequence[float],
        class_name: str = "vehicle",
    ) -> tuple[tuple[float, float, float], ...]:
        del class_name
        return tuple(self.predict(state, horizon) for horizon in horizons)


class CTRAPredictor:
    name = "ctra"

    def __init__(
        self,
        omega_epsilon: float = 1.0e-3,
        acceleration_limit: float | None = 6.0,
    ):
        self.omega_epsilon = omega_epsilon
        self.acceleration_limit = acceleration_limit

    def predict(self, state: MotionState, horizon: float) -> tuple[float, float, float]:
        acceleration = state.acceleration
        if self.acceleration_limit is not None:
            acceleration = max(
                -self.acceleration_limit,
                min(self.acceleration_limit, acceleration),
            )

        if abs(state.yaw_rate) < self.omega_epsilon:
            distance = state.speed * horizon + 0.5 * acceleration * horizon**2
            return (
                state.x + distance * math.cos(state.yaw),
                state.y + distance * math.sin(state.yaw),
                state.yaw,
            )

        omega = state.yaw_rate
        next_yaw = state.yaw + omega * horizon
        x_constant_velocity = state.speed / omega * (
            math.sin(next_yaw) - math.sin(state.yaw)
        )
        y_constant_velocity = state.speed / omega * (
            math.cos(state.yaw) - math.cos(next_yaw)
        )
        x_acceleration = acceleration * (
            horizon * math.sin(next_yaw) / omega
            + (math.cos(next_yaw) - math.cos(state.yaw)) / omega**2
        )
        y_acceleration = acceleration * (
            -horizon * math.cos(next_yaw) / omega
            + (math.sin(next_yaw) - math.sin(state.yaw)) / omega**2
        )
        return (
            state.x + x_constant_velocity + x_acceleration,
            state.y + y_constant_velocity + y_acceleration,
            next_yaw,
        )

    def predict_path(
        self,
        state: MotionState,
        horizons: Sequence[float],
        class_name: str = "vehicle",
    ) -> tuple[tuple[float, float, float], ...]:
        del class_name
        return tuple(self.predict(state, horizon) for horizon in horizons)


class MapBasedP1Predictor:
    """Adapter from the ROS-independent Lanelet network to the E0 evaluator.

    The highest-confidence route is evaluated as the planning-facing top-1
    prediction. Non-vehicle or unmatched samples use the same CTRV fallback as
    the runtime ROS node.
    """

    name = "map_p1"

    def __init__(self, network, fallback: Predictor | None = None):
        self.network = network
        self.fallback = fallback or CTRVPredictor()
        self.mapped_origin_count = 0
        self.fallback_origin_count = 0

    def predict_path(
        self,
        state: MotionState,
        horizons: Sequence[float],
        class_name: str = "vehicle",
    ) -> tuple[tuple[float, float, float], ...]:
        trajectories = ()
        if class_name == "vehicle":
            trajectories = self.network.predict(
                x=state.x,
                y=state.y,
                z=state.z,
                yaw=state.yaw,
                speed=state.speed,
                horizons=horizons,
            )
        if trajectories:
            self.mapped_origin_count += 1
            top_path = max(trajectories, key=lambda item: item.confidence)
            return tuple((x, y, yaw) for x, y, _z, yaw in top_path.poses)

        self.fallback_origin_count += 1
        predict_path = getattr(self.fallback, "predict_path", None)
        if predict_path is not None:
            return tuple(predict_path(state, horizons, class_name))
        return tuple(self.fallback.predict(state, horizon) for horizon in horizons)

    def predict(self, state: MotionState, horizon: float) -> tuple[float, float, float]:
        return self.predict_path(state, (horizon,), "vehicle")[0]


def default_predictors() -> list[Predictor]:
    return [CTRVPredictor(), CTRAPredictor()]
