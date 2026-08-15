"""Causal kinematic prediction models used by the oracle evaluator."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Protocol


@dataclass(frozen=True)
class MotionState:
    x: float
    y: float
    yaw: float
    speed: float
    yaw_rate: float
    acceleration: float


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


def default_predictors() -> list[Predictor]:
    return [CTRVPredictor(), CTRAPredictor()]
