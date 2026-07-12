#!/usr/bin/env python3

from copy import deepcopy
from dataclasses import dataclass

import rospy

def _param(prefix, name, default):
    return rospy.get_param("{}/{}".format(prefix.rstrip("/"), name.lstrip("/")), default)


def _bool(value):
    if isinstance(value, str):
        return value.lower() in ("1", "true", "yes", "on")
    return bool(value)


@dataclass
class GoalPlannerParameters:
    parking_enabled: bool = False
    allow_goal_modification: bool = True

    @classmethod
    def from_ros(cls, prefix="~goal_planner"):
        p = cls()
        p.parking_enabled = _bool(_param(prefix, "parking_enabled", p.parking_enabled))
        p.allow_goal_modification = _bool(
            _param(prefix, "allow_goal_modification", p.allow_goal_modification)
        )
        return p


@dataclass
class GoalPlannerResult:
    reference_path: object
    output_path: object
    modified_goal_pose: object = None
    goal_station: float = 0.0
    goal_projection_error: float = 0.0
    status: str = "goal_planner: inactive, parking mission excluded"


class GoalPlannerEngine:
    """Autoware-compatible dormant Goal Planner for a non-parking competition."""

    def __init__(self, parameters=None):
        self.parameters = parameters or GoalPlannerParameters()

    def plan(self, path, route):
        reference = deepcopy(path)
        output = deepcopy(path)
        if self.parameters.parking_enabled:
            raise RuntimeError(
                "parking_enabled is outside the ROS1 competition port scope"
            )
        return GoalPlannerResult(reference, output)


class GoalPlannerScene:
    def __init__(self, prefix="~goal_planner"):
        self.engine = GoalPlannerEngine(GoalPlannerParameters.from_ros(prefix))

    def process(self, path, route):
        return self.engine.plan(path, route)
