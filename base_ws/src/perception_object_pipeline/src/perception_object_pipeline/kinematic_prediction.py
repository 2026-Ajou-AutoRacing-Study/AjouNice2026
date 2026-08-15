"""Shared kinematic prediction helpers for ROS1 object predictors."""

from __future__ import annotations

import math

from autoware_perception_msgs.msg import ObjectClassification
from geometry_msgs.msg import Pose, Quaternion


OMEGA_EPS = 1.0e-3
VEHICLE_LABELS = {
    ObjectClassification.CAR,
    ObjectClassification.TRUCK,
    ObjectClassification.BUS,
    ObjectClassification.TRAILER,
}


def quaternion_to_yaw(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def yaw_to_quaternion(yaw: float) -> Quaternion:
    half = 0.5 * yaw
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


def ctrv_predict(
    x: float,
    y: float,
    yaw: float,
    speed: float,
    yaw_rate: float,
    dt: float,
):
    if abs(yaw_rate) < OMEGA_EPS:
        return (
            x + speed * dt * math.cos(yaw),
            y + speed * dt * math.sin(yaw),
            yaw,
        )
    next_yaw = yaw + yaw_rate * dt
    return (
        x + speed / yaw_rate * (math.sin(next_yaw) - math.sin(yaw)),
        y + speed / yaw_rate * (math.cos(yaw) - math.cos(next_yaw)),
        next_yaw,
    )


def dominant_label(tracked) -> int:
    if not tracked.classification:
        return ObjectClassification.UNKNOWN
    return max(tracked.classification, key=lambda item: item.probability).label


def predict_pose(tracked, horizon: float) -> Pose:
    current = tracked.kinematics.pose_with_covariance.pose
    twist = tracked.kinematics.twist_with_covariance.twist
    yaw = quaternion_to_yaw(current.orientation)
    label = dominant_label(tracked)

    if label in VEHICLE_LABELS:
        x, y, predicted_yaw = ctrv_predict(
            current.position.x,
            current.position.y,
            yaw,
            twist.linear.x,
            twist.angular.z,
            horizon,
        )
    else:
        # Autoware TrackedObject twist is expressed in the object body frame.
        velocity_x = math.cos(yaw) * twist.linear.x - math.sin(yaw) * twist.linear.y
        velocity_y = math.sin(yaw) * twist.linear.x + math.cos(yaw) * twist.linear.y
        x = current.position.x + velocity_x * horizon
        y = current.position.y + velocity_y * horizon
        predicted_yaw = yaw
        if math.hypot(velocity_x, velocity_y) > 0.2:
            predicted_yaw = math.atan2(velocity_y, velocity_x)

    output = Pose()
    output.position.x = x
    output.position.y = y
    output.position.z = current.position.z
    output.orientation = yaw_to_quaternion(predicted_yaw)
    return output


def build_prediction_horizons(time_step: float, prediction_horizon: float):
    """Return positive, uniformly spaced samples up to the final horizon."""
    if time_step <= 0.0:
        raise ValueError("~prediction_time_step must be positive")
    if prediction_horizon <= 0.0:
        raise ValueError("~prediction_horizon must be positive")
    step_count = int(round(prediction_horizon / time_step))
    if step_count < 1 or abs(step_count * time_step - prediction_horizon) > 1.0e-6:
        raise ValueError(
            "~prediction_horizon must be an integer multiple of "
            f"~prediction_time_step; received {prediction_horizon} and {time_step}"
        )
    return [time_step * index for index in range(1, step_count + 1)]
