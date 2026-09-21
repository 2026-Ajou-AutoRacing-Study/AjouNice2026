"""Geometry helpers for MORAI Base4-Temporal-3S."""

from __future__ import annotations

import math

import numpy as np


def rotation_z(angle):
    c = math.cos(float(angle))
    s = math.sin(float(angle))

    return np.asarray(
        [
            [c, -s, 0.0],
            [s,  c, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def ego_pose_matrix(ego):
    """Ego frame -> MORAI world frame."""
    heading = math.radians(float(ego.heading))

    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation_z(heading)
    transform[:3, 3] = [
        float(ego.position.x),
        float(ego.position.y),
        float(ego.position.z),
    ]

    return transform


def past_lidar_to_current_lidar(
    past_ego,
    current_ego,
    lidar_to_ego,
    ego_to_lidar,
):
    """Return homogeneous transform from a past LiDAR frame to current LiDAR."""

    world_from_past_ego = ego_pose_matrix(past_ego)
    world_from_current_ego = ego_pose_matrix(current_ego)

    current_ego_from_world = np.linalg.inv(world_from_current_ego)

    return (
        np.asarray(ego_to_lidar, dtype=np.float64)
        @ current_ego_from_world
        @ world_from_past_ego
        @ np.asarray(lidar_to_ego, dtype=np.float64)
    )


def transform_points(points, transform):
    """Transform Nx4 xyz/intensity points while preserving intensity."""
    xyz1 = np.concatenate(
        [
            np.asarray(points[:, :3], dtype=np.float64),
            np.ones((len(points), 1), dtype=np.float64),
        ],
        axis=1,
    )

    xyz = (np.asarray(transform, dtype=np.float64) @ xyz1.T).T[:, :3]

    return np.concatenate(
        [
            xyz.astype(np.float32),
            np.asarray(points[:, 3:4], dtype=np.float32),
        ],
        axis=1,
    )


def actor_velocity_in_current_lidar(actor, current_ego, ego_to_lidar):
    """Absolute actor velocity expressed in current LiDAR axes.

    MORAI ObjectStatus velocity is interpreted as actor-body-frame velocity:
      x = forward
      y = lateral
      z = vertical
    """

    actor_heading = math.radians(float(actor.heading))
    ego_heading = math.radians(float(current_ego.heading))

    body_velocity = np.asarray(
        [
            float(actor.velocity.x),
            float(actor.velocity.y),
            float(actor.velocity.z),
        ],
        dtype=np.float64,
    )

    # actor body -> world
    velocity_world = rotation_z(actor_heading) @ body_velocity

    # world -> current ego
    velocity_ego = rotation_z(-ego_heading) @ velocity_world

    # current ego -> current lidar; rotation only, no translation
    ego_to_lidar = np.asarray(ego_to_lidar, dtype=np.float64)
    velocity_lidar = ego_to_lidar[:3, :3] @ velocity_ego

    return velocity_lidar.astype(np.float32)
