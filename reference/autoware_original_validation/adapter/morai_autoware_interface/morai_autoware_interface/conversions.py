"""Pure conversion helpers shared by the MORAI interface and its tests."""

import math


KMH_PER_MPS = 3.6


def clamp(value: float, lower: float, upper: float) -> float:
    """Clamp value to the inclusive [lower, upper] interval."""
    return max(lower, min(upper, value))


def kmh_to_mps(value: float) -> float:
    """Convert kilometres per hour to metres per second."""
    return value / KMH_PER_MPS


def mps_to_kmh(value: float) -> float:
    """Convert metres per second to kilometres per hour."""
    return value * KMH_PER_MPS


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def heading_deg_to_yaw(heading_deg: float) -> float:
    """Convert MORAI heading in degrees to a ROS yaw angle in radians."""
    return normalize_angle(math.radians(heading_deg))


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    """Return an x, y, z, w quaternion for a planar yaw angle."""
    half_yaw = 0.5 * yaw
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


def deterministic_uuid(unique_id: int, namespace: int = 0) -> list[int]:
    """Create a stable ROS UUID byte array from MORAI list and object IDs."""
    namespace_bytes = int(namespace & ((1 << 64) - 1)).to_bytes(8, 'big')
    object_bytes = int(unique_id & ((1 << 64) - 1)).to_bytes(8, 'big')
    return list(namespace_bytes + object_bytes)
