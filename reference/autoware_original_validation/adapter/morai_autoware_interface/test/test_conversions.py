"""Unit tests for MORAI/Autoware value conversions."""

import math

from morai_autoware_interface.conversions import clamp
from morai_autoware_interface.conversions import deterministic_uuid
from morai_autoware_interface.conversions import heading_deg_to_yaw
from morai_autoware_interface.conversions import kmh_to_mps
from morai_autoware_interface.conversions import mps_to_kmh
from morai_autoware_interface.conversions import normalize_angle
from morai_autoware_interface.conversions import quaternion_from_yaw
import pytest


def test_velocity_round_trip() -> None:
    """Velocity conversions preserve their value in a round trip."""
    assert kmh_to_mps(36.0) == pytest.approx(10.0)
    assert mps_to_kmh(10.0) == pytest.approx(36.0)


def test_heading_and_quaternion() -> None:
    """The MORAI heading produces the same yaw after quaternion conversion."""
    yaw = heading_deg_to_yaw(-119.14796447753906)
    quaternion = quaternion_from_yaw(yaw)
    reconstructed = math.atan2(
        2.0 * quaternion[3] * quaternion[2],
        1.0 - 2.0 * quaternion[2] * quaternion[2],
    )
    assert normalize_angle(reconstructed) == pytest.approx(yaw)


def test_angle_wrap() -> None:
    """Angles are normalized to the ROS planar convention."""
    assert normalize_angle(3.0 * math.pi) == pytest.approx(-math.pi)
    assert normalize_angle(-3.0 * math.pi) == pytest.approx(-math.pi)


def test_clamp() -> None:
    """Values outside steering limits are clipped."""
    assert clamp(2.0, -1.0, 1.0) == 1.0
    assert clamp(-2.0, -1.0, 1.0) == -1.0
    assert clamp(0.5, -1.0, 1.0) == 0.5


def test_deterministic_uuid_separates_object_lists() -> None:
    """The same ID in distinct MORAI lists cannot collide."""
    npc = deterministic_uuid(42, 1)
    pedestrian = deterministic_uuid(42, 2)
    assert len(npc) == 16
    assert npc != pedestrian
    assert npc == deterministic_uuid(42, 1)
