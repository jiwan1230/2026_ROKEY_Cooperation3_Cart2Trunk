import math

import pytest

from cart2trunk_common.geometry import (
    IDENTITY_QUATERNION, add_offset, center_to_corner, corner_to_center,
    quaternion_from_z_yaw, subtract_offset, z_yaw_from_quaternion,
)


def test_corner_to_center_and_back_are_inverses():
    corner = (1.0, 2.0, 0.5)
    size = (0.4, 0.3, 0.2)
    center = corner_to_center(corner, size)
    assert center == pytest.approx((1.2, 2.15, 0.6))
    assert center_to_corner(center, size) == pytest.approx(corner)


def test_offset_round_trip():
    p = (3.5, -0.2, 1.0)
    offset = (3.0, -0.5, 0.4)
    local = subtract_offset(p, offset)
    assert local == pytest.approx((0.5, 0.3, 0.6))
    assert add_offset(local, offset) == pytest.approx(p)


def test_quaternion_from_zero_yaw_is_identity():
    assert quaternion_from_z_yaw(0.0) == pytest.approx(IDENTITY_QUATERNION)


def test_quaternion_from_90deg_yaw():
    q = quaternion_from_z_yaw(math.pi / 2)
    assert q == pytest.approx((0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)))


@pytest.mark.parametrize('yaw_deg', [0, 30, 90, 135, 180, -45, 270])
def test_yaw_quaternion_round_trip(yaw_deg):
    yaw_rad = math.radians(yaw_deg)
    q = quaternion_from_z_yaw(yaw_rad)
    recovered = z_yaw_from_quaternion(q)
    # atan2 결과는 (-pi, pi] 범위이므로 원래 각도와 2pi 배수 차이까지 허용
    diff = (recovered - yaw_rad + math.pi) % (2 * math.pi) - math.pi
    assert diff == pytest.approx(0.0, abs=1e-9)
