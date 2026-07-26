import math

import pytest

from cart2trunk_motion.control_loops import base_frame_to_world, down_quat_with_yaw, DOWN_QUAT_XYZW


def test_base_frame_to_world_identity_anchor():
    anchor = ((0.0, 0.0, 0.0), 0.0, 0.0)
    assert base_frame_to_world(anchor, (1.0, 2.0, 3.0)) == pytest.approx((1.0, 2.0, 3.0))


def test_base_frame_to_world_translates_and_adds_lift_to_z():
    anchor = ((1.0, 2.0, 0.5), 0.0, 0.2)
    wx, wy, wz = base_frame_to_world(anchor, (0.0, 0.0, 0.0))
    assert (wx, wy, wz) == pytest.approx((1.0, 2.0, 0.7))


def test_base_frame_to_world_rotates_90_degrees():
    # anchor_yaw=90deg: base +x -> world +y, base +y -> world -x (오른손 좌표계 회전)
    anchor = ((0.0, 0.0, 0.0), math.pi / 2, 0.0)
    wx, wy, wz = base_frame_to_world(anchor, (1.0, 0.0, 0.0))
    assert (wx, wy) == pytest.approx((0.0, 1.0), abs=1e-9)


def test_base_frame_to_world_roundtrips_with_manual_inverse():
    anchor = ((0.3, -0.4, 0.1), 1.1, 0.05)
    point = (0.2, 0.6, 0.9)
    wx, wy, wz = base_frame_to_world(anchor, point)

    ax, ay, az = anchor[0]
    yaw, lift = anchor[1], anchor[2]
    dx, dy = wx - ax, wy - ay
    x_back = dx * math.cos(yaw) + dy * math.sin(yaw)
    y_back = -dx * math.sin(yaw) + dy * math.cos(yaw)
    z_back = wz - az - lift
    assert (x_back, y_back, z_back) == pytest.approx(point)


def test_down_quat_with_yaw_zero_matches_down_quat_constant():
    assert down_quat_with_yaw(0.0) == pytest.approx(DOWN_QUAT_XYZW)


def test_down_quat_with_yaw_is_unit_quaternion():
    q = down_quat_with_yaw(0.7)
    norm = math.sqrt(sum(c * c for c in q))
    assert norm == pytest.approx(1.0)


def test_down_quat_with_yaw_full_turn_returns_to_start():
    q0 = down_quat_with_yaw(0.0)
    q2pi = down_quat_with_yaw(2 * math.pi)
    # 쿼터니언은 2pi 회전 시 부호가 반전될 수 있음(같은 회전을 나타냄) - 부호까지
    # 정확히 같은지 대신 성분별 절댓값으로 비교.
    assert [abs(c) for c in q0] == pytest.approx([abs(c) for c in q2pi], abs=1e-6)
