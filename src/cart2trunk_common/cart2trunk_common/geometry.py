"""AABB 코너/중심 좌표 변환, yaw<->쿼터니언 변환 - 여러 패키지가 각자 구현하던 것을 공통화.

perception(trunk_scan_action_server: 장애물 AABB 중심 계산)과 planning
(loading_planner_action_server: 박스 중심<->최소코너 변환)에서 같은 종류의
변환을 각각 따로 작성하고 있었다. geometry_msgs 메시지가 아니라 순수
(x, y, z) 튜플을 주고받아서 rclpy 없이도 테스트/재사용 가능하게 한다.
"""
from typing import Tuple

Point = Tuple[float, float, float]


def corner_to_center(corner: Point, size: Point) -> Point:
    """최소 코너(x,y,z) + 크기(width,depth,height) -> 중심점."""
    cx, cy, cz = corner
    sx, sy, sz = size
    return (cx + sx / 2.0, cy + sy / 2.0, cz + sz / 2.0)


def center_to_corner(center: Point, size: Point) -> Point:
    """중심점 + 크기(width,depth,height) -> 최소 코너(x,y,z)."""
    cx, cy, cz = center
    sx, sy, sz = size
    return (cx - sx / 2.0, cy - sy / 2.0, cz - sz / 2.0)


def subtract_offset(point: Point, offset: Point) -> Point:
    """base 프레임 좌표 -> 로컬(offset 기준) 좌표. local_to_base_frame()의 역변환."""
    return tuple(p - o for p, o in zip(point, offset))


def add_offset(point: Point, offset: Point) -> Point:
    """로컬(offset 기준) 좌표 -> base 프레임 좌표."""
    return tuple(p + o for p, o in zip(point, offset))


Quaternion = Tuple[float, float, float, float]  # (x, y, z, w) - ROS 관례

IDENTITY_QUATERNION: Quaternion = (0.0, 0.0, 0.0, 1.0)


def quaternion_from_z_yaw(yaw_rad: float) -> Quaternion:
    """Z축 기준 yaw 회전(라디안) -> (x,y,z,w) 쿼터니언. 트렁크/카트 좌표계는
    바닥에 놓인 박스만 다루므로 roll/pitch 없이 yaw 회전만 지원한다."""
    import math
    half = yaw_rad / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def z_yaw_from_quaternion(q: Quaternion) -> float:
    """(x,y,z,w) 쿼터니언 -> Z축 yaw(라디안). quaternion_from_z_yaw()의 역변환
    (입력이 순수 Z회전이 아니면 그 성분만 근사 추출됨)."""
    import math
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
