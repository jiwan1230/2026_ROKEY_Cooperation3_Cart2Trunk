"""
test_02_trunk_space_state.py
② to_bounding_trunk()이 "입구 방향"을 로봇 base 원점 기준으로 올바르게 추정하는지 검증.

배경: 로봇은 M0609 base 좌표계의 원점(0,0,0)에 고정되어 있고, 트렁크에는 항상 정해진
한 방향(x축)으로만 접근한다 (사용자 확인 + 실제 스캔 데이터의 "x, +deep" 라벨과 일치).
즉 y축(좌우 위치)은 입구와 아예 무관하고, x축만 "입구에서 먼 정도"를 결정한다.
그래서 Trunk는 entrance_near_x 하나만 가진다 (처음엔 x/y 둘 다 만들었다가, 실제로는
y축이 입구와 무관하다는 게 밝혀져서 x만 남김 - y축까지 반영했던 첫 버전은 "좌우 위치가
다르면 점수도 달라지는" 실제 버그를 만들었었다).
"""
import sys, pathlib
from importlib import import_module

_m02 = import_module("cart2trunk_planning.core.trunk_space_state")

TrunkWorldMap = _m02.TrunkWorldMap


def test_entrance_near_local_origin_when_trunk_is_positive_x_offset_from_robot():
    """
    트렁크 전체가 로봇 원점(x=0) 기준 x:[0.5,1.0] 범위에 있으면, 로컬 (0,0,0) 코너
    (=base 기준 x=0.5)가 반대쪽(=base 기준 x=1.0)보다 로봇에 더 가깝다 -> 입구는
    로컬 원점 쪽(entrance_near_x=True)이어야 한다.
    """
    world_map = TrunkWorldMap(vertices=[
        (0.5, 0.5, 0.0), (1.0, 0.5, 0.0), (0.5, 1.0, 0.0), (1.0, 1.0, 0.3),
    ])
    trunk, offset = world_map.to_bounding_trunk()

    assert trunk.entrance_near_x is True


def test_entrance_near_far_corner_when_trunk_is_negative_x_offset_from_robot():
    """
    트렁크 전체가 로봇 원점 기준 x:[-1.0,-0.5] 범위(음수)에 있으면, 로컬 max쪽 코너
    (=base 기준 x=-0.5)가 로컬 원점 코너(=base 기준 x=-1.0)보다 로봇에 더 가깝다 ->
    입구는 로컬 원점 반대쪽(entrance_near_x=False)이어야 한다.
    """
    world_map = TrunkWorldMap(vertices=[
        (-1.0, -1.0, 0.0), (-0.5, -1.0, 0.0), (-1.0, -0.5, 0.0), (-0.5, -0.5, 0.3),
    ])
    trunk, offset = world_map.to_bounding_trunk()

    assert trunk.entrance_near_x is False
