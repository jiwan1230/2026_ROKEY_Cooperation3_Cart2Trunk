"""
test_07_placement_plan_stacking.py
⑦ place_one_box()의 allow_stacking 플래그 배선 확인.

시나리오: 트렁크 바닥 전체를 (⑰ 벽 마진 감안하고) 정확히 채우는 박스를 먼저
놓으면, 두 번째 같은 박스는 바닥에 더 놓을 자리가 없다. allow_stacking=False
(기본값)면 이때 "놓을 자리 없음"(None)이어야 하고, allow_stacking=True면 첫 번째
박스 위에(받침 100%로) 정확히 쌓여야 한다.
"""
import sys, pathlib
from importlib import import_module

import pytest

_m02 = import_module("cart2trunk_planning.core.trunk_space_state")
_m03 = import_module("cart2trunk_planning.core.extreme_point_candidates")
_m07 = import_module("cart2trunk_planning.core.placement_plan")
_m17 = import_module("cart2trunk_planning.core.margin_check")

Trunk = _m02.Trunk
Box = _m03.Box
ExtremePointState = _m03.ExtremePointState
place_one_box = _m07.place_one_box
MARGIN = _m17.MARGIN


def _fills_floor_trunk():
    # 바닥 면적을 (⑰ 벽 마진까지 감안해서) 정확히 채우는 박스 하나가 들어갈
    # 트렁크 - 박스(0.3 x 0.3) 양옆으로 MARGIN씩 남도록 0.3+2*MARGIN. 높이는 박스
    # 2개(0.3+0.3)를 쌓고도 ⑮ 상단 여유 공간(0.2m)까지 남도록 0.8로 잡음.
    return Trunk(width=0.3 + 2 * MARGIN, depth=0.3 + 2 * MARGIN, height=0.8)


def test_second_box_has_nowhere_to_go_when_stacking_disabled():
    trunk = _fills_floor_trunk()
    state = ExtremePointState()
    filler = Box("Floor", width=0.3, depth=0.3, height=0.3)

    first = place_one_box(filler, trunk, state, order=1)
    assert first is not None
    assert first.position == pytest.approx((MARGIN, MARGIN, 0.0))

    second = place_one_box(filler, trunk, state, order=2, allow_stacking=False)
    assert second is None  # 바닥엔 자리 없고, z>0은 플래그가 꺼져 있어 거부됨


def test_second_box_stacks_on_top_when_stacking_enabled():
    trunk = _fills_floor_trunk()
    state = ExtremePointState()
    filler = Box("Floor", width=0.3, depth=0.3, height=0.3)

    first = place_one_box(filler, trunk, state, order=1)
    assert first is not None

    second = place_one_box(filler, trunk, state, order=2, allow_stacking=True)
    assert second is not None
    assert second.position == pytest.approx((MARGIN, MARGIN, 0.3))  # 첫 박스 바로 위, 받침 100% (z는 마진 대상 아님)
