"""
test_07_placement_plan_margin_override.py
⑦ place_one_box()에 margin을 시나리오별로 덮어쓸 수 있는지 확인.

배경: 냉동/냉장 물류(industry_scenarios/scenario3_cold_chain.py)처럼 냉기
순환을 위해 기본 마진(17_margin_check.MARGIN, 로봇 실행 오차 흡수용 2cm)보다
훨씬 큰 간격이 필요한 현장이 있다. margin=None(기본값)이면 지금까지처럼
17_margin_check.MARGIN을 그대로 쓴다 (하위 호환).
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


def test_default_margin_unchanged_behavior():
    # 트렁크를 박스+2*MARGIN에 정확히 맞춰서, 입구 쪽 코너(margin, margin, 0)가
    # 유일하게 들어갈 수 있는 자리가 되게 한다(다른 테스트들과 같은 패턴 -
    # test_07_placement_plan_stacking.py의 _fills_floor_trunk() 참고).
    trunk = Trunk(width=0.3 + 2 * MARGIN, depth=0.3 + 2 * MARGIN, height=0.4)
    box = Box("A", 0.3, 0.3, 0.15)

    plan_default = place_one_box(box, trunk, ExtremePointState(), order=1)
    plan_explicit_none = place_one_box(box, trunk, ExtremePointState(), order=1, margin=None)

    assert plan_default.position == plan_explicit_none.position
    assert plan_default.position == pytest.approx((MARGIN, MARGIN, 0.0))


def test_larger_margin_pushes_box_further_from_wall():
    """margin=0.05를 넘기면, 벽에서 기본(2cm)보다 훨씬 더 뗀 자리(5cm)에 놓여야 한다."""
    custom_margin = 0.05
    trunk = Trunk(width=0.3 + 2 * custom_margin, depth=0.3 + 2 * custom_margin, height=0.4)
    box = Box("A", 0.3, 0.3, 0.15)

    plan = place_one_box(box, trunk, ExtremePointState(), order=1, margin=custom_margin)

    assert plan.position == pytest.approx((custom_margin, custom_margin, 0.0))
