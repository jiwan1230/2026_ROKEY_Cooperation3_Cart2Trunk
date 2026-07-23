"""
test_07_placement_plan_score_fn.py
⑦ place_one_box()에 score_fn을 주입할 수 있는지 확인.

배경: 산업 현장 시나리오(industry_scenarios/)마다 "어떤 자리가 좋은 자리인지"
기준(⑤ 점수 함수)이 다르다 - 예: 창고는 빈틈 없이 빽빽하게, 트렁크는 입구
접근성 우선. score_fn=None(기본값)이면 지금까지처럼 05_candidate_scoring.
score_candidate를 그대로 쓰고(하위 호환), 다른 함수를 넘기면 그걸로 후보를
고른다.
"""
import sys, pathlib
from importlib import import_module

_m02 = import_module("cart2trunk_planning.core.trunk_space_state")
_m03 = import_module("cart2trunk_planning.core.extreme_point_candidates")
_m07 = import_module("cart2trunk_planning.core.placement_plan")

Trunk = _m02.Trunk
Box = _m03.Box
ExtremePointState = _m03.ExtremePointState
place_one_box = _m07.place_one_box


def test_default_score_fn_unchanged_behavior():
    """score_fn을 안 넘기면 기존과 똑같은 자리를 고른다 (하위 호환)."""
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    state_default = ExtremePointState()
    state_explicit = ExtremePointState()
    box = Box("A", 0.3, 0.3, 0.2)

    plan_default = place_one_box(box, trunk, state_default, order=1)
    plan_explicit = place_one_box(box, trunk, state_explicit, order=1, score_fn=None)

    assert plan_default.position == plan_explicit.position


def test_custom_score_fn_overrides_candidate_choice():
    """
    일부러 기본 점수 함수와 정반대로 동작하는 score_fn(입구에서 가까운 자리를
    선호)을 넘기면, 기본값이 고르는 자리(입구에서 먼 안쪽)와 다른 자리를
    골라야 한다.
    """
    trunk = Trunk(width=1.0, depth=1.0, height=0.5)
    box = Box("A", 0.2, 0.2, 0.2)

    # 기존 장애물 두 개를 놓아서, 안쪽(x 큰 쪽)과 입구 쪽(x 작은 쪽) 둘 다에
    # 빈 자리가 남게 만든다.
    state_default = ExtremePointState()
    for obs in [
        _m03.PlacedBox(box=Box("Obs1", 0.2, 1.0, 0.2), x=0.4, y=0.0, z=0.0),
    ]:
        state_default.register_placement(obs)
    state_custom = ExtremePointState()
    for obs in [
        _m03.PlacedBox(box=Box("Obs1", 0.2, 1.0, 0.2), x=0.4, y=0.0, z=0.0),
    ]:
        state_custom.register_placement(obs)

    def score_prefer_entrance(x, y, z, box, trunk, placed):
        # x가 작을수록(입구에 가까울수록) 좋은 점수 - 기본 함수와 정반대 방향.
        return x, 0

    plan_default = place_one_box(box, trunk, state_default, order=2)
    plan_custom = place_one_box(box, trunk, state_custom, order=2, score_fn=score_prefer_entrance)

    assert plan_default.position != plan_custom.position
    assert plan_custom.position[0] < plan_default.position[0], (
        "커스텀 score_fn(입구 우선)이 기본값(안쪽 우선)보다 입구에 더 가까운 "
        "자리를 골랐어야 함"
    )
