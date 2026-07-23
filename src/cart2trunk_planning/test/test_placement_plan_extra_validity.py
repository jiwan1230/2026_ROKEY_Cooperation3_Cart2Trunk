"""
test_07_placement_plan_extra_validity.py
⑦ place_one_box()에 시나리오별 추가 하드 컷(extra_validity_fn)을 끼워 넣을 수
있는지 확인.

배경: 위험물 창고(industry_scenarios/scenario4_hazmat.py)처럼 코어에 없는
완전히 새로운 규칙(비호환 물질 인접 금지)이 필요한 현장이 있다. 기존
④⑬⑮⑯⑰ 체인은 코어 고유 규칙이라 안 건드리고, 그 뒤에 AND로 하나 더
끼워 넣을 수 있는 자리를 연다. extra_validity_fn=None(기본값)이면 지금까지와
동일하게 동작한다(하위 호환).
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


def test_default_extra_validity_fn_unchanged_behavior():
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    box = Box("A", 0.3, 0.3, 0.2)

    plan_default = place_one_box(box, trunk, ExtremePointState(), order=1)
    plan_explicit_none = place_one_box(box, trunk, ExtremePointState(), order=1, extra_validity_fn=None)

    assert plan_default.position == plan_explicit_none.position


def test_extra_validity_fn_can_reject_every_candidate():
    """extra_validity_fn이 항상 False를 반환하면, 자리가 있어도 못 놓아야 한다(None)."""
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    box = Box("A", 0.3, 0.3, 0.2)

    plan = place_one_box(box, trunk, ExtremePointState(), order=1, extra_validity_fn=lambda x, y, z, box, trunk, placed: False)

    assert plan is None


def test_extra_validity_fn_can_steer_to_a_different_candidate():
    """
    extra_validity_fn이 기본값이 고르는 자리(안쪽 깊은 곳)만 거부하면, 다른
    유효한 자리로 옮겨가야 한다 - 실제로 후보 필터링에 관여한다는 뜻. 이미
    놓인 박스 하나를 둬서 후보가 최소 2개 이상 생기게 만든다.
    """
    trunk = Trunk(width=1.2, depth=0.4, height=0.4)
    existing = _m03.PlacedBox(box=Box("Existing", 0.3, 0.36, 0.15), x=0.02, y=0.02, z=0.0)
    box = Box("A", 0.2, 0.2, 0.15)

    state_default = ExtremePointState()
    state_default.register_placement(existing)
    plan_default = place_one_box(box, trunk, state_default, order=2)

    def reject_default_choice(x, y, z, box, trunk, placed):
        return abs(x - plan_default.position[0]) > 1e-6  # 기본값이 고른 그 x만 거부

    state_filtered = ExtremePointState()
    state_filtered.register_placement(existing)
    plan_filtered = place_one_box(box, trunk, state_filtered, order=2, extra_validity_fn=reject_default_choice)

    assert plan_filtered is not None
    assert plan_filtered.position != plan_default.position
