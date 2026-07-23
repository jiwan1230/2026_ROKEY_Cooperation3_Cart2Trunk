"""
test_05_score_count_first.py
⑤ score_count_first() - "개수 우선" 모드용 점수 함수 (원래 industry_scenarios/
scenario2_warehouse_density.py의 score_density_first를 코어로 승격).

기존 박스들이 차지한 바운딩 영역을 새 후보가 얼마나 밖으로 넓히는지를 점수로
삼는다 - 새 영역을 여는 것보다 기존 영역 재사용을 우선.
"""
import sys, pathlib
from importlib import import_module

_m02 = import_module("cart2trunk_planning.core.trunk_space_state")
_m03 = import_module("cart2trunk_planning.core.extreme_point_candidates")
_m05 = import_module("cart2trunk_planning.core.candidate_scoring")
_m07 = import_module("cart2trunk_planning.core.placement_plan")

Trunk = _m02.Trunk
Box = _m03.Box
PlacedBox = _m03.PlacedBox
ExtremePointState = _m03.ExtremePointState
place_one_box = _m07.place_one_box
score_count_first = _m05.score_count_first


def _existing_footprint_setup():
    trunk = Trunk(width=1.5, depth=0.4, height=0.5)
    state = ExtremePointState()
    state.register_placement(PlacedBox(box=Box("Existing", 0.3, 0.36, 0.2), x=0.02, y=0.02, z=0.0))
    return trunk, state


def test_score_count_first_prefers_reusing_existing_footprint_over_going_deeper():
    trunk, state_default = _existing_footprint_setup()
    _, state_count_first = _existing_footprint_setup()
    box = Box("New", 0.2, 0.2, 0.2)

    plan_default = place_one_box(box, trunk, state_default, order=2)
    plan_count_first = place_one_box(box, trunk, state_count_first, order=2, score_fn=score_count_first)

    assert plan_default is not None and plan_count_first is not None
    assert plan_default.position != plan_count_first.position
    assert plan_count_first.position[0] < 0.4, f"개수우선인데 Existing 옆이 아니라 x={plan_count_first.position[0]}로 감"
    assert plan_default.position[0] > 1.0, f"기본값인데 안쪽 깊은 자리로 안 감 (x={plan_default.position[0]})"
