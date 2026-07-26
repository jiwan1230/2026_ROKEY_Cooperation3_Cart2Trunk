"""
test_scenario4_hazmat.py
시나리오4(위험물/화학물질 창고) - 비호환 물질 인접 배치 금지 검증.
"""
import sys, pathlib
from importlib import import_module

_ALGORISM_DIR = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_ALGORISM_DIR))
sys.path.insert(0, str(_ALGORISM_DIR / "industry_scenarios"))

_m02 = import_module("02_trunk_space_state")
_m03 = import_module("03_extreme_point_candidates")
scenario4 = import_module("scenario4_hazmat")

Trunk = _m02.Trunk
Box = _m03.Box
PlacedBox = _m03.PlacedBox
ExtremePointState = _m03.ExtremePointState
has_hazmat_clearance = scenario4.has_hazmat_clearance
generate_loading_plan_hazmat = scenario4.generate_loading_plan_hazmat
HAZMAT_SAFE_DISTANCE = scenario4.HAZMAT_SAFE_DISTANCE


def test_compatible_boxes_can_sit_right_next_to_each_other():
    """같은 분류(또는 hazard_class 없음)끼리는 안전거리 규칙 대상이 아니다."""
    trunk = Trunk(width=1.0, depth=1.0, height=0.5)
    existing = PlacedBox(box=Box("Ox1", 0.2, 0.2, 0.2, hazard_class="oxidizer"), x=0.02, y=0.02, z=0.0)
    candidate_box = Box("Ox2", 0.2, 0.2, 0.2, hazard_class="oxidizer")

    # Ox1 바로 옆(안전거리 없이)
    assert has_hazmat_clearance(0.24, 0.02, 0.0, candidate_box, trunk, [existing]) is True


def test_incompatible_boxes_must_keep_safe_distance():
    """산화제(oxidizer)와 인화물(flammable)은 안전거리 미만이면 거부돼야 한다."""
    trunk = Trunk(width=1.0, depth=1.0, height=0.5)
    existing = PlacedBox(box=Box("Ox1", 0.2, 0.2, 0.2, hazard_class="oxidizer"), x=0.02, y=0.02, z=0.0)
    flammable = Box("Fuel1", 0.2, 0.2, 0.2, hazard_class="flammable")

    # Ox1 바로 옆(간격 0) - 안전거리 미달이라 거부돼야 함
    assert has_hazmat_clearance(0.22, 0.02, 0.0, flammable, trunk, [existing]) is False
    # 안전거리 이상 떨어진 자리 - 허용돼야 함
    far_x = 0.22 + HAZMAT_SAFE_DISTANCE + 0.05
    assert has_hazmat_clearance(far_x, 0.02, 0.0, flammable, trunk, [existing]) is True


def test_generate_loading_plan_hazmat_keeps_incompatible_boxes_apart():
    trunk = Trunk(width=2.0, depth=1.0, height=0.5)
    oxidizer_box = Box("Oxidizer1", 0.2, 0.2, 0.2, hazard_class="oxidizer")
    flammable_box = Box("Flammable1", 0.2, 0.2, 0.2, hazard_class="flammable")
    boxes = [oxidizer_box, flammable_box]

    plans, unloadable = generate_loading_plan_hazmat(boxes, trunk)

    assert len(unloadable) == 0
    plan_by_id = {p.box_id: p for p in plans}
    ox_plan = plan_by_id["Oxidizer1"]
    fl_plan = plan_by_id["Flammable1"]
    ox_placed = PlacedBox(box=oxidizer_box, x=ox_plan.position[0], y=ox_plan.position[1], z=ox_plan.position[2])

    # 최종 배치 결과가 실제로 안전거리 규칙(has_hazmat_clearance)을 만족하는지
    # 그대로 재확인 - 단일 축 거리만 보면 다른 축에서 떨어진 경우를 놓칠 수 있어서
    # 실제 판정 함수를 그대로 재사용한다.
    assert has_hazmat_clearance(
        fl_plan.position[0], fl_plan.position[1], fl_plan.position[2],
        flammable_box, trunk, [ox_placed],
    ) is True
