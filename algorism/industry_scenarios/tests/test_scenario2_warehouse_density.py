"""
test_scenario2_warehouse_density.py
시나리오2(창고/물류센터 - 공간 활용 최대화) - 이제 코어 mode="count_first"에
위임하는 얇은 래퍼라, 여기서는 "제대로 위임하는지"만 확인한다. 실제 점수/순서
로직 자체의 상세 검증은 코어 테스트(tests/test_05_score_count_first.py,
tests/test_06_loading_order_decision.py, tests/test_08_generate_loading_plan_
mode.py)가 담당한다.
"""
import sys, pathlib
from importlib import import_module

_ALGORISM_DIR = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_ALGORISM_DIR))
sys.path.insert(0, str(_ALGORISM_DIR / "industry_scenarios"))

_m02 = import_module("02_trunk_space_state")
_m03 = import_module("03_extreme_point_candidates")
_m08 = import_module("08_unloadable_reason")
scenario2 = import_module("scenario2_warehouse_density")

Trunk = _m02.Trunk
Box = _m03.Box
generate_loading_plan_count_first = scenario2.generate_loading_plan_count_first


def test_delegates_to_core_count_first_mode():
    trunk = Trunk(width=0.6, depth=0.4, height=0.45)  # ⑮ 상단 여유(0.2m) 감안
    boxes = (
        [Box(f"Small{i}", 0.1, 0.1, 0.1) for i in range(6)]
        + [Box(f"Big{i}", 0.3, 0.2, 0.2) for i in range(2)]
    )

    via_scenario, _ = generate_loading_plan_count_first(boxes, trunk)
    via_core_directly, _ = _m08.generate_loading_plan(boxes, trunk, mode="count_first")

    assert [p.box_id for p in via_scenario] == [p.box_id for p in via_core_directly]
    assert [p.position for p in via_scenario] == [p.position for p in via_core_directly]


def test_count_first_still_fits_more_than_default_in_a_tight_trunk():
    trunk = Trunk(width=0.6, depth=0.4, height=0.45)
    boxes = (
        [Box(f"Small{i}", 0.1, 0.1, 0.1) for i in range(6)]
        + [Box(f"Big{i}", 0.3, 0.2, 0.2) for i in range(2)]
    )

    default_plans, _ = _m08.generate_loading_plan(boxes, trunk)
    count_first_plans, _ = generate_loading_plan_count_first(boxes, trunk)

    assert len(count_first_plans) > len(default_plans)
