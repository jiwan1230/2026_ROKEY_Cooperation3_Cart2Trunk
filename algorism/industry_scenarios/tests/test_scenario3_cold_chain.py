"""
test_scenario3_cold_chain.py
시나리오3(냉동/냉장 물류 - 냉기 순환용 큰 마진) 검증.
"""
import sys, pathlib
from importlib import import_module

import pytest

_ALGORISM_DIR = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_ALGORISM_DIR))
sys.path.insert(0, str(_ALGORISM_DIR / "industry_scenarios"))

_m02 = import_module("02_trunk_space_state")
_m03 = import_module("03_extreme_point_candidates")
scenario3 = import_module("scenario3_cold_chain")

Trunk = _m02.Trunk
Box = _m03.Box
COLD_CHAIN_MARGIN = scenario3.COLD_CHAIN_MARGIN
generate_loading_plan_cold_chain = scenario3.generate_loading_plan_cold_chain


def test_cold_chain_margin_is_much_larger_than_default():
    from importlib import import_module as im
    default_margin = im("17_margin_check").MARGIN
    assert COLD_CHAIN_MARGIN > default_margin * 2


def test_two_boxes_keep_cold_chain_gap_between_them():
    """나란히 놓인 두 박스 사이 간격이 COLD_CHAIN_MARGIN 이상이어야 한다(냉기 순환)."""
    trunk = Trunk(width=1.2, depth=0.5, height=0.4)
    boxes = [Box("Frozen1", 0.3, 0.3, 0.2), Box("Frozen2", 0.3, 0.3, 0.2)]

    plans, unloadable = generate_loading_plan_cold_chain(boxes, trunk)

    assert len(unloadable) == 0
    x_ranges = sorted((p.position[0], p.position[0] + p.dimensions[0]) for p in plans)
    gap = x_ranges[1][0] - x_ranges[0][1]
    assert gap >= COLD_CHAIN_MARGIN - 1e-9, f"박스 사이 간격 {gap}이 냉기 순환 마진({COLD_CHAIN_MARGIN})보다 좁음"
