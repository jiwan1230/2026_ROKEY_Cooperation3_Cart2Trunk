"""
test_09_rescan_replan_mode.py
⑨ replan_after_rescan()에도 mode/margin이 전달되는지 검증 - 이게 실제로
trunk_map_planner_node.py(ROS2, 로봇 결합 경로)가 호출하는 함수라 08과
별개로 여기도 지원해야 한다.
"""
import sys, pathlib
from importlib import import_module

import pytest

_m02 = import_module("cart2trunk_planning.core.trunk_space_state")
_m03 = import_module("cart2trunk_planning.core.extreme_point_candidates")
_m09 = import_module("cart2trunk_planning.core.rescan_replan")

Trunk = _m02.Trunk
Box = _m03.Box
replan_after_rescan = _m09.replan_after_rescan


def test_default_mode_unchanged_behavior():
    trunk = Trunk(width=1.5, depth=1.5, height=0.9)
    remaining = [Box("Small", 0.30, 0.20, 0.15), Box("Large", 0.50, 0.35, 0.30)]

    plans_default, _ = replan_after_rescan(remaining, trunk, [])
    plans_explicit, _ = replan_after_rescan(remaining, trunk, [], mode="large_first")

    assert [p.box_id for p in plans_default] == [p.box_id for p in plans_explicit]
    assert [p.position for p in plans_default] == [p.position for p in plans_explicit]


def test_count_first_mode_changes_order():
    trunk = Trunk(width=1.5, depth=1.5, height=0.9)
    remaining = [Box("Small", 0.30, 0.20, 0.15), Box("Large", 0.50, 0.35, 0.30)]

    plans_large_first, _ = replan_after_rescan(remaining, trunk, [], mode="large_first")
    plans_count_first, _ = replan_after_rescan(remaining, trunk, [], mode="count_first")

    assert [p.box_id for p in plans_large_first] == ["Large", "Small"]
    assert [p.box_id for p in plans_count_first] == ["Small", "Large"]


def test_margin_override_is_respected():
    custom_margin = 0.05
    trunk = Trunk(width=0.3 + 2 * custom_margin, depth=0.3 + 2 * custom_margin, height=0.4)
    box = Box("A", 0.3, 0.3, 0.15)

    plans, unloadable = replan_after_rescan([box], trunk, [], margin=custom_margin)

    assert len(unloadable) == 0
    assert plans[0].position[0] == pytest.approx(custom_margin)
