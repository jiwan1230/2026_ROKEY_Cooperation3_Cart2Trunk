"""
test_13_support_check.py
⑬ 받침(지지대) 확인 로직 검증.

배경: 극점 알고리즘은 박스 윗면도 다음 후보로 등록하지만(③), 그 자리 밑에
실제로 받쳐주는 게 있는지는 확인하지 않았다(④는 겹침/경계만 봄). 이 테스트는
04_candidate_validity_check.is_candidate_valid를 감싸는 13의 지지대 확인이
"밑면의 80% 이상이 실제로 닿아있어야 유효"하다는 규칙을 지키는지 확인한다.
"""
import sys, pathlib
from importlib import import_module

_m02 = import_module("cart2trunk_planning.core.trunk_space_state")
_m03 = import_module("cart2trunk_planning.core.extreme_point_candidates")
_m13 = import_module("cart2trunk_planning.core.support_check")

Trunk = _m02.Trunk
Box = _m03.Box
PlacedBox = _m03.PlacedBox
compute_support_ratio = _m13.compute_support_ratio
is_candidate_valid_with_stacking = _m13.is_candidate_valid_with_stacking


def test_floor_candidate_always_fully_supported():
    """z=0(바닥) 후보는 placed가 비어 있어도 받침 비율 1.0, allow_stacking=False여도 유효."""
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    box = Box("A", width=0.4, depth=0.3, height=0.2)

    assert compute_support_ratio(0.0, 0.0, 0.0, box, placed=[]) == 1.0
    assert is_candidate_valid_with_stacking(
        0.0, 0.0, 0.0, box, trunk, placed=[], allow_stacking=False
    ) is True


def test_fully_supported_box_on_top_is_valid_when_stacking_allowed():
    """아래 박스와 같은 크기의 박스를 정확히 그 위에 놓으면 받침 비율 1.0 -> 유효."""
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    below = PlacedBox(box=Box("Base", width=0.4, depth=0.4, height=0.2), x=0.0, y=0.0, z=0.0)
    on_top = Box("Top", width=0.4, depth=0.4, height=0.2)

    ratio = compute_support_ratio(0.0, 0.0, 0.2, on_top, placed=[below])
    assert ratio == 1.0
    assert is_candidate_valid_with_stacking(
        0.0, 0.0, 0.2, on_top, trunk, placed=[below], allow_stacking=True
    ) is True


def test_small_base_rejects_much_larger_box_on_top():
    """
    회귀 테스트: 작은 박스(0.2x0.2x0.3) 위에 훨씬 넓은 박스(0.6x0.6x0.2)를 놓으면
    받침 비율이 0.04/0.36 ≈ 11%로 80% 기준에 크게 못 미쳐 거부되어야 한다.
    (브레인스토밍 단계에서 발견한 실제 실패 사례)
    """
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    small_base = PlacedBox(box=Box("Small", width=0.2, depth=0.2, height=0.3), x=0.0, y=0.0, z=0.0)
    big_top = Box("Big", width=0.6, depth=0.6, height=0.2)

    ratio = compute_support_ratio(0.0, 0.0, 0.3, big_top, placed=[small_base])
    assert ratio < 0.8

    assert is_candidate_valid_with_stacking(
        0.0, 0.0, 0.3, big_top, trunk, placed=[small_base], allow_stacking=True
    ) is False


def test_combined_support_from_two_adjacent_boxes():
    """
    박스 하나만으로는 50%(0.8 기준 미달)지만, 옆에 딱 붙은 박스 두 개를 합치면
    100%가 되어 유효해져야 한다 - 여러 박스의 받침 넓이를 합산하는지 확인.
    """
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    left = PlacedBox(box=Box("Left", width=0.25, depth=0.4, height=0.2), x=0.0, y=0.0, z=0.0)
    right = PlacedBox(box=Box("Right", width=0.25, depth=0.4, height=0.2), x=0.25, y=0.0, z=0.0)
    on_top = Box("Top", width=0.5, depth=0.4, height=0.2)

    ratio_left_only = compute_support_ratio(0.0, 0.0, 0.2, on_top, placed=[left])
    assert abs(ratio_left_only - 0.5) < 1e-9  # 하나만 있으면 절반만 지지

    ratio_combined = compute_support_ratio(0.0, 0.0, 0.2, on_top, placed=[left, right])
    assert ratio_combined == 1.0

    assert is_candidate_valid_with_stacking(
        0.0, 0.0, 0.2, on_top, trunk, placed=[left, right], allow_stacking=True
    ) is True


def test_allow_stacking_false_rejects_even_fully_supported_candidate():
    """받침이 100%여도 allow_stacking=False(기본값)면 z>0 후보는 무조건 거부되어야 한다."""
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    below = PlacedBox(box=Box("Base", width=0.4, depth=0.4, height=0.2), x=0.0, y=0.0, z=0.0)
    on_top = Box("Top", width=0.4, depth=0.4, height=0.2)

    assert is_candidate_valid_with_stacking(
        0.0, 0.0, 0.2, on_top, trunk, placed=[below], allow_stacking=False
    ) is False
