"""
test_17_margin_check.py
⑰ 박스-벽 / 박스-박스 최소 간격(margin) 확인 검증.

배경: "박스끼리 아예 딱 붙지 말고 아주 조금 여유 공간을 두는 게 좋겠다"는 요청
(박스-벽, 박스-박스 둘 다 적용). 고정 마진(기본 0.01m = 1cm)을 하드 컷으로
확인한다. 바닥 접촉(z=0)이나 쌓기 지지면(z가 딱 맞닿는 경우)은 마진 대상이
아니다 - 그건 오히려 완전히 맞닿아야(⑬ 받침) 하는 관계라서 마진을 넣으면 안 된다.
"""
import sys, pathlib
from importlib import import_module

_m02 = import_module("cart2trunk_planning.core.trunk_space_state")
_m03 = import_module("cart2trunk_planning.core.extreme_point_candidates")
_m07 = import_module("cart2trunk_planning.core.placement_plan")
_m17 = import_module("cart2trunk_planning.core.margin_check")

Trunk = _m02.Trunk
Box = _m03.Box
PlacedBox = _m03.PlacedBox
ExtremePointState = _m03.ExtremePointState
place_one_box = _m07.place_one_box
has_wall_margin = _m17.has_wall_margin
has_box_margin = _m17.has_box_margin
has_sufficient_margin = _m17.has_sufficient_margin
MARGIN = _m17.MARGIN


def test_wall_margin_rejects_when_touching_near_wall():
    """x=0(벽에 딱 붙음)이면 마진(0.01m) 미달로 거부."""
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    box = Box("A", width=0.2, depth=0.2, height=0.2)
    assert has_wall_margin(0.0, 0.5, 0.0, box, trunk) is False


def test_wall_margin_accepts_at_exactly_margin_distance():
    """x=MARGIN(정확히 마진만큼 떨어짐)이면 경계값 통과."""
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    box = Box("A", width=0.2, depth=0.2, height=0.2)
    assert has_wall_margin(MARGIN, 0.5, 0.0, box, trunk) is True


def test_wall_margin_rejects_when_touching_far_wall():
    """반대쪽 벽(x+width == trunk.width)에 딱 붙어도 거부."""
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    box = Box("A", width=0.2, depth=0.2, height=0.2)
    assert has_wall_margin(0.8, 0.5, 0.0, box, trunk) is False  # 0.8+0.2=1.0, 여유 0


def test_wall_margin_rejects_when_touching_side_wall_y():
    """y=0 또는 y+depth==trunk.depth(옆벽)에 딱 붙어도 거부."""
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    box = Box("A", width=0.2, depth=0.2, height=0.2)
    assert has_wall_margin(0.5, 0.0, 0.0, box, trunk) is False
    assert has_wall_margin(0.5, 0.8, 0.0, box, trunk) is False


def test_wall_margin_does_not_apply_to_floor_or_ceiling():
    """z=0(바닥 접촉)은 마진 대상이 아니다 - 바닥엔 딱 붙어야 정상."""
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    box = Box("A", width=0.2, depth=0.2, height=0.2)
    assert has_wall_margin(0.5, 0.5, 0.0, box, trunk) is True


def test_box_margin_rejects_when_flush_against_neighbor_at_same_height():
    """옆에 딱 붙은(간격 0) 박스가 있으면(z 겹침) 마진 미달로 거부."""
    neighbor = PlacedBox(box=Box("N", 0.3, 0.3, 0.2), x=0.0, y=0.0, z=0.0)
    candidate = Box("C", width=0.2, depth=0.2, height=0.2)
    assert has_box_margin(0.3, 0.0, 0.0, candidate, [neighbor]) is False  # x=0.3에 딱 붙음


def test_box_margin_accepts_at_exactly_margin_distance():
    """이웃과 정확히 MARGIN만큼 떨어지면 경계값 통과."""
    neighbor = PlacedBox(box=Box("N", 0.3, 0.3, 0.2), x=0.0, y=0.0, z=0.0)
    candidate = Box("C", width=0.2, depth=0.2, height=0.2)
    assert has_box_margin(0.3 + MARGIN, 0.0, 0.0, candidate, [neighbor]) is True


def test_box_margin_ignores_neighbor_when_stacked_on_top():
    """z 범위가 안 겹치는(딱 위에 쌓인) 관계는 마진 대상이 아니다 - 맞닿아야 정상."""
    base = PlacedBox(box=Box("Base", 0.3, 0.3, 0.2), x=0.0, y=0.0, z=0.0)
    candidate = Box("Top", width=0.3, depth=0.3, height=0.1)
    # Top이 Base 바로 위(z=0.2, Base와 같은 x/y footprint)에 딱 붙어도 통과해야 함
    assert has_box_margin(0.0, 0.0, 0.2, candidate, [base]) is True


def test_has_sufficient_margin_combines_both():
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    neighbor = PlacedBox(box=Box("N", 0.3, 0.3, 0.2), x=0.0, y=0.0, z=0.0)
    candidate = Box("C", width=0.2, depth=0.2, height=0.2)
    # 벽에서는 충분히 떨어졌지만 이웃과는 딱 붙음 -> 거부
    assert has_sufficient_margin(0.3, 0.0, 0.0, candidate, trunk, [neighbor]) is False
    # 벽/이웃 둘 다에서 충분히 떨어짐 -> 통과
    assert has_sufficient_margin(0.3 + MARGIN, MARGIN, 0.0, candidate, trunk, [neighbor]) is True


def test_place_one_box_leaves_margin_from_wall():
    """통합 테스트: 실제 place_one_box()가 벽에 딱 붙이지 않고 MARGIN만큼 띄운다."""
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    state = ExtremePointState()
    box = Box("A", width=0.3, depth=0.3, height=0.2)

    plan = place_one_box(box, trunk, state, order=1)

    assert plan is not None
    x, y, z = plan.position
    assert x >= MARGIN - 1e-9
    assert y >= MARGIN - 1e-9


def test_place_one_box_leaves_margin_between_boxes():
    """통합 테스트: 두 번째 박스가 첫 번째 박스에 딱 붙지 않고 MARGIN만큼 띄워진다."""
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    state = ExtremePointState()
    box1 = Box("A", width=0.2, depth=0.2, height=0.2)
    box2 = Box("B", width=0.2, depth=0.2, height=0.2)

    plan1 = place_one_box(box1, trunk, state, order=1)
    plan2 = place_one_box(box2, trunk, state, order=2)

    assert plan1 is not None and plan2 is not None
    x1, y1, z1 = plan1.position
    x2, y2, z2 = plan2.position
    x_gap = max(x1 - (x2 + box2.width), x2 - (x1 + box1.width))
    y_gap = max(y1 - (y2 + box2.depth), y2 - (y1 + box1.depth))
    assert x_gap >= MARGIN - 1e-9 or y_gap >= MARGIN - 1e-9
