"""
test_06_loading_order_decision.py
⑥ decide_loading_order()가 "카트 위에서 뭐가 뭘 누르고 있는지"(rests_on_id)를
반영해서, 물리적으로 불가능한 픽업 순서를 내지 않는지 검증.

배경: 7/20 결론("Vision은 지금 보이는 박스만 준다 -> 코드 수정 불필요")은 실제
비전 데이터(all_boxes_corners_*.json)로 확인해보니 틀렸다 - 비전이 발전해서 지금은
깔려있는 박스도 "뭐 위에 얹혀있는지" 관계 정보(support_type 등)와 함께 한 번에
같이 인식한다. 그래서 순서 결정 로직이 이 관계를 직접 반영해야 한다.

원칙: 아무것도 위에 안 얹힌(픽업 가능한) 박스들 중에서만 매 단계 부피가 큰 걸
고른다 - 물리적으로 불가능한 순서(밑에 깔린 걸 먼저 집기)는 아예 후보에서 제외.
"""
import sys, pathlib
from importlib import import_module

_m03 = import_module("cart2trunk_planning.core.extreme_point_candidates")
_m06 = import_module("cart2trunk_planning.core.loading_order_decision")

Box = _m03.Box
decide_loading_order = _m06.decide_loading_order


def test_boxes_without_rests_on_id_still_sort_by_volume_desc():
    """회귀 방지 - rests_on_id가 전부 없으면(기존 방식) 그대로 부피 큰 순."""
    boxes = [Box("Small", 0.3, 0.2, 0.15), Box("Large", 0.5, 0.35, 0.3), Box("Medium", 0.4, 0.3, 0.25)]
    order = decide_loading_order(boxes)
    assert [b.id for b in order] == ["Large", "Medium", "Small"]


def test_box_resting_on_bigger_box_is_picked_first():
    """
    작은 박스가 큰 박스 위에 얹혀 있으면, 부피만 보면 큰 박스가 먼저여야 하지만
    물리적으로는 위에 있는 작은 박스부터 집어야 한다.
    """
    bottom = Box("Bottom", width=0.5, depth=0.5, height=0.3)  # 부피 훨씬 큼
    top = Box("Top", width=0.1, depth=0.1, height=0.1, rests_on_id="Bottom")  # Bottom 위에 얹힘

    order = decide_loading_order([bottom, top])

    assert [b.id for b in order] == ["Top", "Bottom"]


def test_three_level_chain_orders_top_to_bottom():
    """C가 B 위에, B가 A 위에 있으면 픽업 순서는 C -> B -> A여야 한다."""
    a = Box("A", width=0.5, depth=0.5, height=0.3)
    b = Box("B", width=0.3, depth=0.3, height=0.2, rests_on_id="A")
    c = Box("C", width=0.1, depth=0.1, height=0.1, rests_on_id="B")

    order = decide_loading_order([a, b, c])

    assert [x.id for x in order] == ["C", "B", "A"]


def test_multiple_boxes_resting_on_same_box_still_prefer_larger_volume_between_themselves():
    """
    카트 재적재 시나리오 재현: 파란 박스 2개(Blue1, Blue2)가 둘 다 초록 박스(Green)
    위에 얹혀 있으면, 부피 순 원칙은 Blue1/Blue2끼리 비교할 때만 적용되고 Green은
    Blue1/Blue2보다 항상 나중이어야 한다 (실제로 발견된 순서 뒤바뀜 버그 재현).
    """
    green = Box("Cart_Green", width=0.20, depth=0.18, height=0.15)
    blue1 = Box("Cart_Blue1", width=0.09, depth=0.10, height=0.12, rests_on_id="Cart_Green")
    blue2 = Box("Cart_Blue2", width=0.12, depth=0.14, height=0.12, rests_on_id="Cart_Green")  # Blue1보다 부피 큼

    order = decide_loading_order([green, blue1, blue2])

    assert order[-1].id == "Cart_Green"  # Green은 항상 맨 마지막(맨 아래라서 제일 늦게 집힘)
    assert [b.id for b in order[:2]] == ["Cart_Blue2", "Cart_Blue1"]  # 둘 사이에선 부피 큰 Blue2 먼저


def test_mode_count_first_sorts_ascending():
    """mode="count_first"면 부피 오름차순(작은 것부터) - 큰 거 우선(large_first, 기본값)과 반대."""
    boxes = [Box("Small", 0.3, 0.2, 0.15), Box("Large", 0.5, 0.35, 0.3), Box("Medium", 0.4, 0.3, 0.25)]
    order = decide_loading_order(boxes, mode="count_first")
    assert [b.id for b in order] == ["Small", "Medium", "Large"]


def test_mode_count_first_still_respects_pickup_constraint():
    """count_first 모드에서도 rests_on_id 픽업 순서 제약(물리적으로 위부터 집기)은 그대로 지켜야 한다."""
    bottom = Box("Bottom", width=0.1, depth=0.1, height=0.1)  # 부피 작음
    top = Box("Top", width=0.5, depth=0.5, height=0.3, rests_on_id="Bottom")  # 부피 큼, Bottom 위에 얹힘

    order = decide_loading_order([bottom, top], mode="count_first")

    # count_first면 부피만 보면 Bottom(작음)이 먼저겠지만, Top이 위에 얹혀 있어서
    # 물리적으로 Top부터 집어야 한다 - 픽업 제약이 부피 정렬보다 우선.
    assert [b.id for b in order] == ["Top", "Bottom"]
