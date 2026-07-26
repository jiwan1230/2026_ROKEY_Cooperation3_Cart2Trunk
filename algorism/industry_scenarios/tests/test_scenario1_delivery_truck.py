"""
test_scenario1_delivery_truck.py
시나리오1(택배 배송 트럭 LIFO) - 순서 결정 + 실제 배치 위치까지 검증.
"""
import sys, pathlib
from importlib import import_module

_ALGORISM_DIR = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_ALGORISM_DIR))
sys.path.insert(0, str(_ALGORISM_DIR / "industry_scenarios"))

_m02 = import_module("02_trunk_space_state")
_m03 = import_module("03_extreme_point_candidates")
scenario1 = import_module("scenario1_delivery_truck")

Trunk = _m02.Trunk
Box = _m03.Box
decide_loading_order_lifo_delivery = scenario1.decide_loading_order_lifo_delivery
generate_loading_plan_lifo_delivery = scenario1.generate_loading_plan_lifo_delivery


def test_later_stops_are_ordered_first():
    """정류장 번호가 큰(나중에 내리는) 박스부터 순서가 매겨져야 한다."""
    boxes = [
        Box("Stop1", 0.3, 0.2, 0.15, delivery_stop=1),
        Box("Stop3", 0.3, 0.2, 0.15, delivery_stop=3),
        Box("Stop2", 0.3, 0.2, 0.15, delivery_stop=2),
    ]
    order = decide_loading_order_lifo_delivery(boxes)
    assert [b.id for b in order] == ["Stop3", "Stop2", "Stop1"]


def test_first_stop_box_ends_up_closer_to_entrance():
    """
    실제로 트렁크에 배치했을 때, 첫 정류장(delivery_stop=1) 박스가 나중 정류장
    박스보다 입구(x=0)에 더 가까운 자리에 놓여야 한다 - 문 열자마자 바로 손이
    닿아야 순서대로 빨리 내릴 수 있다.
    """
    trunk = Trunk(width=1.0, depth=0.6, height=0.5)
    boxes = [
        Box("Stop1", 0.25, 0.25, 0.2, delivery_stop=1),
        Box("Stop2", 0.25, 0.25, 0.2, delivery_stop=2),
        Box("Stop3", 0.25, 0.25, 0.2, delivery_stop=3),
    ]
    plans, unloadable = generate_loading_plan_lifo_delivery(boxes, trunk)

    assert len(unloadable) == 0
    pos_by_id = {p.box_id: p.position for p in plans}
    x_stop1 = pos_by_id["Stop1"][0]
    x_stop3 = pos_by_id["Stop3"][0]
    assert x_stop1 < x_stop3, (
        f"Stop1(x={x_stop1})이 Stop3(x={x_stop3})보다 입구에 가까워야 하는데 아님"
    )
