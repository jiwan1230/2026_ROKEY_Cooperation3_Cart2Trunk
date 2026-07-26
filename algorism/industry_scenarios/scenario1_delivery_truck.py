"""
scenario1_delivery_truck.py
산업 현장 시나리오 ① 택배 배송 트럭 (LIFO)
==============================================

[정책]
"나중에 내릴 물건을 먼저 싣는다" - 트럭 문을 열었을 때 첫 배송지 물건이 바로
손에 닿아야 한다. Box.delivery_stop(1=첫 배송지, 숫자가 클수록 나중 배송지)을
기준으로, **숫자가 큰(나중 배송지) 박스부터** 트렁크에 싣는다.

[왜 이 순서가 맞는지 - 코어 알고리즘(⑤ score_candidate)과의 관계]
코어의 점수화 함수는 "입구에서 먼 자리"를 우선한다(05_candidate_scoring.py
wall_a_term 참고) - 즉 먼저 배치되는 박스일수록 입구에서 먼(트렁크 깊숙한)
자리를 먼저 차지하고, 나중에 배치되는 박스일수록 이미 깊은 자리가 차 있어서
입구 쪽으로 밀린다. 그래서 "나중 배송지 박스를 먼저 배치"하면 자동으로
"나중 배송지 박스가 더 깊숙이, 첫 배송지 박스가 입구 쪽에" 놓이게 된다 - 코어의
점수 함수를 전혀 손대지 않고 ⑥(순서 결정)만 바꿔서 원하는 물리적 결과를 얻는다.

[코어와의 관계 - 뭘 재사용하고 뭘 새로 만들었나]
place_one_box(⑦)/classify_unloadable_reason(⑧)는 순서와 무관하게 동작하는
"자리 찾기 메커니즘"이라 그대로 재사용한다 (09_rescan_replan.py가 이미 같은
패턴 - 08_unloadable_reason.generate_loading_plan()을 복붙하지 않고 자체
루프에서 같은 부품을 재사용함). 이 파일이 새로 만드는 건 ⑥(순서 결정) 하나뿐.
"""

import sys
import pathlib
from importlib import import_module
from typing import List, Tuple

_ALGORISM_DIR = pathlib.Path(__file__).resolve().parent.parent
if str(_ALGORISM_DIR) not in sys.path:
    sys.path.insert(0, str(_ALGORISM_DIR))

_m03 = import_module("03_extreme_point_candidates")
_m07 = import_module("07_placement_plan")
_m08 = import_module("08_unloadable_reason")

Box = _m03.Box
ExtremePointState = _m03.ExtremePointState
place_one_box = _m07.place_one_box
classify_unloadable_reason = _m08.classify_unloadable_reason
UnloadableItem = _m08.UnloadableItem


def decide_loading_order_lifo_delivery(boxes: List["Box"]) -> List["Box"]:
    """
    delivery_stop이 큰(나중 배송지) 박스부터 정렬. delivery_stop이 없는(None)
    박스는 이 정책 대상이 아니므로 가장 낮은 우선순위(맨 뒤, 입구 쪽)로 보낸다 -
    "언제 내릴지 모르면 일단 접근하기 쉬운 곳에 둔다"는 안전한 기본값.
    """
    return sorted(boxes, key=lambda b: (b.delivery_stop is None, -(b.delivery_stop or 0)))


def generate_loading_plan_lifo_delivery(boxes: List["Box"], trunk) -> Tuple[list, list]:
    """08_unloadable_reason.generate_loading_plan()과 동일한 루프, ⑥만 교체."""
    order = decide_loading_order_lifo_delivery(boxes)
    state = ExtremePointState()

    plans = []
    unloadable = []
    order_counter = 1

    for box in order:
        plan = place_one_box(box, trunk, state, order_counter)
        if plan is not None:
            plans.append(plan)
            order_counter += 1
        else:
            reason = classify_unloadable_reason(box, trunk, state)
            unloadable.append(UnloadableItem(
                box_id=box.id, reason=reason,
                detail=f"{box.id}(부피 {box.volume*1000:.1f}L) - 사유: {reason.value}",
            ))

    return plans, unloadable


if __name__ == "__main__":
    _m02 = import_module("02_trunk_space_state")
    Trunk = _m02.Trunk

    trunk = Trunk(width=1.2, depth=0.8, height=0.6)
    boxes = [
        Box("정류장1_박스", 0.3, 0.25, 0.2, delivery_stop=1),
        Box("정류장2_박스", 0.3, 0.25, 0.2, delivery_stop=2),
        Box("정류장3_박스", 0.3, 0.25, 0.2, delivery_stop=3),
        Box("정류장4_박스", 0.3, 0.25, 0.2, delivery_stop=4),
    ]

    plans, unloadable = generate_loading_plan_lifo_delivery(boxes, trunk)
    print("=== 적재 순서 및 위치 (x가 작을수록 입구에 가까움) ===")
    for p in sorted(plans, key=lambda p: p.position[0]):
        print(f"  {p.box_id}: order={p.order}, x={p.position[0]:.3f}")
    for u in unloadable:
        print(f"  [미적재] {u.box_id}: {u.reason.value}")
