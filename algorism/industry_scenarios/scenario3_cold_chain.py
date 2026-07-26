"""
scenario3_cold_chain.py
산업 현장 시나리오 ③ 냉동/냉장 물류 (냉기 순환 마진)
==============================================

[정책]
냉동/냉장 컨테이너는 박스 사이·박스와 벽 사이로 찬 공기가 흘러야 전체가
고르게 냉각된다 - 로봇 실행 오차 흡수용 기본 마진(⑰, 2cm)보다 훨씬 큰 간격이
필요하다. 순서·점수 기준은 코어 기본값 그대로 두고(냉기 순환은 "어디에
놓는지"가 아니라 "얼마나 띄우는지"의 문제라서), 마진 값만 바꾼다.

[코어와의 관계]
07_placement_plan.place_one_box()에 이번 시나리오 작업 중 추가한 margin
확장점(margin=None이면 17_margin_check.MARGIN 그대로, 값을 넘기면 그걸로
덮어씀)을 그대로 쓴다. 순서 결정(⑥)·점수화(⑤)·자리 찾기 메커니즘(③④⑦⑧)은
전부 코어 그대로 - 이 시나리오가 바꾸는 건 상수 하나(COLD_CHAIN_MARGIN)뿐이라
세 시나리오 중 가장 단순하다.
"""

import sys
import pathlib
from importlib import import_module
from typing import List, Tuple

_ALGORISM_DIR = pathlib.Path(__file__).resolve().parent.parent
if str(_ALGORISM_DIR) not in sys.path:
    sys.path.insert(0, str(_ALGORISM_DIR))

_m03 = import_module("03_extreme_point_candidates")
_m06 = import_module("06_loading_order_decision")
_m07 = import_module("07_placement_plan")
_m08 = import_module("08_unloadable_reason")

Box = _m03.Box
ExtremePointState = _m03.ExtremePointState
decide_loading_order = _m06.decide_loading_order
place_one_box = _m07.place_one_box
classify_unloadable_reason = _m08.classify_unloadable_reason
UnloadableItem = _m08.UnloadableItem

# 냉기가 흐를 수 있는 최소 간격 (팀/현장 협의로 조정 가능한 값 - 일반적인 냉동
# 컨테이너 공기 순환 가이드라인 기준 5cm로 잡음). 기본 마진(2cm)의 2.5배.
COLD_CHAIN_MARGIN = 0.05


def generate_loading_plan_cold_chain(boxes: List["Box"], trunk) -> Tuple[list, list]:
    """08_unloadable_reason.generate_loading_plan()과 동일한 루프, 마진만 확대."""
    order = decide_loading_order(boxes)
    state = ExtremePointState()

    plans = []
    unloadable = []
    order_counter = 1

    for box in order:
        plan = place_one_box(box, trunk, state, order_counter, margin=COLD_CHAIN_MARGIN)
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

    trunk = Trunk(width=1.2, depth=0.6, height=0.5)
    boxes = [Box(f"냉동박스{i}", 0.3, 0.25, 0.2) for i in range(3)]

    default_plans, default_unloadable = _m08.generate_loading_plan(boxes, trunk)
    cold_plans, cold_unloadable = generate_loading_plan_cold_chain(boxes, trunk)

    print(f"[기본 마진(2cm)] 배치 {len(default_plans)}/{len(boxes)}, 미적재 {len(default_unloadable)}")
    for p in sorted(default_plans, key=lambda p: p.position[0]):
        print(f"  {p.box_id}: x=[{p.position[0]:.3f}, {p.position[0]+p.dimensions[0]:.3f}]")

    print(f"[냉기순환 마진(5cm)] 배치 {len(cold_plans)}/{len(boxes)}, 미적재 {len(cold_unloadable)}")
    for p in sorted(cold_plans, key=lambda p: p.position[0]):
        print(f"  {p.box_id}: x=[{p.position[0]:.3f}, {p.position[0]+p.dimensions[0]:.3f}]")
