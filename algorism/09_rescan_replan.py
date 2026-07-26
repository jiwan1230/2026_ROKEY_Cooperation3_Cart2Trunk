"""
09_rescan_replan.py
⑨ 재스캔 후 재계획
=====================
상태: 🟡 트리거 정책 확정, ID 추적은 "추후 해결 예정" — 보수적 가정으로 임시 구현 가능

[지완 답변 - 트리거 정책 확정]
박스를 하나 놓을 때마다("PER_PLACEMENT") 무조건 재스캔해서 트렁크 공간을
갱신하고 다음 작업을 수행.

[준형 답변 - 박스 ID 지속성]
지금은 재스캔 후에도 같은 박스가 같은 id를 유지한다는 보장이 없지만,
"나중에 해결될 예정"이라는 답변을 받음. → 급하게 팀 이슈로 escalate할
필요는 없어졌고, 대신 지금은 "ID가 유지 안 된다"는 보수적 가정 하에
안전하게 동작하는 임시 버전으로 구현해둠. 1차 통합(7/22) 전에 진행
상황 한 번 더 확인하면 됨.

[보수적 가정이란]
"재스캔마다 트렁크를 처음부터 완전히 새로 인식한다"고 가정. 즉 이전에
어떤 박스를 처리했는지 기억하지 않고, 재스캔 결과에 있는 박스는 전부
"이미 트렁크 안에 있는 것"으로 취급해서 상태를 재구성한다. 이러면 ID가
바뀌어도 안전하지만, "이 박스는 아직 카트에 있는데 이미 처리한 걸로
착각" 같은 오류는 못 막는다 - ID 추적이 실제로 붙으면 이 부분을
더 정교하게 만들 수 있음.
"""

import logging
import sys, pathlib
from typing import List, Optional
from importlib import import_module

logger = logging.getLogger(__name__)

sys.path.insert(0, str(pathlib.Path(__file__).parent))
_m03 = import_module("03_extreme_point_candidates")
_m05 = import_module("05_candidate_scoring")
_m06 = import_module("06_loading_order_decision")
_m07 = import_module("07_placement_plan")
_m08 = import_module("08_unloadable_reason")

Box = _m03.Box
ExtremePointState = _m03.ExtremePointState
PlacedBox = _m03.PlacedBox
generate_loading_plan = _m08.generate_loading_plan
decide_loading_order = _m06.decide_loading_order
place_one_box = _m07.place_one_box
score_count_first = _m05.score_count_first
make_weighted_score_fn = _m05.make_weighted_score_fn

RESCAN_TRIGGER_POLICY = "PER_PLACEMENT"  # 지완 답변 확정: 박스 1개 놓을 때마다 1회


def rebuild_state_from_rescan(rescanned_placed_boxes: List["PlacedBox"]) -> "ExtremePointState":
    """
    재스캔 결과(이미 트렁크 안에 있는 박스들의 위치)로 ExtremePointState를
    다시 만든다. 후보 좌표는 각 박스를 register_placement()로 다시 등록하면서
    자동으로 재계산된다.

    지금은 "보수적 가정"으로 동작: rescanned_placed_boxes에 있는 건 전부
    이미 놓인 것으로 그대로 믿고 상태를 재구성한다. ID 추적이 붙으면
    "이전 상태와 비교해서 진짜 새로 놓인 것만 반영" 같은 정교한 버전으로
    바꿀 수 있음.
    """
    state = ExtremePointState()  # 완전히 빈 상태에서 다시 시작 (이전 state는 버림)
    for pb in rescanned_placed_boxes:
        # 재스캔으로 확인된 박스들을 하나씩 "이미 놓인 것"으로 재등록.
        # register_placement가 알아서 그 박스 기준 새 후보 3개도 같이 만들어주므로
        # 후보 집합을 따로 계산할 필요 없음.
        state.register_placement(pb)
    return state


def _run_strategy(order, trunk, rescanned_placed_boxes, score_fn, margin, allow_stacking,
                   allow_rotation=True, wall_margin=None, obstacle_margin=None, ceiling_margin=None,
                   entrance_margin=None):
    """정해진 순서(order)로 재스캔 상태를 새로 구성하고 하나씩 배치 시도하는 공통 루프."""
    state = rebuild_state_from_rescan(rescanned_placed_boxes)
    plans, unloadable = [], []
    # 순번은 "실제로 이미 놓인 카트 박스" 개수 다음부터 이어간다 - 장애물
    # (휠하우스 등, is_obstacle=True)은 로봇이 실은 게 아니라 원래 트렁크에
    # 있던 고정 구조물이라 순번에서 빼야 한다. 버그로 발견됨: 이걸 안 뺐더니
    # (지금 파이프라인에서 rescanned_placed_boxes가 사실상 항상 장애물뿐이라)
    # 첫 카트 박스가 order=1이 아니라 order=(장애물 개수+1)로 나왔었다 - Task
    # JSON의 sequence 필드에도 그대로 나가는 값이라 MSI2에 잘못 전달될 뻔함.
    already_placed_cart_boxes = sum(1 for pb in rescanned_placed_boxes if not pb.box.is_obstacle)
    order_counter = already_placed_cart_boxes + 1

    for box in order:
        plan = place_one_box(box, trunk, state, order_counter, score_fn=score_fn, margin=margin,
                              allow_stacking=allow_stacking, allow_rotation=allow_rotation,
                              wall_margin=wall_margin, obstacle_margin=obstacle_margin,
                              ceiling_margin=ceiling_margin, entrance_margin=entrance_margin)
        if plan is not None:
            plans.append(plan)
            order_counter += 1
        else:
            reason = _m08.classify_unloadable_reason(box, trunk, state)
            logger.info(f"[{box.id}] 미적재 - 사유: {reason.value}")
            unloadable.append(_m08.UnloadableItem(
                box_id=box.id, reason=reason,
                detail=f"{box.id} - 사유: {reason.value}",
            ))

    return plans, unloadable


def replan_after_rescan(
    remaining_boxes: List["Box"], trunk, rescanned_placed_boxes: List["PlacedBox"],
    mode: str = "large_first", margin: Optional[float] = None, allow_stacking: bool = False,
    allow_rotation: bool = True, wall_margin: Optional[float] = None,
    obstacle_margin: Optional[float] = None, ceiling_margin: Optional[float] = None,
    entrance_margin: Optional[float] = None,
    entrance_preference: float = 1.0, contact_preference: float = 1.0, height_preference: float = 1.0,
    fixed_order: Optional[List[str]] = None,
):
    """
    재스캔 트리거(PER_PLACEMENT)가 발생할 때마다 호출.
    보수적 가정 버전: 재스캔으로 확인된 박스들로 상태를 다시 만들고,
    remaining_boxes(카트에 남은 것으로 알려진 박스)에 대해 이어서 계획한다.

    모든 파라미터는 08_unloadable_reason.generate_loading_plan()과 같은 뜻
    (그쪽 docstring 참고) - trunk_map_planner_node.py(ROS2)가 실제로 호출하는
    게 08의 generate_loading_plan()이 아니라 이 함수라서, 사용자가 고른
    설정이 로봇까지 실제로 전달되려면 여기도 08과 똑같이 지원해야 한다.
    mode="count_first"의 best-of-two 로직도, fixed_order가 주어지면 그 로직
    자체를 건너뛰는 것도 08과 동일하게 여기서 수행한다.
    """
    weighted_fn = None
    if entrance_preference != 1.0 or contact_preference != 1.0 or height_preference != 1.0:
        weighted_fn = make_weighted_score_fn(entrance_preference, contact_preference, height_preference)

    extra_kwargs = dict(allow_rotation=allow_rotation, wall_margin=wall_margin,
                         obstacle_margin=obstacle_margin, ceiling_margin=ceiling_margin,
                         entrance_margin=entrance_margin)

    if fixed_order is not None:
        order = decide_loading_order(remaining_boxes, fixed_order=fixed_order)
        return _run_strategy(order, trunk, rescanned_placed_boxes, weighted_fn, margin, allow_stacking, **extra_kwargs)

    if mode == "count_first":
        order_small_first = decide_loading_order(remaining_boxes, mode="count_first")
        score_a = weighted_fn if weighted_fn is not None else score_count_first
        plans_a, unloadable_a = _run_strategy(
            order_small_first, trunk, rescanned_placed_boxes, score_a, margin, allow_stacking, **extra_kwargs)
        order_large_first = decide_loading_order(remaining_boxes, mode="large_first")
        plans_b, unloadable_b = _run_strategy(
            order_large_first, trunk, rescanned_placed_boxes, weighted_fn, margin, allow_stacking, **extra_kwargs)

        if len(plans_b) > len(plans_a):
            logger.info(
                f"count_first: 작은 것부터 전략({len(plans_a)}개)보다 큰 것부터 전략"
                f"({len(plans_b)}개)이 더 많이 담겨 그쪽을 채택"
            )
            return plans_b, unloadable_b
        return plans_a, unloadable_a

    # 남은 박스만 대상으로 [⑥][⑦]을 그대로 다시 수행 (기존 배치는 이미 state에 반영됨)
    order = decide_loading_order(remaining_boxes, mode=mode)
    return _run_strategy(order, trunk, rescanned_placed_boxes, weighted_fn, margin, allow_stacking, **extra_kwargs)


# TODO: 트리거 신호 연동 (트리거 정책은 확정, 신호 자체 연동은 미구현)
# def on_rescan_trigger(remaining_boxes, trunk, rescanned_placed_boxes):
#     """박스 1개 배치 완료 시점마다 지완 쪽에서 신호를 주면 호출."""
#     return replan_after_rescan(remaining_boxes, trunk, rescanned_placed_boxes)


if __name__ == "__main__":
    _m02 = import_module("02_trunk_space_state")
    Trunk = _m02.Trunk

    trunk = Trunk(width=1.5, depth=1.5, height=0.9)
    already_placed = [PlacedBox(box=Box("Medium", 0.4, 0.3, 0.25), x=0.0, y=0.0, z=0.0)]
    remaining = [Box("Small", 0.30, 0.20, 0.15), Box("Large", 0.50, 0.35, 0.30)]

    plans, unloadable = replan_after_rescan(remaining, trunk, already_placed)
    for p in plans:
        print("PLACED:", p)
    for u in unloadable:
        print("UNLOADABLE:", u)