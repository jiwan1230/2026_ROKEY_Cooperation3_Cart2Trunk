"""
08_unloadable_reason.py
⑧ 미적재 판단
===============
상태: 🟢 완료·확정

[비유] 이사 갈 때 가구를 못 넣는 상황 세 가지와 같다.
    SIZE_EXCEEDS_TRUNK            : 소파가 현관문보다 커서, 집이 비어있어도 절대 못 들어감
    INSUFFICIENT_REMAINING_VOLUME : 문은 통과하는데 집 안 남은 공간 자체가 이미 꽉 참
    NO_VALID_CANDIDATE_POSITION   : 남은 공간을 합치면 충분한데 가구 배치 모양 때문에 못 들어감
                                     (이 경우만 재배치하면 들어갈 수도 있음)

1번·2번은 재배치를 시도해봐야 소용없어서 바로 담당자 호출.
3번만 재배치(reshuffle) 시도 가치가 있음 (decide_reshuffle_or_call 참고).

이 파일 하단에는 ⑥⑦⑧을 하나로 묶은 통합 파이프라인
generate_loading_plan()도 함께 있음.
"""

import logging
import sys, pathlib
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Tuple
from importlib import import_module

logger = logging.getLogger(__name__)

sys.path.insert(0, str(pathlib.Path(__file__).parent))
_m03 = import_module("03_extreme_point_candidates")
_m05 = import_module("05_candidate_scoring")
_m06 = import_module("06_loading_order_decision")
_m07 = import_module("07_placement_plan")
_m18 = import_module("18_rotation")

Box = _m03.Box
ExtremePointState = _m03.ExtremePointState
fits_dims_any_rotation = _m18.fits_dims_any_rotation
decide_loading_order = _m06.decide_loading_order
place_one_box = _m07.place_one_box
PlacementPlan = _m07.PlacementPlan
score_count_first = _m05.score_count_first
make_weighted_score_fn = _m05.make_weighted_score_fn


class UnloadableReason(Enum):
    SIZE_EXCEEDS_TRUNK = "SIZE_EXCEEDS_TRUNK"
    INSUFFICIENT_REMAINING_VOLUME = "INSUFFICIENT_REMAINING_VOLUME"
    NO_VALID_CANDIDATE_POSITION = "NO_VALID_CANDIDATE_POSITION"


class LoadingAction(Enum):
    RESHUFFLE = "RESHUFFLE"
    CALL_OPERATOR = "CALL_OPERATOR"


@dataclass
class UnloadableItem:
    box_id: str
    reason: UnloadableReason
    detail: str


def _remaining_free_volume(trunk, state: "ExtremePointState") -> float:
    """단순 근사치: 트렁크 전체 부피 - 이미 놓인 박스 부피 합."""
    used = sum(p.box.volume for p in state.placed)
    return trunk.width * trunk.depth * trunk.height - used


def classify_unloadable_reason(box: "Box", trunk, state: "ExtremePointState") -> UnloadableReason:
    # 검사 순서가 중요: 셋 다 걸릴 수 있어도 "가장 근본적인 이유"부터 확인해서 반환한다.
    # 1순위: 박스 자체가 트렁크보다 큼 (자리 배치와 무관하게 애초에 불가능) - ⑱ 90도
    # 회전한 자세까지 감안해서, 둘 다 안 맞을 때만 진짜 SIZE_EXCEEDS_TRUNK로 본다.
    if not fits_dims_any_rotation(box, trunk):
        return UnloadableReason.SIZE_EXCEEDS_TRUNK

    # 2순위: 남은 부피 자체가 박스 부피보다 적음 (배치를 어떻게 하든 물리적으로 불가능)
    if _remaining_free_volume(trunk, state) < box.volume:
        return UnloadableReason.INSUFFICIENT_REMAINING_VOLUME

    # 여기까지 왔으면 이론상 공간은 충분한데 현재 배치 모양 때문에 못 놓는 것
    # -> 재배치(reshuffle)하면 들어갈 가능성이 있는 케이스
    return UnloadableReason.NO_VALID_CANDIDATE_POSITION


def decide_reshuffle_or_call(reason: UnloadableReason) -> LoadingAction:
    """
    사유 코드에 따라 재배치를 시도할지, 바로 담당자를 호출할지 판단.
    SIZE_EXCEEDS_TRUNK / INSUFFICIENT_REMAINING_VOLUME -> 재배치해도 소용없음 -> 바로 호출
    NO_VALID_CANDIDATE_POSITION -> 배치 모양 문제라 재배치 시도 가치 있음
    """
    if reason == UnloadableReason.NO_VALID_CANDIDATE_POSITION:
        return LoadingAction.RESHUFFLE
    return LoadingAction.CALL_OPERATOR


# ---------------------------------------------------------------------------
# ⑥⑦⑧ 통합 파이프라인
# ---------------------------------------------------------------------------

def _run_strategy(
    order: List["Box"], trunk, score_fn, margin: Optional[float], allow_stacking: bool,
    allow_rotation: bool = True, wall_margin: Optional[float] = None,
    obstacle_margin: Optional[float] = None, ceiling_margin: Optional[float] = None,
    entrance_margin: Optional[float] = None,
) -> Tuple[List["PlacementPlan"], List[UnloadableItem]]:
    """정해진 순서(order)를 [⑦] 하나씩 배치 시도하고 [⑧] 실패 사유를 분류하는 공통 루프."""
    state = ExtremePointState()          # 빈 트렁크 상태(후보는 (0,0,0) 하나)에서 시작
    plans: List["PlacementPlan"] = []
    unloadable: List[UnloadableItem] = []
    order_counter = 1

    for box in order:
        # [⑦] 현재 state 기준으로 이 박스의 최적 자리를 찾아 배치 시도
        plan = place_one_box(box, trunk, state, order_counter, score_fn=score_fn, margin=margin,
                              allow_stacking=allow_stacking, allow_rotation=allow_rotation,
                              wall_margin=wall_margin, obstacle_margin=obstacle_margin,
                              ceiling_margin=ceiling_margin, entrance_margin=entrance_margin)
        if plan is not None:
            plans.append(plan)
            order_counter += 1  # 실제로 배치된 것만 순번을 늘림
        else:
            # 자리를 못 찾음 -> [⑧] 왜 못 찾았는지 사유를 분류.
            # 이 박스만 건너뛰고 계속 돌아 다음 박스는 계속 시도한다 (전체 중단 X)
            reason = classify_unloadable_reason(box, trunk, state)
            logger.info(f"[{box.id}] 미적재 - 사유: {reason.value}")
            unloadable.append(UnloadableItem(
                box_id=box.id, reason=reason,
                detail=f"{box.id}(부피 {box.volume*1000:.1f}L) - 사유: {reason.value}",
            ))

    return plans, unloadable


def generate_loading_plan(
    boxes: List["Box"], trunk, mode: str = "large_first", margin: Optional[float] = None,
    allow_stacking: bool = False, allow_rotation: bool = True,
    wall_margin: Optional[float] = None, obstacle_margin: Optional[float] = None,
    ceiling_margin: Optional[float] = None, entrance_margin: Optional[float] = None,
    entrance_preference: float = 1.0, contact_preference: float = 1.0, height_preference: float = 1.0,
    fixed_order: Optional[List[str]] = None,
) -> Tuple[List["PlacementPlan"], List[UnloadableItem]]:
    """
    boxes, trunk를 받아서:
      1) [⑥] 정해진 순서로 시도 순서 고정
      2) [⑦] 순서대로 하나씩 Extreme Point 최적 자리 찾아 배치
      3) 자리를 못 찾으면 [⑧] 사유 코드 부여 (다음 박스는 계속 시도)

    [mode] 사용자가 적재 정책을 고를 수 있게 하는 스위치.
    - "large_first"(기본값): 부피 큰 것부터, 코어 기본 점수(입구 접근성 우선) -
      지금까지의 동작과 완전히 동일 (하위 호환).
    - "count_first": 최대한 많은 개수를 담는 게 목표인 현장용. "작은 것부터 +
      footprint growth 최소화 점수(⑤ score_count_first)" 전략과 "큰 것부터 +
      기본 점수(=large_first와 동일)" 전략을 둘 다 계산해서 더 많이 담기는
      쪽을 채택한다(best-of-two).
        - 이유(실측으로 확인된 문제): "항상 작은 것부터"만 쓰면, 박스 크기가
          고르게 섞여 있을 때 작은 박스들이 공간을 잘게 조각내버려서 맨 뒤로
          밀린 큰 박스가 들어갈 자리를 잃는 역효과가 있었다(큰 거 우선 모드
          보다 오히려 더 적게 담김 - 예: 10개 중 large_first 10개, 이전
          count_first 9개). large_first 전략이 항상 후보 두 개 중 하나로
          포함되므로, count_first가 large_first보다 적게 담는 일은 이제 없다.
        - 반대로 작은 박스가 많고 큰 박스는 적은 세트에서는 "작은 것부터"
          전략이 여전히 이긴다(실측: large_first 4/8 vs count_first 6/8, 지금도
          유지됨 - 정확한 수치는 ③ 후보 생성기가 바뀔 때마다(예: 입구쪽 벽
          플러시 후보 추가) greedy 선택 경로가 살짝 달라져 조금씩 변할 수 있음,
          "count_first가 large_first보다 못하지 않다"는 게 불변식).

    [margin] 벽/박스 최소 간격도 사용자가 조절 가능 (예: 냉동 물류는 냉기 순환용
    으로 훨씬 큰 간격 필요). None(기본값)이면 17_margin_check.MARGIN 그대로.

    [allow_stacking] 트렁크 1층이 꽉 찼을 때 2층·3층으로 자동으로 쌓을지 여부.
    False(기본값)면 지금까지처럼 1층 전용(하위 호환). True로 켜면 ⑤ 점수 기준이
    원래 "낮은 자리 우선"이라, 바닥에 자리가 있는 동안은 대체로 바닥부터 채우는
    쪽으로 기울지만 절대 규칙은 아니다(입구 접근성 등 다른 점수 요소가 더 크면
    바닥에 자리가 남아있어도 위층을 고를 수 있음) - "몇 층까지"를 따로 지정할
    필요 없이 트렁크 높이가 허락하는 한 필요한 만큼만 쌓인다. ⑬(받침 비율)·
    ⑮(상단 여유 공간)가 이미 안전 기준을 지키면서 배치되는지 확인해준다.

    [allow_rotation/wall_margin/obstacle_margin/ceiling_margin] ⑦
    place_one_box()의 같은 이름 파라미터를 그대로 전달만 한다 - 자세한 설명은
    거기 docstring 참고.

    [entrance_margin] "트렁크 입구 여유 거리" 슬라이더 - wall_margin(정해졌으면
    그 값, 아니면 margin) 위에서 입구 쪽 벽만 한 번 더 덮어쓴다. 17_margin_check.
    has_sufficient_margin 참고.

    [entrance_preference/contact_preference/height_preference] "적재 우선순위"
    슬라이더 3축(입구 우선↔깊은 위치 우선 / 공간활용률 우선↔안정성 우선 /
    바닥부터 채우기 강도, ⑤ make_weighted_score_fn 참고). 셋 다 기본값(1.0,
    1.0, 1.0)이 아니면, mode가 원래 고르는 점수 함수(large_first의 기본
    score_candidate, count_first의 score_count_first) 대신 이 가중치로 만든
    점수 함수를 쓴다 - 사용자가 슬라이더를 직접 조정했다는 건 mode의 기본
    전략보다 그 선호를 우선하겠다는 뜻으로 본다. 셋 다 기본값이면 지금까지와
    완전히 동일(하위 호환).

    [fixed_order] "적재 순서 고정 여부" - 박스 id 리스트를 주면 mode의 순서
    결정을 완전히 건너뛰고 이 순서로만 시도한다(⑥ decide_loading_order 참고,
    픽업 순서 제약은 여전히 지켜짐). mode="count_first"의 best-of-two 로직도
    건너뛴다 - 순서가 고정되면 "작은 것부터 vs 큰 것부터" 비교 자체가 의미
    없어서다. None(기본값)이면 지금까지와 동일(하위 호환).
    """
    weighted_fn = None
    if entrance_preference != 1.0 or contact_preference != 1.0 or height_preference != 1.0:
        weighted_fn = make_weighted_score_fn(entrance_preference, contact_preference, height_preference)

    extra_kwargs = dict(allow_rotation=allow_rotation, wall_margin=wall_margin,
                         obstacle_margin=obstacle_margin, ceiling_margin=ceiling_margin,
                         entrance_margin=entrance_margin)

    if fixed_order is not None:
        order = decide_loading_order(boxes, fixed_order=fixed_order)
        return _run_strategy(order, trunk, weighted_fn, margin, allow_stacking, **extra_kwargs)

    if mode == "count_first":
        # 후보 A: 작은 것부터 + 공간재사용 점수 (기존 count_first) - 가중치 슬라이더가
        # 조정됐으면 그쪽을 우선
        order_small_first = decide_loading_order(boxes, mode="count_first")
        score_a = weighted_fn if weighted_fn is not None else score_count_first
        plans_a, unloadable_a = _run_strategy(order_small_first, trunk, score_a, margin, allow_stacking, **extra_kwargs)
        # 후보 B: 큰 것부터 + 기본 점수(또는 가중치 점수) - large_first와 동일한 전략
        order_large_first = decide_loading_order(boxes, mode="large_first")
        plans_b, unloadable_b = _run_strategy(order_large_first, trunk, weighted_fn, margin, allow_stacking, **extra_kwargs)

        if len(plans_b) > len(plans_a):
            logger.info(
                f"count_first: 작은 것부터 전략({len(plans_a)}개)보다 큰 것부터 전략"
                f"({len(plans_b)}개)이 더 많이 담겨 그쪽을 채택"
            )
            return plans_b, unloadable_b
        return plans_a, unloadable_a

    order = decide_loading_order(boxes, mode=mode)  # [⑥] 모드에 맞는 순서로 정렬
    return _run_strategy(order, trunk, weighted_fn, margin, allow_stacking, **extra_kwargs)


if __name__ == "__main__":
    _m02 = import_module("02_trunk_space_state")
    Trunk = _m02.Trunk

    boxes = [
        Box("Small", 0.30, 0.20, 0.15),
        Box("Medium", 0.40, 0.30, 0.25),
        Box("Large", 0.50, 0.35, 0.30),
    ]
    trunk = Trunk(width=0.57, depth=1.12, height=0.25)  # 실제 트렁크 실측값

    plans, unloadable = generate_loading_plan(boxes, trunk)
    for p in plans:
        print("PLACED:", p)
    for u in unloadable:
        print("UNLOADABLE:", u, "-> 판단:", decide_reshuffle_or_call(u.reason).value)
