"""
07_placement_plan.py
⑦ 적재 위치 결정 (PlacementPlan)
==================================
상태: 🟢 완료·확정

③(후보 생성) + ④(유효성 검사) + ⑤(점수화)를 하나로 묶어서,
박스 하나를 실제로 어디에 놓을지 결정하고 PlacementPlan을 만든다.

    1. 현재 후보 좌표 집합에서 [④] 유효한 후보만 추림
    2. 유효한 후보들을 [⑤] 점수화해서 가장 낮은 점수(가장 좋은 자리) 선택
    3. 그 좌표에 배치하고 [③] 새 후보 3개 추가
"""

import logging
import math
import sys, pathlib
from dataclasses import dataclass
from typing import List, Optional, Tuple
from importlib import import_module

logger = logging.getLogger(__name__)

sys.path.insert(0, str(pathlib.Path(__file__).parent))
_m03 = import_module("03_extreme_point_candidates")
_m05 = import_module("05_candidate_scoring")
_m13 = import_module("13_support_check")
_m15 = import_module("15_overhead_clearance_check")
_m17 = import_module("17_margin_check")
_m18 = import_module("18_rotation")

Box = _m03.Box
PlacedBox = _m03.PlacedBox
ExtremePointState = _m03.ExtremePointState
generate_wall_flush_candidates = _m03.generate_wall_flush_candidates
generate_box_flush_candidates = _m03.generate_box_flush_candidates
is_candidate_valid_with_stacking = _m13.is_candidate_valid_with_stacking
has_overhead_clearance = _m15.has_overhead_clearance
has_clear_approach_path = _m15.has_clear_approach_path
has_sufficient_margin = _m17.has_sufficient_margin
MARGIN = _m17.MARGIN
rotate_box = _m18.rotate_box
score_candidate = _m05.score_candidate


@dataclass
class PlacementPlan:
    """협동로봇 Pick & Place 단계(BR-10)에 그대로 전달 가능한 정식 결과 구조."""
    box_id: str
    order: int
    position: Tuple[float, float, float]      # 트렁크 로컬 좌표 (x, y, z)
    dimensions: Tuple[float, float, float]     # (width, depth, height) - 실제로 놓인 자세 기준
    score: float
    touches: int
    rotated: bool = False  # True면 ⑱ 90도 회전(가로/세로 교환)된 자세로 놓인 것
    # MSI2(민결)에 넘길 최종 목표 회전각(라디안) - box.initial_yaw(①에서 넘어온
    # 카트 위 초기 자세) + (rotated면 90도, 아니면 0). "HMI 화면 설계 가이드라인"
    # 문서의 Task 출력 필수 필드 "Target Yaw"에 대응.
    target_yaw: float = 0.0


def place_one_box(
    box: "Box", trunk, state: "ExtremePointState", order: int,
    allow_stacking: bool = False, score_fn=None, margin: Optional[float] = None,
    extra_validity_fn=None, allow_rotation: bool = True,
    wall_margin: Optional[float] = None, obstacle_margin: Optional[float] = None,
    ceiling_margin: Optional[float] = None, entrance_margin: Optional[float] = None,
) -> Optional["PlacementPlan"]:
    """
    현재 상태(state)에서 box 하나를 놓을 최선의 자리를 찾아 배치한다.
    자리가 없으면 None (이 경우 ⑧ 미적재 판단으로 넘어가야 함).

    allow_stacking=False(기본값)면 z>0 후보(박스 위에 놓는 자리)는 ⑬에서
    무조건 거부되어 지금의 1층 전용 동작과 동일하게 동작한다.

    [⑱ 회전] 먼저 정자세로 시도하고, 자리가 없을 때만 90도 돌린 자세(가로/세로
    교환, 높이는 그대로 - 로봇이 눕히거나 뒤집는 건 불가능)로 한 번 더 시도한다.
    정자세가 이미 되면 굳이 돌리지 않는다 (회전은 그리퍼 동작이 하나 더 필요함).

    [score_fn] "어떤 자리가 좋은 자리인지" 기준은 현장마다 다르다(예: 창고는
    빈틈 없이 빽빽하게, 트렁크는 입구 접근성 우선) - industry_scenarios/의
    시나리오별 정책이 이 자리에 자기 점수 함수를 끼워 넣을 수 있게 열어둔
    확장점이다. None(기본값)이면 지금까지처럼 05_candidate_scoring.
    score_candidate를 그대로 쓴다 (하위 호환 - 코어 자체 동작은 안 바뀜).
    score_candidate와 같은 시그니처 (x, y, z, box, trunk, placed) -> (score, touches)
    를 따라야 한다.

    [margin] 벽/박스와 최소 얼마나 띄울지도 현장마다 다르다(예: 냉동 물류는
    냉기 순환용으로 로봇 오차 흡수용 기본값보다 훨씬 큰 간격이 필요). None
    (기본값)이면 17_margin_check.MARGIN을 그대로 쓴다 (하위 호환).

    [extra_validity_fn] 코어에 없는 완전히 새로운 하드 컷 규칙이 필요한 현장용
    확장점(예: 위험물 비호환 인접 금지). 기존 ④⑬⑮⑯⑰ 체인 뒤에 AND로 추가로
    끼워 넣는다 - None(기본값)이면 아무 것도 추가로 거르지 않는다(하위 호환).
    has_sufficient_margin과 같은 시그니처 (x, y, z, box, trunk, placed) -> bool.

    [allow_rotation] "HMI 화면 설계 가이드라인" 문서의 "박스 90도 회전 허용"
    토글. True(기본값)면 지금까지처럼 정자세 실패 시 90도 회전을 재시도한다
    (하위 호환). False면 정자세로 안 들어가면 바로 실패 처리 - 재시도 자체를
    안 한다.

    [wall_margin/obstacle_margin/ceiling_margin/entrance_margin] 문서의 나머지
    마진 슬라이더 4종(벽면 간격/장애물 간격/천장 간격/입구 여유 거리) - 전부
    None(기본값)이면 margin(박스 간격)과 17_margin_check.MARGIN / 15_overhead_
    clearance_check.OVERHEAD_CLEARANCE 그대로 쓴다(하위 호환). entrance_margin은
    wall_margin(정해졌으면 그 값, 아니면 margin) 위에서 입구 쪽 벽만 한 번 더
    덮어쓴다. 17_margin_check.has_sufficient_margin/
    15_overhead_clearance_check.has_overhead_clearance 참고.
    """
    logger.info(f"[{box.id}] 시도 (부피 {box.volume*1000:.1f}L, {box.width}x{box.depth}x{box.height})")
    kwargs = dict(score_fn=score_fn, margin=margin, extra_validity_fn=extra_validity_fn,
                  wall_margin=wall_margin, obstacle_margin=obstacle_margin, ceiling_margin=ceiling_margin,
                  entrance_margin=entrance_margin)
    plan = _place_one_orientation(box, trunk, state, order, allow_stacking, rotated=False, **kwargs)
    if plan is not None:
        return plan
    if not allow_rotation:
        return None  # 회전 자체가 비허용 - 재시도 안 함
    if box.width == box.depth:
        return None  # 정사각형이면 돌려도 후보가 똑같아서 재시도할 의미 없음
    logger.debug(f"[{box.id}] 정자세 실패 -> 90도 회전 재시도")
    return _place_one_orientation(rotate_box(box), trunk, state, order, allow_stacking, rotated=True, **kwargs)


def _place_one_orientation(
    box: "Box", trunk, state: "ExtremePointState", order: int,
    allow_stacking: bool, rotated: bool, score_fn=None, margin: Optional[float] = None,
    extra_validity_fn=None,
    wall_margin: Optional[float] = None, obstacle_margin: Optional[float] = None,
    ceiling_margin: Optional[float] = None, entrance_margin: Optional[float] = None,
) -> Optional["PlacementPlan"]:
    """place_one_box()의 실제 배치 로직 - 주어진 box의 치수(정자세 또는 이미
    회전된 치수)를 그대로 하나의 "자세"로 취급해서 자리를 찾는다."""
    _margin = margin if margin is not None else MARGIN
    _wall_margin = wall_margin if wall_margin is not None else _margin
    _ceiling_margin = ceiling_margin if ceiling_margin is not None else _m15.OVERHEAD_CLEARANCE

    # [③ 보강] 순수 모서리 확장만으로는 못 만드는 "이 박스라면 벽에 딱 붙는 자리"
    # + "이미 놓인 다른 박스 옆면에 딱 붙는 자리"를 지금 놓으려는 box 크기 기준으로
    # 추가 생성 - state.candidates에는 저장하지 않고 이번 배치 판단에만 잠깐 섞어
    # 쓴다 (다른 박스 크기에는 안 맞을 수 있어서). wall_flush는 wall_margin/
    # entrance_margin으로 생성해야 한다 - margin(box_margin)으로만 생성하면(과거
    # 버그) has_wall_margin이 더 큰 값을 요구할 때 그 후보가 거부되고 대안이 없어
    # 배치가 실패한다 (obstacle_margin과 같은 종류의 버그였음 -
    # generate_box_flush_candidates 참고).
    wall_flush = generate_wall_flush_candidates(box, trunk, state.candidates, margin=_wall_margin,
                                                 entrance_margin=entrance_margin)
    box_flush = generate_box_flush_candidates(box, trunk, state.candidates, state.placed, margin=_margin,
                                               obstacle_margin=obstacle_margin)
    # ⑰(마진) 도입 후 발견: "벽에 마진만큼 띄운 자리"가 하필 다른 박스와는 마진
    # 미달로 너무 가까운 경우가 있다 - 그 자리에서 다시 그 박스를 피해 마진만큼
    # 더 띄우는 조합("벽 마진" + "박스 마진" 둘 다 적용)은 각 생성기를 한 번씩만
    # 돌려서는 안 나온다. wall_flush 결과를 다시 box-flush 생성기에 넣어서 조합을
    # 만든다 (⑦이 아니라 ③+⑦ 조합 자체 확장 - 잘못된 후보가 섞여도 유효성 검사가
    # 그대로 걸러내므로 안전하다).
    combo_flush = generate_box_flush_candidates(box, trunk, wall_flush, state.placed, margin=_margin,
                                                 obstacle_margin=obstacle_margin)
    candidate_pool = state.candidates | wall_flush | box_flush | combo_flush

    # [④+⑬+⑮+⑯+⑰] 후보 좌표들 중, 겹치지도 밖으로 나가지도 않고(④) 충분히
    # 받쳐지고(⑬, allow_stacking일 때만) 로봇 팔이 위쪽으로 뺄 여유 공간도
    # 충분하고(⑮, 최종 자리 기준) 거기까지 가는 길에 더 높이 솟은 걸 타고 넘지
    # 않아도 되고(⑯, 입구~목표 사이 같은 y폭 장애물 기준) 벽/다른 박스와 딱
    # 붙지 않고 최소 간격을 유지하는(⑰) 것만 추림
    valid_candidates = [
        (x, y, z) for (x, y, z) in candidate_pool
        if is_candidate_valid_with_stacking(x, y, z, box, trunk, state.placed, allow_stacking=allow_stacking)
        and has_overhead_clearance(z, box, trunk, clearance=_ceiling_margin)
        and has_clear_approach_path(x, y, z, box, trunk, state.placed, clearance=_ceiling_margin)
        and has_sufficient_margin(x, y, z, box, trunk, state.placed, margin=_margin,
                                   wall_margin=wall_margin, obstacle_margin=obstacle_margin,
                                   entrance_margin=entrance_margin)
        and (extra_validity_fn is None or extra_validity_fn(x, y, z, box, trunk, state.placed))
    ]

    logger.debug(f"[{box.id}] 후보 {len(candidate_pool)}개 생성, 유효 {len(valid_candidates)}개")

    if not valid_candidates:
        return None  # 놓을 자리가 하나도 없음 -> 호출한 쪽(⑧)이 미적재 사유를 판단해야 함

    # [⑤] 유효한 후보 각각을 점수화 (score, touches)까지 같이 계산해서 보관
    _score = score_fn if score_fn is not None else score_candidate
    scored = [
        (pos, *_score(pos[0], pos[1], pos[2], box, trunk, state.placed))
        for pos in valid_candidates
    ]
    # 점수(t[1])가 가장 낮은(=가장 좋은) 후보 하나를 최종 선택
    best_pos, best_score, best_touches = min(scored, key=lambda t: t[1])
    logger.info(
        f"[{box.id}] 배치 완료: pos=({best_pos[0]:.2f},{best_pos[1]:.2f},{best_pos[2]:.2f}), "
        f"회전={'예' if rotated else '아니오'}, score={best_score:.3f}, 접촉면={best_touches}"
    )

    # [③] 실제로 그 자리에 박스를 배치하고, 그 박스 기준 새 후보 3개를 상태에 등록
    placed_box = PlacedBox(box=box, x=best_pos[0], y=best_pos[1], z=best_pos[2])
    state.register_placement(placed_box)

    target_yaw = box.initial_yaw + (math.pi / 2 if rotated else 0.0)

    return PlacementPlan(
        box_id=box.id,
        order=order,
        position=best_pos,
        dimensions=(box.width, box.depth, box.height),
        score=best_score,
        touches=best_touches,
        rotated=rotated,
        target_yaw=target_yaw,
    )


if __name__ == "__main__":
    _m02 = import_module("02_trunk_space_state")
    Trunk = _m02.Trunk

    trunk = Trunk(width=1.5, depth=1.5, height=0.9)
    state = ExtremePointState()
    box = Box("Medium", 0.40, 0.30, 0.25)

    plan = place_one_box(box, trunk, state, order=1)
    print(plan)
