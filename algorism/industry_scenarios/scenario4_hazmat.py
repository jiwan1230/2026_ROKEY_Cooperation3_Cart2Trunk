"""
scenario4_hazmat.py
산업 현장 시나리오 ④ 위험물/화학물질 창고 (비호환 물질 인접 금지)
==============================================

[정책]
특정 위험물 분류끼리는(예: 산화제 oxidizer + 인화물 flammable) 최소 안전거리
이상 떨어뜨려야 한다 - 코어에 없는 완전히 새로운 종류의 규칙이라, 순서/점수/
마진을 바꾸는 게 아니라 하드 컷 규칙 자체를 하나 새로 추가해야 한다.

[코어와의 관계]
이번 시나리오 작업 중 07_placement_plan.place_one_box()에 추가한
extra_validity_fn 확장점(기존 ④⑬⑮⑯⑰ 체인 뒤에 AND로 하나 더 끼워 넣는 자리)
에 has_hazmat_clearance()를 꽂아 쓴다. 순서 결정(⑥)·점수화(⑤)는 코어 기본값
그대로 - "비호환 물질을 안전거리 밖에 두는 것" 자체가 배치 가능/불가능을
가르는 하드 제약이지, "어디를 더 선호하는지"의 문제가 아니라서다.

[안전거리 계산 방식]
04_candidate_validity_check._boxes_too_close()와 같은 스타일(AABB를 안전거리
만큼 부풀려서 겹침 판정)이지만, x/y(수평)만 보는 ⑰과 달리 z(높이)까지 포함한
3축 전체에 적용한다 - 위험물은 위/아래로 쌓여도 위험하기 때문(⑰은 쌓임
관계는 오히려 딱 맞닿아야 정상이라고 보지만, 위험물은 정반대: 쌓여도 안전거리
필요).
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
PlacedBox = _m03.PlacedBox
ExtremePointState = _m03.ExtremePointState
decide_loading_order = _m06.decide_loading_order
place_one_box = _m07.place_one_box
classify_unloadable_reason = _m08.classify_unloadable_reason
UnloadableItem = _m08.UnloadableItem

# 비호환으로 취급할 위험물 분류 쌍 (순서 무관 - frozenset으로 대칭 비교).
# 실제 운영에서는 GHS(세계조화시스템) 분류표 기준으로 팀/현장이 확정해야 함 -
# 여기선 가장 널리 알려진 예시(산화제-인화물) 하나만 데모로 넣어둠.
INCOMPATIBLE_PAIRS = {
    frozenset({"oxidizer", "flammable"}),
}

# 비호환 물질 사이에 반드시 둬야 하는 최소 거리 (팀/현장 협의 대상 - 여기선
# ⑰ 기본 마진(2cm)보다 훨씬 큰 30cm로 데모).
HAZMAT_SAFE_DISTANCE = 0.3


def _is_incompatible(class_a, class_b) -> bool:
    if class_a is None or class_b is None:
        return False
    return frozenset({class_a, class_b}) in INCOMPATIBLE_PAIRS


def has_hazmat_clearance(x: float, y: float, z: float, box: "Box", trunk,
                          placed: List["PlacedBox"]) -> bool:
    """
    비호환 관계인 기존 박스들과 3축 전체(x/y/z) 기준 HAZMAT_SAFE_DISTANCE
    이상 떨어져 있는지 확인. box.hazard_class가 없으면 항상 통과.
    """
    if box.hazard_class is None:
        return True

    x0, x1 = x, x + box.width
    y0, y1 = y, y + box.depth
    z0, z1 = z, z + box.height

    for p in placed:
        if not _is_incompatible(box.hazard_class, p.box.hazard_class):
            continue
        px0, px1 = p.x_range
        py0, py1 = p.y_range
        pz0, pz1 = p.z_range
        d = HAZMAT_SAFE_DISTANCE
        # candidate를 3축 모두 d만큼 부풀린 뒤 p(안 부풀림)와 겹치면 "너무 가까움"
        overlap_x = (x0 - d) < px1 and (x1 + d) > px0
        overlap_y = (y0 - d) < py1 and (y1 + d) > py0
        overlap_z = (z0 - d) < pz1 and (z1 + d) > pz0
        if overlap_x and overlap_y and overlap_z:
            return False

    return True


def generate_loading_plan_hazmat(boxes: List["Box"], trunk) -> Tuple[list, list]:
    """08_unloadable_reason.generate_loading_plan()과 동일한 루프, 하드 컷 하나 추가."""
    order = decide_loading_order(boxes)
    state = ExtremePointState()

    plans = []
    unloadable = []
    order_counter = 1

    for box in order:
        plan = place_one_box(box, trunk, state, order_counter, extra_validity_fn=has_hazmat_clearance)
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

    trunk = Trunk(width=1.5, depth=1.0, height=0.5)
    boxes = [
        Box("산화제_드럼1", 0.3, 0.3, 0.3, hazard_class="oxidizer"),
        Box("인화물_드럼1", 0.3, 0.3, 0.3, hazard_class="flammable"),
        Box("일반박스1", 0.3, 0.3, 0.3),
    ]

    plans, unloadable = generate_loading_plan_hazmat(boxes, trunk)
    print("=== 배치 결과 ===")
    for p in plans:
        b = next(b for b in boxes if b.id == p.box_id)
        print(f"  {p.box_id} ({b.hazard_class}): x=[{p.position[0]:.2f},{p.position[0]+p.dimensions[0]:.2f}] "
              f"y=[{p.position[1]:.2f},{p.position[1]+p.dimensions[1]:.2f}]")
    for u in unloadable:
        print(f"  [미적재] {u.box_id}: {u.reason.value}")

    ox_plan = next(p for p in plans if p.box_id == "산화제_드럼1")
    fl_plan = next(p for p in plans if p.box_id == "인화물_드럼1")
    oxidizer_box = next(b for b in boxes if b.id == "산화제_드럼1")
    ox_placed = PlacedBox(box=oxidizer_box, x=ox_plan.position[0], y=ox_plan.position[1], z=ox_plan.position[2])
    flammable_box = next(b for b in boxes if b.id == "인화물_드럼1")
    ok = has_hazmat_clearance(fl_plan.position[0], fl_plan.position[1], fl_plan.position[2],
                               flammable_box, trunk, [ox_placed])
    print(f"\n산화제-인화물 안전거리({HAZMAT_SAFE_DISTANCE}m) 준수: {'예' if ok else '아니오'}")
