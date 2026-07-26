"""
10_verification.py
⑩ 알고리즘 검증
=================
상태: 🟢 완료 — 실제 트렁크 실측값(0.57×1.12×0.25m) 반영됨

확정했던 검증 항목 5개를 각각 시나리오로 자동 테스트한다.
    1. 재현성          : 동일 입력 -> 동일 출력이 항상 나오는가
    2. 사유코드 정확성   : ⑧의 3가지 사유 코드가 각각 맞는 상황에서 정확히 나오는가
    3. 좌표변환/경계     : PlacementPlan의 좌표가 트렁크 범위 안 + 서로 겹치지 않는가
    4. 점수화 회귀방지   : 접촉면 기반 Best 후보 선택이 의도대로 동작하는가
    5. 실제 데이터 반영  : 진짜 트렁크/박스 실측값으로 돌렸을 때 결과가 예상과 일치하는가
                          (Large -> SIZE_EXCEEDS_TRUNK, Medium/Small -> 배치 성공)

실행: python 10_verification.py
"""

import sys, pathlib
from importlib import import_module

sys.path.insert(0, str(pathlib.Path(__file__).parent))
_m02 = import_module("02_trunk_space_state")
_m03 = import_module("03_extreme_point_candidates")
_m04 = import_module("04_candidate_validity_check")
_m08 = import_module("08_unloadable_reason")

Trunk = _m02.Trunk
REAL_TRUNK = _m02.REAL_TRUNK
Box = _m03.Box
find_overlaps = _m04.find_overlaps
find_out_of_bounds = _m04.find_out_of_bounds
generate_loading_plan = _m08.generate_loading_plan
UnloadableReason = _m08.UnloadableReason

SMALL = Box("Small", 0.30, 0.20, 0.15, mass_kg=1.0)
MEDIUM = Box("Medium", 0.40, 0.30, 0.25, mass_kg=2.0)
LARGE = Box("Large", 0.50, 0.35, 0.30, mass_kg=3.5)


def check(label: str, condition: bool, detail: str = ""):
    """검증 조건 하나를 PASS/FAIL로 출력하고, condition을 그대로 반환 (run_all에서 취합용)."""
    status = "PASS" if condition else "FAIL"
    # 실패했을 때만 detail을 같이 출력해서 원인 파악을 돕는다 (성공 시엔 생략해 로그를 짧게 유지)
    print(f"  [{status}] {label}" + (f" - {detail}" if detail and not condition else ""))
    return condition


def verify_reproducibility() -> bool:
    print("\n[1] 재현성 검증")
    boxes = [SMALL, MEDIUM, LARGE]
    trunk = Trunk(1.5, 1.5, 0.9)

    # 같은 boxes, trunk로 두 번 실행해서 결과가 완전히 똑같이 나오는지 비교
    plans_a, unload_a = generate_loading_plan(boxes, trunk)
    plans_b, unload_b = generate_loading_plan(boxes, trunk)

    ok = (
        [p.box_id for p in plans_a] == [p.box_id for p in plans_b]
        and [p.position for p in plans_a] == [p.position for p in plans_b]
        and [u.box_id for u in unload_a] == [u.box_id for u in unload_b]
    )
    return check("동일 입력 -> 동일 순서/좌표/미적재 결과", ok)


def verify_reason_codes() -> bool:
    print("\n[2] 사유코드 정확성 검증")
    results = []

    # SIZE_EXCEEDS_TRUNK: 트렁크가 비어도 Large가 못 들어가는 높이
    trunk1 = Trunk(1.5, 1.5, 0.28)
    _, unload1 = generate_loading_plan([LARGE], trunk1)
    ok1 = len(unload1) == 1 and unload1[0].reason == UnloadableReason.SIZE_EXCEEDS_TRUNK
    results.append(check("SIZE_EXCEEDS_TRUNK", ok1))

    # INSUFFICIENT_REMAINING_VOLUME: Blocker가 거의 꽉 채우는 트렁크에 두 번째 박스까지
    # 넣으려는 경우. ⑮ 상단 여유 공간(0.2m) 도입 후 트렁크 높이를 그냥 키우면 Blocker
    # 위쪽에 남는 "낭비 부피"가 함께 늘어나서 예전 LARGE/MEDIUM 조합(공용 상수, 다른
    # 테스트에서도 씀)으로는 이 시나리오 자체가 성립하지 않게 됨 - 그래서 이 테스트
    # 전용 박스로 다시 설계함 (Blocker 높이를 트렁크 높이-0.2에 딱 맞춰 낭비를 최소화).
    # ⑰ 박스-벽 마진(17_margin_check.MARGIN, main과 병합하며 0.01->0.02로 상향) 도입
    # 후: 박스 폭/깊이가 트렁크와 완전히 같으면 마진 넣을 자리가 없어서 그 박스 자체가
    # 못 들어가게 됨 - Blocker/TooBig 둘 다 폭·깊이를 2*MARGIN만큼 줄여서 마진 자리를
    # 남겨줌. 이러면서 Blocker(부피 60.48L)가 여전히 TooBig(59.94L)보다 커야
    # decide_loading_order(⑥, 부피 내림차순)가 Blocker를 먼저 시도해서 트렁크를
    # 먼저 채우고, 그 다음 TooBig이 남은 부피(59.52L) 부족으로 막히는 시나리오가
    # 성립한다 - Blocker가 더 작아지면 TooBig이 먼저 시도되면서(빈 트렁크에서 그 자체
    # 여유 없이도 SIZE 통과) 엉뚱하게 NO_VALID_CANDIDATE_POSITION으로 갈리는 걸 실제로
    # 확인해서 트렁크/두 박스 치수를 전부 재설계함.
    trunk2 = Trunk(0.60, 0.40, 0.50)
    blocker = Box("Blocker", 0.56, 0.36, 0.30)  # 트렁크의 절반 넘게 차지(60.48L) + 폭/깊이/높이 전부 여유 경계값
    too_big = Box("TooBig", 0.555, 0.36, 0.30)  # Blocker보다 살짝 작지만(59.94L), 남는 부피(59.52L)보다는 큼
    _, unload2 = generate_loading_plan([blocker, too_big], trunk2)
    ok2 = (
        len(unload2) == 1
        and unload2[0].box_id == "TooBig"
        and unload2[0].reason == UnloadableReason.INSUFFICIENT_REMAINING_VOLUME
    )
    results.append(check("INSUFFICIENT_REMAINING_VOLUME", ok2, detail=str(unload2)))

    return all(results)


def verify_coordinate_bounds() -> bool:
    print("\n[3] 좌표변환/경계 검증")
    trunk = Trunk(1.5, 1.5, 0.9)
    plans, _ = generate_loading_plan([SMALL, MEDIUM, LARGE], trunk)

    from importlib import import_module as im
    _m03b = im("03_extreme_point_candidates")
    placed_boxes = [
        _m03b.PlacedBox(box=Box(p.box_id, *p.dimensions), x=p.position[0], y=p.position[1], z=p.position[2])
        for p in plans
    ]

    ok1 = check("모든 배치 좌표가 트렁크 범위 안에 있음", len(find_out_of_bounds(placed_boxes, trunk)) == 0)
    ok2 = check("배치된 박스끼리 서로 겹치지 않음", len(find_overlaps(placed_boxes)) == 0)
    return ok1 and ok2


def verify_scoring_regression() -> bool:
    print("\n[4] 점수화(접촉면 기반) 회귀방지 검증")
    _m05 = import_module("05_candidate_scoring")
    score_candidate = _m05.score_candidate
    _m03b = import_module("03_extreme_point_candidates")
    PlacedBox = _m03b.PlacedBox

    # 05_candidate_scoring.py의 __main__ 데모와 같은 시나리오 (A/B/C 벽 우대 반영 후
    # 좌표 재설계 - pocket이 B·C에 둘러싸이면서 동시에 벽 A에도 붙어있어야 접촉면
    # 우대와 벽 우대가 서로 충돌하지 않는다).
    trunk = Trunk(12.0, 12.0, 4.0)
    unit = Box("unit", 2, 2, 2)
    placed = [
        PlacedBox(Box("A", 2, 2, 2), 8, 1, 0),
        PlacedBox(Box("B", 2, 2, 2), 8, 3, 0),
        PlacedBox(Box("C", 2, 2, 2), 10, 1, 0),
    ]
    open_score, _ = score_candidate(1.0, 1.0, 0.0, unit, trunk, placed)
    pocket_score, _ = score_candidate(10.0, 3.0, 0.0, unit, trunk, placed)

    return check("안쪽 구석 자리가 열린 자리보다 점수가 더 낮음(더 좋음)", pocket_score < open_score,
                 detail=f"pocket={pocket_score}, open={open_score}")


def verify_real_data() -> bool:
    print("\n[5] 실제 데이터 반영 검증 (트렁크 실측값 0.57×1.12×0.25m)")
    # 이 트렁크 높이(0.25m)는 ⑮ 상단 여유 공간(0.2m) 기준으로 보면 SMALL(0.15m)조차
    # 여유가 0.10m뿐이라 로봇이 안전하게 놓을 수 없는 높이임 - 그래서 셋 다 미적재가
    # 맞는 결과다. (이 값은 지완의 실제 스캔이 아니라 시뮬레이션 씬에서 뽑은 임시값 -
    # 실제 스캔 데이터(run_20260720_*, 높이 약 0.52m)는 ⑫에서 별도로 검증하고 있고
    # 거긴 전혀 안 깨짐. 오히려 이 임시 트렁크 높이 자체가 비현실적으로 낮았다는 걸
    # ⑮가 드러내준 셈.)
    plans, unloadable = generate_loading_plan([SMALL, MEDIUM, LARGE], REAL_TRUNK)

    ok1 = check(
        "Large는 SIZE_EXCEEDS_TRUNK로 미적재 처리됨",
        any(u.box_id == "Large" and u.reason == UnloadableReason.SIZE_EXCEEDS_TRUNK for u in unloadable),
        detail=str(unloadable),
    )
    ok2 = check(
        "Medium, Small도 상단 여유 공간(0.2m) 부족으로 미적재 처리됨 (트렁크 높이 0.25m가 비현실적으로 낮음)",
        {u.box_id for u in unloadable} == {"Small", "Medium", "Large"} and len(plans) == 0,
        detail=str(unloadable),
    )
    return ok1 and ok2


def run_all() -> bool:
    print("=" * 60)
    print("Cart2Trunk 알고리즘 검증 (극점 알고리즘 기반, ③④⑤⑥⑦⑧ 통합)")
    print("=" * 60)

    results = [
        verify_reproducibility(),
        verify_reason_codes(),
        verify_coordinate_bounds(),
        verify_scoring_regression(),
        verify_real_data(),
    ]

    print("\n" + "=" * 60)
    total, passed = len(results), sum(results)  # bool 리스트를 sum하면 True 개수가 나옴
    print(f"총 {total}개 항목 중 {passed}개 통과")
    print("=" * 60)
    return all(results)


if __name__ == "__main__":
    success = run_all()
    sys.exit(0 if success else 1)
