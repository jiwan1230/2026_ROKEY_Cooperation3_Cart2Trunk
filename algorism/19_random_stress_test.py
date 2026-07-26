"""
19_random_stress_test.py
⑲ 무작위 스트레스 테스트
==========================
상태: 🟡 신규 (7/22)

[배경]
지금까지의 테스트는 전부 "이 버그를 잡아야지" 하고 사람이 손으로 설계한
시나리오였다 (박스 개수·크기·순서를 직접 정함). 실전에서는 카트에 어떤
박스가 몇 개, 어떤 크기로 올지 알 수 없다 - 손으로 만든 시나리오만으로는
"아무도 예상 못한 조합"에서 뭔가 깨지는지 확인이 안 된다.

[이 파일이 하는 것]
트렁크 크기 + 박스 개수/치수를 매 시행마다 무작위로 만들어서
generate_loading_plan()(⑥⑦⑧ 통합 파이프라인, 08_unloadable_reason.py)에
그대로 돌리고, 결과가 "정확히 이 좌표에 놓여야 한다"가 아니라 **절대 깨지면
안 되는 규칙(불변조건)**을 지키는지만 확인한다:

    1. 예외 없이 끝까지 실행됨
    2. 입력한 박스 전부가 배치 결과 또는 미적재 목록 중 정확히 한 곳에만 있음
       (사라지거나 중복되지 않음)
    3. 배치된 박스끼리 서로 안 겹침 (④)
    4. 배치된 박스가 트렁크 밖으로 안 나감 (④)
    5. 배치된 박스가 벽/다른 박스와 최소 마진(⑰)을 지킴
    6. 회전(⑱)됐다고 표시된 박스는 실제로 가로/세로가 바뀐 치수임 (안 됐다고
       표시된 박스는 원래 치수 그대로임)

실패하면 그 시행의 시드+번호+트렁크/박스 정의를 그대로 출력해서 재현 가능하게
한다 - 무작위 테스트는 "왜 실패했는지"를 재현 못 하면 쓸모가 없어서, 항상
고정된 시드(seed)로 돌리고 실패 시 그 시드를 정확히 찍어준다.

[비목표]
카트에서의 픽업 순서(rests_on_id, ⑥)는 이미 tests/test_06_loading_order_decision.py
에서 충분히 다루고 있어서 여기서는 안 섞는다 (뒤섞으면 실패 원인을 "픽업 순서
문제냐 배치 문제냐" 구분하기 어려워짐). generate_loading_plan()이 기본적으로
allow_stacking 없이(=바닥 전용) 호출하는 실제 동작 그대로 검증한다.
"""

import sys, pathlib, random
from importlib import import_module

sys.path.insert(0, str(pathlib.Path(__file__).parent))
m02 = import_module("02_trunk_space_state")
m03 = import_module("03_extreme_point_candidates")
m04 = import_module("04_candidate_validity_check")
m08 = import_module("08_unloadable_reason")
m17 = import_module("17_margin_check")

Trunk = m02.Trunk
Box = m03.Box
PlacedBox = m03.PlacedBox
find_overlaps = m04.find_overlaps
find_out_of_bounds = m04.find_out_of_bounds
generate_loading_plan = m08.generate_loading_plan
has_sufficient_margin = m17.has_sufficient_margin


def _random_trunk(rng: random.Random) -> "Trunk":
    return Trunk(
        width=round(rng.uniform(0.40, 0.90), 3),
        depth=round(rng.uniform(0.50, 1.20), 3),
        height=round(rng.uniform(0.35, 0.55), 3),
    )


def _random_boxes(rng: random.Random, count: int) -> list:
    boxes = []
    for i in range(count):
        boxes.append(Box(
            id=f"Box{i}",
            width=round(rng.uniform(0.05, 0.50), 3),
            depth=round(rng.uniform(0.05, 0.50), 3),
            height=round(rng.uniform(0.05, 0.35), 3),
        ))
    return boxes


def _check_invariants(boxes, trunk, plans, unloadable):
    """깨지면 절대 안 되는 규칙들을 확인한다. 위반 사항 문자열 목록을 반환
    (빈 목록이면 전부 통과)."""
    problems = []
    box_by_id = {b.id: b for b in boxes}

    # 2. 입력 박스 전부가 정확히 한 곳(배치 or 미적재)에만 있는지
    placed_ids = [p.box_id for p in plans]
    unloadable_ids = [u.box_id for u in unloadable]
    seen = placed_ids + unloadable_ids
    if len(set(seen)) != len(seen):
        problems.append(f"박스 id 중복 등장: {seen}")
    missing = set(box_by_id) - set(seen)
    if missing:
        problems.append(f"입력했는데 결과에 아예 없는 박스: {missing}")
    extra = set(seen) - set(box_by_id)
    if extra:
        problems.append(f"입력에 없는 박스가 결과에 나옴: {extra}")

    # 배치 결과를 PlacedBox로 재구성 (③④⑬⑮⑯⑰의 실제 좌표계 검증용)
    placed_boxes = []
    for p in plans:
        b = box_by_id[p.box_id]
        w, d, h = p.dimensions
        placed = Box(id=b.id, width=w, depth=d, height=h)
        placed_boxes.append(PlacedBox(box=placed, x=p.position[0], y=p.position[1], z=p.position[2]))

        # 6. 회전 표시와 실제 치수가 일치하는지
        if p.rotated:
            if not (w == b.depth and d == b.width):
                problems.append(f"{p.box_id}: rotated=True인데 치수가 안 바뀜 (원래 {b.width}x{b.depth}, 결과 {w}x{d})")
        else:
            if not (w == b.width and d == b.depth):
                problems.append(f"{p.box_id}: rotated=False인데 치수가 다름 (원래 {b.width}x{b.depth}, 결과 {w}x{d})")

    # 3. 서로 안 겹침 / 4. 트렁크 밖으로 안 나감 (④)
    overlaps = find_overlaps(placed_boxes)
    if overlaps:
        problems.append(f"겹치는 박스 쌍: {overlaps}")
    oob = find_out_of_bounds(placed_boxes, trunk)
    if oob:
        problems.append(f"트렁크 밖으로 나간 박스: {oob}")

    # 5. 벽/다른 박스와 최소 마진(⑰) - 자기 자신을 제외한 나머지를 기준으로 확인
    for i, pb in enumerate(placed_boxes):
        others = placed_boxes[:i] + placed_boxes[i + 1:]
        if not has_sufficient_margin(pb.x, pb.y, pb.z, pb.box, trunk, others):
            problems.append(f"{pb.box.id}: 벽 또는 다른 박스와 마진(1cm) 미달")

    return problems


def run_stress_test(num_trials: int, seed: int, min_boxes: int = 1, max_boxes: int = 10) -> tuple:
    """무작위 시행을 num_trials번 돌리고 (성공 횟수, 실패 상세 목록)을 반환한다."""
    rng = random.Random(seed)
    failures = []

    for trial in range(num_trials):
        trunk = _random_trunk(rng)
        count = rng.randint(min_boxes, max_boxes)
        boxes = _random_boxes(rng, count)

        try:
            plans, unloadable = generate_loading_plan(boxes, trunk)
        except Exception as e:  # 예외 자체도 실패로 취급 (규칙 1)
            failures.append({
                "trial": trial, "seed": seed, "trunk": trunk, "boxes": boxes,
                "problems": [f"예외 발생: {type(e).__name__}: {e}"],
            })
            continue

        problems = _check_invariants(boxes, trunk, plans, unloadable)
        if problems:
            failures.append({"trial": trial, "seed": seed, "trunk": trunk, "boxes": boxes, "problems": problems})

    return num_trials - len(failures), failures


def _print_failure(f):
    print(f"\n  [실패] trial={f['trial']} (seed={f['seed']}로 재현 가능)")
    print(f"    트렁크: {f['trunk']}")
    for b in f["boxes"]:
        print(f"    박스: {b.id} {b.width}x{b.depth}x{b.height}")
    for p in f["problems"]:
        print(f"    위반: {p}")


if __name__ == "__main__":
    import time

    # 발표/포트폴리오용: 서로 다른 시드 5개로 총 1,700회를 한 번의 실행으로
    # 재현한다 - 원래는 이 세션 중 5번 따로 실행해서 나온 숫자였는데(500 +
    # 300*4), 명령어 하나로 재현 가능하게 정리해둠.
    SEED_BATCHES = [(42, 500), (1, 300), (7, 300), (999, 300), (123456, 300)]

    start = time.time()
    total_trials = 0
    total_passed = 0
    all_failures = []

    for seed, num_trials in SEED_BATCHES:
        print(f"시드 {seed}: {num_trials}번 시행 중...")
        passed, failures = run_stress_test(num_trials, seed)
        total_trials += num_trials
        total_passed += passed
        all_failures.extend(failures)
        print(f"  -> {passed}/{num_trials} 통과")

    elapsed = time.time() - start
    print(f"\n{'=' * 60}")
    print(f"전체 합계: {total_passed}/{total_trials} 통과 ({elapsed:.1f}초 소요)")
    if all_failures:
        print(f"실패: {len(all_failures)}건")
        for f in all_failures[:10]:  # 너무 많으면 앞 10건만
            _print_failure(f)
        if len(all_failures) > 10:
            print(f"  ... 외 {len(all_failures) - 10}건 더 있음")
    else:
        print("전부 통과 - 위반 사항 없음")
    print("=" * 60)
