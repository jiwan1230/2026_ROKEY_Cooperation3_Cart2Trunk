"""
05_candidate_scoring.py
⑤ 후보 평가 — 정렬(sort) → 점수화(score) 재설계
====================================================
상태: 🟢 완료·확정 ← 오늘(7/20)의 핵심 작업

[기존 방식의 문제]
후보들을 (z, y, x) 순으로 정렬해서 제일 앞의 유효한 후보를 그냥 골랐다.
"낮은 위치"는 z를 먼저 보니 어느 정도 반영되지만, 팀 완료 기준에 있는
"구석"과 "공간 활용도"는 전혀 반영하지 못한다 — 좌표값이 작다고 실제로
더 구석진(둘러싸인) 자리라는 보장이 없다.

[새 방식]
후보 지점에 박스를 놓았다고 가정하고, 그 박스의 6개 면 중 몇 개가 벽이나
이미 놓인 다른 박스에 "붙는지"(접촉면)를 센다. 접촉면이 많을수록 더
구석에 박히고 빈틈을 덜 남기는(=공간 활용도 높은) 자리라고 본다.

    score = HEIGHT_WEIGHT * (z / trunk.height) - CONTACT_WEIGHT * (접촉면수 / 6)

점수가 낮을수록(더 음수일수록) 좋은 자리. 유효한 후보 중 점수가 가장 낮은
걸 고른다. 가중치는 팀 튜닝 대상 — 지금은 "높이가 1순위, 접촉면은 그 안에서
보정"하도록 기본값을 잡았다 (BR-09: 낮고 안정적인 배치 우선).

[증명된 사실]
아래 데모에서, 좌표값만 보면 더 작아서 기존 방식이 고르는 "열린 자리"보다
"안쪽 구석" 자리가 접촉면이 많아 점수가 더 낮게 나와 새 방식이 실제로
다른(더 나은) 답을 낸다는 걸 확인함.
"""

import sys, pathlib
from typing import List, Tuple
from importlib import import_module

sys.path.insert(0, str(pathlib.Path(__file__).parent))
_m03 = import_module("03_extreme_point_candidates")
_m04 = import_module("04_candidate_validity_check")

Box = _m03.Box
PlacedBox = _m03.PlacedBox
boxes_overlap = _m04.boxes_overlap

HEIGHT_WEIGHT = 1.0
CONTACT_WEIGHT = 0.5
# 트렁크 벽 3개(A/B/C)를 우대하는 정도 - 사용자가 손그림으로 직접 지정한 우선순위:
#   A(가장 안쪽 벽, x=width, 입구 반대쪽) > B(측면 벽, y=depth) = C(측면 벽, y=0)
#
# 첫 버전(0.6/0.3)은 "벽에 딱 붙기만 하면" 받는 WALL_BC 보너스가 A쪽으로 조금 더
# 들어가는 이득보다 커서, 박스가 A로 계속 전진하지 않고 얕은 채로 옆벽에 붙어버리는
# 문제가 있었다 (사용자가 실제로 발견함). 그래서 A는 CONTACT_WEIGHT보다도 확실히
# 크게, BC는 "완전히 벽에 붙어도" A쪽으로 어느 정도 더 들어간 것보다는 못 이기도록
# 확 낮춤 - 이제 A가 "닿았냐 안 닿았냐"가 아니라 "얼마나 가까우냐"로 계속 우세하게
# 작용하고, BC는 A가 비슷할 때만 갈리는 진짜 '중간 정도' 보정으로 동작한다.
WALL_A_WEIGHT = 0.9
WALL_BC_WEIGHT = 0.2


def _ranges_overlap(a: Tuple[float, float], b: Tuple[float, float]) -> bool:
    """1차원 구간 두 개가 겹치는지 확인 (면끼리 실제로 마주보는지 판단할 때 사용)."""
    return a[0] < b[1] and a[1] > b[0]


def count_touching_faces(x: float, y: float, z: float, box: "Box", trunk,
                          placed: List["PlacedBox"]) -> int:
    """박스를 (x,y,z)에 놓았을 때, 6개 면 중 벽 또는 다른 박스에 붙는 면 개수(0~6)."""
    touches = 0
    # 후보 박스가 차지할 x/y/z 구간 (0이 시작점, 1이 끝점)
    x0, x1 = x, x + box.width
    y0, y1 = y, y + box.depth
    z0, z1 = z, z + box.height

    # --- 1) 트렁크 벽 6면과 붙는지 확인 (바닥/천장/좌/우/앞/뒤) ---
    # 박스의 시작점이 0(바닥/좌/앞 벽)에 딱 붙어있거나,
    # 끝점이 trunk 크기(천장/우/뒤 벽)에 딱 붙어있으면 그 면은 벽에 접촉한 것.
    if abs(x0 - 0.0) < 1e-9:
        touches += 1
    if abs(x1 - trunk.width) < 1e-9:
        touches += 1
    if abs(y0 - 0.0) < 1e-9:
        touches += 1
    if abs(y1 - trunk.depth) < 1e-9:
        touches += 1
    if abs(z0 - 0.0) < 1e-9:
        touches += 1
    if abs(z1 - trunk.height) < 1e-9:
        touches += 1

    # --- 2) 이미 놓인 다른 박스들과 맞닿는 면 확인 ---
    for p in placed:
        px0, px1 = p.x_range
        py0, py1 = p.y_range
        pz0, pz1 = p.z_range

        # x축 방향으로 딱 붙어있고(내 오른쪽=상대 왼쪽 or 내 왼쪽=상대 오른쪽),
        # 동시에 y축·z축 구간도 겹쳐야 진짜로 "면이 마주보고 접촉"하는 것.
        # (x만 붙어있어도 y/z가 아예 다른 위치면 실제로는 접촉이 아님)
        if abs(x1 - px0) < 1e-9 or abs(x0 - px1) < 1e-9:
            if _ranges_overlap((y0, y1), (py0, py1)) and _ranges_overlap((z0, z1), (pz0, pz1)):
                touches += 1
        if abs(y1 - py0) < 1e-9 or abs(y0 - py1) < 1e-9:
            if _ranges_overlap((x0, x1), (px0, px1)) and _ranges_overlap((z0, z1), (pz0, pz1)):
                touches += 1
        if abs(z1 - pz0) < 1e-9 or abs(z0 - pz1) < 1e-9:
            if _ranges_overlap((x0, x1), (px0, px1)) and _ranges_overlap((y0, y1), (py0, py1)):
                touches += 1

    return min(touches, 6)  # 박스는 면이 6개뿐이므로 상한선을 6으로 고정


def entrance_distance_ratio(x: float, box: "Box", trunk) -> float:
    """
    후보 박스가 입구에서 얼마나 안쪽으로 들어가 있는지를 0(입구 바로 앞)~1(제일 안쪽)
    사이 값으로 정규화해서 반환한다.

    x축만 본다 - 로봇은 M0609 base 좌표계 원점에 고정돼 있고, 트렁크에는 항상 정해진
    한 방향(x축)으로만 접근한다는 게 확인됐다 (실제 스캔 데이터의 "x, +deep" 라벨과도
    일치). 즉 y(좌우 위치)는 입구에서 먼 정도와 아예 무관하다 - 로봇이 왼쪽에 있든
    오른쪽에 있든 트렁크 안쪽으로 뻗는 거리는 x만으로 결정되기 때문. (첫 버전은 x/y를
    평균 냈다가, 좌우 위치만 달라도 점수가 달라지는 버그가 돼서 x만 보도록 수정함.)

    trunk.entrance_near_x는 ②(to_bounding_trunk)가 로봇 base 원점 기준으로 미리
    계산해 둔 값이다 - 로컬 x=0쪽이 입구에 더 가까우면 True, 반대쪽이 더 가까우면 False.
    """
    # entrance_near_x가 True면 입구가 x=0 쪽이므로 x좌표 자체가 곧 "입구로부터 거리".
    # False면 입구가 x=width 쪽이므로, 박스의 반대쪽 끝(x+width)에서 벽까지 남은 거리를 잰다.
    depth_x = x if trunk.entrance_near_x else (trunk.width - (x + box.width))
    return depth_x / trunk.width


def side_wall_distance_ratio(y: float, box: "Box", trunk) -> float:
    """
    후보 박스가 두 측면 벽(B: y=depth쪽, C: y=0쪽) 중 더 가까운 쪽에서 얼마나
    떨어져 있는지 0(벽에 붙음)~1(트렁크 정중앙, 제일 멂)로 정규화해서 반환한다.

    B/C는 로봇의 접근 방향(x축, 벽 A)과는 완전히 별개인 좌우 측면 벽이다 - "측면
    벽에도 붙여서 중앙 통로를 비워두자"는 아이디어를 반영한다. B와 C는 같은 값으로
    우대하므로(WALL_BC_WEIGHT 하나만 씀) 이 함수는 어느 쪽이 더 가까운지만 보고
    둘을 구분하지 않는다.
    """
    dist_to_c = y                              # y=0쪽 벽(C)까지 남은 거리
    dist_to_b = trunk.depth - (y + box.depth)  # y=depth쪽 벽(B)까지 남은 거리
    nearest_wall_dist = min(dist_to_c, dist_to_b)

    # 박스가 정중앙에 있을 때 nearest_wall_dist가 가질 수 있는 최댓값 (그때를 1.0으로 정규화)
    max_possible = (trunk.depth - box.depth) / 2
    if max_possible < 1e-9:
        return 0.0  # 박스가 트렁크 깊이를 거의 다 차지해서 '중앙'이라는 개념 자체가 무의미
    return min(nearest_wall_dist / max_possible, 1.0)


def score_candidate(x: float, y: float, z: float, box: "Box", trunk,
                     placed: List["PlacedBox"]) -> Tuple[float, int]:
    """(score, 접촉면수)를 같이 반환해서 '왜 이 점수인지' 설명 가능하게 한다."""
    touches = count_touching_faces(x, y, z, box, trunk, placed)
    height_term = HEIGHT_WEIGHT * (z / trunk.height)   # 높을수록(z 클수록) 점수가 커짐 = 불리
    contact_term = CONTACT_WEIGHT * (touches / 6)       # 접촉면 많을수록 점수가 깎임 = 유리
    # 벽 A(안쪽)에 가까울수록 점수가 깎임 = 유리 - 입구부터 막아버리는 걸 방지 (최우선)
    wall_a_term = WALL_A_WEIGHT * entrance_distance_ratio(x, box, trunk)
    # 벽 B/C(측면) 중 가까운 쪽에 붙을수록 점수가 깎임 = 유리 (A보다는 약하게)
    wall_bc_term = WALL_BC_WEIGHT * (1 - side_wall_distance_ratio(y, box, trunk))
    return height_term - contact_term - wall_a_term - wall_bc_term, touches  # 낮을수록 좋은 자리


def make_weighted_score_fn(
    entrance_preference: float = 1.0, contact_preference: float = 1.0, height_preference: float = 1.0,
):
    """
    "HMI 화면 설계 가이드라인" 문서 4절의 적재 우선순위 슬라이더 중 3개 축을
    반영하는 score_fn을 만들어 반환한다 (place_one_box의 score_fn 확장점에
    바로 꽂아 쓸 수 있음). 기본값 (1.0, 1.0, 1.0)은 기존 score_candidate와
    완전히 동일하게 동작한다(하위 호환).

    [entrance_preference] "입구 우선 ↔ 깊은 위치 우선" 축. 양수면 지금까지처럼
    깊은 자리를 우대(WALL_A_WEIGHT 부호 그대로), 음수면 부호가 뒤집혀 입구에
    가까운 자리를 우대한다. 크기는 배율(1.0=기존 강도 그대로, 0=이 축 무시).

    [contact_preference] "공간활용률 우선 ↔ 안정성 우선" 축의 근사치. ⚠️ 정확히
    말하면 이 문서가 말하는 두 개념(공간 활용도 / 안정성)을 지금 알고리즘이
    독립적으로 계산할 수 있는 신호가 따로 없다 - count_touching_faces(접촉면
    수) 하나가 "접촉면이 많을수록 빈틈을 덜 남긴다(공간 활용)"와 "여러 면이
    벽/다른 박스에 지지된다(안정성 근사)"를 동시에 대변하는 유일한 지표라서,
    이 하나의 손잡이로 두 축을 근사한다. 진짜 지지력 기반 안정성 점수(예:
    ⑬ 받침 비율을 그레이디드 점수로)는 팀 논의 후 별도로 설계해야 한다.

    [height_preference] "바닥부터 채우기" 축의 강도 배율 - HEIGHT_WEIGHT(낮은
    자리 우대)에 곱한다. 1.0=기존 강도, 0=높이를 아예 신경 안 씀(쌓기를 꺼리지
    않게 됨 - allow_stacking=True와 같이 쓰면 더 적극적으로 위층을 활용하는
    쪽으로 기움), 1보다 크면 바닥 선호를 더 강하게. 음수는 의미가 없어서
    (쌓인 자리를 오히려 우대) 지원하지 않는다 - 반대 방향이 필요하면
    allow_stacking=False로 아예 쌓기를 막는 게 맞는 접근.
    """
    def _score(x: float, y: float, z: float, box: "Box", trunk,
               placed: List["PlacedBox"]) -> Tuple[float, int]:
        touches = count_touching_faces(x, y, z, box, trunk, placed)
        height_term = HEIGHT_WEIGHT * height_preference * (z / trunk.height)
        contact_term = CONTACT_WEIGHT * contact_preference * (touches / 6)
        wall_a_term = WALL_A_WEIGHT * entrance_preference * entrance_distance_ratio(x, box, trunk)
        wall_bc_term = WALL_BC_WEIGHT * (1 - side_wall_distance_ratio(y, box, trunk))
        return height_term - contact_term - wall_a_term - wall_bc_term, touches

    return _score


# "개수 우선" 모드(08_unloadable_reason.generate_loading_plan(mode="count_first"))용
# 가중치 - 입구 우대(WALL_A/BC)는 완전히 제거. 새 영역을 넓히는 데는 큰 페널티를
# 줘서, 이미 쓰는 영역 안에 채워 넣는 걸 최우선으로 한다.
COUNT_FIRST_HEIGHT_WEIGHT = 1.0
COUNT_FIRST_FOOTPRINT_GROWTH_WEIGHT = 5.0


def score_count_first(x: float, y: float, z: float, box: "Box", trunk,
                       placed: List["PlacedBox"]) -> Tuple[float, int]:
    """
    "개수 우선" 모드 전용 점수 함수 (원래 industry_scenarios/scenario2_warehouse_
    density.py의 score_density_first였던 걸 실제 운용 모드로 승격).

    지금까지 놓인 박스들이 차지한 바운딩 영역(x/y 최대 범위)을 이 후보가 얼마나
    더 밖으로 넓히는지를 점수의 핵심으로 삼는다 - 0이면 "이미 쓰던 영역 안에
    딱 맞게 채워 넣음"(밀도 최댓값), 클수록 새 빈 영역을 여는 것(밀도 손해).

    [접촉면 가중치 대신 이 방식을 쓰는 이유] 처음엔 접촉면(count_touching_faces)
    가중치를 올리는 방식을 시도했는데, ⑰(마진)가 벽/박스 어디에도 딱 붙는 자리를
    금지해서 접촉면이 거의 항상 0~1로 고정돼버려 후보를 구분하는 신호가 안
    됐다(실측 확인). footprint growth는 마진이 있어도 "얼마나 가까운지"가 아니라
    "새 영역을 여는지 아닌지"를 보므로 margin 유무와 무관하게 항상 유효하다.
    """
    touches = count_touching_faces(x, y, z, box, trunk, placed)
    if placed:
        used_max_x = max(p.x_range[1] for p in placed)
        used_max_y = max(p.y_range[1] for p in placed)
    else:
        used_max_x = 0.0
        used_max_y = 0.0
    growth_x = max(0.0, (x + box.width) - used_max_x)
    growth_y = max(0.0, (y + box.depth) - used_max_y)
    footprint_growth_term = COUNT_FIRST_FOOTPRINT_GROWTH_WEIGHT * (growth_x + growth_y)
    height_term = COUNT_FIRST_HEIGHT_WEIGHT * (z / trunk.height)
    return height_term + footprint_growth_term, touches


if __name__ == "__main__":
    _m02 = import_module("02_trunk_space_state")
    Trunk = _m02.Trunk

    # ---- 데모: 정렬 방식과 점수화 방식이 실제로 다른 답을 낼 수 있음을 증명 ----
    # A/B/C 벽 우대가 생긴 뒤로 좌표를 다시 잡았다 (예전 버전은 pocket이 트렁크
    # 정중앙에 우연히 걸려서 벽 B/C 보너스를 하나도 못 받는 우연한 충돌이 있었음).
    # 지금 pocket은 B·C 박스에 둘러싸이면서 동시에 벽 A(안쪽)에도 붙어있어서,
    # "접촉면 많음"과 "벽 우대" 두 기준이 서로 부딪히지 않고 같은 방향을 가리킨다.
    trunk = Trunk(width=12.0, depth=12.0, height=4.0)
    box_size = Box(id="unit", width=2, depth=2, height=2)

    placed = [
        PlacedBox(box=Box(id="A", width=2, depth=2, height=2), x=8, y=1, z=0),
        PlacedBox(box=Box(id="B", width=2, depth=2, height=2), x=8, y=3, z=0),
        PlacedBox(box=Box(id="C", width=2, depth=2, height=2), x=10, y=1, z=0),
    ]

    candidate_open = (1.0, 1.0, 0.0)     # 입구 쪽 구석, 아무것도 없는 열린 자리
    candidate_pocket = (10.0, 3.0, 0.0)  # B·C 사이 + 벽 A에 붙은 안쪽 구석 자리

    print("=== 후보 비교: '열린 자리' vs '안쪽 구석' ===\n")
    for label, (cx, cy, cz) in [("열린 자리", candidate_open), ("안쪽 구석", candidate_pocket)]:
        score, touches = score_candidate(cx, cy, cz, box_size, trunk, placed)
        print(f"[{label}] 좌표=({cx},{cy},{cz})  접촉면={touches}/6  점수={score:.4f}")

    print("\n--- 기존 방식(정렬, z,y,x 순) ---")
    old_pick = min([candidate_open, candidate_pocket], key=lambda p: (p[2], p[1], p[0]))
    print(f"선택: {old_pick} (단순히 y좌표가 더 작다는 이유로 선택됨)")

    print("\n--- 새 방식(점수화) ---")
    scored = [(label, score_candidate(*p, box_size, trunk, placed)[0])
              for label, p in [("열린 자리", candidate_open), ("안쪽 구석", candidate_pocket)]]
    new_pick_label = min(scored, key=lambda t: t[1])[0]
    print(f"선택: {new_pick_label} (접촉면이 더 많아서 점수가 더 낮음 = 더 좋음)")

    print("\n=== 결론 ===")
    if old_pick == candidate_open and new_pick_label == "안쪽 구석":
        print("기존 방식과 새 방식이 다른 자리를 골랐다 -> 좌표값만으로는 '구석'을 못 잡아낸다는 증거.")
