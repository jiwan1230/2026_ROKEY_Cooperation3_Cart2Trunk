"""
13_support_check.py
⑬ 지지대(받침) 확인 — 적층(2층 이상) 대비
=============================================
상태: 🟡 신규 (allow_stacking 플래그로 잠긴 채 대기)

[배경]
③(register_placement)이 박스 윗면(z+height)도 다음 후보로 등록하기 때문에,
"박스 위에 놓을 자리"는 이미 후보 목록에 들어가 있다. 그런데 ④(is_candidate_valid)는
경계/겹침만 확인하고 "그 자리 밑에 실제로 뭔가 받쳐주고 있는가"는 전혀 안 물어봤다.

실제로 확인된 문제: 작은 박스(0.2x0.2x0.3) 위에 훨씬 넓은 박스(0.6x0.6x0.2)를
놓는 후보도 ④는 "문제없음"으로 판단한다. 실제로는 밑면 대부분이 허공이라
로봇이 이대로 실행하면 박스가 떨어진다.

[이 파일이 하는 일]
1. compute_support_ratio() - 후보 자리 밑면 중 실제로 닿아있는 비율을 계산
   (근사 아님 - AABB 사각형 겹침 넓이를 정확히 계산)
2. is_candidate_valid_with_stacking() - ④의 is_candidate_valid()를 그대로
   감싸서, 받침 비율이 기준(기본 80%) 미만이면 하드 컷으로 무효 처리

[아직 안 켜짐]
allow_stacking 기본값은 False. 꺼져 있으면 z>0 후보는 무조건 거부되어
지금의 1층 전용 동작이 그대로 유지된다. 실제 적층 데이터/운영 결정이
나오면 07_placement_plan.py 호출부에서 True로 켠다.

[비목표 - 스펙 참고: docs/superpowers/specs/2026-07-21-support-check-design.md]
is_fragile 활용, 무게(mass_kg) 기반 강도 검사, 무게중심 안정성, 최대 층수 제한
— 전부 이번 범위 밖.
"""

import sys, pathlib
from typing import List
from importlib import import_module

sys.path.insert(0, str(pathlib.Path(__file__).parent))
_m03 = import_module("03_extreme_point_candidates")
_m04 = import_module("04_candidate_validity_check")

Box = _m03.Box
PlacedBox = _m03.PlacedBox
is_candidate_valid = _m04.is_candidate_valid


def compute_support_ratio(x: float, y: float, z: float, box: "Box", placed: List["PlacedBox"]) -> float:
    """
    후보 (x, y, z)에 box를 놓았을 때, 밑면 중 실제로 뭔가에 닿아있는 비율(0.0~1.0).

    z가 바닥(0)이면 트렁크 바닥이 항상 전체를 받쳐주므로 무조건 1.0.
    z가 0보다 크면, placed 중 "윗면이 정확히 z에 닿는" 박스들만 지지대 후보로
    보고, 그 박스들과 candidate 밑면의 x/y 겹침 사각형 넓이를 전부 더한다.
    placed 안의 박스들은 서로 겹치지 않는다는 게 ④에서 이미 보장되므로,
    겹침 넓이를 단순 합산해도 이중 계산될 위험이 없다.
    """
    if z < 1e-9:
        return 1.0

    x0, x1 = x, x + box.width
    y0, y1 = y, y + box.depth
    footprint_area = box.width * box.depth

    supported_area = 0.0
    for p in placed:
        if abs(p.z_range[1] - z) > 1e-9:
            continue  # 이 박스의 윗면이 후보 밑면과 안 맞닿아 있으면 지지대가 아님
        px0, px1 = p.x_range
        py0, py1 = p.y_range
        overlap_x = max(0.0, min(x1, px1) - max(x0, px0))
        overlap_y = max(0.0, min(y1, py1) - max(y0, py0))
        supported_area += overlap_x * overlap_y

    return min(supported_area / footprint_area, 1.0)


def is_candidate_valid_with_stacking(
    x: float, y: float, z: float, box: "Box", trunk, placed: List["PlacedBox"],
    allow_stacking: bool = False,
    min_support_ratio: float = 0.8,
) -> bool:
    """
    ④의 is_candidate_valid()(경계/겹침 확인)에 받침 확인을 추가로 얹은 버전.
    ④는 수정하지 않고 그대로 재사용한다.
    """
    if not is_candidate_valid(x, y, z, box, trunk, placed):
        return False

    if z < 1e-9:
        return True  # 바닥은 항상 통과 - 지지 확인 자체가 불필요

    if not allow_stacking:
        return False  # 적층 기능이 꺼져 있으면 z>0 후보는 무조건 탈락

    return compute_support_ratio(x, y, z, box, placed) >= min_support_ratio - 1e-9


if __name__ == "__main__":
    _m02 = import_module("02_trunk_space_state")
    Trunk = _m02.Trunk

    # 데모: 브레인스토밍 단계에서 발견한 실패 사례가 이제 거부되는지 확인
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    small_base = PlacedBox(box=Box("Small", width=0.2, depth=0.2, height=0.3), x=0.0, y=0.0, z=0.0)
    big_top = Box("Big", width=0.6, depth=0.6, height=0.2)

    ratio = compute_support_ratio(0.0, 0.0, 0.3, big_top, placed=[small_base])
    print(f"작은 박스(0.2x0.2) 위 큰 박스(0.6x0.6) 받침 비율: {ratio:.1%}")
    print("allow_stacking=True에서 유효한가:",
          is_candidate_valid_with_stacking(0.0, 0.0, 0.3, big_top, trunk, [small_base], allow_stacking=True))
    print("(기대: False - 받침 비율이 80% 기준에 크게 못 미침)")
