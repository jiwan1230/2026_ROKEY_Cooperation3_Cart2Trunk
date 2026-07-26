"""
04_candidate_validity_check.py
④ 후보 유효성 검사
=====================
상태: 🟢 완료·확정

③에서 만든 후보 좌표들이 전부 "쓸 수 있는" 자리는 아니다. 실제로 박스를
놓았을 때 두 가지 기하학 규칙을 어기지 않는지 확인해야 한다.

    1. 겹치지 않는가 — 두 박스가 3D 공간에서 서로 침범하면 물리적으로 불가능
    2. 트렁크 밖으로 나가지 않는가 — 모든 박스가 트렁크 경계 안에 있어야 함

이 두 체크를 통과한 후보만 "유효한 후보"로 ⑤(점수화)에 넘긴다.
"""

import sys, pathlib
from typing import List, Tuple
from importlib import import_module

sys.path.insert(0, str(pathlib.Path(__file__).parent))
_m03 = import_module("03_extreme_point_candidates")

Box = _m03.Box
PlacedBox = _m03.PlacedBox


def boxes_overlap(a: "PlacedBox", b: "PlacedBox") -> bool:
    """두 박스가 3축(x,y,z) 모두에서 겹치면 실제로 겹치는 것 (AABB 충돌 판정)."""
    ax0, ax1 = a.x_range  # a박스의 x축 구간 [ax0, ax1]
    bx0, bx1 = b.x_range  # b박스의 x축 구간
    ay0, ay1 = a.y_range
    by0, by1 = b.y_range
    az0, az1 = a.z_range
    bz0, bz1 = b.z_range
    # 각 축마다 "구간이 겹치는가"(ax0 < bx1 and ax1 > bx0)를 확인.
    # 3축 전부 겹쳐야만 실제 3D 공간에서 두 박스가 겹치는 것 (하나라도 안 겹치면 안전).
    return (ax0 < bx1 and ax1 > bx0) and (ay0 < by1 and ay1 > by0) and (az0 < bz1 and az1 > bz0)


def find_overlaps(placed: List["PlacedBox"]) -> List[Tuple[str, str]]:
    """겹치는 박스 쌍의 (id, id) 리스트를 반환. 빈 리스트면 겹침 없음 = 통과."""
    pairs = []
    for i in range(len(placed)):
        for j in range(i + 1, len(placed)):
            if boxes_overlap(placed[i], placed[j]):
                pairs.append((placed[i].box.id, placed[j].box.id))
    return pairs


def find_out_of_bounds(placed: List["PlacedBox"], trunk) -> List[str]:
    """트렁크 경계를 벗어난 박스 id 리스트를 반환. 빈 리스트면 전부 경계 안 = 통과."""
    violations = []
    for p in placed:
        if (p.x < 0 or p.y < 0 or p.z < 0
                or p.x_range[1] > trunk.width + 1e-9
                or p.y_range[1] > trunk.depth + 1e-9
                or p.z_range[1] > trunk.height + 1e-9):
            violations.append(p.box.id)
    return violations


def is_candidate_valid(x: float, y: float, z: float, box: "Box", trunk, placed: List["PlacedBox"]) -> bool:
    """
    후보 좌표 (x,y,z)에 box를 놓았을 때 유효한지 종합 판단.
    (경계 안 + 기존 박스와 안 겹침)
    """
    # 박스를 (x,y,z)에 놓았을 때 반대쪽 끝(x+width 등)이 트렁크 경계를 넘는지 확인.
    # +1e-9는 부동소수점 오차로 인해 "딱 맞는 경우"가 false로 튕기는 걸 방지하는 여유값.
    if x + box.width > trunk.width + 1e-9:
        return False
    if y + box.depth > trunk.depth + 1e-9:
        return False
    if z + box.height > trunk.height + 1e-9:
        return False

    # 아래쪽 경계(x<0 등)도 확인 - 지금까지는 register_placement()가 만드는 후보가
    # 항상 0에서 시작해서 이 구멍이 드러난 적 없었는데, ③의
    # generate_box_flush_candidates()가 "박스 폭이 앞쪽 빈 틈보다 넓은" 경우 음수
    # 좌표를 계산해낼 수 있어서 실제로 필요해짐 (-1e-9는 위와 같은 이유의 여유값).
    if x < -1e-9 or y < -1e-9 or z < -1e-9:
        return False

    # 경계는 통과했으니, 이미 놓인 박스들과 하나라도 겹치면 무효 (마진은 여기서
    # 안 다룸 - ⑰ has_sufficient_margin()이 07 파이프라인에서 별도로 확인한다)
    candidate = PlacedBox(box=box, x=x, y=y, z=z)
    return not any(boxes_overlap(candidate, p) for p in placed)


if __name__ == "__main__":
    from importlib import import_module
    _m02 = import_module("02_trunk_space_state")
    Trunk = _m02.Trunk

    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    b1 = PlacedBox(box=Box("A", 0.4, 0.3, 0.2), x=0.0, y=0.0, z=0.0)
    b2 = PlacedBox(box=Box("B", 0.4, 0.3, 0.2), x=0.2, y=0.0, z=0.0)  # A와 겹침

    print("겹침 쌍:", find_overlaps([b1, b2]))
    print("경계 이탈:", find_out_of_bounds([b1, b2], trunk))
    print("새 후보 (0.4,0,0)에 같은 박스 놓기 유효한가:",
          is_candidate_valid(0.4, 0.0, 0.0, Box("C", 0.4, 0.3, 0.2), trunk, [b1, b2]))
