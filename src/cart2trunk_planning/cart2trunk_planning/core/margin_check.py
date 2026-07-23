"""
17_margin_check.py
⑰ 박스-벽 / 박스-박스 최소 간격(Margin) 확인
==============================================
상태: 🟢 완료 (7/22, main과 두 번 병합하며 설계/값 정리)

[배경]
지금까지는 박스를 벽이나 다른 박스에 딱(간격 0으로) 붙여서 배치했다. 실제
그리퍼로 놓고 빼는 동작을 생각하면, 완전히 맞닿은 자리는 살짝만 어긋나도
옆 박스나 벽에 긁힐 위험이 있다. 그래서 "박스끼리도, 박스-벽 사이도 아주
조금은 여유를 두자"는 요청으로 고정 마진을 하드 컷으로 추가한다.

[마진 값 = 0.02m, 왜 이 파일이 유일한 소스인가]
처음엔 1cm로 시작했는데, 지완님이 실제 로봇으로 PLACE 하강을 반복 검증하며
1cm는 벽 코너(두 벽이 동시에 margin 딱 걸리는 자리)에서 실행 오차를 못
흡수해 박스가 튕겨나가는 걸 실측으로 확인하고 main의 03/04에 직접
PLACEMENT_SAFETY_MARGIN_M=0.02 상수를 박아넣는 방식으로 2cm로 올렸었다.
그런데 지완님이 이후 main<->algorism을 다시 병합하면서, 마진을 후보
생성(③)에 하드코딩하는 대신 이 파일(⑰) 하나로 일원화하는 설계를 택했고
(더 깔끔한 단일 소스 - ③/④는 이제 마진을 전혀 모른다), 그 병합 과정에서
값 자체는 옛 1cm 버전으로 되돌아가는 걸 놓쳤다 - 설계는 그 판단을 따르되
(③에 상수를 다시 넣지 않음), 값은 실측으로 검증된 0.02로 여기서 복구한다.
1cm로는 실제 로봇 PLACE가 벽 코너에서 튕겨나간다는 게 이미 실측된 사실이라,
설계와 무관하게 값 자체가 틀리면 안 된다.

[이 파일이 하는 것 / 안 하는 것]
- 좌우(x/y, 트렁크 옆벽·안쪽벽) 방향으로만 마진을 확인한다.
- 바닥(z=0)은 대상이 아니다 - 박스는 바닥에 딱 붙어야(접촉해야) 정상이다.
- 천장 쪽 여유는 이미 ⑮(OVERHEAD_CLEARANCE=0.20m)가 훨씬 큰 마진으로 다루고
  있어서 여기서 따로 안 다룬다.
- 박스끼리도, z 범위가 실제로 겹치는(=옆으로 나란한) 경우에만 마진을 요구한다.
  z가 안 겹치는(한쪽이 다른 쪽 위에 쌓인) 관계는 오히려 완전히 맞닿아야
  (⑬ 받침) 하므로 마진 대상이 아니다.
"""

from typing import List
from importlib import import_module

_m03 = import_module("cart2trunk_planning.core.extreme_point_candidates")

Box = _m03.Box
PlacedBox = _m03.PlacedBox

# 그리퍼가 옆 박스/벽에 안 긁히도록 두는 최소 여유 - 이 파일이 유일한 소스(③/④는
# 마진을 모름). 0.01(1cm)은 실제 로봇 PLACE 하강 중 벽 코너에서 실행 오차를 못
# 흡수해 박스가 튕겨나가는 게 실측으로 확인된 값이라 쓰면 안 된다 - 위 모듈
# docstring 참고.
MARGIN = 0.02  # 2cm (실측 검증값)


def has_wall_margin(x: float, y: float, z: float, box: "Box", trunk, margin: float = MARGIN) -> bool:
    """트렁크 옆벽/안쪽벽까지 x·y 방향으로 margin 이상 떨어져 있는지 확인 (바닥/천장 제외)."""
    return (
        x >= margin - 1e-9
        and x + box.width <= trunk.width - margin + 1e-9
        and y >= margin - 1e-9
        and y + box.depth <= trunk.depth - margin + 1e-9
    )


def has_box_margin(x: float, y: float, z: float, box: "Box", placed: List["PlacedBox"], margin: float = MARGIN) -> bool:
    """
    z 범위가 실제로 겹치는(옆으로 나란히 놓이는) 다른 박스와 x 또는 y 방향으로
    margin 이상 떨어져 있는지 확인. 셋 중 하나라도 겹치는 게 있으면 그 박스와는
    x_gap, y_gap 둘 다 margin 미만이면 안 된다(대각선으로 너무 가까운 것도 거부).
    """
    x0, x1 = x, x + box.width
    y0, y1 = y, y + box.depth
    z0, z1 = z, z + box.height

    for p in placed:
        pz0, pz1 = p.z_range
        if not (z0 < pz1 - 1e-9 and z1 > pz0 + 1e-9):
            continue  # z가 안 겹침 - 쌓기 관계라 마진 대상 아님

        px0, px1 = p.x_range
        py0, py1 = p.y_range
        x_gap = max(px0 - x1, x0 - px1)
        y_gap = max(py0 - y1, y0 - py1)
        if x_gap < margin - 1e-9 and y_gap < margin - 1e-9:
            return False

    return True


def has_sufficient_margin(
    x: float, y: float, z: float, box: "Box", trunk, placed: List["PlacedBox"], margin: float = MARGIN
) -> bool:
    """벽 마진 + 박스 간 마진을 모두 확인 (하드 컷 - ⑬/⑮와 같은 원칙)."""
    return has_wall_margin(x, y, z, box, trunk, margin) and has_box_margin(x, y, z, box, placed, margin)


if __name__ == "__main__":
    _m02 = import_module("cart2trunk_planning.core.trunk_space_state")
    Trunk = _m02.Trunk

    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    box = Box("demo", width=0.2, depth=0.2, height=0.2)
    print("벽에 딱 붙은(x=0) 자리:", has_wall_margin(0.0, 0.5, 0.0, box, trunk), "(기대: False)")
    print(f"벽에서 {MARGIN}m 뗀 자리:", has_wall_margin(MARGIN, 0.5, 0.0, box, trunk), "(기대: True)")
