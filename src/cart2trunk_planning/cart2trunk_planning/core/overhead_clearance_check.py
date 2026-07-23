"""
15_overhead_clearance_check.py
⑮ 상단 여유 공간(Overhead Clearance) 확인
==============================================
상태: 🟡 신규 (7/22) - 로봇 미연결, 사전 대비 차원

[배경]
로봇이 아직 연결되지 않아서 실제로 문제가 생기는지는 모르지만, 사용자가 트렁크
천장에 로봇 팔(엔드이펙터+조인트)이 걸리는 그림을 보여주며 미리 대비를 요청함.

[중요 - 이 파일이 안 하는 것]
실제 역기구학(IK)이나 팔 전체의 충돌 검사는 여기서 하지 않는다. 그건 실제
로봇이 연결된 뒤 모션 플래너(MoveIt 등)가 담당할 영역이고, 정적인 CAD/USD
데이터만으로는 "이 자세에서 팔이 어디를 지나가는지" 자체를 알 수 없다 (실제로
M0609 로봇의 진짜 USD를 열어서 확인했지만, 그건 "관절 전부 0도인 기본 자세"의
지오메트리일 뿐, 트렁크에 옆으로 팔을 뻗는 실제 자세와는 무관함).

[이 파일이 하는 것]
"박스 윗면과 트렁크 천장 사이에 최소 이만큼은 비어있어야 한다"는 단순하고
보수적인 안전 마진 하나만 하드 컷으로 확인한다. 그리퍼(VGP20) 실측 CAD가 아직
없어서 OVERHEAD_CLEARANCE는 보수적인 추정치 - 나중에 실측치가 나오면 이 상수만
바꾸면 된다. z=0(바닥)이든 z>0(2층)이든 모든 후보에 똑같이 적용된다 (키 큰
박스는 바닥에 놓아도 천장에 가까울 수 있어서, "쌓기 전용"으로 한정하지 않음).
"""

from importlib import import_module

_m03 = import_module("cart2trunk_planning.core.extreme_point_candidates")

Box = _m03.Box

# 그리퍼(VGP20) 실측 CAD가 아직 없어서 잡은 보수적 추정치 (팀 튜닝 대상).
# VGP20 본체+패드 길이가 보통 0.1~0.15m대라, 팀 안전 마진까지 더해 0.2m로 설정함.
OVERHEAD_CLEARANCE = 0.20


def has_overhead_clearance(z: float, box: "Box", trunk, clearance: float = OVERHEAD_CLEARANCE) -> bool:
    """
    후보 (z)에 box를 놓았을 때, 박스 윗면(z+height)과 트렁크 천장(trunk.height)
    사이에 clearance만큼 여유가 있는지 확인한다. 부족하면 False(하드 컷) -
    ⑬ 받침 확인과 같은 원칙으로, 점수를 깎는 게 아니라 후보 자체를 무효 처리한다.
    """
    return (trunk.height - (z + box.height)) >= clearance - 1e-9


def has_clear_approach_path(x: float, y: float, z: float, box: "Box", trunk, placed, clearance: float = OVERHEAD_CLEARANCE) -> bool:
    """
    has_overhead_clearance()는 "최종 자리 바로 위" 천장 여유만 본다. 근데 로봇은
    입구(x=0)에서 +x로 들어오면서 자리를 잡으므로, 목표보다 입구에 더 가까운
    (x가 더 작은) 자리에 목표보다 높이 솟은 장애물/박스가 같은 y 폭에 걸쳐
    있으면 그걸 타고 넘어가야 한다 - 그 경우 "넘어가는 높이" 기준으로 다시 천장
    여유를 확인해야 안전하다.

    z=0(바닥) 배치는 검사 대상이 아니다 - 옆으로 스치는 정도는 실제로도 흔히
    감수하는 수준이라고 보고, ⑬/⑮와 마찬가지로 "쌓기(z>0)"에서 특히 위험한
    경우만 하드 컷으로 잡는다. 완전한 3D 충돌/IK 계산이 아니라 ⑮와 동일한
    보수적 근사 - 실제 정교한 경로(예: 더 높이 들었다 내리기)는 여전히 하류
    모션 플래너 몫.
    """
    if z < 1e-9:
        return True

    y0, y1 = y, y + box.depth
    peak_z = z
    for p in placed:
        px0, _px1 = p.x_range
        if px0 >= x - 1e-9:
            continue  # 목표보다 입구에서 먼(더 깊은) 물체는 지나갈 필요 없음
        py0, py1 = p.y_range
        if py1 <= y0 + 1e-9 or py0 >= y1 - 1e-9:
            continue  # y축이 안 겹치면 다른 통로로 지나갈 수 있다고 가정
        peak_z = max(peak_z, p.z_range[1])

    return (trunk.height - (peak_z + box.height)) >= clearance - 1e-9


if __name__ == "__main__":
    _m02 = import_module("cart2trunk_planning.core.trunk_space_state")
    Trunk = _m02.Trunk

    # 데모: 천장이 낮은 트렁크에서 박스 높이별로 여유 공간 확인이 어떻게 갈리는지
    trunk = Trunk(width=0.6, depth=0.73, height=0.4)
    for h in (0.10, 0.15, 0.20, 0.25):
        box = Box("demo", width=0.2, depth=0.2, height=h)
        gap = trunk.height - h
        ok = has_overhead_clearance(0.0, box, trunk)
        print(f"박스 높이={h:.2f}m  z=0에 놓으면 남는 여유={gap:.2f}m  "
              f"{'통과' if ok else '거부(여유 부족)'}")
