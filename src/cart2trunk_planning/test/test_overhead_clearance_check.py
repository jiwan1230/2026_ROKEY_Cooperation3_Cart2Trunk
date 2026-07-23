"""
test_15_overhead_clearance_check.py
⑮ 상단 여유 공간(Overhead Clearance) 확인 검증.

배경: 로봇이 아직 연결 전이라 실제 그리퍼 충돌 여부는 모르지만, 사용자가 첨부한
그림(로봇 팔이 트렁크 천장에 걸리는 상황)을 미리 대비하기 위해 추가함. 저희
알고리즘이 정밀한 IK/충돌검사를 대신하는 게 아니라(그건 나중에 모션플래너 역할),
"박스 윗면과 트렁크 천장 사이 최소 0.2m는 비워야 한다"는 단순하고 보수적인 안전
마진만 하드 컷으로 확인한다. z=0(바닥)이든 z>0(2층)이든 모든 후보에 똑같이 적용됨
(키 큰 박스는 바닥에 놓아도 천장에 가까울 수 있어서).
"""
import sys, pathlib
from importlib import import_module

_m02 = import_module("cart2trunk_planning.core.trunk_space_state")
_m03 = import_module("cart2trunk_planning.core.extreme_point_candidates")
_m07 = import_module("cart2trunk_planning.core.placement_plan")
_m15 = import_module("cart2trunk_planning.core.overhead_clearance_check")

Trunk = _m02.Trunk
Box = _m03.Box
PlacedBox = _m03.PlacedBox
ExtremePointState = _m03.ExtremePointState
place_one_box = _m07.place_one_box
has_overhead_clearance = _m15.has_overhead_clearance
has_clear_approach_path = _m15.has_clear_approach_path
OVERHEAD_CLEARANCE = _m15.OVERHEAD_CLEARANCE


def test_rejects_when_gap_smaller_than_clearance():
    """트렁크 높이 0.25m에 박스 높이 0.15m면, 남는 여유는 0.10m라 0.2m 기준 미달 -> 거부."""
    trunk = Trunk(width=1.0, depth=1.0, height=0.25)
    box = Box("Small", width=0.3, depth=0.2, height=0.15)
    assert has_overhead_clearance(0.0, box, trunk) is False


def test_accepts_when_gap_meets_clearance():
    """트렁크 높이 0.6m에 박스 높이 0.15m면, 남는 여유는 0.45m라 기준 충족 -> 통과."""
    trunk = Trunk(width=1.0, depth=1.0, height=0.6)
    box = Box("Small", width=0.3, depth=0.2, height=0.15)
    assert has_overhead_clearance(0.0, box, trunk) is True


def test_applies_to_stacked_candidates_too():
    """z>0(2층) 후보도 똑같이 확인 - 쌓인 위치에서 천장까지 남는 여유로 판단."""
    trunk = Trunk(width=1.0, depth=1.0, height=0.5)
    box = Box("Blue", width=0.1, depth=0.1, height=0.15)
    # z=0.2에 놓으면 위쪽 끝은 0.35, 남는 여유는 0.5-0.35=0.15m -> 0.2m 미달로 거부
    assert has_overhead_clearance(0.2, box, trunk) is False
    # z=0.1에 놓으면 위쪽 끝은 0.25, 남는 여유는 0.5-0.25=0.25m -> 통과
    assert has_overhead_clearance(0.1, box, trunk) is True


def test_boundary_exactly_at_clearance_is_valid():
    """회귀 방지 - 여유가 정확히 0.2m면(경계값) 통과해야 한다 (부동소수점 여유값 확인)."""
    trunk = Trunk(width=1.0, depth=1.0, height=0.35)
    box = Box("unit", width=0.2, depth=0.2, height=0.15)  # 0.35-0.15=0.20 정확히 경계
    assert has_overhead_clearance(0.0, box, trunk) is True


def test_place_one_box_reports_no_placement_when_no_overhead_clearance_anywhere():
    """
    통합 테스트: 트렁크 전체가 낮아서(높이 0.25m) 바닥에 놓아도 여유가 0.2m 미달인
    박스는, 자리가 물리적으로 남아있어도 place_one_box()가 배치하지 않아야 한다.
    """
    trunk = Trunk(width=1.0, depth=1.0, height=0.25)
    state = ExtremePointState()
    box = Box("TooTallForClearance", width=0.2, depth=0.2, height=0.15)  # 0.25-0.15=0.10 < 0.2

    plan = place_one_box(box, trunk, state, order=1)

    assert plan is None


# ---------------------------------------------------------------------------
# ⑯ 접근 경로 확인 (has_clear_approach_path) - has_overhead_clearance()는 "최종
# 자리 바로 위" 천장 여유만 본다. 근데 로봇은 입구(x=0)에서 +x로 들어오면서
# 자리를 잡으므로, 목표보다 입구에 더 가까운 자리에 목표 높이보다 높이 솟은
# 장애물/박스가 같은 y 폭에 걸쳐 있으면 그걸 타고 넘어야 한다 - 이것도 확인해야
# 실제로 안전한 경로인지 알 수 있다. (계기: 카트 시나리오에서 파란 박스가 빨간
# 장애물 뒤의 바퀴 위에 배치됐는데, "그 장애물을 넘어가야 하는데 계산한 거야?"라는
# 질문 - 확인해보니 계산 안 하고 있었음.)
# ---------------------------------------------------------------------------

def test_floor_placement_always_passes_regardless_of_whats_in_front():
    """z=0(바닥) 배치는 옆으로 스치는 정도라 대상이 아님 - 앞에 뭐가 있어도 통과."""
    trunk = Trunk(width=1.0, depth=1.0, height=0.5)
    box = Box("Floor", width=0.1, depth=0.1, height=0.15)
    tall_obstacle = PlacedBox(box=Box("Tall", width=0.1, depth=0.1, height=0.45), x=0.0, y=0.0, z=0.0)
    assert has_clear_approach_path(0.5, 0.0, 0.0, box, trunk, [tall_obstacle]) is True


def test_no_obstacle_in_front_matches_plain_overhead_clearance():
    """입구 쪽에 아무것도 없으면 has_overhead_clearance()와 똑같이 동작해야 한다."""
    trunk = Trunk(width=1.0, depth=1.0, height=0.5)
    box = Box("Blue", width=0.1, depth=0.1, height=0.12)
    assert has_clear_approach_path(0.5, 0.0, 0.2, box, trunk, []) == has_overhead_clearance(0.2, box, trunk)


def test_taller_obstacle_in_same_lane_in_front_blocks_even_when_own_spot_has_clearance():
    """
    목표 자리 자체는 천장 여유가 충분해도, 입구 쪽에 더 높이 솟은 장애물이 같은
    y 폭을 막고 있으면 거부해야 한다 (그 위를 넘어가려면 추가 여유가 더 필요함).
    """
    trunk = Trunk(width=1.0, depth=1.0, height=0.5)
    box = Box("Blue", width=0.1, depth=0.1, height=0.1)
    # 목표 z=0.1: has_overhead_clearance만 보면 0.5-(0.1+0.1)=0.3 >= 0.2로 통과.
    assert has_overhead_clearance(0.1, box, trunk) is True
    # 근데 입구 쪽(x=0.1)에, 같은 y 폭(0.0~0.5)을 덮는 장애물이 z=0.35까지 솟아있다면
    # 그 위를 넘어야 하는데, 0.5-(0.35+0.1)=0.05 < 0.2라서 안전하지 않음 -> 거부.
    tall_in_front = PlacedBox(box=Box("Tall", width=0.2, depth=0.5, height=0.35), x=0.1, y=0.0, z=0.0)
    assert has_clear_approach_path(0.6, 0.0, 0.1, box, trunk, [tall_in_front]) is False


def test_obstacle_in_different_y_lane_does_not_block():
    """입구 쪽에 있어도 y 폭이 안 겹치면(다른 통로) 방해 안 되는 걸로 본다."""
    trunk = Trunk(width=1.0, depth=1.0, height=0.5)
    box = Box("Blue", width=0.1, depth=0.1, height=0.1)
    tall_other_lane = PlacedBox(box=Box("Tall", width=0.2, depth=0.1, height=0.45), x=0.1, y=0.6, z=0.0)
    assert has_clear_approach_path(0.6, 0.0, 0.1, box, trunk, [tall_other_lane]) is True


def test_obstacle_behind_target_does_not_block():
    """목표보다 더 깊은(입구에서 먼) 곳에 있는 건 지나갈 필요가 없으니 방해 안 됨."""
    trunk = Trunk(width=1.0, depth=1.0, height=0.5)
    box = Box("Blue", width=0.1, depth=0.1, height=0.1)
    tall_behind = PlacedBox(box=Box("Tall", width=0.2, depth=0.5, height=0.45), x=0.7, y=0.0, z=0.0)
    assert has_clear_approach_path(0.6, 0.0, 0.1, box, trunk, [tall_behind]) is True
