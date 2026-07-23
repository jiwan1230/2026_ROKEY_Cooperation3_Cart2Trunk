"""
test_box_flush_candidates.py
③+⑦ "이미 놓인 다른 박스 옆면에 딱 붙는 자리" 후보 누락 버그 재현 및 수정 검증.

배경: generate_wall_flush_candidates()(⑦에서 이미 씀)는 트렁크 바깥쪽 벽(A/B/C)에
딱 붙는 자리는 커버했지만, "이미 놓인 다른 박스" 옆면에 딱 붙는 자리는 다루지
않는다. 실제 발견된 사례: 큰 박스(F_BigRight)가 먼저 벽 A 근처에 놓인 뒤, 그보다
작은 박스(F_BigLeft)를 놓을 차례가 되면 F_BigRight 바로 앞(입구 쪽)에 딱 붙는
자리가 물리적으로는 비어있는데도 후보가 안 만들어져서, 결국 입구까지 밀려나
배치됐다.
"""
import sys, pathlib
from importlib import import_module

_m02 = import_module("cart2trunk_planning.core.trunk_space_state")
_m03 = import_module("cart2trunk_planning.core.extreme_point_candidates")
_m04 = import_module("cart2trunk_planning.core.candidate_validity_check")
_m07 = import_module("cart2trunk_planning.core.placement_plan")
_m17 = import_module("cart2trunk_planning.core.margin_check")

Trunk = _m02.Trunk
Box = _m03.Box
PlacedBox = _m03.PlacedBox
ExtremePointState = _m03.ExtremePointState
generate_box_flush_candidates = _m03.generate_box_flush_candidates
is_candidate_valid = _m04.is_candidate_valid
place_one_box = _m07.place_one_box
MARGIN = _m17.MARGIN


def test_generate_box_flush_candidates_adds_near_face_variant():
    """
    기존 후보 (0, 0, 0)이 있고, x=[0.38,0.60] y=[0.15,0.45]에 다른 박스가 놓여 있을 때,
    폭 0.18짜리 박스라면 "그 박스 입구 쪽 면(x=0.38)에 딱 붙는" 좌표
    (0.38-0.18, 0, 0) = (0.20, 0, 0)도 추가로 만들어져야 한다 (y,z는 겹치지 않지만
    후보 생성 자체는 만들어두고, 실제 유효성은 ④가 걸러낸다 - 여기서는 그냥 후보
    집합에 들어있는지만 확인).
    """
    trunk = Trunk(width=0.6, depth=0.73, height=0.4)
    box = Box("F_BigLeft", width=0.18, depth=0.21, height=0.15)
    neighbor = PlacedBox(box=Box("F_BigRight", 0.22, 0.30, 0.15), x=0.38, y=0.15, z=0.0)
    existing = {(0.0, 0.15, 0.0)}  # neighbor와 같은 y를 쓰는 기존 후보

    extra = generate_box_flush_candidates(box, trunk, existing, [neighbor])

    assert (0.38 - 0.18, 0.15, 0.0) in extra


def test_generate_box_flush_candidates_adds_far_face_variant():
    """같은 상황에서 '그 박스를 지나 더 안쪽(far face)'에 붙는 좌표도 만들어져야 한다."""
    trunk = Trunk(width=0.6, depth=0.73, height=0.4)
    box = Box("Small", width=0.10, depth=0.10, height=0.15)
    neighbor = PlacedBox(box=Box("Neighbor", 0.20, 0.20, 0.15), x=0.10, y=0.10, z=0.0)
    existing = {(0.0, 0.10, 0.0)}

    extra = generate_box_flush_candidates(box, trunk, existing, [neighbor])

    assert (0.10 + 0.20, 0.10, 0.0) in extra  # neighbor의 먼 쪽 면(x=0.30)에 붙음


def test_place_one_box_finds_spot_flush_against_neighboring_box():
    """
    실제로 발견된 사례의 최소 재현. 트렁크 깊이를 F_BigRight의 깊이와 똑같이
    잡아서(0.30m), F_BigRight가 y 전체를 다 차지하게 만들었다 - 이러면 벽 A
    쪽에 F_BigRight를 피해서 갈 수 있는 다른 y밴드 자체가 없으므로, F_BigLeft가
    깊이 들어갈 수 있는 유일한 방법은 F_BigRight 바로 앞(x축)에 딱 붙는 것뿐이다
    (다른 y밴드로 벽 A에 바로 붙는 대안이 있으면 그쪽이 더 깊어서 그게 이기는 게
    맞는 동작이라 - 이 테스트는 그 대안 자체를 없애서 이번에 고친 후보만
    순수하게 검증한다).
    """
    # 트렁크 깊이는 F_BigRight의 깊이(0.30) + 벽 마진 2번(⑰, MARGIN 양쪽)만큼 잡아서,
    # F_BigLeft(깊이도 0.30)가 y방향으로 들어갈 수 있는 자리가 딱 하나(y=MARGIN)뿐이게
    # 만든다 - 그래야 "다른 y밴드로 벽 A에 바로 붙는 대안"이 여전히 없다.
    trunk = Trunk(width=0.6, depth=0.30 + 2 * MARGIN, height=0.4)
    state = ExtremePointState()
    f_big_right = PlacedBox(box=Box("F_BigRight", 0.22, 0.30, 0.15), x=0.38, y=0.0, z=0.0)
    state.register_placement(f_big_right)

    f_big_left = Box("F_BigLeft", width=0.18, depth=0.30, height=0.15)
    expected_x = 0.38 - 0.18 - MARGIN  # F_BigRight 바로 앞(x축)에서 MARGIN만큼 뗀 자리

    # 전제 확인: 이 자리가 실제로 겹치지 않는 유효한 자리인지
    assert is_candidate_valid(expected_x, MARGIN, 0.0, f_big_left, trunk, state.placed)
    # 전제 확인: 이 좌표가 순수 모서리 확장 + 벽 후보만으로는 없었는지 (버그 재현)
    m03 = import_module("cart2trunk_planning.core.extreme_point_candidates")
    wall_only_pool = state.candidates | m03.generate_wall_flush_candidates(f_big_left, trunk, state.candidates, margin=MARGIN)
    assert (expected_x, MARGIN, 0.0) not in wall_only_pool

    plan = place_one_box(f_big_left, trunk, state, order=1)

    assert plan is not None
    x, y, z = plan.position
    assert abs(x - expected_x) < 1e-9, f"F_BigRight 바로 앞(x={expected_x})이 아니라 x={x}에 배치됨"
