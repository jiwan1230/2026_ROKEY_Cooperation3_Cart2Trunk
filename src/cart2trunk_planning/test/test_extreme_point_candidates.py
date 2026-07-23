"""
test_03_extreme_point_candidates.py
③ register_placement()의 "장애물 사이 틈 미탐지" 버그를 재현하는 회귀 테스트.

배경: 12_verify_real_coords.py로 실제 트렁크 스캔 데이터를 돌려보니, 극점 알고리즘이
"자리 없음"이라 판단한 자리를 브루트포스(격자 전수조사)는 찾아냈다 (Medium 572곳,
Large 117곳). 원인은 register_placement()가 새로 놓인 박스 자기 자신의 모서리 3개만
후보로 추가하고, "이미 등록된 다른 장애물에 의해 생기는 틈"은 고려하지 않기 때문이다.

아래 테스트는 이 실패 패턴의 최소 재현이다.
"""
import sys, pathlib
from importlib import import_module

_m02 = import_module("cart2trunk_planning.core.trunk_space_state")
_m03 = import_module("cart2trunk_planning.core.extreme_point_candidates")
_m04 = import_module("cart2trunk_planning.core.candidate_validity_check")

Trunk = _m02.Trunk
Box = _m03.Box
PlacedBox = _m03.PlacedBox
ExtremePointState = _m03.ExtremePointState
is_candidate_valid = _m04.is_candidate_valid


def test_register_placement_finds_gap_behind_independent_obstacle():
    """
    장애물 A(왼쪽, 깊이 전체를 가로막음)와 장애물 B(A 뒤쪽 안쪽에만 있는 좁은 블록)를
    등록하면, "B의 오른쪽 면(x=0.3)을 앞쪽 벽(y=0)까지 밀어서 만든 자리" (0.3, 0, 0)에
    큰 박스가 들어갈 수 있어야 한다.

    이 좌표는 A의 모서리(A 자신의 y=0 안에서만 나옴)나 B의 모서리(B 자신의 y=0.5에서만
    나옴) 중 어느 쪽에서도 그대로 나오지 않는다 - B의 x면과 A가 없는 y=0을 조합해야만
    나오는 좌표라서, "자기 모서리 3개만 보는" 지금 로직은 절대 못 찾는다.
    """
    trunk = Trunk(width=1.0, depth=1.0, height=0.5)
    state = ExtremePointState()

    obstacle_a = PlacedBox(box=Box("A", width=0.15, depth=1.0, height=0.15), x=0.0, y=0.0, z=0.0)
    obstacle_b = PlacedBox(box=Box("B", width=0.15, depth=0.5, height=0.15), x=0.15, y=0.5, z=0.0)
    state.register_placement(obstacle_a)
    state.register_placement(obstacle_b)

    target_box = Box("Test", width=0.7, depth=0.5, height=0.15)
    expected_pos = (0.3, 0.0, 0.0)

    assert is_candidate_valid(*expected_pos, target_box, trunk, state.placed), (
        "테스트 전제 오류: 기대 좌표 자체가 유효한 자리가 아님"
    )
    assert expected_pos in state.candidates, (
        "장애물 B의 오른쪽 면을 앞쪽 벽까지 밀어서 만든 후보 (0.3, 0.0, 0.0)이 "
        "candidates에 없음 -> register_placement가 이 틈을 못 찾고 있음"
    )
