"""
test_05_candidate_scoring.py
⑤ score_candidate()의 "벽 우대" 로직(A/B/C 3단계)이 의도대로 동작하는지 검증.

배경: 사용자가 실제로 지적한 문제 - 지금까지 스코어링은 높이/접촉면만 보고 "입구"와
"안쪽"을 전혀 구분하지 않아서, 매번 로컬 원점(=하필 입구 쪽인 경우가 많음) 근처부터
채워버려 트렁크 입구를 막는 결과가 나왔다.

첫 버전은 entrance_distance_ratio를 x/y 두 축 평균으로 계산했는데, 이것도 버그였다 -
로봇은 x축(정해진 한 방향)으로만 트렁크에 접근하고 y축(좌우 위치)은 입구와 아예
무관한데, 평균을 내다 보니 y좌표만 달라도 점수가 달라지는 잘못된 결과가 나왔다.
그래서 entrance_distance_ratio는 x만 본다 (= 안쪽 벽 A에 대한 선호).

이후 사용자가 손그림으로 벽 3개를 지정해서 우대 순서를 정했다:
    A(가장 안쪽 벽, x=width) > B(측면 벽, y=depth) = C(측면 벽, y=0)
그래서 y축도 다시 쓰이게 됐는데, 이번엔 "입구 거리"가 아니라 "측면 벽에 붙는지"라는
별개의 개념으로 - entrance_distance_ratio(A)와는 독립된 side_wall_distance_ratio(B/C)
함수로 분리해서 반영한다.
"""
import sys, pathlib
from importlib import import_module

_m02 = import_module("cart2trunk_planning.core.trunk_space_state")
_m03 = import_module("cart2trunk_planning.core.extreme_point_candidates")
_m05 = import_module("cart2trunk_planning.core.candidate_scoring")

Trunk = _m02.Trunk
Box = _m03.Box
score_candidate = _m05.score_candidate
entrance_distance_ratio = _m05.entrance_distance_ratio
side_wall_distance_ratio = _m05.side_wall_distance_ratio
WALL_A_WEIGHT = _m05.WALL_A_WEIGHT
WALL_BC_WEIGHT = _m05.WALL_BC_WEIGHT


def test_score_prefers_candidate_farther_from_entrance_when_height_and_contact_tie():
    """
    높이(z=0)와 접촉면 수(둘 다 바닥에만 닿음, 벽/다른 박스와는 안 닿음)가 완전히 같은
    두 후보 중, 입구(로컬 x=0)에서 더 먼(=벽 A에 더 가까운) 쪽이 더 낮은(=더 좋은)
    점수를 받아야 한다. y는 일부러 똑같이 둬서 x 차이만으로 비교되게 한다.
    """
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)  # entrance_near_x 기본값 True(로컬 원점=입구)
    box = Box("unit", width=0.2, depth=0.2, height=0.2)

    near_entrance_score, near_touches = score_candidate(0.3, 0.3, 0.0, box, trunk, placed=[])
    far_from_entrance_score, far_touches = score_candidate(0.6, 0.3, 0.0, box, trunk, placed=[])

    # 접촉면 조건이 진짜로 동일한지 먼저 확인 - 아니면 벽 A 거리 항이 아니라 접촉면 차이 때문에
    # 점수가 달라진 것일 수 있어서, 이 전제가 깨지면 테스트 자체가 무의미해짐
    assert near_touches == far_touches == 1
    assert far_from_entrance_score < near_entrance_score


def test_entrance_distance_ratio_flips_when_entrance_is_on_far_side():
    """trunk.entrance_near_x가 False(입구가 반대쪽)면, 같은 x좌표라도 벽 A까지 거리 계산이 뒤집혀야 한다."""
    trunk_entrance_near = Trunk(width=1.0, depth=1.0, height=1.0, entrance_near_x=True)
    trunk_entrance_far = Trunk(width=1.0, depth=1.0, height=1.0, entrance_near_x=False)
    box = Box("unit", width=0.2, depth=0.2, height=0.2)

    ratio_when_entrance_near = entrance_distance_ratio(0.0, box, trunk_entrance_near)
    ratio_when_entrance_far = entrance_distance_ratio(0.0, box, trunk_entrance_far)

    assert ratio_when_entrance_near < ratio_when_entrance_far


def test_side_wall_distance_ratio_is_zero_when_touching_c():
    """박스가 y=0쪽 벽(C)에 딱 붙으면 side_wall_distance_ratio는 0(벽에 붙음)이어야 한다."""
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    box = Box("unit", width=0.2, depth=0.2, height=0.2)

    assert side_wall_distance_ratio(0.0, box, trunk) == 0.0


def test_side_wall_distance_ratio_is_zero_when_touching_b():
    """박스가 y=depth쪽 벽(B)에 딱 붙어도 마찬가지로 0이어야 한다 (B/C는 대칭)."""
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    box = Box("unit", width=0.2, depth=0.2, height=0.2)

    y_touching_b = trunk.depth - box.depth  # 박스 뒤쪽 끝이 정확히 y=depth에 닿는 좌표
    assert side_wall_distance_ratio(y_touching_b, box, trunk) == 0.0


def test_side_wall_distance_ratio_is_max_at_center():
    """박스가 B/C 양쪽에서 똑같이 먼(=정중앙) 위치면 side_wall_distance_ratio는 1(제일 멂)이어야 한다."""
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    box = Box("unit", width=0.2, depth=0.2, height=0.2)

    center_y = (trunk.depth - box.depth) / 2
    assert abs(side_wall_distance_ratio(center_y, box, trunk) - 1.0) < 1e-9


def test_score_prefers_candidate_near_side_wall_over_center():
    """
    x와 접촉면 조건이 같을 때, 측면 벽(B 또는 C)에 붙은 후보가 트렁크 중앙에 있는
    후보보다 더 낮은(=더 좋은) 점수를 받아야 한다 - 사용자가 요청한 B/C 우대 반영 확인.
    """
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    box = Box("unit", width=0.2, depth=0.2, height=0.2)

    center_y = (trunk.depth - box.depth) / 2
    at_wall_c_score, wall_touches = score_candidate(0.4, 0.0, 0.0, box, trunk, placed=[])
    at_center_score, center_touches = score_candidate(0.4, center_y, 0.0, box, trunk, placed=[])

    # x=0.4는 벽이 아니라서 두 후보 다 "y쪽 벽 접촉 여부"만 다르고 나머지 접촉 조건은 같아야 함
    assert wall_touches == center_touches + 1  # 벽쪽 후보만 y벽 접촉이 하나 더 있음
    assert at_wall_c_score < at_center_score


def test_wall_a_weighted_more_than_wall_bc():
    """
    "A(안쪽 벽)를 가장 높이, B/C(측면 벽)는 그보다 낮게" 우대해달라는 요청 검증.
    가중치 자체가 A > B/C 순서를 지키는지 직접 확인한다 (score_candidate가 이 두
    가중치를 그대로 곱해서 쓰므로, 이 순서가 실제 우선순위를 결정한다).
    """
    assert WALL_A_WEIGHT > WALL_BC_WEIGHT > 0
