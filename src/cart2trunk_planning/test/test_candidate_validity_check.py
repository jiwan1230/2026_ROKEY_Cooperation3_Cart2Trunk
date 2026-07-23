"""
test_04_candidate_validity_check.py
④ is_candidate_valid()이 트렁크 경계의 "아래쪽"(x<0, y<0, z<0)도 제대로
거부하는지 검증.

배경: 지금까지는 x+width<=trunk.width 같은 윗쪽 경계만 확인하고, x>=0 같은
아래쪽 경계는 전혀 확인하지 않았다. register_placement()가 만드는 후보는 항상
0에서 시작해서 이 구멍이 그동안 드러나지 않았는데, ③에 새로 추가한
generate_box_flush_candidates()가 "박스 폭이 앞쪽 빈 틈보다 넓으면" 음수 좌표를
계산해낼 수 있다는 게 실제로 발견됨 (y=-0.3인데도 유효한 후보로 통과했었음).
"""
import sys, pathlib
from importlib import import_module

_m02 = import_module("cart2trunk_planning.core.trunk_space_state")
_m03 = import_module("cart2trunk_planning.core.extreme_point_candidates")
_m04 = import_module("cart2trunk_planning.core.candidate_validity_check")

Trunk = _m02.Trunk
Box = _m03.Box
is_candidate_valid = _m04.is_candidate_valid


def test_negative_x_is_rejected():
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    box = Box("unit", width=0.2, depth=0.2, height=0.2)
    assert is_candidate_valid(-0.1, 0.0, 0.0, box, trunk, placed=[]) is False


def test_negative_y_is_rejected():
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    box = Box("unit", width=0.2, depth=0.2, height=0.2)
    assert is_candidate_valid(0.0, -0.3, 0.0, box, trunk, placed=[]) is False


def test_negative_z_is_rejected():
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    box = Box("unit", width=0.2, depth=0.2, height=0.2)
    assert is_candidate_valid(0.0, 0.0, -0.05, box, trunk, placed=[]) is False


def test_origin_corner_still_valid():
    """회귀 방지 - 경계값(0,0,0)은 여전히 유효해야 한다 (부동소수점 여유값 유지 확인)."""
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    box = Box("unit", width=0.2, depth=0.2, height=0.2)
    assert is_candidate_valid(0.0, 0.0, 0.0, box, trunk, placed=[]) is True
