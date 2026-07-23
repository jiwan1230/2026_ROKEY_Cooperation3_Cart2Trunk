"""
test_18_rotation.py
⑱ 회전(Yaw) 지원 검증.

배경: 박스가 정자세로 안 들어가면 지금까지는 그냥 미적재였다. 로봇 그리퍼는
박스를 눕히거나(가로/세로<->높이 교환) 뒤집는 건 불가능하고, 세운 채로 z축
기준 90도 돌리는 것(가로<->세로 교환)만 가능하다는 제약을 확인받아서, 정자세로
안 되면 90도 돌린 자세로 한 번 더 시도하도록 한다. 항상 정자세를 먼저 시도하고
(불필요한 회전 동작을 피하려고), 그래도 안 될 때만 회전판을 시도한다.
"""
import sys, pathlib
from importlib import import_module

_m02 = import_module("cart2trunk_planning.core.trunk_space_state")
_m03 = import_module("cart2trunk_planning.core.extreme_point_candidates")
_m07 = import_module("cart2trunk_planning.core.placement_plan")
_m08 = import_module("cart2trunk_planning.core.unloadable_reason")
_m18 = import_module("cart2trunk_planning.core.rotation")

Trunk = _m02.Trunk
Box = _m03.Box
ExtremePointState = _m03.ExtremePointState
place_one_box = _m07.place_one_box
classify_unloadable_reason = _m08.classify_unloadable_reason
UnloadableReason = _m08.UnloadableReason
rotate_box = _m18.rotate_box
fits_dims_any_rotation = _m18.fits_dims_any_rotation


def test_rotate_box_swaps_width_and_depth_only():
    box = Box("A", width=0.65, depth=0.30, height=0.15, mass_kg=2.0, is_fragile=True, rests_on_id="Base")
    rotated = rotate_box(box)
    assert rotated.width == 0.30
    assert rotated.depth == 0.65
    assert rotated.height == 0.15  # 높이는 절대 안 바뀜 (눕히기/뒤집기 불가)
    assert rotated.id == "A"
    assert rotated.mass_kg == 2.0
    assert rotated.is_fragile is True
    assert rotated.rests_on_id == "Base"


def test_fits_dims_any_rotation_true_when_only_rotated_fits():
    trunk = Trunk(width=0.6, depth=0.73, height=0.5)
    box = Box("Wide", width=0.65, depth=0.30, height=0.15)  # 정자세 폭 0.65 > 트렁크 폭 0.6
    assert fits_dims_any_rotation(box, trunk) is True


def test_fits_dims_any_rotation_false_when_neither_fits():
    trunk = Trunk(width=0.6, depth=0.6, height=0.5)
    box = Box("TooBig", width=0.65, depth=0.65, height=0.15)  # 돌려도 둘 다 트렁크보다 큼
    assert fits_dims_any_rotation(box, trunk) is False


def test_place_one_box_rotates_when_normal_orientation_does_not_fit():
    """정자세(가로 0.65)로는 안 들어가지만 90도 돌리면(가로 0.30) 들어가는 경우."""
    trunk = Trunk(width=0.6, depth=0.73, height=0.5)
    state = ExtremePointState()
    box = Box("Wide", width=0.65, depth=0.30, height=0.15)

    plan = place_one_box(box, trunk, state, order=1)

    assert plan is not None
    assert plan.box_id == "Wide"
    assert plan.rotated is True
    assert plan.dimensions == (0.30, 0.65, 0.15)  # 실제로 놓인 치수는 회전된 값


def test_place_one_box_prefers_normal_orientation_when_it_already_fits():
    """정자세로 이미 들어가면 굳이 회전 안 함 (불필요한 그리퍼 동작 회피)."""
    trunk = Trunk(width=1.0, depth=1.0, height=0.5)
    state = ExtremePointState()
    box = Box("Normal", width=0.3, depth=0.2, height=0.15)

    plan = place_one_box(box, trunk, state, order=1)

    assert plan is not None
    assert plan.rotated is False
    assert plan.dimensions == (0.3, 0.2, 0.15)


def test_place_one_box_still_fails_when_neither_orientation_fits():
    trunk = Trunk(width=0.5, depth=0.5, height=0.5)
    state = ExtremePointState()
    box = Box("TooBig", width=0.8, depth=0.8, height=0.15)

    plan = place_one_box(box, trunk, state, order=1)

    assert plan is None


def test_classify_unloadable_reason_not_size_exceeds_when_rotation_would_fit():
    """
    돌리면 트렁크 자체 크기엔 들어가는 박스는 SIZE_EXCEEDS_TRUNK(재배치해도 소용없음)로
    잘못 판단하면 안 된다 - 실제로 자리가 없어서 못 놓은 거라면 NO_VALID_CANDIDATE_POSITION
    이어야 재배치(reshuffle) 시도라도 해볼 수 있다.
    """
    trunk = Trunk(width=0.6, depth=0.3, height=0.5)
    state = ExtremePointState()
    # 폭 0.65는 트렁크 폭(0.6)보다 크지만, 돌리면 0.3 x 0.65 -> 깊이(0.3)보다 커서
    # 그래도 안 들어감(회전해도 트렁크 자체 크기를 넘음) - 이건 진짜 SIZE_EXCEEDS_TRUNK
    box_still_too_big = Box("StillTooBig", width=0.65, depth=0.65, height=0.15)
    assert classify_unloadable_reason(box_still_too_big, trunk, state) == UnloadableReason.SIZE_EXCEEDS_TRUNK

    # 돌리면 실제로 트렁크 크기 안에는 들어가는 경우 -> SIZE_EXCEEDS_TRUNK가 아니어야 함
    trunk2 = Trunk(width=0.6, depth=0.73, height=0.5)
    box_fits_rotated = Box("FitsRotated", width=0.65, depth=0.30, height=0.15)
    reason = classify_unloadable_reason(box_fits_rotated, trunk2, state)
    assert reason != UnloadableReason.SIZE_EXCEEDS_TRUNK
