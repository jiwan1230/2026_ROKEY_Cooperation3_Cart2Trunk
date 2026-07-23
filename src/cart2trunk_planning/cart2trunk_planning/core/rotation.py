"""
18_rotation.py
⑱ 회전(Yaw Rotation) 지원
==========================
상태: 🟡 신규 (7/22)

[배경]
회전이 필요한 경우(가로/세로를 바꾸면 들어가는 박스)에도 지금까지는 시도조차
안 하고 그냥 미적재 처리했다 (③의 fits_dims()에 "회전은 고려 안 함 - MVP 범위"로
명시돼 있던 한계). 확인 결과 로봇 그리퍼는 박스를 눕히거나(높이를 가로/세로와
맞바꾸기) 뒤집는 건 불가능하고, 세운 채로 **z축 기준 90도 돌리는 것(가로<->세로
교환)만** 가능하다.

[이 파일이 하는 것 / 안 하는 것]
- rotate_box(): 가로/세로만 맞바꾼 새 Box를 만든다. 높이는 절대 안 바뀐다.
- fits_dims_any_rotation(): 정자세든 90도 돌린 자세든 트렁크 자체 크기 안에는
  들어갈 수 있는지 확인한다 (⑧의 SIZE_EXCEEDS_TRUNK 판단에 씀 - 자리 배치와는
  무관한, "애초에 가능한가"만 보는 체크).
- "언제 실제로 돌릴지"는 여기서 정하지 않는다 - ⑦(place_one_box)이 정자세로
  먼저 시도해보고, 그래도 자리가 없을 때만 회전판을 다시 시도한다 (회전은
  그리퍼 동작이 하나 더 필요해서, 꼭 필요할 때만 쓴다).
"""

from importlib import import_module

_m03 = import_module("cart2trunk_planning.core.extreme_point_candidates")

Box = _m03.Box
fits_dims = _m03.fits_dims


def rotate_box(box: "Box") -> "Box":
    """가로/세로를 맞바꾼 새 Box를 반환한다 (높이·id·질량 등 나머지는 그대로)."""
    return Box(
        id=box.id,
        width=box.depth,
        depth=box.width,
        height=box.height,
        mass_kg=box.mass_kg,
        is_fragile=box.is_fragile,
        rests_on_id=box.rests_on_id,
    )


def fits_dims_any_rotation(box: "Box", trunk) -> bool:
    """정자세든 90도 돌린 자세든 트렁크 자체 크기 안에 들어갈 수 있는지 확인."""
    return fits_dims(box, trunk) or fits_dims(rotate_box(box), trunk)


if __name__ == "__main__":
    _m02 = import_module("cart2trunk_planning.core.trunk_space_state")
    Trunk = _m02.Trunk

    trunk = Trunk(width=0.6, depth=0.73, height=0.5)
    box = Box("Wide", width=0.65, depth=0.30, height=0.15)
    print(f"정자세(가로 {box.width}m x 세로 {box.depth}m) 트렁크(폭 {trunk.width}m)에 들어감:", fits_dims(box, trunk))
    rotated = rotate_box(box)
    print(f"90도 돌리면(가로 {rotated.width}m x 세로 {rotated.depth}m) 들어감:", fits_dims(rotated, trunk))
