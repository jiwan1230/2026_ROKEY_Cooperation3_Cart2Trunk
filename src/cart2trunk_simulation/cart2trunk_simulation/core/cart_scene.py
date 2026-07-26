"""EDU 저장소 100.py의 "카트 + 박스 구성" 섹션 이관 - 카트 위에 3박스 적층
구조(Large1/Large2가 바닥, Medium이 Large1 위, Small이 Large2 위)를 물리
낙하시킨다. box_geometry.py/multiview_scan.py(cart2trunk_perception이 이미
포팅한 박스 검출 알고리즘)가 검증된 것과 똑같은 3-box 시나리오라, 이 씬으로
검출 파이프라인을 실물 스캔처럼 검증할 수 있다.

여기 있는 크기/간격/오프셋 상수들은 전부 100.py가 여러 차례(2026-07-26,
2026-07-27) 실측으로 튜닝한 값 그대로다 - 임의로 바꾸지 말 것(각 상수 옆 주석에
왜 그 값인지 원본 그대로 남겨둠).

이관하지 않은 것: 카트 옆 접근 standoff/CART_CLEAR_X(회전 안전지대)/듀얼사이드
XOR 접근 로직 - 그건 "로봇이 카트 어느 쪽에서 집을지" 판단하는 모션/알고리즘
문제라 cart2trunk_motion이 실제로 그 접근 로직을 구현할 때 같이 가져올 것.
이 모듈은 순수하게 "카트+박스가 물리적으로 존재한다"까지만 책임진다.
"""
from dataclasses import dataclass

import numpy as np
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from isaacsim.core.api.objects import DynamicCuboid
from pxr import Gf, UsdGeom

from cart2trunk_simulation.core.scene_assets import add_asset, add_sdf_collision, bbox_of

# 카트 바스켓 바닥의 world z(카트 자체 스케일/스폰 기준 실측값) - 카트 원점이
# 아니라 실제로 박스가 앉는 바스켓 안쪽 바닥 높이.
CART_BASKET_FLOOR_Z = 0.68

CART_LARGE_SIZE_XY = 0.18  # Large의 x/y 한 변
CART_LARGE_GAP_M = 0.15  # 낙하 중 두 Large가 모서리로 부딪혀 넘어지지 않을 만큼 넉넉한 간격
_CART_LARGE_DX = (CART_LARGE_SIZE_XY + CART_LARGE_GAP_M) / 2.0

# Medium/Small을 한 Large 위에만 하나씩 올린다(둘 다 올리면 노출면이 얇은
# 테두리만 남아 RANSAC 검출이 불안정해짐 - 실측 확인).
_MEDIUM_SIZE = (0.085, 0.085, 0.11)
_SMALL_SIZE = (0.07, 0.08, 0.07)
CART_STACK_BASE_NAMES = ("Large1", "Large2")
_STACK_PARENT = {"Medium": "Large1", "Small": "Large2"}

# (name, size(x,y,z), 카트 중심 기준 offset(dx=카트 길이축, dy=카트 폭축), mass_kg)
CART_BOX_SPECS = [
    ("Large1", (CART_LARGE_SIZE_XY, CART_LARGE_SIZE_XY, 0.12), (-_CART_LARGE_DX, 0.0), 1.2),
    ("Large2", (CART_LARGE_SIZE_XY, CART_LARGE_SIZE_XY, 0.12), (_CART_LARGE_DX, 0.0), 1.2),
    ("Medium", _MEDIUM_SIZE, (-_CART_LARGE_DX, 0.0), 0.6),
    ("Small", _SMALL_SIZE, (_CART_LARGE_DX, 0.0), 0.3),
]

CART_BOX_DROP_HEIGHT_ABOVE_FLOOR = 0.07  # 낙하 높이(바닥 기준) - 너무 높으면 착지 충격/바운스가 큼
_CART_STACK_TOP_SPAWN_MARGIN_M = 0.05
CART_BOX_FRONT_SHIFT_M = 0.07  # 박스 그룹 전체를 카트 앞쪽(입구쪽)으로 이동
CART_LARGE1_EXTRA_FRONT_SHIFT_M = 0.04  # Large1(+Medium)이 손잡이쪽 턱에 걸려서 추가 이동
_EXTRA_FRONT_SHIFT_BY_NAME = {
    "Large1": CART_LARGE1_EXTRA_FRONT_SHIFT_M, "Medium": CART_LARGE1_EXTRA_FRONT_SHIFT_M,
}


@dataclass
class CartSceneResult:
    cart_prim_path: str
    cart_min: np.ndarray
    cart_max: np.ndarray
    cart_center_xy: tuple
    cart_half_x: float
    cart_half_y: float
    box_objects: dict  # prim_path -> DynamicCuboid
    box_known_size: dict  # prim_path -> (sx, sy, sz) 스폰 시점 크기


def build_cart_with_boxes(
    stage, simulation_app, usd_path: str, position=(0.0, 0.0, 0.0), extra_scale: float = 0.55,
    rot_z: float = 90.0, basket_floor_z: float = CART_BASKET_FLOOR_Z,
    prim_path: str = "/World/ShoppingCart",
) -> CartSceneResult:
    target_mpu = UsdGeom.GetStageMetersPerUnit(stage)
    target_up = UsdGeom.GetStageUpAxis(stage)
    add_asset(
        stage, prim_path, usd_path, Gf.Vec3d(*position), extra_scale, target_mpu, target_up,
        rot_z=rot_z)
    for _ in range(20):
        simulation_app.update()
    add_sdf_collision(stage, prim_path)

    cart_min, cart_max = bbox_of(stage, prim_path)
    cart_center_xy = ((cart_min[0] + cart_max[0]) / 2.0, (cart_min[1] + cart_max[1]) / 2.0)
    cart_half_x = (cart_max[0] - cart_min[0]) / 2.0
    cart_half_y = (cart_max[1] - cart_min[1]) / 2.0
    print(f"[카트 bbox] min={cart_min} max={cart_max} center_xy={cart_center_xy} "
          f"half_x={cart_half_x:.3f} half_y={cart_half_y:.3f}", flush=True)

    box_material = PhysicsMaterial(
        prim_path="/World/Physics_Materials/box_material",
        static_friction=1.2, dynamic_friction=1.0, restitution=0.0,
    )

    size_by_name = {name: size for name, size, _off, _m in CART_BOX_SPECS}
    large_spawn_z = basket_floor_z + CART_BOX_DROP_HEIGHT_ABOVE_FLOOR
    box_objects = {}
    for name, size, (dx, dy), mass_kg in CART_BOX_SPECS:
        if name in CART_STACK_BASE_NAMES:
            spawn_z = large_spawn_z
        else:
            parent_size = size_by_name[_STACK_PARENT[name]]
            spawn_z = (large_spawn_z + parent_size[2] / 2.0 + size[2] / 2.0
                       + _CART_STACK_TOP_SPAWN_MARGIN_M)
        box_prim_path = f"/World/Box_{name}"
        front_shift = CART_BOX_FRONT_SHIFT_M + _EXTRA_FRONT_SHIFT_BY_NAME.get(name, 0.0)
        box_objects[box_prim_path] = DynamicCuboid(
            prim_path=box_prim_path, name=name.lower(),
            position=np.array([
                cart_center_xy[0] + dx + front_shift,
                cart_center_xy[1] + dy, spawn_z,
            ]),
            scale=np.array(size), color=np.array([0.85, 0.55, 0.15]), mass=mass_kg,
            physics_material=box_material,
        )
    print(f"[박스 배치] 카트 안에 적층 구조 {len(CART_BOX_SPECS)}개 낙하 예정 "
          f"(바닥=Large1/Large2, Large1 위에 Medium, Large2 위에 Small)", flush=True)

    box_known_size = {f"/World/Box_{name}": size for name, size, _off, _m in CART_BOX_SPECS}

    return CartSceneResult(
        cart_prim_path=prim_path, cart_min=cart_min, cart_max=cart_max,
        cart_center_xy=cart_center_xy, cart_half_x=cart_half_x, cart_half_y=cart_half_y,
        box_objects=box_objects, box_known_size=box_known_size,
    )
