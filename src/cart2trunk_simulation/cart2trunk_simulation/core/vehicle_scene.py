"""EDU 저장소 100.py의 차량(트렁크) 스폰 부분 이관 - "씬 구성" 섹션의
add_asset(.../World/Vehicle, CAR_USD, ...) + add_sdf_collision() 그대로.

100.py가 여기 덧붙였던 ArtificialBackWall(트렁크 안쪽 벽 메시가 실측 도달범위보다
얕아서 그 오차를 메우려고 세운 얇은 가상 벽)은 옮기지 않았다 - 그건 100.py의
STAGE 3.2.1(정밀 접근 모션)이 이 정확한 트렁크 메시/스케일 조합에서 실측으로
튜닝한 보정값이라, cart2trunk_motion이 아직 그 정밀 접근 단계 자체를 구현하기
전인 지금 먼저 들여오면 근거 없는 상수만 남는다 - 그 모션을 포팅할 때 같이
들여올 것.
"""
from pxr import Gf, UsdGeom

from cart2trunk_simulation.core.scene_assets import add_asset, add_sdf_collision


def build_vehicle(
    stage, simulation_app, usd_path: str, position=(5.0, 0.0, 0.0), extra_scale: float = 0.53,
    rot_z: float = 0.0, prim_path: str = "/World/Vehicle",
):
    """차량(열린 트렁크 포함) 에셋을 스폰하고 SDF 콜리전을 입힌다.

    simulation_app.update()를 몇 프레임 돌려주는 이유(100.py와 동일) - 참조 붙인
    직후에는 USD 컴포지션이 아직 완전히 로드되지 않아, add_sdf_collision()이
    Mesh 프림을 순회해도 하나도 못 찾을 수 있다."""
    target_mpu = UsdGeom.GetStageMetersPerUnit(stage)
    target_up = UsdGeom.GetStageUpAxis(stage)
    xform = add_asset(
        stage, prim_path, usd_path, Gf.Vec3d(*position), extra_scale, target_mpu, target_up,
        rot_z=rot_z)
    for _ in range(20):
        simulation_app.update()
    add_sdf_collision(stage, prim_path)
    return xform
