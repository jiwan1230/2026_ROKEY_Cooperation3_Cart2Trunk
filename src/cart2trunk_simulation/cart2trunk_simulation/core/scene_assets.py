"""EDU 저장소 100.cart_to_trunk_dual_side_holonomic.py의 add_asset()/
add_sdf_collision()/bbox_of() 그대로 이관 - 외부 USD/USDZ 에셋(차량, 카트 등)을
씬에 참조로 붙이고, 메시에 SDF 콜리전을 입히고, 월드 AABB를 재는 범용 헬퍼.

cart2trunk_platform/platform_controller_node.py처럼 Isaac Sim 프로세스 안에서만
동작한다(pxr/omni 의존) - rclpy 없이도 순수 씬 조작 함수라 그 노드가 그대로
import해서 쓴다.
"""
import numpy as np
from pxr import PhysxSchema, Usd, UsdGeom, UsdPhysics


def add_asset(stage, prim_path, usd_path, position, extra_scale, target_mpu, target_up, rot_z=0.0):
    """usd_path의 에셋을 prim_path에 참조로 붙인다. 원본 에셋의 단위(meters-per-unit)/
    up-axis가 현재 스테이지와 다를 수 있어(외부에서 받아온 usdz 등), 그 차이를
    스케일/회전으로 보정한다."""
    src_stage = Usd.Stage.Open(usd_path)
    src_mpu = UsdGeom.GetStageMetersPerUnit(src_stage)
    src_up = UsdGeom.GetStageUpAxis(src_stage)
    scale = (src_mpu / target_mpu if target_mpu else src_mpu) * extra_scale
    xform = UsdGeom.Xform.Define(stage, prim_path)
    prim = xform.GetPrim()
    prim.GetReferences().AddReference(usd_path)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(position)
    if rot_z:
        xform.AddRotateZOp().Set(rot_z)
    if src_up == UsdGeom.Tokens.y and target_up == UsdGeom.Tokens.z:
        xform.AddRotateXOp().Set(90.0)
    xform.AddScaleOp().Set((scale, scale, scale))
    return xform


def add_sdf_collision(stage, root_prim_path, sdf_resolution=256):
    """root_prim_path 아래 모든 Mesh 프림에 SDF(signed distance field) 콜리전을
    입힌다 - 외부 에셋(차량/카트 등)은 볼록 껍질(convex hull) 콜리전만으로는
    트렁크 내부 같은 오목한 형상을 제대로 표현 못 해서, 원본 메시 형상 그대로
    콜리전을 만드는 SDF 방식이 필요하다."""
    root_prim = stage.GetPrimAtPath(root_prim_path)
    n = 0
    for prim in Usd.PrimRange(root_prim):
        if prim.GetTypeName() == "Mesh":
            UsdPhysics.CollisionAPI.Apply(prim)
            mc = UsdPhysics.MeshCollisionAPI.Apply(prim)
            mc.CreateApproximationAttr().Set("sdf")
            sdf_api = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(prim)
            sdf_api.CreateSdfResolutionAttr().Set(sdf_resolution)
            n += 1
    print(f"[SDF] {root_prim_path}: {n} mesh", flush=True)
    return n


def bbox_of(stage, prim_path):
    """prim_path의 현재 월드 AABB(min, max) - 스폰 후 실제 크기/위치를 코드가
    직접 재서 뒤따르는 배치 계산(카트 중심 좌표 등)에 쓴다."""
    prim = stage.GetPrimAtPath(prim_path)
    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    bbox = bbox_cache.ComputeWorldBound(prim)
    rng = bbox.ComputeAlignedRange()
    return np.array(rng.GetMin()), np.array(rng.GetMax())
