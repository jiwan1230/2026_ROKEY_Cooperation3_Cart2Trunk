#!/usr/bin/env python3
"""
box_geometry.py
box_top_extractor.py(단일 시점, 카메라 좌표계 전용 라이브 ROS 도구)의 순수 기하 검출
로직을, 여러 시점을 base_link 좌표계로 합친 point cloud에도 쓸 수 있도록 프레임에
무관하게 새로 정리한 모듈.

box_top_extractor.py는 이 모듈이 존재하기 전부터 12개 이상의 다른 스크립트
(32/33/36/38~44.py 등)가 그대로 의존하고 있어서, 이번 다중 시점 작업 때문에 그
파일의 동작을 조금이라도 바꾸는 위험을 지지 않기로 했다 - 그래서 이 모듈은
box_top_extractor.py를 "리팩터링해서 공유"하는 대신, 같은 검출 원리를 프레임
파라미터를 받는 형태로 다시 구현한다(로직은 동일, import 관계는 없음).

box_top_extractor.py와 다른 점은 딱 하나: 카메라 좌표계에서는 "박스 윗면이 어느
방향을 향하는지"를 알 수 없어서(카메라가 얼마나 기울었는지에 따라 다름) "카메라를
향하는 평면" 필터 + "그 중 대표 법선을 뽑아 나머지를 비교"하는 2단계 동적 필터를
써야 했다 - 이 대표 법선 재선정 로직이 실제로 오탐/미검출의 최대 원인이었다
(perception/STACKED_BOX_DETECTION_DEBUG_GUIDE.md 참고). 여러 시점을 base_link
좌표계(Z가 실제로 위쪽)로 합치면 "박스 윗면 법선은 항상 (0,0,1) 근처"라는 절대
기준이 생기므로, 그 2단계 동적 필터를 "up_vector와의 내적 ≥ 임계값" 단일 필터로
교체할 수 있다 - detect_box_top_candidates_fixed_up()이 그 버전이다.

나머지(전처리, DBSCAN 클러스터링, 사각형 채움비 검사, 아래 지지면 ray-cast 매칭,
8꼭짓점 복원, 표면 복원)는 프레임에 의존하지 않는 순수 기하 연산이라 그대로 옮겼다.
"""

from __future__ import annotations

import copy
from collections import Counter
from dataclasses import dataclass
from typing import Callable, Optional

import cv2
import numpy as np
import open3d as o3d


# ============================================================
# Open3D 전처리 (box_top_extractor.py의 VOXEL_SIZE_M 등과 동일 기본값)
# ============================================================

VOXEL_SIZE_M = 0.005
OUTLIER_NB_NEIGHBORS = 20
OUTLIER_STD_RATIO = 2.0


# ============================================================
# RANSAC 평면 검출 (box_top_extractor.py와 동일 기본값)
# ============================================================

PLANE_DISTANCE_THRESHOLD_M = 0.010
PLANE_RANSAC_N = 3
PLANE_RANSAC_ITERATIONS = 500
MIN_PLANE_POINTS = 150
MAX_PLANES = 12

# 단일 시점(카메라 좌표계) 경로 전용 - box_top_extractor.py와 동일
CAMERA_FACING_NORMAL_DOT_MIN = 0.70
TOP_NORMAL_CONSISTENCY_DOT_MIN = 0.94
REFERENCE_NORMAL_ALIGNMENT_POWER = 2.0

# 다중 시점(base_link 좌표계) 경로 전용 - 절대 위쪽(0,0,1)과의 내적 임계값.
# TOP_NORMAL_CONSISTENCY_DOT_MIN(0.94, 약 20도)과 같은 엄격도로 시작한다.
UP_FACING_NORMAL_DOT_MIN = 0.94


# ============================================================
# 평면 내부의 박스 윗면 분리
# ============================================================

DBSCAN_EPS_M = 0.025
DBSCAN_MIN_POINTS = 15
MIN_CLUSTER_POINTS = 70

MIN_BOX_SIDE_M = 0.04
MAX_BOX_SIDE_M = 1.50
MIN_RECTANGULAR_FILL_RATIO = 0.50
MAX_SIDE_ASPECT_RATIO = 8.0


# ============================================================
# 8개 꼭짓점 계산 / 지지면 매칭
# ============================================================

MIN_RAY_DISTANCE_M = 0.020
MAX_RAY_DISTANCE_M = 1.00
MAX_RAY_DISTANCE_SPREAD_M = 0.060
BOUNDARY_HIT_DISTANCE_M = 0.040
MIN_BOUNDARY_RAY_HITS = 3
SUPPORT_NORMAL_DOT_MIN = 0.90
BOTTOM_CUT_MARGIN_M = 0.010

# "위 박스는 반드시 아래 박스보다 작거나 같다" 물리적 사전 지식을 지지면 선택의
# 타이브레이크로 쓴다 - 완전 배제가 아니라, 이 조건을 만족하는 후보가 있으면
# 우선하고 없으면(예: 관측 노이즈로 살짝 작게 잡힌 경우) 기존 순위를 그대로 쓴다.
SUPPORT_MIN_AREA_RATIO = 0.85

# 지지면 후보 중 관측 점 수가 가장 많은 후보 대비 이 비율 미만인 것은 RANSAC이
# 같은 평면을 쪼개서 생긴 노이즈 조각일 가능성이 높다고 보고 순위에서 뒤로 미룬다.
SUPPORT_SIZE_PREFERENCE_RATIO = 0.3

FLOOR_RANSAC_MAX_PLANES = 8
FLOOR_MIN_POINTS = 300
FLOOR_DISTANCE_THRESHOLD_M = 0.012
FLOOR_RANSAC_ITERATIONS = 1200
FLOOR_NORMAL_DOT_MIN = 0.94
FLOOR_MIN_AREA_M2 = 0.12
FLOOR_BOUNDARY_HIT_DISTANCE_M = 0.30

MIN_BOX_HEIGHT_M = 0.035
MAX_BOX_HEIGHT_M = 1.00

COMPLETED_SURFACE_POINT_SPACING_M = 0.005
COMPLETED_SURFACE_VOXEL_SIZE_M = 0.003


DebugLog = Optional[Callable[[str], None]]


@dataclass
class PlaneClusterCandidate:
    candidate_id: int
    points: np.ndarray

    normal: np.ndarray
    median_depth: float

    width: float
    height: float
    area: float
    fill_ratio: float

    center: np.ndarray
    plane_d: float
    corners_3d: np.ndarray
    pixel_polygon: Optional[np.ndarray]

    # find_hidden_stacked_box()가 만들어낸 "관측된 독립 평면이 아니라, 추정으로
    # 복원한" 후보인지 표시한다 - RANSAC/DBSCAN을 거치지 않았으므로 fill_ratio 등
    # 일부 필드는 의미가 없거나 근사값이다. 기본값 False라 기존 생성 코드는 전혀
    # 안 바뀐다.
    synthetic: bool = False


def _normalize(v: np.ndarray, eps: float = 1e-8) -> np.ndarray:
    v = np.asarray(v, dtype=np.float64)
    n = float(np.linalg.norm(v))
    if n < eps:
        raise ValueError(f"영벡터는 방향으로 사용할 수 없습니다: {v}")
    return v / n


def preprocess_cloud(
    points: np.ndarray,
    voxel_size: float = VOXEL_SIZE_M,
    outlier_nb_neighbors: int = OUTLIER_NB_NEIGHBORS,
    outlier_std_ratio: float = OUTLIER_STD_RATIO,
) -> o3d.geometry.PointCloud:
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    pcd = pcd.voxel_down_sample(voxel_size=voxel_size)

    if len(pcd.points) >= outlier_nb_neighbors:
        pcd, _ = pcd.remove_statistical_outlier(
            nb_neighbors=outlier_nb_neighbors,
            std_ratio=outlier_std_ratio,
        )

    return pcd


def make_plane_basis(normal: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    normal = normal / np.linalg.norm(normal)

    reference = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    if abs(float(np.dot(reference, normal))) > 0.9:
        reference = np.array([0.0, 1.0, 0.0], dtype=np.float64)

    axis_u = reference - np.dot(reference, normal) * normal
    axis_u /= np.linalg.norm(axis_u)

    axis_v = np.cross(normal, axis_u)
    axis_v /= np.linalg.norm(axis_v)

    return axis_u, axis_v


def order_rectangle_corners(corners: np.ndarray) -> np.ndarray:
    """4개 꼭짓점을 둘레 순서로 정렬한다."""
    center = corners.mean(axis=0)

    edge_1 = corners[1] - corners[0]
    edge_2 = corners[2] - corners[0]
    normal = np.cross(edge_1, edge_2)
    normal_norm = np.linalg.norm(normal)

    if normal_norm < 1e-8:
        return corners

    normal /= normal_norm
    axis_u = edge_1 / max(np.linalg.norm(edge_1), 1e-8)
    axis_v = np.cross(normal, axis_u)
    axis_v /= max(np.linalg.norm(axis_v), 1e-8)

    relative = corners - center
    angles = np.arctan2(relative @ axis_v, relative @ axis_u)
    return corners[np.argsort(angles)]


def make_candidate(
    candidate_id: int,
    cluster: o3d.geometry.PointCloud,
    plane_normal: np.ndarray,
    *,
    min_cluster_points: int = MIN_CLUSTER_POINTS,
    min_box_side_m: float = MIN_BOX_SIDE_M,
    max_box_side_m: float = MAX_BOX_SIDE_M,
    max_side_aspect_ratio: float = MAX_SIDE_ASPECT_RATIO,
    min_rectangular_fill_ratio: float = MIN_RECTANGULAR_FILL_RATIO,
    project_points_fn: Optional[Callable[[np.ndarray], Optional[np.ndarray]]] = None,
    debug: bool = False,
    debug_log: DebugLog = None,
    debug_tag: str = "",
) -> Optional[PlaneClusterCandidate]:
    points = np.asarray(cluster.points, dtype=np.float64)

    if len(points) < min_cluster_points:
        return None

    center = points.mean(axis=0)
    axis_u, axis_v = make_plane_basis(plane_normal)

    relative = points - center
    local_u = relative @ axis_u
    local_v = relative @ axis_v
    points_2d = np.column_stack((local_u, local_v)).astype(np.float32)

    rectangle = cv2.minAreaRect(points_2d)
    rect_center, rect_size, _ = rectangle
    side_a = float(rect_size[0])
    side_b = float(rect_size[1])

    if side_a <= 0.0 or side_b <= 0.0:
        return None

    width = max(side_a, side_b)
    height = min(side_a, side_b)

    if not (min_box_side_m <= width <= max_box_side_m):
        if debug and debug_log:
            debug_log(
                f"[DEBUG make_candidate]{debug_tag} points={len(points)} "
                f"center={np.round(center, 3).tolist()}: width={width:.3f} "
                f"out of [{min_box_side_m},{max_box_side_m}] -> reject"
            )
        return None

    if not (min_box_side_m <= height <= max_box_side_m):
        if debug and debug_log:
            debug_log(
                f"[DEBUG make_candidate]{debug_tag} points={len(points)} "
                f"center={np.round(center, 3).tolist()}: height={height:.3f} "
                f"out of [{min_box_side_m},{max_box_side_m}] -> reject"
            )
        return None

    aspect_ratio = width / max(height, 1e-8)
    if aspect_ratio > max_side_aspect_ratio:
        if debug and debug_log:
            debug_log(
                f"[DEBUG make_candidate]{debug_tag} points={len(points)} "
                f"center={np.round(center, 3).tolist()}: aspect_ratio={aspect_ratio:.2f} "
                f"> {max_side_aspect_ratio} (width={width:.3f}, height={height:.3f}) -> reject"
            )
        return None

    rectangle_area = width * height

    hull = cv2.convexHull(points_2d)
    observed_area = float(cv2.contourArea(hull))
    fill_ratio = observed_area / max(rectangle_area, 1e-8)

    if fill_ratio < min_rectangular_fill_ratio:
        if debug and debug_log:
            debug_log(
                f"[DEBUG make_candidate]{debug_tag} points={len(points)} "
                f"center={np.round(center, 3).tolist()} footprint=({width:.3f},{height:.3f}): "
                f"fill_ratio={fill_ratio:.3f} < {min_rectangular_fill_ratio} -> reject"
            )
        return None

    rectangle_points_2d = cv2.boxPoints(rectangle).astype(np.float64)
    corners_3d = (
        center[None, :]
        + rectangle_points_2d[:, 0:1] * axis_u[None, :]
        + rectangle_points_2d[:, 1:2] * axis_v[None, :]
    )

    pixel_polygon = project_points_fn(corners_3d) if project_points_fn is not None else None
    median_depth = float(np.median(points[:, 2]))
    plane_d = -float(np.dot(plane_normal, center))

    return PlaneClusterCandidate(
        candidate_id=candidate_id,
        points=points.astype(np.float32),
        normal=plane_normal.copy(),
        median_depth=median_depth,
        width=width,
        height=height,
        area=rectangle_area,
        fill_ratio=fill_ratio,
        center=center.astype(np.float64),
        plane_d=plane_d,
        corners_3d=corners_3d.astype(np.float64),
        pixel_polygon=pixel_polygon,
    )


def _cluster_plane_into_candidates(
    plane_cloud: o3d.geometry.PointCloud,
    normal: np.ndarray,
    *,
    candidate_id_start: int,
    dbscan_eps_m: float,
    dbscan_min_points: int,
    min_cluster_points: int,
    min_box_side_m: float,
    max_box_side_m: float,
    max_side_aspect_ratio: float,
    min_rectangular_fill_ratio: float,
    project_points_fn,
    debug: bool,
    debug_log: DebugLog,
    debug_tag: str,
) -> list[PlaneClusterCandidate]:
    labels = np.asarray(
        plane_cloud.cluster_dbscan(eps=dbscan_eps_m, min_points=dbscan_min_points, print_progress=False)
    )
    if labels.size == 0:
        return []

    out = []
    candidate_id = candidate_id_start
    for label in np.unique(labels):
        if label < 0:
            continue
        cluster_indices = np.flatnonzero(labels == label)
        if len(cluster_indices) < min_cluster_points:
            if debug and debug_log and len(cluster_indices) >= 10:
                cluster_pts = np.asarray(plane_cloud.points)[cluster_indices]
                debug_log(
                    f"[DEBUG cluster]{debug_tag} label={label}: points={len(cluster_indices)} "
                    f"< MIN_CLUSTER_POINTS({min_cluster_points}) "
                    f"center={np.round(cluster_pts.mean(axis=0), 3).tolist()} -> drop"
                )
            continue

        cluster = plane_cloud.select_by_index(cluster_indices.tolist())
        candidate = make_candidate(
            candidate_id=candidate_id,
            cluster=cluster,
            plane_normal=normal,
            min_cluster_points=min_cluster_points,
            min_box_side_m=min_box_side_m,
            max_box_side_m=max_box_side_m,
            max_side_aspect_ratio=max_side_aspect_ratio,
            min_rectangular_fill_ratio=min_rectangular_fill_ratio,
            project_points_fn=project_points_fn,
            debug=debug,
            debug_log=debug_log,
            debug_tag=debug_tag,
        )
        if candidate is None:
            continue
        out.append(candidate)
        candidate_id += 1

    return out


def detect_box_top_candidates_fixed_up(
    scene_pcd: o3d.geometry.PointCloud,
    up_vector: np.ndarray,
    *,
    plane_distance_threshold_m: float = PLANE_DISTANCE_THRESHOLD_M,
    plane_ransac_n: int = PLANE_RANSAC_N,
    plane_ransac_iterations: int = PLANE_RANSAC_ITERATIONS,
    min_plane_points: int = MIN_PLANE_POINTS,
    max_planes: int = MAX_PLANES,
    up_facing_dot_min: float = UP_FACING_NORMAL_DOT_MIN,
    dbscan_eps_m: float = DBSCAN_EPS_M,
    dbscan_min_points: int = DBSCAN_MIN_POINTS,
    min_cluster_points: int = MIN_CLUSTER_POINTS,
    min_box_side_m: float = MIN_BOX_SIDE_M,
    max_box_side_m: float = MAX_BOX_SIDE_M,
    max_side_aspect_ratio: float = MAX_SIDE_ASPECT_RATIO,
    min_rectangular_fill_ratio: float = MIN_RECTANGULAR_FILL_RATIO,
    debug: bool = False,
    debug_log: DebugLog = None,
) -> list[PlaneClusterCandidate]:
    """다중 시점(base_link 좌표계) 전용 - "절대 위쪽"과의 내적만으로 윗면 평면을
    거른다. 단일 시점(box_top_extractor.py)이 쓰는 "카메라를 향하는 평면 + 대표
    법선 재선정" 2단계 동적 필터가 필요 없다 - 위쪽 방향을 이미 알기 때문이다."""
    up_vector = _normalize(up_vector)
    remaining = o3d.geometry.PointCloud(scene_pcd)

    candidates: list[PlaneClusterCandidate] = []
    candidate_id = 0

    for plane_index in range(max_planes):
        if len(remaining.points) < min_plane_points:
            break

        plane_model, inliers = remaining.segment_plane(
            distance_threshold=plane_distance_threshold_m,
            ransac_n=plane_ransac_n,
            num_iterations=plane_ransac_iterations,
        )

        if len(inliers) < min_plane_points:
            break

        plane_cloud = remaining.select_by_index(inliers)
        remaining = remaining.select_by_index(inliers, invert=True)

        normal = np.asarray(plane_model[:3], dtype=np.float64)
        normal_norm = float(np.linalg.norm(normal))
        if normal_norm < 1e-8:
            continue
        normal /= normal_norm

        if float(np.dot(normal, up_vector)) < 0.0:
            normal = -normal

        alignment = float(np.dot(normal, up_vector))
        if alignment < up_facing_dot_min:
            if debug and debug_log:
                pts = np.asarray(plane_cloud.points)
                debug_log(
                    f"[DEBUG plane] plane_idx={plane_index} points={len(inliers)} "
                    f"center={np.round(pts.mean(axis=0), 3).tolist()}: "
                    f"up_alignment={alignment:.3f} < {up_facing_dot_min} -> reject whole plane"
                )
            continue

        # 이 평면은 이미 "충분히 위를 향한다"는 검증을 통과했다 - 실측 확인:
        # RANSAC이 fit한 실제 법선은 완전한 수직이 아니라 몇 도(최대 ~20도, 매
        # 시도마다 방향까지 똑같이 일관됨 - 노이즈가 아니라 물체가 실제로 살짝
        # 기울어져 정착했거나 depth 관측 자체가 그 방향으로 살짝 치우친 것으로
        # 추정)씩 기울어질 수 있는데, 그 법선을 그대로 쓰면 minAreaRect의 2D 기준축
        # (axis_u/axis_v)까지 같이 기울어져서 복원된 윗면 4꼭짓점이 비스듬하게
        # 잘린 것처럼 나온다. 우리는 이미 "박스는 테이블/다른 박스 위에 수평으로
        # 놓인다"는 물리적 전제(up_vector를 절대 기준으로 아는 것 자체가 다중 시점
        # 설계의 핵심)를 갖고 있으므로, 평면 판정에만 실제 법선을 쓰고 이후 사각형
        # 복원은 절대 up_vector 기준으로 강제한다 - 노이즈든 실제 미세 기울임이든
        # 최종 8꼭짓점은 항상 평평한 직육면체가 된다.
        found = _cluster_plane_into_candidates(
            plane_cloud,
            up_vector,
            candidate_id_start=candidate_id,
            dbscan_eps_m=dbscan_eps_m,
            dbscan_min_points=dbscan_min_points,
            min_cluster_points=min_cluster_points,
            min_box_side_m=min_box_side_m,
            max_box_side_m=max_box_side_m,
            max_side_aspect_ratio=max_side_aspect_ratio,
            min_rectangular_fill_ratio=min_rectangular_fill_ratio,
            project_points_fn=None,
            debug=debug,
            debug_log=debug_log,
            debug_tag=f" plane_idx={plane_index}",
        )
        candidates.extend(found)
        candidate_id += len(found)

    return candidates


def get_down_direction_camera_relative(top_normal: np.ndarray) -> np.ndarray:
    """단일 시점(카메라 좌표계) 전용 - box_top_extractor.py의 get_down_direction()과
    동일: optical frame +Z(카메라 전방)에서 멀어지는 쪽을 아래로 삼는다."""
    direction = _normalize(top_normal)
    camera_forward = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    if float(np.dot(direction, camera_forward)) < 0.0:
        direction = -direction
    return direction


def select_support_candidate(
    top: PlaneClusterCandidate,
    candidates: list[PlaneClusterCandidate],
    down_direction: np.ndarray,
    *,
    support_normal_dot_min: float = SUPPORT_NORMAL_DOT_MIN,
    min_ray_distance_m: float = MIN_RAY_DISTANCE_M,
    max_ray_distance_m: float = MAX_RAY_DISTANCE_M,
    max_ray_distance_spread_m: float = MAX_RAY_DISTANCE_SPREAD_M,
    boundary_hit_distance_m: float = BOUNDARY_HIT_DISTANCE_M,
    min_boundary_ray_hits: int = MIN_BOUNDARY_RAY_HITS,
    support_min_area_ratio: Optional[float] = None,
    max_support_area_ratio: Optional[float] = None,
    allow_plane_only_fallback: bool = False,
    plane_only_bounds_margin_m: Optional[float] = None,
    debug: bool = False,
    debug_log: DebugLog = None,
) -> Optional[PlaneClusterCandidate]:
    """초록색 윗면의 네 꼭짓점에서 아래 방향으로 ray를 내리고, 네 ray가 가장 먼저
    만나는 평행 후보 평면을 경계면(다른 박스의 윗면)으로 고른다.

    support_min_area_ratio가 주어지면, "위 박스는 항상 아래 박스보다 작다"는
    적층 규칙을 타이브레이크로 쓴다 - ray 조건을 통과한 후보 중 top보다 충분히 큰
    후보가 있으면 그중 최선을, 없으면(관측 노이즈로 지지면이 작게 잡힌 경우 등)
    기존 순위 1위를 그대로 반환한다(완전 배제 아님).

    max_support_area_ratio가 주어지면, top 자신의 면적보다 이 배수 이상 큰 후보는
    아예 지지면 후보에서 제외한다 - 실측 확인(SAME_SIZE_STACK_DETECTION_LOG.md
    3-6절): 테이블 표면 전체가 우연히 "유효한 평면 후보"로 잡혀 있으면, 이 함수가
    그걸 정상적인 박스 지지면(support_type="box_top")으로 잘못 채택해버려서 실제로는
    "바닥까지 내려간" 것인데 라벨만 정상처럼 보이는 문제가 있었다. 진짜 박스 위에
    놓인 물체라면 지지면 크기가 top과 비슷한 자릿수여야 한다는 물리적 사실을 이용해
    테이블처럼 압도적으로 큰 평면을 미리 걸러낸다.

    allow_plane_only_fallback(기본 False, 하위호환 유지 - live 단일 시점 경로/
    box_top_extractor.py는 이 인자를 안 넘기므로 기존 동작 그대로)이 True일 때만:
    엄격한 검사(hit_count >= min_boundary_ray_hits, "그 지점에 실제 관측 포인트가
    있어야 함")를 통과하는 후보가 하나도 없으면, 이 포인트 근접 조건만 뺀 나머지
    (법선 정렬/거리 범위/편차/면적비) 기하 조건은 그대로 만족하는 후보들로 2차
    후보군을 다시 만들어 그중 최선을 대신 채택한다. 실측 확인(88.cart_scan_holonomic.py
    다중 시점 적층 시나리오): 한 박스(Large) 위에 다른 박스 2개(Medium/Small)가
    나란히 얹혀 서로 다른 영역을 가리면, Medium 바로 밑처럼 카메라가 원천적으로
    못 본 자리는 Large의 관측 포인트가 존재하지 않는다 - Large 자체는 다른 부분
    (가려지지 않은 앞쪽)에서 이미 확실한 박스 윗면으로 검출됐는데도, 이 특정
    top 아래쪽만 놓고 보면 "포인트가 없다"는 이유로 엄격한 검사가 실패해서 진짜
    바닥까지 뚫고 내려간 것으로 잘못 복원됐다(실측: 높이 0.11m여야 할 박스가
    0.21m로 나옴). 여러 박스가 하나의 큰 아래 박스를 나눠서 가리는 배치라면 이
    시나리오에 국한되지 않고 재현되는 일반적인 문제라, live 경로에는 아직 없는
    검증(오프라인 다중 시점 배치에서만 우선 적용 - 시점을 여러 번 합쳐 이미
    노이즈에 강하다는 전제가 있어야 안전하다)으로 분리해 켠다.

    plane_only_bounds_margin_m(기본 None=비활성, 하위호환 유지)이 주어지면
    plane_only_ranked 후보에 한해 추가 조건을 건다: ray-plane 교점이 그 후보
    자신의 관측된 사각형 XY 범위(+margin) 안에 들어야 한다. 실측 확인(88.py 5개
    적층 시나리오): allow_plane_only_fallback은 거리(ray_distance)만으로 순위를
    매겨서, 특정 trial에서 진짜 부모(M2)가 아예 후보로 안 잡히면 전혀 무관한 다른
    박스(M1 - 다른 부모 밑, 카트 반대편)의 평면이 "우연히 비슷한 ray_distance"라는
    이유만으로 지지면으로 잘못 채택되는 사례가 있었다(실측: 서로 다른 위치의 두
    자식(XS1, XS2)이 같은 trial에서 완전히 동일한 지지면 z값을 받음 - 사실은 둘
    다 XS2의 진짜 부모(M2) 대신 M1으로 스킵 매칭된 것이었다). candidate 자신의
    관측 범위 안에 교점이 있는지 보면 "위에 있는 척 하는 좁은 남의 박스"는
    걸러지면서도, Base처럼 실제로 그 아래를 넓게 받치는 진짜 지지 박스는(자기
    범위가 넓으므로) 그대로 통과한다."""
    top_corners = order_rectangle_corners(top.corners_3d.copy())
    down_direction = _normalize(down_direction)
    top_area = top.width * top.height

    ranked = []
    plane_only_ranked = []
    for candidate in candidates:
        if candidate.candidate_id == top.candidate_id:
            continue

        if max_support_area_ratio is not None and candidate.width * candidate.height > top_area * max_support_area_ratio:
            continue

        normal_dot = abs(float(np.dot(top.normal, candidate.normal)))
        if normal_dot < support_normal_dot_min:
            continue

        candidate_normal = candidate.normal.astype(np.float64)
        candidate_normal /= max(np.linalg.norm(candidate_normal), 1e-8)

        denominator = float(np.dot(candidate_normal, down_direction))
        if abs(denominator) < 1e-6:
            continue

        ray_distances = -(top_corners @ candidate_normal + float(candidate.plane_d)) / denominator
        if np.any(~np.isfinite(ray_distances)):
            continue
        if np.any(ray_distances < min_ray_distance_m):
            continue

        median_distance = float(np.median(ray_distances))
        if not (min_ray_distance_m <= median_distance <= max_ray_distance_m):
            continue

        spread = float(np.max(ray_distances) - np.min(ray_distances))
        if spread > max_ray_distance_spread_m:
            continue

        intersections = top_corners + ray_distances[:, None] * down_direction[None, :]

        if plane_only_bounds_margin_m is not None:
            cand_corners_2d = candidate.corners_3d[:, :2]
            cand_min = cand_corners_2d.min(axis=0) - plane_only_bounds_margin_m
            cand_max = cand_corners_2d.max(axis=0) + plane_only_bounds_margin_m
            within_bounds = np.all(
                (intersections[:, :2] >= cand_min) & (intersections[:, :2] <= cand_max), axis=1
            )
            if not np.any(within_bounds):
                continue

        plane_only_ranked.append((median_distance, spread, candidate))

        candidate_points = np.asarray(candidate.points, dtype=np.float64)
        if len(candidate_points) == 0:
            continue

        nearest_distances = np.asarray(
            [float(np.min(np.linalg.norm(candidate_points - p[None, :], axis=1))) for p in intersections],
            dtype=np.float64,
        )

        hit_count = int(np.count_nonzero(nearest_distances <= boundary_hit_distance_m))
        if hit_count < min_boundary_ray_hits:
            continue

        ranked.append((median_distance, -hit_count, spread, float(np.min(nearest_distances)), candidate))

    if not ranked and allow_plane_only_fallback and plane_only_ranked:
        plane_only_ranked.sort(key=lambda item: (item[0], item[1]))
        best = plane_only_ranked[0]
        if debug and debug_log:
            debug_log(
                f"[DEBUG box_top support] top={top.candidate_id}: 엄격한 포인트 근접 검사를 "
                f"통과한 후보가 없음(가려진 영역으로 추정) -> 평면 방정식만으로 candidate="
                f"{best[2].candidate_id} (median_distance={best[0]:.3f}) 채택"
            )
        return best[2]

    if not ranked:
        if debug and debug_log:
            debug_log(
                f"[DEBUG box_top support] top={top.candidate_id}: "
                "no other top candidate qualifies as support -> falling back to floor"
            )
        return None

    # 순수 거리 순위만으로는, 같은 물리적 평면(보통 테이블)이 RANSAC 반복마다
    # 다르게 쪼개져 생긴 작고 노이즈 많은 조각이 우연히 더 가깝게 나오면 그걸 지지면
    # 으로 채택해버릴 수 있다 - 실측 확인: 15709점짜리 평평한 테이블(up 정렬 1.000)
    # 대신 228점짜리 기울어진 조각(up 정렬 0.967)이 뽑혀서 복원된 아래쪽 4꼭짓점이
    # 평평하지 않고 비뚤어진 사례가 있었다. 관측 점 수가 가장 큰 후보의
    # SUPPORT_SIZE_PREFERENCE_RATIO 미만인 후보는 "신뢰도 낮은 조각"으로 보고
    # 순위에서 뒤로 미룬다(완전 배제는 아님 - 다른 후보가 전혀 없으면 그대로 씀).
    max_support_points = max(len(item[4].points) for item in ranked)
    ranked.sort(
        key=lambda item: (
            0 if len(item[4].points) >= SUPPORT_SIZE_PREFERENCE_RATIO * max_support_points else 1,
            *item[:4],
        )
    )

    chosen = ranked[0]
    if support_min_area_ratio is not None:
        top_area = top.width * top.height
        area_ok = [r for r in ranked if (r[4].width * r[4].height) >= top_area * support_min_area_ratio]
        if area_ok:
            chosen = area_ok[0]
        elif debug and debug_log:
            debug_log(
                f"[DEBUG box_top support] top={top.candidate_id}: no support candidate has "
                f"area >= top_area*{support_min_area_ratio} (top_area={top_area:.4f}) - "
                f"falling back to best geometric match anyway (candidate={ranked[0][4].candidate_id}, "
                f"area={ranked[0][4].width * ranked[0][4].height:.4f})"
            )

    if debug and debug_log:
        debug_log(
            f"[DEBUG box_top support] top={top.candidate_id}: "
            f"ACCEPTED support candidate={chosen[4].candidate_id} (median_distance={chosen[0]:.3f})"
        )
    return chosen[4]


def detect_floor_boundary(
    top: PlaneClusterCandidate,
    scene_pcd: o3d.geometry.PointCloud,
    down_direction: np.ndarray,
    *,
    floor_ransac_max_planes: int = FLOOR_RANSAC_MAX_PLANES,
    floor_min_points: int = FLOOR_MIN_POINTS,
    floor_distance_threshold_m: float = FLOOR_DISTANCE_THRESHOLD_M,
    floor_ransac_iterations: int = FLOOR_RANSAC_ITERATIONS,
    floor_normal_dot_min: float = FLOOR_NORMAL_DOT_MIN,
    floor_min_area_m2: float = FLOOR_MIN_AREA_M2,
    floor_boundary_hit_distance_m: float = FLOOR_BOUNDARY_HIT_DISTANCE_M,
    min_ray_distance_m: float = MIN_RAY_DISTANCE_M,
    max_ray_distance_m: float = MAX_RAY_DISTANCE_M,
    max_ray_distance_spread_m: float = MAX_RAY_DISTANCE_SPREAD_M,
    min_boundary_ray_hits: int = MIN_BOUNDARY_RAY_HITS,
    project_points_fn=None,
    debug: bool = False,
    debug_log: DebugLog = None,
) -> Optional[PlaneClusterCandidate]:
    """아래쪽 박스 윗면 후보가 없을 때 전체 장면에서 바닥을 찾는다."""
    remaining = o3d.geometry.PointCloud(scene_pcd)

    top_corners = order_rectangle_corners(top.corners_3d.copy())
    down_direction = _normalize(down_direction)

    floor_candidates = []

    if debug and debug_log:
        debug_log(
            f"[DEBUG floor] top={top.candidate_id} footprint=({top.width:.3f},{top.height:.3f}) "
            f"corners_z={np.round(top_corners[:, 2], 3).tolist()} scene_points={len(remaining.points)}"
        )

    for plane_index in range(floor_ransac_max_planes):
        if len(remaining.points) < floor_min_points:
            break

        plane_model, inliers = remaining.segment_plane(
            distance_threshold=floor_distance_threshold_m,
            ransac_n=3,
            num_iterations=floor_ransac_iterations,
        )
        if len(inliers) < floor_min_points:
            break

        plane_cloud = remaining.select_by_index(inliers)
        remaining = remaining.select_by_index(inliers, invert=True)

        points = np.asarray(plane_cloud.points, dtype=np.float64)
        if len(points) < floor_min_points:
            continue

        normal = np.asarray(plane_model[:3], dtype=np.float64)
        normal_norm = float(np.linalg.norm(normal))
        if normal_norm < 1e-8:
            continue
        normal /= normal_norm
        plane_d = float(plane_model[3] / normal_norm)

        parallel_score = abs(float(np.dot(normal, top.normal)))
        if parallel_score < floor_normal_dot_min:
            continue

        denominator = float(np.dot(normal, down_direction))
        if abs(denominator) < 1e-6:
            continue

        ray_distances = -(top_corners @ normal + plane_d) / denominator
        if np.any(~np.isfinite(ray_distances)):
            continue
        if np.any(ray_distances < min_ray_distance_m):
            continue

        median_distance = float(np.median(ray_distances))
        if not (min_ray_distance_m <= median_distance <= max_ray_distance_m):
            continue

        spread = float(np.max(ray_distances) - np.min(ray_distances))
        if spread > max_ray_distance_spread_m:
            continue

        center = points.mean(axis=0)
        axis_u, axis_v = make_plane_basis(normal)
        relative = points - center
        local_points = np.column_stack((relative @ axis_u, relative @ axis_v)).astype(np.float32)

        hull = cv2.convexHull(local_points)
        observed_area = float(cv2.contourArea(hull))
        if observed_area < floor_min_area_m2:
            continue

        intersections = top_corners + ray_distances[:, None] * down_direction[None, :]
        nearest_distances = np.asarray(
            [float(np.min(np.linalg.norm(points - p[None, :], axis=1))) for p in intersections],
            dtype=np.float64,
        )

        hit_count = int(np.count_nonzero(nearest_distances <= floor_boundary_hit_distance_m))
        if hit_count < min_boundary_ray_hits:
            if debug and debug_log:
                debug_log(
                    f"[DEBUG floor] top={top.candidate_id} plane_index={plane_index}: "
                    f"nearest_distances={np.round(nearest_distances, 3).tolist()} "
                    f"hit_count={hit_count} < MIN_BOUNDARY_RAY_HITS({min_boundary_ray_hits}) -> reject"
                )
            continue

        if debug and debug_log:
            debug_log(
                f"[DEBUG floor] top={top.candidate_id} plane_index={plane_index}: ACCEPTED as floor "
                f"(median_distance={median_distance:.3f}, spread={spread:.3f}, hit_count={hit_count})"
            )

        rectangle = cv2.minAreaRect(local_points)
        rectangle_points_2d = cv2.boxPoints(rectangle).astype(np.float64)
        corners_3d = (
            center[None, :]
            + rectangle_points_2d[:, 0:1] * axis_u[None, :]
            + rectangle_points_2d[:, 1:2] * axis_v[None, :]
        )

        rect_size = rectangle[1]
        width = float(max(rect_size))
        height = float(min(rect_size))
        rectangle_area = max(width * height, 1e-8)
        fill_ratio = observed_area / rectangle_area

        pixel_polygon = project_points_fn(corners_3d) if project_points_fn is not None else None

        floor_candidate = PlaneClusterCandidate(
            candidate_id=-(plane_index + 1),
            points=points,
            normal=normal,
            median_depth=float(np.median(points[:, 2])),
            width=width,
            height=height,
            area=observed_area,
            fill_ratio=fill_ratio,
            center=center,
            plane_d=plane_d,
            corners_3d=corners_3d,
            pixel_polygon=pixel_polygon,
        )

        floor_candidates.append((median_distance, -observed_area, -hit_count, floor_candidate))

    if not floor_candidates:
        return None

    floor_candidates.sort(key=lambda item: item[:3])
    return floor_candidates[0][3]


def compute_box_corners(
    top: PlaneClusterCandidate,
    support: PlaneClusterCandidate,
    down_direction: np.ndarray,
    *,
    min_ray_distance_m: float = MIN_RAY_DISTANCE_M,
    min_box_height_m: float = MIN_BOX_HEIGHT_M,
    max_box_height_m: float = MAX_BOX_HEIGHT_M,
    max_ray_distance_spread_m: float = MAX_RAY_DISTANCE_SPREAD_M,
    bottom_cut_margin_m: float = BOTTOM_CUT_MARGIN_M,
) -> Optional[np.ndarray]:
    """초록색 윗면 네 꼭짓점에서 아래 방향 ray를 내리고, 선택된 경계 평면과 만나는
    네 교점으로 아래 꼭짓점을 만든다."""
    top_corners = order_rectangle_corners(top.corners_3d.copy())
    down_direction = _normalize(down_direction)

    support_normal = support.normal.astype(np.float64)
    support_normal /= max(np.linalg.norm(support_normal), 1e-8)
    support_d = float(support.plane_d)

    denominator = float(np.dot(support_normal, down_direction))
    if abs(denominator) < 1e-6:
        return None

    ray_distances = -(top_corners @ support_normal + support_d) / denominator
    if np.any(~np.isfinite(ray_distances)):
        return None
    if np.any(ray_distances < min_ray_distance_m):
        return None

    median_height = float(np.median(ray_distances))
    if not (min_box_height_m <= median_height <= max_box_height_m):
        return None

    spread = float(np.max(ray_distances) - np.min(ray_distances))
    if spread > max_ray_distance_spread_m:
        return None

    effective_distances = ray_distances - bottom_cut_margin_m
    if np.any(effective_distances <= 0.0):
        return None

    bottom_corners = top_corners + effective_distances[:, None] * down_direction[None, :]
    return np.vstack((top_corners, bottom_corners)).astype(np.float64)


# ============================================================
# 동일 크기로 적층된(위 박스가 아래 박스와 같은 footprint) 박스 탐색
#
# 일반 파이프라인(detect_box_top_candidates_fixed_up + make_candidate)은 "독립된
# 사각형 후보"를 요구한다(MIN_BOX_SIDE_M=0.04 이상, DBSCAN으로 뭉친 클러스터). 위/
# 아래 박스 크기가 같으면 아래 박스의 노출된 테두리가 아예 없거나(완전히 같은
# XY - 이 경우는 어떤 시점에서 봐도 원천적으로 구분 불가능) 4cm보다 얇아서, 그
# 테두리가 절대 독립 후보로 안 잡힌다. 이 아래 함수는 "이미 정확히 검출된 위 박스의
# 사각형"을 템플릿으로 삼아, 그 경계 바로 바깥쪽 + 더 낮은 위치에 있는 점들만 보고
# (변 하나가 몇 mm만 튀어나와 있어도) 오프셋을 추정한다 - 최소 변 길이 조건이 없다.
# 자세한 설계 배경/시행착오는 perception/SAME_SIZE_STACK_DETECTION_LOG.md 참고.
# ============================================================

MIN_HIDDEN_BOX_GAP_M = 0.03  # 위 박스 자신의 윗면/모서리 노이즈와 구분하기 위한 최소 깊이차
MAX_HIDDEN_BOX_GAP_M = 0.60  # 이보다 훨씬 아래 점은 다른 물체/바닥일 가능성이 높아 제외
EDGE_SEARCH_BAND_M = 0.06    # 사각형 경계 바깥쪽 이 거리 이내의 점만 "돌출 후보"로 본다
MIN_PROTRUSION_SUPPORT_POINTS = 12  # 한쪽 변에서 이 이상 점이 모여야 신뢰할 만한 돌출로 인정
PROTRUSION_Z_BAND_M = 0.02   # 같은 높이(아래 박스 윗면)로 볼 z 허용 오차
# top 자신의 옆면(경계선에 딱 붙어 위아래로 이어지는 면)과 진짜 노출된 테두리를
# 구분하는 최소 돌출량 - 이보다 작으면 노이즈/옆면으로 보고 무시한다. 실측
# 확인(SAME_SIZE_STACK_DETECTION_LOG.md 3-7절): 1cm로는 독립 박스(회전/정렬
# 오차가 섞인 실제 시뮬레이션 데이터)에서도 이따금 통과할 만큼 노이즈가 있었다.
MIN_MEANINGFUL_PROTRUSION_M = 0.020


def _rectangle_frame(top: PlaneClusterCandidate) -> tuple[np.ndarray, np.ndarray, np.ndarray, float, float]:
    """top의 4개 꼭짓점에서 (중심, 변 방향 축 rect_u/rect_v, 반폭) 을 뽑는다.
    make_plane_basis()가 주는 일반 평면 기준축이 아니라, 실제로 minAreaRect가 잡은
    사각형 자신의 변 방향을 쓴다(회전된 박스에도 정확히 맞아야 하므로)."""
    corners = order_rectangle_corners(top.corners_3d.copy())
    center = corners.mean(axis=0)
    edge_u = corners[1] - corners[0]
    edge_v = corners[3] - corners[0]
    half_u = float(np.linalg.norm(edge_u)) / 2.0
    half_v = float(np.linalg.norm(edge_v)) / 2.0
    rect_u = edge_u / max(half_u * 2.0, 1e-8)
    rect_v = edge_v / max(half_v * 2.0, 1e-8)
    return center, rect_u, rect_v, half_u, half_v


# ============================================================
# find_stacked_layers() - 반복적 바닥 재탐색(recursive floor descent)으로
# "top 아래에 몇 겹이 쌓여 있는가"를 찾는다.
#
# 이 함수를 추가하게 된 계기(SAME_SIZE_STACK_DETECTION_LOG.md 3-8절 참고): 처음엔
# find_hidden_stacked_box()의 변별 돌출량 히스토그램만으로 "숨겨진 박스가 있는지,
# 깊이가 얼마인지"를 한 번에 찾으려 했다. 그런데 실측 확인 결과 detect_floor_boundary()
# 자체가 이미 흥미로운 성질을 갖고 있었다: RANSAC으로 찾은 여러 평면 후보 중
# "top 바로 아래에서 가장 가까운(median_distance 최솟값)" 것을 채택하도록
# 설계돼 있어서(정렬 키가 (median_distance, -area, -hit_count)), 아래에 숨겨진
# 동일 크기 박스가 있으면 그 박스 자신의 노출된 윗면 테두리를 "가장 가까운 평면"
# 으로 우연히 찾아낸다(실측: 15회 반복 중 15회 모두 0.136~0.151m 범위로 안정적,
# 진짜 참값인 단일 박스 높이 0.14m와 거의 일치) - 이걸 몰랐던 이유는, 이 결과를
# "그냥 바닥을 찾았다"고 가정하고 top의 지지면(support)으로만 쓰고 끝냈기
# 때문이다(그 지지면 자체가 사실은 "또 다른 박스의 윗면"일 수 있다는 걸
# 검증하지 않았다).
#
# 그래서 이제는: detect_floor_boundary()가 찾아준 지지면을, top과 같은
# footprint를 가진 "새로운 top 후보"로 간주하고 거기서 다시 detect_floor_boundary()를
# 호출해본다 - 만약 그 아래에서 또 유효한 지지면을 찾으면, 방금 찾은 지지면은
# 사실 "바닥"이 아니라 "또 다른 박스"였다는 뜻이다(진짜 바닥이라면, 스캔
# 크롭 범위(TABLE_TOP_Z - 0.05m 아래는 애초에 point cloud에 없음) 밖이라 더
# 아래에서는 유효한 평면을 못 찾는다 - 실측 확인: Medium처럼 실제로 안 쌓인
# 독립 박스에서는 20회 중 12회 정확히 "더 없음"으로 멈췄다). 이 과정을 반복하면
# 몇 겹이 쌓여 있는지(스택 깊이)까지 일반적으로 알아낼 수 있다.
#
# 다만 detect_floor_boundary() 자신이 RANSAC 비결정성 노이즈를 갖고 있어서(실측:
# Medium에서도 20회 중 2회는 크롭 경계 부근 노이즈를 "층"으로 오인 - 다만 그
# 두께가 6cm 미만으로 비정상적으로 얇았다), 단 한 번의 재귀 탐색 결과를 그대로
# 믿지 않는다. 같은 재귀 탐색을 여러 번 반복해서 "몇 겹으로 내려가는가"를
# 다수결로 정하고(더 깊이 내려간 시도는 항상 얕은 단계도 포함하므로, "이 겹수
# 이상 내려간 시도가 전체의 min_agree_fraction 이상"인 가장 깊은 겹수를 채택),
# 채택된 겹수만큼만 각 단계의 깊이를 중앙값으로 취한다 - 이미 다른 곳(RANSAC
# 평면 검출, find_hidden_stacked_box의 옆면 판정)에서 쓰던 것과 같은
# "다중 시도 + 다수결/중앙값" 원칙을 여기에도 적용한 것이다.
# ============================================================

MIN_PLAUSIBLE_LAYER_HEIGHT_M = 0.06  # 실제 박스 카탈로그(0.12~0.14m)보다 넉넉히 낮은 하한
STACK_DESCENT_TRIALS = 9
STACK_DESCENT_MIN_AGREE_FRACTION = 0.5
STACK_DESCENT_MAX_LEVELS = 4
# 2단계 이상 재귀 탐색 시 detect_floor_boundary()가 전체 scene을 대상으로 RANSAC을
# 돌리면, 진짜 바닥(테이블)과 "우연히 비슷한 깊이에 있는 완전히 다른 물체"(예: 옆에
# 놓인 다른 독립 박스의 윗면)를 같은 평면으로 잘못 합칠 수 있다 - 실측 확인: Large
# 자신의 rim에서 진짜 테이블까지 2단계로 내려가는 탐색에서, 참값(2층 합 ~0.28m)보다
# 3~4cm 짧게 나옴(Medium의 top이 우연히 비슷한 깊이라 테이블 평면 피팅에 섞여
# 들어간 것으로 추정). top 자신의 footprint 국소 영역(margin만큼 확장)으로 미리
# 잘라내서, RANSAC이 애초에 먼 곳의 다른 물체 점을 볼 수 없게 한다.
STACK_DESCENT_LOCAL_CROP_MARGIN_M = 0.12


def _synthetic_top_at_cumulative_depth(
    top: PlaneClusterCandidate, cumulative_depth_m: float, down_direction: np.ndarray
) -> PlaneClusterCandidate:
    """top과 같은 footprint를, top 윗면에서 down_direction 방향으로 cumulative_depth_m
    만큼 내려간 위치에 복사한다 - detect_floor_boundary()를 그 위치에서 다시 돌려보기
    위한 임시 "top" 역할(회전/모양은 top과 같다고 가정 - 같은 방향으로 쌓인 경우만
    다룬다는 기존 범위 제한과 동일)."""
    synthetic = copy.deepcopy(top)
    offset = cumulative_depth_m * down_direction
    synthetic.corners_3d = top.corners_3d + offset[None, :]
    synthetic.center = top.center + offset
    synthetic.candidate_id = -999999
    return synthetic


def _crop_local_region(
    scene_pcd: o3d.geometry.PointCloud,
    top: PlaneClusterCandidate,
    margin_m: float,
) -> o3d.geometry.PointCloud:
    """top의 사각형 로컬 축(rect_u/rect_v) 기준으로, footprint를 margin_m만큼 확장한
    영역 밖의 점을 잘라낸다. 회전된 박스에도 정확히 맞도록 XY가 아니라 top 자신의
    변 방향 축을 쓴다."""
    points = np.asarray(scene_pcd.points)
    center, rect_u, rect_v, half_u, half_v = _rectangle_frame(top)
    relative = points - center[None, :]
    u = relative @ rect_u
    v = relative @ rect_v
    mask = (np.abs(u) <= half_u + margin_m) & (np.abs(v) <= half_v + margin_m)
    cropped = o3d.geometry.PointCloud()
    cropped.points = o3d.utility.Vector3dVector(points[mask])
    return cropped


def _single_descent_trial(
    top: PlaneClusterCandidate,
    full_scene_pcd: o3d.geometry.PointCloud,
    local_scene_pcd: o3d.geometry.PointCloud,
    down_direction: np.ndarray,
    max_levels: int,
    min_layer_height_m: float,
) -> list[float]:
    """top에서 시작해서 detect_floor_boundary()를 반복 호출하며 아래로 내려간다.
    각 단계에서 찾은 "top 윗면 기준 누적 깊이"의 리스트를 반환한다(마지막 원소가
    이번 시도에서 도달한 최종 바닥).

    1단계(원래 top 바로 아래)는 전체 scene(full_scene_pcd)으로 찾고, 2단계부터는
    국소 영역(local_scene_pcd)으로 찾는다 - 실측 확인: 둘 다 국소 영역으로 하면
    1단계 자체가 사라진다(숨겨진 박스의 노출된 테두리는 그 자체로 점이 너무 적어서,
    국소 영역만으로는 detect_floor_boundary()의 최소 점 개수/면적 기준을 못 채워
    아예 무시되고 곧바로 진짜 바닥까지 건너뛰어 버림 - 반대로 전체 scene에서는
    "우연히 비슷한 높이의 먼 평면과 합쳐진 덕에" 기준을 통과해서 오히려 찾아진다).
    반대로 둘 다 전체 scene으로 하면 2단계(진짜 바닥 찾기)에서 그 "우연히 비슷한
    높이의 먼 물체"(예: 옆에 놓인 다른 독립 박스의 윗면)와 진짜 바닥이 하나의
    평면으로 잘못 합쳐져 깊이가 짧게 나온다(실측: 참값 대비 3~4cm). 그래서 1단계는
    전체로, 2단계부터는 국소로 나눈다."""
    depths: list[float] = []
    cumulative = 0.0
    current = top
    for level in range(max_levels):
        scene_for_this_level = full_scene_pcd if level == 0 else local_scene_pcd
        floor = detect_floor_boundary(current, scene_for_this_level, down_direction)
        if floor is None:
            break
        corners = compute_box_corners(current, floor, down_direction)
        corners = np.asarray(corners) if corners is not None else None
        if corners is None or corners.shape != (8, 3):
            break
        layer_height = float(np.mean(corners[:4, 2] - corners[4:, 2])) + BOTTOM_CUT_MARGIN_M
        if layer_height < min_layer_height_m:
            break
        cumulative += layer_height
        depths.append(cumulative)
        current = _synthetic_top_at_cumulative_depth(top, cumulative, down_direction)
    return depths


def find_stacked_layers(
    top: PlaneClusterCandidate,
    scene_pcd: o3d.geometry.PointCloud,
    down_direction: np.ndarray,
    *,
    trials: int = STACK_DESCENT_TRIALS,
    min_agree_fraction: float = STACK_DESCENT_MIN_AGREE_FRACTION,
    max_levels: int = STACK_DESCENT_MAX_LEVELS,
    min_layer_height_m: float = MIN_PLAUSIBLE_LAYER_HEIGHT_M,
    debug: bool = False,
    debug_log: DebugLog = None,
) -> list[float]:
    """top 아래에 (같은 footprint를 가정하고) 몇 겹의 박스가 있는지, 각 겹의 top 윗면
    기준 누적 깊이를 반환한다. 마지막 원소는 "진짜 최종 바닥"까지의 깊이다 - 즉
    반환 리스트 길이가 N이면, 숨겨진 박스는 N-1개다(마지막은 지지면일 뿐 새 박스가
    아님). 숨겨진 박스가 전혀 없으면(top이 바로 바닥/최종 지지면 위에 있으면)
    길이 1인 리스트(바닥까지의 깊이만)를 반환한다. 아무 지지면도 못 찾으면 빈
    리스트를 반환한다."""
    down_direction = _normalize(down_direction)
    local_pcd = _crop_local_region(scene_pcd, top, STACK_DESCENT_LOCAL_CROP_MARGIN_M)
    all_depths = [
        _single_descent_trial(top, scene_pcd, local_pcd, down_direction, max_levels, min_layer_height_m)
        for _ in range(trials)
    ]
    level_counts = [len(d) for d in all_depths]
    if not level_counts or max(level_counts) == 0:
        if debug and debug_log:
            debug_log(f"[DEBUG stack_descent] top={top.candidate_id}: 어떤 시도에서도 지지면을 못 찾음")
        return []

    counts = Counter(level_counts)
    min_trials_required = trials * min_agree_fraction
    chosen_level = 0
    # 레벨 수가 큰 시도는 항상 그보다 얕은 레벨도 포함한다(더 깊이 내려갔다는 뜻)
    # - 그래서 "이 레벨 수 이상 도달한 시도가 전체의 min_agree_fraction 이상"을
    # 만족하는 가장 큰 레벨 수를 채택한다(다수결이 큰 값 쪽으로 과대평가되지
    # 않도록, 정확히 그 레벨 이상 도달한 시도의 비율로 판정).
    for level in sorted(counts.keys(), reverse=True):
        support = sum(c for lvl, c in counts.items() if lvl >= level)
        if support >= min_trials_required:
            chosen_level = level
            break

    if debug and debug_log:
        debug_log(
            f"[DEBUG stack_descent] top={top.candidate_id}: level_counts={dict(counts)} "
            f"(trials={trials}) -> chosen_level={chosen_level}"
        )

    if chosen_level == 0:
        return []

    result_depths = []
    for level_idx in range(chosen_level):
        values = [d[level_idx] for d in all_depths if len(d) > level_idx]
        result_depths.append(float(np.median(values)))
    return result_depths


def flat_plane_support_at_depth(
    top: PlaneClusterCandidate,
    depth_m: float,
    down_direction: np.ndarray,
    *,
    candidate_id: int = -888888,
) -> PlaneClusterCandidate:
    """top과 같은(회전 없는) footprint를, top 윗면에서 depth_m만큼 내려간 위치에 놓인
    수평 평면으로 간주하고 compute_box_corners()의 support 인자로 바로 쓸 수 있는
    PlaneClusterCandidate를 만든다(normal/plane_d를 그 깊이에 맞게 정확히 계산 -
    _synthetic_top_at_cumulative_depth()와 달리 support 역할이 목적이라 plane_d가
    맞아야 한다). find_stacked_layers()가 찾아낸 각 층의 깊이를 실제 8꼭짓점
    복원에 쓸 때 사용한다."""
    down_direction = _normalize(down_direction)
    offset = depth_m * down_direction
    center = top.center + offset
    normal = -down_direction
    plane_d = -float(np.dot(normal, center))
    return PlaneClusterCandidate(
        candidate_id=candidate_id,
        points=np.zeros((0, 3), dtype=np.float64),
        normal=normal,
        median_depth=float(center[2]),
        width=top.width,
        height=top.height,
        area=top.width * top.height,
        fill_ratio=1.0,
        center=center,
        plane_d=plane_d,
        corners_3d=(top.corners_3d + offset[None, :]),
        pixel_polygon=None,
        synthetic=True,
    )


def find_hidden_stacked_box(
    top: PlaneClusterCandidate,
    scene_points: np.ndarray,
    down_direction: np.ndarray,
    *,
    min_gap_m: float = MIN_HIDDEN_BOX_GAP_M,
    max_gap_m: float = MAX_HIDDEN_BOX_GAP_M,
    edge_search_band_m: float = EDGE_SEARCH_BAND_M,
    min_support_points: int = MIN_PROTRUSION_SUPPORT_POINTS,
    z_band_m: float = PROTRUSION_Z_BAND_M,
    known_far_support_depth_m: Optional[float] = None,
    min_box_height_m: float = MIN_BOX_HEIGHT_M,
    min_meaningful_protrusion_m: float = MIN_MEANINGFUL_PROTRUSION_M,
    forced_depth_m: Optional[float] = None,
    debug: bool = False,
    debug_log: DebugLog = None,
) -> Optional[PlaneClusterCandidate]:
    """top과 같은 크기(width/height/방향)의 박스가 그 바로 아래, 살짝 다른 XY
    위치에 쌓여 있는지 찾는다. top의 사각형 경계 바로 바깥쪽 + top 윗면보다 낮은
    위치에 있는 점들을 변(±u, ±v)별로 모아서 돌출량을 재고, 그 돌출량으로 오프셋
    (du, dv)과 높이(z)를 역산한다.

    같은 방향(회전 없음)으로 쌓였다고 가정한다 - 회전까지 다르게 쌓인 경우는
    범위 밖이다(SAME_SIZE_STACK_DETECTION_LOG.md 참고).

    오프셋이 정말 0에 가까우면(위/아래 박스가 거의 완전히 같은 XY) 아래 박스의
    노출면이 전혀 없어서, 시야가 마침 닿는 "바닥"을 "숨겨진 박스"로 잘못 착각할 수
    있다(실측 확인: offset=0 테스트 케이스에서 실제로 발생 - 이건 카메라를 몇 개
    더 늘려도 못 고치는 정보 이론적 한계라, 알고리즘 쪽에서는 "이럴 땐 모른다고
    답하기"가 맞는 대응이다). `known_far_support_depth_m`(호출부가 이미 알고 있는,
    더 먼 바닥/지지면까지의 깊이)을 넘겨주면, 발견한 높이가 그 바닥과 사실상
    같을 때(최소 박스 높이만큼도 차이가 안 날 때) 기각한다.

    반환값: 찾으면 아래 박스의 top 후보(PlaneClusterCandidate, synthetic=True),
    못 찾으면(테두리가 전혀 안 보이는 경우 포함) None.
    """
    down_direction = _normalize(down_direction)
    center, rect_u, rect_v, half_u, half_v = _rectangle_frame(top)

    relative = scene_points - center[None, :]
    local_u = relative @ rect_u
    local_v = relative @ rect_v
    # top.normal은 항상 절대 up 벡터로 강제 정렬돼 있으므로(detect_box_top_candidates_fixed_up),
    # -down_direction 방향 성분이 곧 "top 평면 기준 높이"다.
    local_h = relative @ (-down_direction)

    below_mask = (local_h <= -min_gap_m) & (local_h >= -max_gap_m)
    if not np.any(below_mask):
        if debug and debug_log:
            debug_log(f"[DEBUG hidden_stack] top={top.candidate_id}: 아래쪽 후보 점 자체가 없음")
        return None

    u_b, v_b, h_b = local_u[below_mask], local_v[below_mask], local_h[below_mask]

    # 경계 바깥쪽(어느 변이든) 밴드 안에 있는 점들을 일단 다 모은다 - 아직 변별로
    # 나누지 않는다. 실측 확인(SAME_SIZE_STACK_DETECTION_LOG.md 3절): 다중 시점
    # 스캔은 원래 그늘져야 할 바닥까지 비스듬한 각도로 일부 봐버릴 수 있어서, 이
    # 단계에서 바닥 점(더 깊은 h)과 진짜 아래 박스 테두리 점(더 얕은 h)이 같은
    # 변의 검색 밴드 안에 섞여 들어올 수 있다 - 변별 중앙값을 바로 믿으면 안 된다.
    near_edge_mask = (
        ((u_b > half_u) & (u_b <= half_u + edge_search_band_m))
        | ((u_b < -half_u) & (u_b >= -half_u - edge_search_band_m))
        | ((v_b > half_v) & (v_b <= half_v + edge_search_band_m))
        | ((v_b < -half_v) & (v_b >= -half_v - edge_search_band_m))
    ) & (np.abs(u_b) <= half_u + edge_search_band_m) & (np.abs(v_b) <= half_v + edge_search_band_m)

    if np.count_nonzero(near_edge_mask) < min_support_points:
        if debug and debug_log:
            debug_log(
                f"[DEBUG hidden_stack] top={top.candidate_id}: 경계 밴드 안에 점이 "
                f"{int(np.count_nonzero(near_edge_mask))}개뿐(<{min_support_points}) -> 아래 박스 없음(또는 완전히 가려짐)"
            )
        return None

    idx_below = np.flatnonzero(below_mask)
    idx_near_edge = idx_below[near_edge_mask]  # scene_points로 바로 인덱싱 가능한 전역 인덱스
    u_e, v_e, h_e = u_b[near_edge_mask], v_b[near_edge_mask], h_b[near_edge_mask]

    if forced_depth_m is not None:
        # find_stacked_layers()가 detect_floor_boundary()를 반복 재탐색 + 다수결
        # 투표로 이미 검증한 깊이를 넘겨준 경우 - 아래 히스토그램/피크 탐지(깊이
        # 자체를 모를 때만 필요한 단계)를 건너뛰고 그 깊이 근처 점만 바로 쓴다.
        # 실측 확인: 히스토그램 방식은 노이즈에 취약해서(같은 물리적 케이스에서도
        # "옆면처럼 이어짐"으로 계속 건너뛰다 결국 못 찾거나, 반대로 Medium 같은
        # 독립 박스에서 옆면을 표면으로 오인하는 등) 신뢰도가 들쭉날쭉했다
        # (SAME_SIZE_STACK_DETECTION_LOG.md 3-8절 참고) - 깊이를 이미 알고 있다면
        # 이 단계 자체가 필요 없다.
        z_hidden_h = -abs(forced_depth_m)
        shallow_mask = np.abs(h_e - z_hidden_h) <= z_band_m
        shallow_idx = np.flatnonzero(shallow_mask)
        if len(shallow_idx) < min_support_points:
            if debug and debug_log:
                debug_log(
                    f"[DEBUG hidden_stack] top={top.candidate_id}: forced_depth={forced_depth_m:.4f}m 근처에 "
                    f"경계 점이 {len(shallow_idx)}개뿐(<{min_support_points}) -> 기각"
                )
            return None
        idx_shallow = idx_near_edge[shallow_idx]
        u_c, v_c = u_e[shallow_idx], v_e[shallow_idx]
        return _build_offset_candidate_from_edge_points(
            top, scene_points, down_direction, center, rect_u, rect_v, half_u, half_v,
            idx_shallow, u_c, v_c, z_hidden_h, edge_search_band_m, min_support_points,
            min_meaningful_protrusion_m, debug, debug_log,
            require_offset_evidence=False,
        )

    # "가장 얕은(= top 윗면에 가장 가까운) 높이 군집"을 먼저 찾는다 - top 바로
    # 아래에 뭔가 있다면 그게 첫 번째로 만나는 표면이어야 하고(바닥은 그보다
    # 항상 더 깊다), 그 군집 바깥(더 깊은) 점은 바닥/다른 물체이므로 버린다.
    #
    # 점 하나를 기준으로 간격(gap)이 좁으면 계속 이어붙이는 방식(예전 버전)은,
    # 진짜 테두리보다 살짝 얕은 곳에 노이즈 점 몇 개가 섞여 있으면 그쪽으로 군집
    # 경계가 끌려가서 높이를 몇 cm씩 잘못 잡는다(실측 확인: support_points=3648인
    # 실제 케이스에서 위/아래 박스 높이가 각각 2.8cm/5cm씩 어긋남). 대신 히스토그램
    # 밀도로 판단한다 - 진짜 표면은 점이 조밀하게 뭉쳐 있고, 노이즈는 성기게
    # 흩어져 있다는 성질을 이용해 "얕은 쪽부터 훑어서 처음으로 충분히 조밀한 bin"을
    # 찾는다.
    # bin 경계를 관측된 점들의 min/max(=시도마다, 노이즈 표본마다 흔들리는 값)가
    # 아니라 h=0(= top 윗면) 기준 절대 위치로 고정한다 - 상대 경계를 쓰면 실행마다
    # (RANSAC 시드가 바뀌어 표본이 바뀌면서) bin 나뉘는 지점이 미묘하게 달라져서,
    # 같은 물리적 군집이 서로 다른 bin으로 쪼개지거나 합쳐지는 게 실측으로 확인됐다
    # (같은 입력을 반복 실행했을 때 찾은 높이가 크게 흔들림).
    bin_width = max(z_band_m, 0.005)
    n_bins = max(1, int(np.ceil(max_gap_m / bin_width)) + 1)
    bin_idx = np.clip((np.abs(h_e) / bin_width).astype(int), 0, n_bins - 1)
    bin_counts = np.bincount(bin_idx, minlength=n_bins)

    # "조밀한 bin 하나"만 요구하면 top 자신의 옆면(수직으로 쭉 이어지는 면)을 잘못
    # 잡는다 - 실측 확인(SAME_SIZE_STACK_DETECTION_LOG.md 3-7절): 옆면은 여러 깊이
    # bin에 걸쳐 "고르게" 점이 있는 반면(뾰족한 피크가 없음), 진짜 아래 박스의
    # 윗면(수평 평면)은 그 깊이에서 점이 뾰족하게 몰려 있고 그 바로 아래는 상대적으로
    # 비어 있어야 한다(박스 옆면과 그 아래 바닥 사이는 관측이 잘 안 되는 좁은 틈).
    # 그래서 "조밀한 bin"이면서 동시에 "바로 다음(더 깊은) bin에서 밀도가 뚜렷하게
    # 떨어지는" bin만 인정한다 - 옆면처럼 이어지는 구간은 이 조건에서 계속 걸러지고
    # 통과할 때까지 더 깊이 훑는다.
    WALL_VS_SURFACE_DROPOFF_RATIO = 0.5

    chosen_bin = None
    for b in range(n_bins):  # bin 0이 h=0(얕은 쪽)에 가장 가까움
        if bin_counts[b] < min_support_points:
            continue
        next_count = bin_counts[b + 1] if b + 1 < n_bins else 0
        if next_count > bin_counts[b] * WALL_VS_SURFACE_DROPOFF_RATIO:
            if debug and debug_log:
                debug_log(
                    f"[DEBUG hidden_stack] top={top.candidate_id}: bin {b}({bin_counts[b]}점) 다음 bin이 "
                    f"{next_count}점으로 안 떨어짐(옆면처럼 이어지는 구간으로 추정) -> 건너뛰고 더 깊이 탐색"
                )
            continue
        chosen_bin = b
        break

    if chosen_bin is None:
        if debug and debug_log:
            debug_log(
                f"[DEBUG hidden_stack] top={top.candidate_id}: 밀도 있으면서(>= {min_support_points}점) "
                f"뒤가 비는(수평 표면다운) 높이 군집을 못 찾음 -> 신뢰 불가"
            )
        return None

    bin_center = -(chosen_bin + 0.5) * bin_width
    shallow_mask = np.abs(h_e - bin_center) <= z_band_m
    shallow_idx = np.flatnonzero(shallow_mask)

    idx_shallow = idx_near_edge[shallow_idx]  # 전역 인덱스
    z_hidden_h = float(np.median(h_e[shallow_idx]))

    if known_far_support_depth_m is not None and abs(z_hidden_h) >= known_far_support_depth_m - min_box_height_m:
        if debug and debug_log:
            debug_log(
                f"[DEBUG hidden_stack] top={top.candidate_id}: 발견한 높이(깊이 {abs(z_hidden_h):.4f}m)가 "
                f"이미 알고 있는 먼 지지면(깊이 {known_far_support_depth_m:.4f}m)과 사실상 같음 -> "
                "그냥 바닥을 본 것으로 보고 기각(오프셋이 0에 가까워 원천적으로 구분 불가능한 경우)"
            )
        return None
    u_c, v_c = u_e[shallow_idx], v_e[shallow_idx]

    return _build_offset_candidate_from_edge_points(
        top, scene_points, down_direction, center, rect_u, rect_v, half_u, half_v,
        idx_shallow, u_c, v_c, z_hidden_h, edge_search_band_m, min_support_points,
        min_meaningful_protrusion_m, debug, debug_log,
    )


def _build_offset_candidate_from_edge_points(
    top: PlaneClusterCandidate,
    scene_points: np.ndarray,
    down_direction: np.ndarray,
    center: np.ndarray,
    rect_u: np.ndarray,
    rect_v: np.ndarray,
    half_u: float,
    half_v: float,
    idx_shallow: np.ndarray,
    u_c: np.ndarray,
    v_c: np.ndarray,
    z_hidden_h: float,
    edge_search_band_m: float,
    min_support_points: int,
    min_meaningful_protrusion_m: float,
    debug: bool,
    debug_log: DebugLog,
    require_offset_evidence: bool = True,
) -> Optional[PlaneClusterCandidate]:
    """find_hidden_stacked_box()의 두 경로(히스토그램으로 깊이를 직접 찾은 경우 /
    find_stacked_layers()가 이미 검증한 깊이를 forced_depth_m으로 받은 경우) 모두
    "이미 깊이를 알고, 그 깊이 근처의 경계 점들만 있는" 상태에서 공통으로 쓰는
    변별 돌출량 측정 + 사각형 후보 조립 로직."""
    sides = {
        "+u": (u_c > half_u) & (np.abs(v_c) <= half_v + edge_search_band_m),
        "-u": (u_c < -half_u) & (np.abs(v_c) <= half_v + edge_search_band_m),
        "+v": (v_c > half_v) & (np.abs(u_c) <= half_u + edge_search_band_m),
        "-v": (v_c < -half_v) & (np.abs(u_c) <= half_u + edge_search_band_m),
    }

    # 돌출 거리는 중앙값이 아니라 상위 백분위수로 잰다 - 노출된 테두리 위의 점들은
    # top 경계(half_u/half_v)부터 아래 박스의 실제 바깥쪽 모서리까지 폭 전체에
    # 걸쳐 고르게 분포하므로, 중앙값을 쓰면 실제 돌출량의 절반 정도로 과소추정된다
    # (실측 확인: 중앙값 사용 시 4cm 오프셋이 2cm로 나옴). 진짜 모서리는 그 분포의
    # 가장 바깥쪽(높은 백분위수) 근처에 있다.
    PROTRUSION_PERCENTILE = 90.0
    _edge_coord = {"+u": ("u", PROTRUSION_PERCENTILE), "-u": ("u", 100.0 - PROTRUSION_PERCENTILE),
                   "+v": ("v", PROTRUSION_PERCENTILE), "-v": ("v", 100.0 - PROTRUSION_PERCENTILE)}

    # top 자신의 옆면(side wall)은 정의상 경계선(half_u/half_v) 바로 그 자리에서
    # 위아래로 쭉 이어지는 면이다 - "근처 밴드" 조건만 걸면 이 옆면 점들도 (튀어나온
    # 양이 노이즈 수준인데도) "돌출 증거"로 잘못 잡힌다(실측 확인: 아무것도 안
    # 쌓인 독립 박스에서도 옆면 점 때문에 "숨겨진 박스"가 나옴). 진짜 노출된
    # 테두리라면 경계선에서 의미 있게(노이즈보다 확실히 크게) 튀어나와 있어야 한다.
    half_by_axis = {"u": half_u, "v": half_v}

    side_stats = {}
    for name, mask in sides.items():
        n = int(np.count_nonzero(mask))
        if n < min_support_points:
            continue
        axis, pct = _edge_coord[name]
        values = u_c if axis == "u" else v_c
        edge_val = float(np.percentile(values[mask], pct))
        protrusion = abs(edge_val) - half_by_axis[axis]
        if protrusion < min_meaningful_protrusion_m:
            if debug and debug_log:
                debug_log(
                    f"[DEBUG hidden_stack] top={top.candidate_id} side={name}: 돌출량 "
                    f"{protrusion*1000:.1f}mm(<{min_meaningful_protrusion_m*1000:.0f}mm) -> "
                    "옆면 노이즈로 보고 이 변은 증거에서 제외"
                )
            continue
        side_stats[name] = {"n": n, axis: edge_val}

    if not side_stats:
        if require_offset_evidence:
            if debug and debug_log:
                debug_log(
                    f"[DEBUG hidden_stack] top={top.candidate_id}: 가장 얕은 군집은 있지만 "
                    f"어느 변도 {min_support_points}점을 못 채움(흩어진 노이즈로 추정) -> 기각"
                )
            return None
        # forced_depth_m 경로: find_stacked_layers()가 반복 재탐색+다수결로 이미
        # "이 깊이에 진짜 층이 있다"는 것 자체는 독립적으로 확인했다 - 그 위에서
        # 가장자리 돌출 증거(변별 du/dv)까지 못 찾았다고 존재 자체를 부정하면
        # (오프셋이 아주 작아 노출 테두리가 거의 안 보이는 경우) 이미 확인된
        # 박스를 최종 결과에서 통째로 놓치게 된다 - 오프셋 0(top과 같은 XY)으로
        # 보수적으로 간주하고 등록한다(옆면 검색 밴드/최소 돌출량 필터는 여전히
        # 통과 못 했으므로 du/dv 추정 자체의 신뢰도는 낮다는 뜻일 뿐).
        if debug and debug_log:
            debug_log(
                f"[DEBUG hidden_stack] top={top.candidate_id}: 깊이는 확인됐지만 "
                f"어느 변도 {min_support_points}점 이상의 의미 있는 돌출을 못 보임(오프셋이 매우 작거나 "
                "노이즈로 추정) -> 오프셋 0(같은 XY)으로 간주"
            )
        du = dv = 0.0
        combined_side_mask = np.zeros(len(u_c), dtype=bool)
        total_support_points = len(u_c)
    else:
        # 오프셋 = "+쪽 변 돌출량"과 "-쪽 변 돌출량"의 차 - 물리적으로(회전 없는
        # 순수 평행이동이라면) 한쪽만 돌출된다.
        #
        # [시행착오] 두 변이 모두 기준을 통과하는 경우(실제 Isaac Sim 데이터에서
        # 자주 발생) 처음엔 평균을 냈다. 그런데 base_link가 world 기준 90도
        # 회전돼 있다는 걸 나중에 확인하고(테이블 자신의 검출된 변 방향으로 역산),
        # 스폰 오프셋을 base_link 프레임으로 정확히 변환해 참값과 직접 비교해보니
        # (참값 (du,dv)≈(-0.02,-0.03)) 평균 방식은 부호까지 틀리는 경우가 있었다
        # (약한 쪽 - 점 개수가 적고 퍼짐이 넓어 노이즈에 가까운 쪽 - 이 강한 쪽의
        # 부호를 뒤집을 만큼 끌어당김). "점 개수가 많은 쪽만 채택"으로 바꾸니
        # 두 축 모두 참값과 부호가 일치하고 절대오차도 1~2.5cm 수준으로 줄었다
        # (SAME_SIZE_STACK_DETECTION_LOG.md 3-13절 참고 - 예전에 이 방식을 한 번
        # 시도했다가 "더 나빠졌다"고 기각한 적이 있는데, 그때는 세계 좌표 오프셋을
        # base_link 회전 없이 그대로(잘못) 비교하고 있었다 - 잘못된 기준과 비교해서
        # 내린 잘못된 결론이었다).
        if "+u" in side_stats and "-u" not in side_stats:
            du = side_stats["+u"]["u"] - half_u
        elif "-u" in side_stats and "+u" not in side_stats:
            du = side_stats["-u"]["u"] + half_u
        elif "+u" in side_stats and "-u" in side_stats:
            if side_stats["+u"]["n"] >= side_stats["-u"]["n"]:
                du = side_stats["+u"]["u"] - half_u
            else:
                du = side_stats["-u"]["u"] + half_u
        else:
            du = 0.0

        if "+v" in side_stats and "-v" not in side_stats:
            dv = side_stats["+v"]["v"] - half_v
        elif "-v" in side_stats and "+v" not in side_stats:
            dv = side_stats["-v"]["v"] + half_v
        elif "+v" in side_stats and "-v" in side_stats:
            if side_stats["+v"]["n"] >= side_stats["-v"]["n"]:
                dv = side_stats["+v"]["v"] - half_v
            else:
                dv = side_stats["-v"]["v"] + half_v
        else:
            dv = 0.0

        combined_side_mask = np.logical_or.reduce([sides[name] for name in side_stats])
        total_support_points = sum(s["n"] for s in side_stats.values())

    hidden_center = center + du * rect_u + dv * rect_v - z_hidden_h * down_direction

    corners_local = np.array([
        [-half_u, -half_v], [half_u, -half_v], [half_u, half_v], [-half_u, half_v],
    ])
    hidden_corners = hidden_center[None, :] + corners_local[:, 0:1] * rect_u[None, :] + corners_local[:, 1:2] * rect_v[None, :]

    normal = -down_direction  # 위를 향하도록(down_direction과 반대)
    plane_d = -float(np.dot(normal, hidden_center))

    if debug and debug_log:
        debug_log(
            f"[DEBUG hidden_stack] top={top.candidate_id}: 발견! sides={list(side_stats.keys())} "
            f"du={du:.4f} dv={dv:.4f} z_offset={z_hidden_h:.4f} support_points={total_support_points}"
        )

    if np.any(combined_side_mask):
        support_points = scene_points[idx_shallow[combined_side_mask]].astype(np.float32)
    else:
        support_points = scene_points[idx_shallow].astype(np.float32)

    return PlaneClusterCandidate(
        candidate_id=-100000 - top.candidate_id,
        points=support_points,
        normal=normal,
        median_depth=float(np.median(hidden_corners[:, 2])),
        width=top.width,
        height=top.height,
        area=top.width * top.height,
        fill_ratio=1.0,
        center=hidden_center.astype(np.float64),
        plane_d=plane_d,
        corners_3d=hidden_corners.astype(np.float64),
        pixel_polygon=None,
        synthetic=True,
    )


def sample_quad_surface(
    corner_00: np.ndarray,
    corner_10: np.ndarray,
    corner_11: np.ndarray,
    corner_01: np.ndarray,
    spacing: float,
) -> np.ndarray:
    """네 꼭짓점으로 정의된 사각형 면을 bilinear interpolation으로 균일하게 샘플링."""
    edge_u_length = max(
        float(np.linalg.norm(corner_10 - corner_00)),
        float(np.linalg.norm(corner_11 - corner_01)),
    )
    edge_v_length = max(
        float(np.linalg.norm(corner_01 - corner_00)),
        float(np.linalg.norm(corner_11 - corner_10)),
    )

    if edge_u_length < 1e-8 or edge_v_length < 1e-8:
        return np.empty((0, 3), dtype=np.float64)

    count_u = max(2, int(np.ceil(edge_u_length / spacing)) + 1)
    count_v = max(2, int(np.ceil(edge_v_length / spacing)) + 1)

    u_values = np.linspace(0.0, 1.0, count_u, dtype=np.float64)
    v_values = np.linspace(0.0, 1.0, count_v, dtype=np.float64)
    uu, vv = np.meshgrid(u_values, v_values)
    uu = uu.reshape(-1, 1)
    vv = vv.reshape(-1, 1)

    points = (
        (1.0 - uu) * (1.0 - vv) * corner_00[None, :]
        + uu * (1.0 - vv) * corner_10[None, :]
        + uu * vv * corner_11[None, :]
        + (1.0 - uu) * vv * corner_01[None, :]
    )
    return points.astype(np.float64)


def generate_completed_box_surface(
    corners: np.ndarray,
    point_spacing: float = COMPLETED_SURFACE_POINT_SPACING_M,
    voxel_size: float = COMPLETED_SURFACE_VOXEL_SIZE_M,
) -> np.ndarray:
    """8개 꼭짓점으로 직육면체의 여섯 면을 샘플링한다 (corner 0-3: 윗면, 4-7: 아랫면)."""
    corners = np.asarray(corners, dtype=np.float64)
    if corners.shape != (8, 3):
        return np.empty((0, 3), dtype=np.float32)

    faces = (
        (0, 1, 2, 3),
        (4, 5, 6, 7),
        (0, 1, 5, 4),
        (1, 2, 6, 5),
        (2, 3, 7, 6),
        (3, 0, 4, 7),
    )

    sampled_faces = []
    for i0, i1, i2, i3 in faces:
        face_points = sample_quad_surface(corners[i0], corners[i1], corners[i2], corners[i3], point_spacing)
        if len(face_points) > 0:
            sampled_faces.append(face_points)

    if not sampled_faces:
        return np.empty((0, 3), dtype=np.float32)

    points = np.vstack(sampled_faces)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    return np.asarray(pcd.points, dtype=np.float32)
