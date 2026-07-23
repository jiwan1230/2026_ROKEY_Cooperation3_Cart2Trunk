"""트렁크 다중 시점 스캔 포인트클라우드 -> M0609 base_link 좌표계 "선/면 지도"(trunk_map) 변환.

EDU 저장소(algorism 브랜치) isaacpjt/Cart2Trunk/13.export_trunk_map.py를 이식.
Isaac Sim 불필요 (numpy + open3d + scipy만 사용) - 원본 그대로 순수 함수만 옮겼고,
matplotlib 미리보기 이미지 저장(save_preview_png)과 --visualize 3D 뷰어는 core/에
포함하지 않았다 (알고리즘이 아니라 디버그 도구라서 필요해지면 tools/에 별도 이식).

[좌표계 원칙]
    - 바닥/좌우벽/안쪽벽 4면 + AABB x/y/floor_z: 이번 스캔 포인트클라우드에서 직접 측정한 값.
    - 천장(문이 닫히는 높이 한계선): 트렁크가 열린 채로 스캔했으므로 포인트클라우드에 실제로
      존재하지 않는 면. meta의 trunk_bounds.wall_top_z(설계/설정값)와 실측값 중 더 낮은
      (보수적인) 쪽을 채택한다.
    - 출력은 전부 M0609 base_link 좌표계 (meta의 base_pos/base_quat로 world -> base 변환).

[알려진 제약]
    - meta에 base_pos/base_quat가 없는 과거 run은 12.trunk_scan_hidden_gripper.py의 고정
      상수(ROBOT_XY, MOUNT_Z, FACE_ROT_Z=0)로 근사 재구성한다.
    - 바닥 위 점유 공간 검출은 grid+connected-component 기반 휴리스틱이며, 원인(휠하우스/
      기존 물건)을 구분하지 않고 전부 obstacle_N으로 출력한다.
"""
from __future__ import annotations

import numpy as np
import open3d as o3d

# 과거 run(base_pos/base_quat 미저장)에 대한 근사 재구성 - 12.trunk_scan_hidden_gripper.py와 동일 상수.
FALLBACK_TRUNK_X_MIN = 3.11
FALLBACK_ROBOT_XY = (FALLBACK_TRUNK_X_MIN - 0.85, -0.15)
FALLBACK_MOUNT_Z = 0.42
FALLBACK_FACE_ROT_Z = 0.0

CROP_MARGIN_XY = 0.30
CROP_MARGIN_Z = 0.15
OUTLIER_NB_NEIGHBORS = 20
OUTLIER_STD_RATIO = 2.0

BOUND_PCTL_LOW = 1.0
BOUND_PCTL_HIGH = 99.0
CEILING_OVERHEAD_PCTL = 5.0
CEILING_MIN_OVERHEAD_POINTS = 500
CEILING_PLANE_DIST_THRESHOLD = 0.02
CEILING_PLANE_RANSAC_ITERS = 2000
CEILING_MIN_PLANE_INLIERS = 500
CEILING_PLANE_MIN_HORIZONTALITY = 0.8

OCCUPIED_CELL = 0.03
OCCUPIED_EDGE_MARGIN = 0.05
OCCUPIED_BUMP_THRESHOLD = 0.03
OCCUPIED_MIN_CELLS = 6
OCCUPIED_MAX_AREA_FRACTION = 0.25
OCCUPIED_MAX_Y_SPAN_FRACTION = 0.5
OCCUPIED_MAX_HEIGHT_ABOVE_FLOOR = 0.40


# --------------------------------------------------------------------------- #
# 좌표 변환
# --------------------------------------------------------------------------- #

def quat_wxyz_to_matrix(q) -> np.ndarray:
    """Isaac Sim 관례(w, x, y, z) 쿼터니언 -> 3x3 회전행렬."""
    w, x, y, z = q
    n = w * w + x * x + y * y + z * z
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    wx, wy, wz = s * w * x, s * w * y, s * w * z
    xx, xy, xz = s * x * x, s * x * y, s * x * z
    yy, yz, zz = s * y * y, s * y * z, s * z * z
    return np.array([
        [1 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1 - (xx + yy)],
    ])


def world_to_base(points_world: np.ndarray, base_pos: np.ndarray, R_base: np.ndarray) -> np.ndarray:
    """p_base = R_base^T @ (p_world - base_pos)."""
    return (points_world - base_pos[None, :]) @ R_base


def load_base_pose(meta: dict) -> tuple[np.ndarray, np.ndarray]:
    if 'base_pos' in meta and 'base_quat' in meta:
        base_pos = np.asarray(meta['base_pos'], dtype=float)
        base_quat = np.asarray(meta['base_quat'], dtype=float)
        return base_pos, base_quat

    base_pos = np.array([FALLBACK_ROBOT_XY[0], FALLBACK_ROBOT_XY[1], FALLBACK_MOUNT_Z])
    base_quat = np.array([1.0, 0.0, 0.0, 0.0])  # identity (w,x,y,z), FACE_ROT_Z=0 가정
    return base_pos, base_quat


# --------------------------------------------------------------------------- #
# 포인트클라우드 정제
# --------------------------------------------------------------------------- #

def filter_points_world(pts_world: np.ndarray, trunk_bounds: dict) -> np.ndarray:
    x_min, x_max = trunk_bounds['x']
    y_min, y_max = trunk_bounds['y']
    floor_z = trunk_bounds['floor_z']
    wall_top_z = trunk_bounds['wall_top_z']

    mask = (
        (pts_world[:, 0] > x_min - CROP_MARGIN_XY) & (pts_world[:, 0] < x_max + CROP_MARGIN_XY) &
        (pts_world[:, 1] > y_min - CROP_MARGIN_XY) & (pts_world[:, 1] < y_max + CROP_MARGIN_XY) &
        (pts_world[:, 2] > floor_z - CROP_MARGIN_Z) & (pts_world[:, 2] < wall_top_z + CROP_MARGIN_Z)
    )
    cropped = pts_world[mask]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(cropped)
    pcd_clean, _ = pcd.remove_statistical_outlier(
        nb_neighbors=OUTLIER_NB_NEIGHBORS, std_ratio=OUTLIER_STD_RATIO,
    )
    return np.asarray(pcd_clean.points)


# --------------------------------------------------------------------------- #
# AABB (실측: 바닥 + 3벽) 산출
# --------------------------------------------------------------------------- #

def compute_measured_aabb(pts_base: np.ndarray) -> dict:
    x_lo, x_hi = np.percentile(pts_base[:, 0], [BOUND_PCTL_LOW, BOUND_PCTL_HIGH])
    y_lo, y_hi = np.percentile(pts_base[:, 1], [BOUND_PCTL_LOW, BOUND_PCTL_HIGH])
    z_lo = np.percentile(pts_base[:, 2], BOUND_PCTL_LOW)
    return {'x_min': float(x_lo), 'x_max': float(x_hi),
            'y_min': float(y_lo), 'y_max': float(y_hi),
            'floor_z': float(z_lo)}


def compute_ceiling_z_base(trunk_bounds: dict, base_pos: np.ndarray, R_base: np.ndarray,
                            pts_base: np.ndarray, floor_z: float) -> float:
    """천장(점선) 높이 = 설계 상수(wall_top_z)와 실측(RANSAC 평면) 중 더 낮은(보수적인) 값."""
    cx = (trunk_bounds['x'][0] + trunk_bounds['x'][1]) / 2.0
    cy = (trunk_bounds['y'][0] + trunk_bounds['y'][1]) / 2.0
    p_world = np.array([[cx, cy, trunk_bounds['wall_top_z']]])
    config_ceiling_z = float(world_to_base(p_world, base_pos, R_base)[0][2])

    overhead = pts_base[pts_base[:, 2] > floor_z + OCCUPIED_MAX_HEIGHT_ABOVE_FLOOR]
    measured_ceiling_z = config_ceiling_z

    if len(overhead) >= CEILING_MIN_OVERHEAD_POINTS:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(overhead)
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=CEILING_PLANE_DIST_THRESHOLD, ransac_n=3,
            num_iterations=CEILING_PLANE_RANSAC_ITERS,
        )
        if len(inliers) >= CEILING_MIN_PLANE_INLIERS:
            a, b, c, _ = plane_model
            horizontality = abs(c) / (np.linalg.norm([a, b, c]) + 1e-9)
            if horizontality >= CEILING_PLANE_MIN_HORIZONTALITY:
                plane_pts = overhead[inliers]
                measured_ceiling_z = float(np.percentile(plane_pts[:, 2], CEILING_OVERHEAD_PCTL))

    return min(config_ceiling_z, measured_ceiling_z)


# --------------------------------------------------------------------------- #
# 바닥 위 점유 공간(obstacle) 검출 - grid 휴리스틱 + connected-component
# --------------------------------------------------------------------------- #

def detect_occupied_regions(pts_base: np.ndarray, aabb: dict) -> list[dict]:
    """열린 트렁크 문(리드)이 공중에 떠서 찍힌 점들은 OCCUPIED_MAX_HEIGHT_ABOVE_FLOOR보다
    훨씬 위에 있으므로 후보 점 집합에서 제외하고 시작한다. 남은 낮은 점들에서 바닥 위로
    솟은 덩어리를 모두 찾아 obstacle_N으로 반환한다 (원인 구분 없음)."""
    from scipy import ndimage

    x_min, x_max = aabb['x_min'], aabb['x_max']
    y_min, y_max = aabb['y_min'], aabb['y_max']
    floor_z = aabb['floor_z']

    band_pts = pts_base[pts_base[:, 2] < floor_z + OCCUPIED_MAX_HEIGHT_ABOVE_FLOOR]

    nx = max(1, int((x_max - x_min) / OCCUPIED_CELL))
    ny = max(1, int((y_max - y_min) / OCCUPIED_CELL))
    x_edges = np.linspace(x_min, x_max, nx + 1)
    y_edges = np.linspace(y_min, y_max, ny + 1)

    ix = np.clip(np.digitize(band_pts[:, 0], x_edges) - 1, 0, nx - 1)
    iy = np.clip(np.digitize(band_pts[:, 1], y_edges) - 1, 0, ny - 1)

    local_min_z = np.full((nx, ny), np.nan)
    for cx in range(nx):
        for cy in range(ny):
            sel = band_pts[(ix == cx) & (iy == cy), 2]
            if len(sel) >= 3:
                local_min_z[cx, cy] = np.percentile(sel, 5)

    edge_margin_cells_x = max(1, int(OCCUPIED_EDGE_MARGIN / OCCUPIED_CELL))
    edge_margin_cells_y = max(1, int(OCCUPIED_EDGE_MARGIN / OCCUPIED_CELL))
    interior = np.zeros_like(local_min_z, dtype=bool)
    interior[edge_margin_cells_x:nx - edge_margin_cells_x, edge_margin_cells_y:ny - edge_margin_cells_y] = True
    interior &= ~np.isnan(local_min_z)
    n_interior = int(np.count_nonzero(interior))
    if n_interior == 0:
        return []

    floor_ref = float(np.median(local_min_z[interior]))
    bump_mask = interior & (local_min_z > (floor_ref + OCCUPIED_BUMP_THRESHOLD))

    labeled, n_blobs = ndimage.label(bump_mask, structure=np.ones((3, 3)))
    y_span_total = y_max - y_min
    blobs = []
    for blob_id in range(1, n_blobs + 1):
        blob = labeled == blob_id
        n_cells = int(np.count_nonzero(blob))
        area_fraction = n_cells / n_interior
        if n_cells < OCCUPIED_MIN_CELLS:
            continue
        if area_fraction > OCCUPIED_MAX_AREA_FRACTION:
            continue

        cx_idx, cy_idx = np.nonzero(blob)
        bx_min, bx_max = x_edges[cx_idx.min()], x_edges[cx_idx.max() + 1]
        by_min, by_max = y_edges[cy_idx.min()], y_edges[cy_idx.max() + 1]
        if (by_max - by_min) > OCCUPIED_MAX_Y_SPAN_FRACTION * y_span_total:
            continue

        top_z = float(np.nanpercentile(local_min_z[blob], 90))
        blobs.append({'x_min': float(bx_min), 'x_max': float(bx_max),
                       'y_min': float(by_min), 'y_max': float(by_max),
                       'floor_z': floor_z, 'top_z': top_z, 'n_cells': n_cells})

    if not blobs:
        return []

    results = []
    for i, b in enumerate(sorted(blobs, key=lambda b: -b['n_cells']), start=1):
        entry = dict(b, name=f'obstacle_{i}')
        results.append(entry)
    return results


# --------------------------------------------------------------------------- #
# JSON 조립
# --------------------------------------------------------------------------- #

def box_vertices(x_min, x_max, y_min, y_max, z_min, z_max):
    return [
        [x_min, y_min, z_min], [x_max, y_min, z_min],
        [x_max, y_max, z_min], [x_min, y_max, z_min],
        [x_min, y_min, z_max], [x_max, y_min, z_max],
        [x_max, y_max, z_max], [x_min, y_max, z_max],
    ]


def build_trunk_map(aabb: dict, ceiling_z: float, detected_objects: list[dict],
                     run_id: str, n_raw: int, n_filtered: int) -> dict:
    v = box_vertices(aabb['x_min'], aabb['x_max'], aabb['y_min'], aabb['y_max'],
                      aabb['floor_z'], ceiling_z)

    edges = [
        {'v': [0, 1], 'style': 'solid'}, {'v': [1, 2], 'style': 'solid'},
        {'v': [2, 3], 'style': 'solid'}, {'v': [3, 0], 'style': 'solid'},
        {'v': [4, 5], 'style': 'dashed'}, {'v': [5, 6], 'style': 'dashed'},
        {'v': [6, 7], 'style': 'dashed'}, {'v': [7, 4], 'style': 'dashed'},
        {'v': [0, 4], 'style': 'solid'}, {'v': [1, 5], 'style': 'solid'},
        {'v': [2, 6], 'style': 'solid'}, {'v': [3, 7], 'style': 'solid'},
    ]
    faces = [
        {'name': 'floor', 'v': [0, 1, 2, 3], 'style': 'solid'},
        {'name': 'wall_y_min', 'v': [0, 1, 5, 4], 'style': 'solid'},
        {'name': 'wall_y_max', 'v': [3, 2, 6, 7], 'style': 'solid'},
        {'name': 'wall_x_max', 'v': [1, 2, 6, 5], 'style': 'solid'},
        {'name': 'ceiling_limit', 'v': [4, 5, 6, 7], 'style': 'dashed'},
    ]

    result = {
        'schema_version': '1.0',
        'run_id': run_id,
        'frame': 'm0609_base_link',
        'note': (
            'floor/wall_y_min/wall_y_max/wall_x_max + AABB x/y/floor_z: 이번 스캔에서 실측 (solid). '
            'ceiling_limit: 박스를 이 높이 넘게 쌓으면 안 되는 한계선(dashed) - 설계 상수(wall_top_z)와 '
            '실측(관측된 가장 낮은 상부 구조물, 정체 불문 보수적으로 반영) 중 더 낮은 값을 채택. '
            'x_min 쪽(입구)은 열린 방향이라 벽 없음.'
        ),
        'vertices': v,
        'edges': edges,
        'faces': faces,
        'source_stats': {
            'n_points_raw': n_raw,
            'n_points_filtered': n_filtered,
            'crop_margin_xy_m': CROP_MARGIN_XY,
            'crop_margin_z_m': CROP_MARGIN_Z,
            'outlier_removal': {'nb_neighbors': OUTLIER_NB_NEIGHBORS, 'std_ratio': OUTLIER_STD_RATIO},
        },
    }

    result['obstacles'] = [
        {
            'name': obj['name'],
            'vertices': box_vertices(obj['x_min'], obj['x_max'], obj['y_min'], obj['y_max'],
                                      obj['floor_z'], obj['top_z']),
            'style': 'solid',
            'note': (
                'grid 휴리스틱 자동 검출 결과 - 휠하우스/기존 물건 구분 없이 바닥 위 점유 공간을 '
                '모두 extreme point(AABB)로 표시. 필요시 손으로 조정'
            ),
        }
        for obj in detected_objects
    ]

    return result


# --------------------------------------------------------------------------- #
# 조합 (npy/meta -> trunk_map dict)
# --------------------------------------------------------------------------- #

def build_trunk_map_from_scan(pts_world: np.ndarray, meta: dict, run_id: str) -> dict:
    """월드 프레임 포인트클라우드(pts_world) + 스캔 메타데이터(meta)로 trunk_map dict를 만든다.
    13.export_trunk_map.py의 main()에서 파일 입출력/시각화를 뺀 순수 계산 부분."""
    trunk_bounds = meta['trunk_bounds']
    n_raw = len(pts_world)

    base_pos, base_quat = load_base_pose(meta)
    R_base = quat_wxyz_to_matrix(base_quat)

    pts_world_filtered = filter_points_world(pts_world, trunk_bounds)
    pts_base = world_to_base(pts_world_filtered, base_pos, R_base)

    aabb = compute_measured_aabb(pts_base)
    ceiling_z = compute_ceiling_z_base(trunk_bounds, base_pos, R_base, pts_base, aabb['floor_z'])
    detected_objects = detect_occupied_regions(pts_base, aabb)

    return build_trunk_map(aabb, ceiling_z, detected_objects, run_id, n_raw, len(pts_base))
