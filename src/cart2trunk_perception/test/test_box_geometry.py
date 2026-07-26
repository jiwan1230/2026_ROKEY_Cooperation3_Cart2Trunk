"""cart2trunk_perception.core.box_geometry 단위 테스트.

box_geometry.py는 EDU 저장소 perception/box_geometry.py를 바이트 단위 그대로 이식한
것(core/multiview_box_detector.py가 이 모듈에 의존) - 실측 fixture 대조 검증은
core/multiview_box_detector.py 포팅 시 별도로 수행함(EDU 저장소
results/holonomic_base/cart_stacked_scan_raw.ply, 3박스 적층 시나리오로 원본과
동일한 RANSAC 변동성 프로파일 확인). 여기서는 rclpy 없이 커밋 가능한 손으로 만든
작은 케이스로 기본 기하 연산만 검증한다.
"""
import numpy as np

from cart2trunk_perception.core import box_geometry as bg


def test_make_plane_basis_orthonormal_for_up_normal():
    axis_u, axis_v = bg.make_plane_basis(np.array([0.0, 0.0, 1.0]))
    assert np.isclose(np.linalg.norm(axis_u), 1.0)
    assert np.isclose(np.linalg.norm(axis_v), 1.0)
    assert np.isclose(np.dot(axis_u, axis_v), 0.0, atol=1e-9)
    normal = np.array([0.0, 0.0, 1.0])
    assert np.isclose(np.dot(axis_u, normal), 0.0, atol=1e-9)
    assert np.isclose(np.dot(axis_v, normal), 0.0, atol=1e-9)


def test_order_rectangle_corners_preserves_perimeter_order():
    # 이미 둘레 순서인 정사각형(반시계) - 순서가 안 바뀌어야 함
    square = np.array([
        [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0],
    ])
    ordered = bg.order_rectangle_corners(square.copy())
    assert ordered.shape == (4, 3)
    # 순서가 뒤섞인 입력을 넣어도 같은 둘레 순서(사이클릭 동치)로 복원되는지 확인
    shuffled = square[[2, 0, 3, 1]]
    ordered_shuffled = bg.order_rectangle_corners(shuffled.copy())
    # 두 결과 모두 인접 변 길이가 1,1,1,1이어야 함(사각형 둘레를 올바로 순회)
    for pts in (ordered, ordered_shuffled):
        edges = np.linalg.norm(np.roll(pts, -1, axis=0) - pts, axis=1)
        assert np.allclose(sorted(edges), [1.0, 1.0, 1.0, 1.0], atol=1e-9)


def test_preprocess_cloud_downsamples_dense_points():
    rng = np.random.default_rng(0)
    # 1mm 간격보다 훨씬 촘촘한 점들을 넣으면 voxel_size=0.005 다운샘플로 줄어들어야 함
    points = rng.uniform(0.0, 0.01, size=(5000, 3))
    pcd = bg.preprocess_cloud(points)
    assert len(pcd.points) < len(points)
    assert len(pcd.points) > 0


def _flat_top_candidate(width=0.15, height=0.12, center_z=0.10):
    """폭 width x height, 법선 (0,0,1)인 평평한 사각형 top 후보(합성)."""
    half_w, half_h = width / 2.0, height / 2.0
    corners = np.array([
        [-half_w, -half_h, center_z], [half_w, -half_h, center_z],
        [half_w, half_h, center_z], [-half_w, half_h, center_z],
    ])
    rng = np.random.default_rng(1)
    u = rng.uniform(-half_w, half_w, 500)
    v = rng.uniform(-half_h, half_h, 500)
    points = np.column_stack([u, v, np.full_like(u, center_z)])
    return bg.PlaneClusterCandidate(
        candidate_id=0, points=points.astype(np.float32),
        normal=np.array([0.0, 0.0, 1.0]), median_depth=center_z,
        width=width, height=height, area=width * height, fill_ratio=0.98,
        center=np.array([0.0, 0.0, center_z]), plane_d=-center_z,
        corners_3d=corners, pixel_polygon=None,
    )


def test_compute_box_corners_height_matches_support_distance():
    top = _flat_top_candidate(center_z=0.10)
    # 바닥(z=0)을 지지면으로 삼는 합성 후보
    floor = _flat_top_candidate(width=1.0, height=1.0, center_z=0.0)
    corners = bg.compute_box_corners(top, floor, down_direction=np.array([0.0, 0.0, -1.0]))
    assert corners is not None
    assert corners.shape == (8, 3)
    top_z = corners[:4, 2].mean()
    bottom_z = corners[4:, 2].mean()
    # bottom_cut_margin_m(0.01m)만큼 바닥보다 살짝 위에서 잘림
    assert np.isclose(top_z - bottom_z, 0.10 - bg.BOTTOM_CUT_MARGIN_M, atol=1e-6)
