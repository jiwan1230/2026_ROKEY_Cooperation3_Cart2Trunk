"""cart2trunk_perception.core.trunk_map_builder 단위/통합 테스트.

큰 실제 스캔 fixture(run_20260720_160153, 17MB npy)는 이 저장소에 커밋하지 않았다 -
이 파일의 테스트는 손으로 만든 작은 합성 포인트클라우드로 검증한다. 포팅 직후
실제 fixture로 원본 13.export_trunk_map.py와 1:1 비교 검증은 이미 별도로 수행함
(obstacles/faces/edges/source_stats 전부 정확히 일치, ceiling_z만 RANSAC
평면검출 특유의 run-to-run 비결정성으로 수~십수 mm 차이).
"""
import numpy as np
import pytest

from cart2trunk_perception.core import trunk_map_builder as tmb


def test_quat_wxyz_to_matrix_identity():
    R = tmb.quat_wxyz_to_matrix([1.0, 0.0, 0.0, 0.0])
    assert np.allclose(R, np.eye(3))


def test_quat_wxyz_to_matrix_90deg_about_z():
    # 90도 z회전: x축 -> y축
    q = [np.cos(np.pi / 4), 0.0, 0.0, np.sin(np.pi / 4)]
    R = tmb.quat_wxyz_to_matrix(q)
    x_axis = np.array([1.0, 0.0, 0.0])
    assert np.allclose(R @ x_axis, [0.0, 1.0, 0.0], atol=1e-9)


def test_world_to_base_pure_translation():
    pts_world = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
    base_pos = np.array([1.0, 1.0, 1.0])
    R_identity = np.eye(3)
    pts_base = tmb.world_to_base(pts_world, base_pos, R_identity)
    assert np.allclose(pts_base, [[0.0, 1.0, 2.0], [3.0, 4.0, 5.0]])


def test_load_base_pose_uses_meta_when_present():
    meta = {'base_pos': [1.0, 2.0, 3.0], 'base_quat': [1.0, 0.0, 0.0, 0.0]}
    base_pos, base_quat = tmb.load_base_pose(meta)
    assert np.allclose(base_pos, [1.0, 2.0, 3.0])
    assert np.allclose(base_quat, [1.0, 0.0, 0.0, 0.0])


def test_load_base_pose_falls_back_when_missing():
    base_pos, base_quat = tmb.load_base_pose({})
    assert np.allclose(base_pos, [
        tmb.FALLBACK_ROBOT_XY[0], tmb.FALLBACK_ROBOT_XY[1], tmb.FALLBACK_MOUNT_Z,
    ])
    assert np.allclose(base_quat, [1.0, 0.0, 0.0, 0.0])


def test_box_vertices_order():
    v = tmb.box_vertices(0, 1, 0, 2, 0, 3)
    assert v[0] == [0, 0, 0]
    assert v[6] == [1, 2, 3]  # 대각선 반대 코너 (x_max,y_max,z_max)


def test_compute_measured_aabb_matches_percentile_bounds():
    rng = np.random.default_rng(0)
    pts = rng.uniform(low=[0.0, 0.0, 0.0], high=[1.0, 2.0, 3.0], size=(5000, 3))
    aabb = tmb.compute_measured_aabb(pts)
    assert aabb['x_min'] == pytest.approx(0.0, abs=0.02)
    assert aabb['x_max'] == pytest.approx(1.0, abs=0.02)
    assert aabb['y_min'] == pytest.approx(0.0, abs=0.04)
    assert aabb['y_max'] == pytest.approx(2.0, abs=0.04)
    assert aabb['floor_z'] == pytest.approx(0.0, abs=0.06)


def _synthetic_room(rng, floor_z=0.0, x=(0.0, 1.0), y=(0.0, 1.5), bump=True):
    """바닥(+옵션 bump 하나)을 나타내는 합성 포인트클라우드.

    실제 depth 카메라는 불투명한 덩어리 아래 바닥을 보지 못하므로(가려짐), bump
    영역에서는 바닥 점을 빼고 bump 표면 점만 남긴다 - 그래야 detect_occupied_regions의
    "그 cell의 최저 높이(5th percentile)" 판정이 실제 스캔과 같은 조건이 된다.
    """
    bump_x, bump_y = (0.5, 0.7), (0.5, 0.7)
    n = 20000
    xs = rng.uniform(x[0], x[1], n)
    ys = rng.uniform(y[0], y[1], n)
    zs = floor_z + rng.normal(0, 0.002, n)  # 바닥 평면 + 약간의 노이즈
    pts = np.stack([xs, ys, zs], axis=1)

    if bump:
        under_bump = (
            (pts[:, 0] >= bump_x[0]) & (pts[:, 0] <= bump_x[1]) &
            (pts[:, 1] >= bump_y[0]) & (pts[:, 1] <= bump_y[1])
        )
        pts = pts[~under_bump]

        bn = 400
        bxs = rng.uniform(*bump_x, bn)
        bys = rng.uniform(*bump_y, bn)
        bzs = floor_z + 0.10 + rng.normal(0, 0.002, bn)  # 바닥+10cm 솟은 덩어리 (윗면만 관측됨)
        bump_pts = np.stack([bxs, bys, bzs], axis=1)
        pts = np.concatenate([pts, bump_pts], axis=0)

    return pts


def test_detect_occupied_regions_finds_synthetic_bump():
    rng = np.random.default_rng(1)
    pts_base = _synthetic_room(rng, bump=True)
    aabb = tmb.compute_measured_aabb(pts_base)

    obstacles = tmb.detect_occupied_regions(pts_base, aabb)

    assert len(obstacles) == 1
    obs = obstacles[0]
    assert obs['x_min'] == pytest.approx(0.5, abs=0.05)
    assert obs['x_max'] == pytest.approx(0.7, abs=0.05)
    assert obs['top_z'] - obs['floor_z'] == pytest.approx(0.10, abs=0.03)


def test_detect_occupied_regions_empty_floor_has_no_obstacles():
    rng = np.random.default_rng(2)
    pts_base = _synthetic_room(rng, bump=False)
    aabb = tmb.compute_measured_aabb(pts_base)

    obstacles = tmb.detect_occupied_regions(pts_base, aabb)

    assert obstacles == []


def test_build_trunk_map_from_scan_end_to_end():
    rng = np.random.default_rng(3)
    floor_z = 0.5
    ceiling_z = 1.5
    x_range, y_range = (2.0, 3.0), (-0.5, 0.5)

    n = 20000
    floor_pts = np.stack([
        rng.uniform(*x_range, n), rng.uniform(*y_range, n), floor_z + rng.normal(0, 0.002, n),
    ], axis=1)
    ceiling_pts = np.stack([
        rng.uniform(*x_range, n), rng.uniform(*y_range, n), ceiling_z + rng.normal(0, 0.002, n),
    ], axis=1)
    pts_world = np.concatenate([floor_pts, ceiling_pts], axis=0)

    meta = {
        'trunk_bounds': {
            'x': list(x_range), 'y': list(y_range), 'floor_z': floor_z, 'wall_top_z': ceiling_z,
        },
        # base_pos/base_quat 없음 -> fallback 경로 검증 겸함
    }

    trunk_map = tmb.build_trunk_map_from_scan(pts_world, meta, run_id='synthetic_test')

    assert trunk_map['run_id'] == 'synthetic_test'
    assert trunk_map['frame'] == 'm0609_base_link'
    assert len(trunk_map['vertices']) == 8
    assert len(trunk_map['edges']) == 12
    assert len(trunk_map['faces']) == 5
    assert trunk_map['obstacles'] == []
