"""cart2trunk_perception.core.multiview_box_detector 통합 테스트.

큰 실제 스캔 fixture(EDU 저장소 results/holonomic_base/cart_stacked_scan_raw.ply,
3.6MB)는 이 저장소에 커밋하지 않았다 - 포팅 직후 그 fixture로 원본
perception/multiview_scan.py와 나란히 3회씩 재실행해서 이미 별도 검증함(둘 다
"복원된 박스 3개"로 수렴, RANSAC 비결정성으로 인한 변동 폭도 동일한 프로파일임을
확인 - trunk_map_builder.py 포팅 때와 같은 방법론). 여기서는 손으로 만든 작은
합성 point cloud로, 실제 depth 카메라의 가려짐(박스 바로 아래 바닥은 안 보임)을
반영해 회귀 테스트를 짠다(trunk_map_builder.py 테스트의 _synthetic_room과 같은 원칙 -
가려짐을 반영 안 하면 지지면 판정 로직이 실제 스캔과 다른 조건으로 검증되어
테스트가 거짓으로 통과할 수 있다).
"""
import numpy as np
import pytest

from cart2trunk_perception.core import multiview_box_detector as mvd


def _single_box_on_floor_cloud(
    box_xy=(0.45, 0.35), box_size=(0.15, 0.12), box_height=0.10,
    floor_extent=(1.0, 1.0), seed=0,
):
    """바닥(z=0, floor_extent 크기) 위 (box_xy)에 놓인 box_size x box_height 박스
    하나. 박스 풋프린트 아래 바닥 점은 제외(가려짐 반영)."""
    rng = np.random.default_rng(seed)
    x0, y0 = box_xy
    bw, bh = box_size
    fx, fy = floor_extent

    n_floor = 40000
    xs = rng.uniform(0.0, fx, n_floor)
    ys = rng.uniform(0.0, fy, n_floor)
    under_box = (
        (xs >= x0) & (xs <= x0 + bw) & (ys >= y0) & (ys <= y0 + bh)
    )
    xs, ys = xs[~under_box], ys[~under_box]
    zs_floor = rng.normal(0.0, 0.001, len(xs))
    floor_points = np.column_stack([xs, ys, zs_floor])

    n_top = 1200
    top_xs = rng.uniform(x0, x0 + bw, n_top)
    top_ys = rng.uniform(y0, y0 + bh, n_top)
    top_zs = box_height + rng.normal(0.0, 0.001, n_top)
    top_points = np.column_stack([top_xs, top_ys, top_zs])

    return np.vstack([floor_points, top_points])


def test_detects_single_box_with_correct_height_and_footprint():
    points = _single_box_on_floor_cloud()
    boxes = mvd.detect_boxes_in_base_frame(points, trials=3)

    assert len(boxes) == 1
    box = boxes[0]
    corners = np.asarray(box["corners"])
    assert corners.shape == (8, 3)

    height = corners[:4, 2].mean() - corners[4:, 2].mean()
    assert height == pytest.approx(0.10, abs=0.02)

    footprint_x = corners[:4, 0].max() - corners[:4, 0].min()
    footprint_y = corners[:4, 1].max() - corners[:4, 1].min()
    sides = sorted([footprint_x, footprint_y])
    assert sides[0] == pytest.approx(0.12, abs=0.02)
    assert sides[1] == pytest.approx(0.15, abs=0.02)


def test_no_boxes_when_scene_is_empty_floor():
    rng = np.random.default_rng(0)
    xs = rng.uniform(0.0, 1.0, 20000)
    ys = rng.uniform(0.0, 1.0, 20000)
    zs = rng.normal(0.0, 0.001, 20000)
    points = np.column_stack([xs, ys, zs])

    boxes = mvd.detect_boxes_in_base_frame(points, trials=2)
    assert boxes == []


def test_load_merged_cloud_rejects_wrong_shape(tmp_path):
    bad = np.zeros((10, 2))
    path = tmp_path / "bad.npy"
    np.save(path, bad)
    with pytest.raises(ValueError):
        mvd.load_merged_cloud(path)
