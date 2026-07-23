"""
test_14_run_full_pipeline.py
⑭ 실제 Vision 데이터 두 파일(trunk_map.json 스타일 + boxes.json 스타일)을 받아서
전체 파이프라인을 돌리고 base frame 좌표를 돌려주는 진입점 검증.
"""
import json
import sys
import pathlib
from importlib import import_module

_m14 = import_module("cart2trunk_planning.core.run_full_pipeline")

run_pipeline = _m14.run_pipeline


def _write_trunk_map(tmp_path, offset=(1.0, 1.0, 0.0), size=(0.6, 0.5, 0.4)):
    """② load_trunk_from_world_map()이 실제로 읽는 최소 필드(vertices/edges)만 채운
    합성 trunk_map.json. 로컬 (0,0,0) 코너가 base 좌표계 offset에 오도록 만든다."""
    ox, oy, oz = offset
    w, d, h = size
    floor = [[ox, oy, oz], [ox + w, oy, oz], [ox + w, oy + d, oz], [ox, oy + d, oz]]
    ceiling = [[x, y, oz + h] for x, y, _ in floor]
    vertices = floor + ceiling
    edges = [{"v": [0, 1]}, {"v": [1, 2]}, {"v": [2, 3]}, {"v": [3, 0]}]
    data = {"frame": "m0609_base_link", "vertices": vertices, "edges": edges, "obstacles": []}
    path = tmp_path / "trunk_map.json"
    path.write_text(json.dumps(data))
    return str(path)


def _write_boxes(tmp_path, box_base_frame_ranges):
    """box_base_frame_ranges: [(x_range, y_range, z_range), ...] - 전부 base 좌표계 기준."""
    boxes = []
    for i, (xr, yr, zr) in enumerate(box_base_frame_ranges):
        x0, x1 = xr
        y0, y1 = yr
        z0, z1 = zr
        corners = [
            [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1],
            [x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
        ]
        boxes.append({"box_id": i, "support_type": "floor", "corners_m": corners})
    data = {"coordinate_frame": "m0609_base_link", "boxes": boxes}
    path = tmp_path / "boxes.json"
    path.write_text(json.dumps(data))
    return str(path)


def test_run_pipeline_places_box_within_trunk_base_frame_bounds(tmp_path):
    """
    작은 박스 하나가 base 좌표계 기준으로 트렁크 범위(offset ~ offset+size) 안의
    base frame 좌표에 배치되는지 확인한다 (로컬 계산 후 base frame으로 정확히
    되돌아오는지가 핵심 - local_to_base_frame 왕복이 안 맞으면 로봇이 엉뚱한
    곳에 손을 뻗게 됨).
    """
    offset = (1.0, 1.0, 0.0)
    trunk_map_path = _write_trunk_map(tmp_path, offset=offset, size=(0.6, 0.5, 0.4))
    # 박스 크기만 의미 있음 (corners 자체의 절대 위치는 트렁크 안이든 밖이든 무관 -
    # ①은 크기만 뽑아서 넘기고, 실제 배치 위치는 ⑦이 새로 계산함)
    boxes_path = _write_boxes(tmp_path, [((0.0, 0.2), (0.0, 0.15), (0.0, 0.1))])

    result = run_pipeline(trunk_map_path, boxes_path, allow_stacking=False)

    assert len(result["placements"]) == 1
    assert len(result["unloadable"]) == 0
    p = result["placements"][0]

    bx, by, bz = p["position_base_frame"]
    ox, oy, oz = offset
    w, d, h = 0.6, 0.5, 0.4
    assert ox - 1e-9 <= bx <= ox + w + 1e-9
    assert oy - 1e-9 <= by <= oy + d + 1e-9
    assert oz - 1e-9 <= bz <= oz + h + 1e-9
    # base frame 좌표가 로컬 좌표 + offset과 정확히 일치하는지 (왕복 변환 검증)
    lx, ly, lz = p["position_local"]
    assert bx == lx + ox and by == ly + oy and bz == lz + oz


def test_run_pipeline_reports_unloadable_when_box_too_big(tmp_path):
    trunk_map_path = _write_trunk_map(tmp_path, offset=(0.0, 0.0, 0.0), size=(0.3, 0.3, 0.2))
    boxes_path = _write_boxes(tmp_path, [((0.0, 0.5), (0.0, 0.5), (0.0, 0.5))])  # 트렁크보다 큼

    result = run_pipeline(trunk_map_path, boxes_path)

    assert len(result["placements"]) == 0
    assert len(result["unloadable"]) == 1
    assert result["unloadable"][0]["reason"] == "SIZE_EXCEEDS_TRUNK"
