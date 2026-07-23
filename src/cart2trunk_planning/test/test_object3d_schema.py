"""
test_01_object3d_schema.py
① load_boxes_from_vision_json()이 지완의 실제 박스 비전 JSON
(all_boxes_corners_*.json 스타일)을 Object3D 리스트로 올바르게 변환하는지 검증.

배경: 실제 샘플 파일을 열어보니 각 박스가 center_xyz/size_xyz가 아니라 8개
모서리 좌표(corners_m, 회전된 3D 박스)로 온다는 걸 확인함. 우리 알고리즘은 회전을
다루지 않으므로(MVP 범위, ③ fits_dims 참고), 트렁크 변환(② to_bounding_trunk)과
같은 방식으로 8개 점의 min/max로 AABB 근사해서 center/size를 뽑는다.

가장 중요한 발견: 실제 샘플의 coordinate_frame이 "depth_camera_optical_frame..."
(카메라 좌표계)였다 - 트렁크 데이터(m0609_base_link)와 다른 좌표계라서, 그대로
섞어 쓰면 엉뚱한 자리에 배치된다. 그래서 로더가 frame을 확인해서, base 좌표계가
아니면 조용히 틀린 결과를 내지 않고 바로 에러를 낸다.
"""
import json
import sys
import pathlib
from importlib import import_module

import pytest

_m01 = import_module("cart2trunk_planning.core.object3d_schema")
_m03 = import_module("cart2trunk_planning.core.extreme_point_candidates")

load_boxes_from_vision_json = _m01.load_boxes_from_vision_json
Object3D = _m01.Object3D
object3d_to_box = _m01.object3d_to_box


def _write_json(tmp_path, data):
    path = tmp_path / "boxes.json"
    path.write_text(json.dumps(data))
    return str(path)


def _box_entry(box_id, x_range, y_range, z_range):
    """AABB 스타일 8개 모서리(회전 없음)로 박스 하나를 만든다 - 계산 검증을 쉽게 하려고
    실제 샘플처럼 회전된 값 대신 축정렬된 값을 쓴다 (min/max 근사 로직은 회전 여부와
    무관하게 똑같이 동작하므로 검증엔 문제없음)."""
    x0, x1 = x_range
    y0, y1 = y_range
    z0, z1 = z_range
    corners = [
        [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1],  # top
        [x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],  # bottom
    ]
    return {"box_id": box_id, "support_type": "floor", "corners_m": corners}


def test_load_boxes_computes_center_and_size_from_corners(tmp_path):
    data = {
        "coordinate_frame": "m0609_base_link",
        "boxes": [_box_entry(0, (1.0, 1.3), (0.5, 0.7), (0.0, 0.15))],
    }
    path = _write_json(tmp_path, data)

    boxes = load_boxes_from_vision_json(path)

    assert len(boxes) == 1
    obj = boxes[0]
    assert obj.id == "0"
    assert obj.size_xyz == pytest.approx((0.3, 0.2, 0.15))
    assert obj.center_xyz == pytest.approx((1.15, 0.6, 0.075))
    assert obj.volume == pytest.approx(0.3 * 0.2 * 0.15)


def test_load_boxes_multiple_entries_get_distinct_ids_and_default_confidence(tmp_path):
    """
    box_id마다 별개 id로 매핑되는지 + confidence를 확인한다. 실제 샘플엔 confidence
    필드가 아예 없음 - Q1 답변대로 "0.7 이하는 Vision 단에서 이미 필터링됨"을
    전제로 남아있는 박스는 전부 1.0으로 채운다.
    """
    data = {
        "coordinate_frame": "m0609_base_link",
        "boxes": [
            _box_entry(0, (0.0, 0.1), (0.0, 0.1), (0.0, 0.1)),
            _box_entry(1, (0.2, 0.3), (0.0, 0.1), (0.0, 0.1)),
        ],
    }
    path = _write_json(tmp_path, data)

    boxes = load_boxes_from_vision_json(path)

    assert [b.id for b in boxes] == ["0", "1"]
    assert all(b.confidence == 1.0 for b in boxes)


def test_load_boxes_rejects_non_base_frame(tmp_path):
    """
    실제로 발견된 문제 재현: coordinate_frame이 m0609_base_link가 아니면(예: 카메라
    좌표계) 조용히 잘못된 좌표를 쓰지 않고 바로 에러를 내야 한다.
    """
    data = {
        "coordinate_frame": "depth_camera_optical_frame_from_message_header",
        "boxes": [_box_entry(0, (0.0, 0.1), (0.0, 0.1), (0.0, 0.1))],
    }
    path = _write_json(tmp_path, data)

    with pytest.raises(ValueError, match="m0609_base_link"):
        load_boxes_from_vision_json(path)


def test_object3d_to_box_carries_rests_on_id_through():
    """
    ⑥ 픽업 순서 제약(rests_on_id)이 Object3D -> Box 변환에서 안 끊기고 그대로
    전달되는지 확인 - 이게 끊기면 ⑥이 아무리 잘 만들어져도 실제로는 항상
    rests_on_id=None만 받게 되어 무용지물이 된다.
    """
    obj = Object3D("Cart_Blue1", (0, 0, 0), (0.09, 0.10, 0.12), 0.09 * 0.10 * 0.12,
                    confidence=1.0, rests_on_id="Cart_Green")
    box = object3d_to_box(obj)
    assert box.rests_on_id == "Cart_Green"


def test_load_boxes_from_vision_json_leaves_rests_on_id_unset_for_now(tmp_path):
    """
    실제 샘플의 support_candidate_id 필드가 정확히 무슨 의미인지(진짜 '깔린 박스
    id'가 맞는지) 아직 지완/준형님 확인 전이라, 지금은 잘못 매핑해서 조용히 틀린
    픽업 순서 제약을 만드는 것보다 안 채우는 쪽이 안전하다 - 확인되면 여기를 고치면 됨.
    """
    data = {
        "coordinate_frame": "m0609_base_link",
        "boxes": [_box_entry(0, (0.0, 0.1), (0.0, 0.1), (0.0, 0.1))],
    }
    path = _write_json(tmp_path, data)

    boxes = load_boxes_from_vision_json(path)

    assert boxes[0].rests_on_id is None
