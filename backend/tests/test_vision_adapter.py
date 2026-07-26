import json
import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
import vision_adapter

# "Cart2Trunk 최종 프로젝트 시나리오 및 시스템 흐름" 문서 4.2절 예시를 그대로 옮김.
_BOX_SCAN_JSON = {
    "snapshot_id": "box_scan_001",
    "frame_id": "m0609_base_link",  # 문서 예시는 "m0609_base"라 오타로 보임 -
    # ①의 확정 상수(EXPECTED_BOX_FRAME="m0609_base_link")를 기준으로 검증하므로 여기선 그 값을 씀.
    "quality_score": 0.94,
    "boxes": [
        {
            "box_id": "BOX_01", "center": [0.1, 0.2, 0.075], "size": [0.3, 0.2, 0.15],
            "yaw": 0.35, "corners": [[0.0, 0.0, 0.0]], "confidence": 0.95,
        },
        {
            "box_id": "BOX_02", "center": [0.1, 0.2, 0.06], "size": [0.25, 0.18, 0.12],
            "yaw": 0.0, "corners": [[0.0, 0.0, 0.0]], "confidence": 0.91,
        },
    ],
}


def test_boxes_from_box_scan_converts_to_simple_dicts():
    boxes, snapshot_id = vision_adapter.boxes_from_box_scan(_BOX_SCAN_JSON)

    assert snapshot_id == "box_scan_001"
    assert boxes == [
        {"id": "BOX_01", "width": 0.3, "depth": 0.2, "height": 0.15, "rests_on_id": None, "initial_yaw": 0.35},
        {"id": "BOX_02", "width": 0.25, "depth": 0.18, "height": 0.12, "rests_on_id": None, "initial_yaw": 0.0},
    ]


def test_boxes_from_box_scan_rejects_wrong_frame():
    bad = {**_BOX_SCAN_JSON, "frame_id": "camera_frame"}
    with pytest.raises(ValueError, match="camera_frame"):
        vision_adapter.boxes_from_box_scan(bad)


# 실제 준형 샘플(~/Downloads/all_boxes_corners_20260721_174311_555644.json)과 같은 구조로,
# 좌표계만 올바르게(m0609_base_link) 바꿔서 만든 합성 데이터 - 로직(AABB, rests_on_id 매핑)을
# 검증하기 위함. box_id=2(바닥)의 top_candidate_id=0 위에 box_id=0,1이 나란히 얹혀 있다.
def _corner_box(box_id, top_candidate_id, support_candidate_id, support_type, x0, y0, z0, w, d, h):
    return {
        "box_id": box_id, "support_type": support_type,
        "top_candidate_id": top_candidate_id, "support_candidate_id": support_candidate_id,
        "corners_m": [
            [x0, y0, z0 + h], [x0 + w, y0, z0 + h], [x0 + w, y0 + d, z0 + h], [x0, y0 + d, z0 + h],
            [x0, y0, z0], [x0 + w, y0, z0], [x0 + w, y0 + d, z0], [x0, y0 + d, z0],
        ],
    }


_VISION_CORNERS_JSON = {
    "coordinate_frame": "m0609_base_link",
    "unit": "meter",
    "box_count": 3,
    "completed_ply_file": "run_test.ply",
    "boxes": [
        _corner_box(0, top_candidate_id=1, support_candidate_id=0, support_type="box_top",
                    x0=0.0, y0=0.0, z0=0.2, w=0.2, d=0.15, h=0.1),
        _corner_box(1, top_candidate_id=2, support_candidate_id=0, support_type="box_top",
                    x0=0.25, y0=0.0, z0=0.2, w=0.18, d=0.14, h=0.09),
        _corner_box(2, top_candidate_id=0, support_candidate_id=-1, support_type="floor",
                    x0=0.0, y0=0.0, z0=0.0, w=0.5, d=0.35, h=0.2),
    ],
}


def test_boxes_from_vision_corners_computes_aabb_and_snapshot_id():
    boxes, snapshot_id = vision_adapter.boxes_from_vision_corners(_VISION_CORNERS_JSON)

    assert snapshot_id == "run_test.ply"
    by_id = {b["id"]: b for b in boxes}
    assert by_id["2"]["width"] == pytest.approx(0.5)
    assert by_id["2"]["depth"] == pytest.approx(0.35)
    assert by_id["2"]["height"] == pytest.approx(0.2)


def test_boxes_from_vision_corners_maps_support_candidate_to_the_box_that_owns_that_top_candidate():
    # box_id=0,1 둘 다 support_candidate_id=0인데, 그건 "box_id=0을 가리키는 게
    # 아니라" top_candidate_id=0을 가진 box_id=2를 가리킨다(실제 샘플로 검증한 규칙).
    boxes, _ = vision_adapter.boxes_from_vision_corners(_VISION_CORNERS_JSON)
    by_id = {b["id"]: b for b in boxes}

    assert by_id["0"]["rests_on_id"] == "2"
    assert by_id["1"]["rests_on_id"] == "2"
    assert by_id["2"]["rests_on_id"] is None  # 바닥


def test_boxes_from_vision_corners_rejects_wrong_frame():
    bad = {**_VISION_CORNERS_JSON, "coordinate_frame": "depth_camera_optical_frame_from_message_header"}
    with pytest.raises(ValueError, match="depth_camera_optical_frame_from_message_header"):
        vision_adapter.boxes_from_vision_corners(bad)


# 준형이 실제로 넘겨준 샘플 파일 - 다운로드돼 있으면 "지금 이 파일을 그대로
# 돌리면 좌표계 불일치로 막힌다"는 걸 회귀 테스트로 고정해둔다(안전 문제라
# 임의로 우회하면 안 됨 - vision_adapter.boxes_from_vision_corners 참고).
_REAL_SAMPLE = pathlib.Path.home() / "Downloads" / "all_boxes_corners_20260721_174311_555644.json"


@pytest.mark.skipif(not _REAL_SAMPLE.exists(), reason="실제 샘플 파일이 이 환경에 없음")
def test_real_vision_sample_currently_blocked_by_frame_mismatch():
    data = json.loads(_REAL_SAMPLE.read_text())
    with pytest.raises(ValueError, match="depth_camera_optical_frame_from_message_header"):
        vision_adapter.boxes_from_vision_corners(data)
