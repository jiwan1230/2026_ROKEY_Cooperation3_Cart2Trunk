import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
from app import create_app

_BOX_SCAN_JSON = {
    "snapshot_id": "box_scan_001",
    "frame_id": "m0609_base_link",
    "quality_score": 0.94,
    "boxes": [
        {"box_id": "BOX_01", "center": [0.1, 0.2, 0.075], "size": [0.3, 0.2, 0.15],
         "yaw": 0.0, "corners": [[0.0, 0.0, 0.0]], "confidence": 0.95},
    ],
}


def _client():
    return create_app().test_client()


def test_post_parse_box_scan_happy_path():
    client = _client()
    resp = client.post("/api/parse-box-scan", json=_BOX_SCAN_JSON)
    assert resp.status_code == 200
    body = resp.get_json()
    assert body["snapshot_id"] == "box_scan_001"
    assert body["boxes"] == [
        {"id": "BOX_01", "width": 0.3, "depth": 0.2, "height": 0.15, "rests_on_id": None, "initial_yaw": 0.0},
    ]


def test_post_parse_box_scan_wrong_frame_returns_400():
    client = _client()
    resp = client.post("/api/parse-box-scan", json={**_BOX_SCAN_JSON, "frame_id": "camera_frame"})
    assert resp.status_code == 400
    assert resp.get_json()["error_code"] == "VISION_FRAME_MISMATCH"


def test_post_parse_vision_corners_wrong_frame_returns_400():
    client = _client()
    data = {
        "coordinate_frame": "depth_camera_optical_frame_from_message_header",
        "boxes": [],
    }
    resp = client.post("/api/parse-vision-corners", json=data)
    assert resp.status_code == 400
    assert resp.get_json()["error_code"] == "VISION_FRAME_MISMATCH"


def test_post_parse_vision_corners_happy_path():
    client = _client()
    data = {
        "coordinate_frame": "m0609_base_link",
        "boxes": [
            {
                "box_id": 0, "support_type": "floor", "top_candidate_id": 0, "support_candidate_id": -1,
                "corners_m": [
                    [0, 0, 0.2], [0.5, 0, 0.2], [0.5, 0.35, 0.2], [0, 0.35, 0.2],
                    [0, 0, 0], [0.5, 0, 0], [0.5, 0.35, 0], [0, 0.35, 0],
                ],
            },
        ],
    }
    resp = client.post("/api/parse-vision-corners", json=data)
    assert resp.status_code == 200
    body = resp.get_json()
    assert body["boxes"] == [
        {"id": "0", "width": 0.5, "depth": 0.35, "height": 0.2, "rests_on_id": None},
    ]
