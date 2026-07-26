"""
routes/vision.py
POST /api/parse-box-scan, POST /api/parse-vision-corners - 비전(준형)이 만드는
원본 JSON을 계획 계산에 바로 쓸 수 있는 단순 박스 목록으로 "형식만" 바꿔준다.
계산 자체는 그대로 기존 POST /api/plan을 쓴다 - 프론트가 여기서 받은 boxes를
그 요청의 boxes 필드에 넣고, snapshot_id를 box_snapshot_id 필드에 넣어서 다시
호출하는 2단계 흐름이다(실제 vision 연동 전에는 boxes 배열만 봐서는 "이게
수동 입력인지 vision에서 온 건지" 구분할 수 없어서, snapshot_id를 진짜 값으로
같이 넘겨야 HMI 8절 원칙 - Box Snapshot/Trunk Map ID 정합성 검증 - 이 의미를
가진다).
"""
from flask import Blueprint, jsonify, request

import vision_adapter
from routes.plan import ApiError

vision_bp = Blueprint("vision", __name__)


@vision_bp.post("/api/parse-box-scan")
def post_parse_box_scan():
    body = request.get_json(force=True, silent=True)
    if not isinstance(body, dict):
        raise ApiError(
            400, "BOX_SCAN_JSON_INVALID", "box_scan.json 본문이 올바른 JSON 객체가 아닙니다.",
            "박스 스캔 결과 파일이 올바른 JSON인지 확인한 뒤 다시 불러오세요.",
        )
    try:
        boxes, snapshot_id = vision_adapter.boxes_from_box_scan(body)
    except ValueError as e:
        raise ApiError(
            400, "VISION_FRAME_MISMATCH", str(e),
            "Vision(준형)/시스템통합(지완)과 좌표계를 맞춘 뒤 다시 시도하세요.",
        )
    except KeyError as e:
        raise ApiError(
            400, "BOX_SCAN_PARSE_FAILED", f"필수 필드가 없습니다: {e}",
            "box_scan.json 형식(HMI 설계 가이드라인 4.2절)에 맞는지 확인하세요.",
        )
    return jsonify({"boxes": boxes, "snapshot_id": snapshot_id})


@vision_bp.post("/api/parse-vision-corners")
def post_parse_vision_corners():
    body = request.get_json(force=True, silent=True)
    if not isinstance(body, dict):
        raise ApiError(
            400, "VISION_CORNERS_JSON_INVALID", "비전 결과 본문이 올바른 JSON 객체가 아닙니다.",
            "all_boxes_corners_*.json 파일이 올바른 JSON인지 확인한 뒤 다시 불러오세요.",
        )
    try:
        boxes, snapshot_id = vision_adapter.boxes_from_vision_corners(body)
    except ValueError as e:
        raise ApiError(
            400, "VISION_FRAME_MISMATCH", str(e),
            "Vision(준형)/시스템통합(지완)과 좌표계를 맞춘 뒤 다시 시도하세요.",
        )
    except KeyError as e:
        raise ApiError(
            400, "VISION_CORNERS_PARSE_FAILED", f"필수 필드가 없습니다: {e}",
            "all_boxes_corners_*.json 형식이 맞는지 확인하세요.",
        )
    return jsonify({"boxes": boxes, "snapshot_id": snapshot_id})
