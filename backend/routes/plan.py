"""
routes/plan.py
POST /api/plan - 파라미터 + 박스 목록을 받아 배치 계획을 계산한다 (핵심 엔드포인트).
"""
import json

from flask import Blueprint, jsonify, request

import algorism_bridge as bridge

plan_bp = Blueprint("plan", __name__)


class ApiError(Exception):
    def __init__(self, status_code: int, error_code: str, cause: str, action: str):
        super().__init__(cause)
        self.status_code = status_code
        self.error_code = error_code
        self.cause = cause
        self.action = action

    def to_response(self):
        return jsonify({
            "error_code": self.error_code, "cause": self.cause, "action": self.action,
        }), self.status_code


@plan_bp.post("/api/plan")
def post_plan():
    body = request.get_json(force=True, silent=True)
    if body is None:
        raise ApiError(
            400, "REQUEST_JSON_INVALID", "요청 본문이 올바른 JSON이 아닙니다.",
            "Content-Type: application/json으로 보냈는지, 본문이 올바른 JSON인지 확인하세요.",
        )

    trunk_map_name = body.get("trunk_map")
    if not trunk_map_name:
        raise ApiError(
            400, "TRUNK_MAP_NOT_SELECTED", "trunk_map 필드가 비어 있습니다.",
            "GET /api/trunk-maps 목록 중 하나를 선택해서 보내세요.",
        )

    boxes_raw = body.get("boxes")
    if not isinstance(boxes_raw, list):
        raise ApiError(
            400, "BOX_JSON_INVALID", "boxes 필드가 배열이 아닙니다.",
            "박스 목록 편집기의 JSON 문법(쉼표, 중괄호, 따옴표)을 확인한 뒤 다시 계산하세요.",
        )

    try:
        trunk_map_path = bridge._trunk_map_path(trunk_map_name)
        trunk_map_data = json.loads(trunk_map_path.read_text())
    except ValueError as e:
        raise ApiError(404, "TRUNK_MAP_NOT_FOUND", str(e),
                        "트렁크 스캔 파일 목록을 새로고침(GET /api/trunk-maps)한 뒤 다시 선택하세요.")

    try:
        result = bridge.compute_plan(
            trunk_map_data, boxes_raw,
            box_source_label=body.get("box_source_label", "custom"),
            mode=body.get("mode", "large_first"),
            margin=body.get("margin"), allow_stacking=body.get("allow_stacking", False),
            allow_rotation=body.get("allow_rotation", True),
            wall_margin=body.get("wall_margin"), obstacle_margin=body.get("obstacle_margin"),
            ceiling_margin=body.get("ceiling_margin"), entrance_margin=body.get("entrance_margin"),
            entrance_preference=body.get("entrance_preference", 1.0),
            contact_preference=body.get("contact_preference", 1.0),
            height_preference=body.get("height_preference", 1.0),
            fixed_order=body.get("fixed_order", False),
        )
    except (KeyError, TypeError) as e:
        raise ApiError(
            400, "BOX_JSON_INVALID", f"박스 목록 필드가 올바르지 않습니다: {e}",
            "각 박스가 id/width/depth/height 필드를 모두 갖고 있는지 확인하세요.",
        )

    return jsonify(result)
