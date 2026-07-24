"""
routes/approval.py
POST /api/approve, POST /api/send - 계획 승인 및 MSI2 전송(현재는 로컬 저장까지만).
"""
from datetime import datetime

from flask import Blueprint, jsonify, request

import algorism_bridge as bridge
from routes.plan import ApiError

approval_bp = Blueprint("approval", __name__)


@approval_bp.post("/api/approve")
def post_approve():
    body = request.get_json(force=True, silent=True) or {}
    placed = body.get("placed")
    if not isinstance(placed, list) or not placed:
        raise ApiError(
            400, "NO_PLAN_TO_APPROVE", "승인할 배치 계획(placed)이 없습니다.",
            "먼저 POST /api/plan으로 계획을 계산한 뒤 그 결과의 placed를 그대로 보내세요.",
        )

    plan_id = f"load_plan_{datetime.now():%Y%m%d_%H%M%S}"
    task = bridge.build_approved_task(
        plan_id=plan_id,
        box_snapshot_id=body.get("box_snapshot_id", "unknown"),
        trunk_map_id=body.get("trunk_map_id", "unknown"),
        parameters=body.get("parameters", {}),
        placed=placed,
    )
    return jsonify({"plan_id": plan_id, "task": task})


@approval_bp.post("/api/send")
def post_send():
    body = request.get_json(force=True, silent=True) or {}
    task = body.get("task")
    if not isinstance(task, dict):
        raise ApiError(
            400, "NO_TASK_TO_SEND", "전송할 task가 없습니다.",
            "먼저 POST /api/approve로 승인한 뒤 그 응답의 task를 그대로 보내세요.",
        )
    try:
        out_path = bridge.send_task(task)
    except ValueError as e:
        raise ApiError(
            400, "TASK_NOT_APPROVED", str(e),
            "POST /api/approve를 먼저 호출해서 approved=True인 task를 받은 뒤 다시 시도하세요.",
        )
    return jsonify({"out_path": out_path})
