"""
routes/resources.py
GET /api/trunk-maps, GET /api/box-presets - 선택 가능한 리소스 목록 조회.
"""
from flask import Blueprint, jsonify

import algorism_bridge as bridge

resources_bp = Blueprint("resources", __name__)


@resources_bp.get("/api/trunk-maps")
def get_trunk_maps():
    return jsonify({"trunk_maps": bridge.list_trunk_maps()})


@resources_bp.get("/api/box-presets")
def get_box_presets():
    return jsonify({"presets": bridge.list_box_presets()})
