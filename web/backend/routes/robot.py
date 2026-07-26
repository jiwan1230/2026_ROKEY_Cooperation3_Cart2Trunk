"""
routes/robot.py
POST /api/robot/cart-scan, /trunk-scan, /pick-and-place - 로봇(MSI2) 동작 트리거.

cart2trunk_hmi/ros_bridge_node.py(시스템 python3.10 rclpy 프로세스, 이 Flask
venv와는 별개)로 HTTP POST해서 실제 ROS2 액션(ScanBoxes/ScanTrunk/
RunLoadingMission)을 호출한다 - 이 파일이 rclpy를 직접 import하지 않는 이유는
algorism_bridge.py 최상단 docstring과 동일(venv 분리 유지).

브릿지가 아직 안 떠 있으면(개발 중 흔한 상태) 명확한 에러로 안내한다 - 예전
더미처럼 조용히 성공한 척하지 않는다.
"""
import os

import requests
from flask import Blueprint, jsonify

from routes.plan import ApiError

robot_bp = Blueprint("robot", __name__)

_BRIDGE_URL = os.environ.get("CART2TRUNK_HMI_BRIDGE_URL", "http://localhost:5050")

# pick-and-place는 ros_bridge_node.py 안에서 RunLoadingMission 전체(카트가 빌
# 때까지 반복)를 끝까지 기다리므로, 그보다 넉넉한 타임아웃이 필요하다.
_SCAN_TIMEOUT_SECONDS = 65
_PICK_AND_PLACE_TIMEOUT_SECONDS = 605


def _post_to_bridge(path: str, step_name: str, timeout_seconds: int):
    try:
        resp = requests.post(f"{_BRIDGE_URL}{path}", timeout=timeout_seconds)
    except requests.exceptions.ConnectionError:
        raise ApiError(
            503, "HMI_BRIDGE_UNAVAILABLE",
            f"cart2trunk_hmi ROS2 브릿지({_BRIDGE_URL})에 연결할 수 없습니다.",
            "MSI2에서 `ros2 run cart2trunk_hmi ros_bridge_node`가 떠 있는지 확인하세요.",
        )
    except requests.exceptions.Timeout:
        raise ApiError(
            504, "HMI_BRIDGE_TIMEOUT", f"{step_name} 응답이 {timeout_seconds}초 안에 오지 않았습니다.",
            "로봇/Isaac Sim이 응답 없는 상태인지 MSI2 쪽 로그를 확인하세요.",
        )

    body = resp.json()
    return jsonify(body), resp.status_code


@robot_bp.post("/api/robot/cart-scan")
def cart_scan():
    return _post_to_bridge("/trigger/cart-scan", "카트 스캔", _SCAN_TIMEOUT_SECONDS)


@robot_bp.post("/api/robot/trunk-scan")
def trunk_scan():
    return _post_to_bridge("/trigger/trunk-scan", "트렁크 스캔", _SCAN_TIMEOUT_SECONDS)


@robot_bp.post("/api/robot/pick-and-place")
def pick_and_place():
    return _post_to_bridge(
        "/trigger/pick-and-place", "픽앤플레이스", _PICK_AND_PLACE_TIMEOUT_SECONDS)
