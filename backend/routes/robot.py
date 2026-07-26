"""
routes/robot.py
POST /api/robot/cart-scan, /trunk-scan, /pick-and-place - 로봇(MSI2 - 신지완/
민결) 동작 트리거. ROS2 노드 구조가 아직 설계 중이라, 지금은 실제 서비스/
액션 호출 없이 DUMMY_DELAY_SECONDS만큼 대기한 뒤 항상 성공 응답을 돌려주는
더미다. 실제 연동 시 _dummy_trigger() 안의 TODO(MSI2) 자리에 실제 ROS2 호출을
넣고, 그 결과에 따라 상태/메시지를 채우도록 바꾸면 된다.
"""
import time

from flask import Blueprint, jsonify

robot_bp = Blueprint("robot", __name__)

# 실제 스캔/동작 시간을 흉내내는 더미 지연(초). 테스트에서는 이 값을
# monkeypatch로 0으로 낮춰서 느려지지 않게 한다.
DUMMY_DELAY_SECONDS = 1.5


def _dummy_trigger(step_name: str):
    time.sleep(DUMMY_DELAY_SECONDS)
    # TODO(MSI2): 여기에 실제 ROS2 서비스/액션 호출을 넣고, 그 결과에 따라
    # status/message를 채운다. 지금은 항상 성공하는 더미.
    return jsonify({
        "status": "ok",
        "dummy": True,
        "message": f"{step_name} 완료 (더미 - 실제 로봇 미연동)",
    })


@robot_bp.post("/api/robot/cart-scan")
def cart_scan():
    return _dummy_trigger("카트 스캔")


@robot_bp.post("/api/robot/trunk-scan")
def trunk_scan():
    return _dummy_trigger("트렁크 스캔")


@robot_bp.post("/api/robot/pick-and-place")
def pick_and_place():
    return _dummy_trigger("픽앤플레이스")
