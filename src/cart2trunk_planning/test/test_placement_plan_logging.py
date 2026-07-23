"""
test_07_placement_plan_logging.py
⑦ place_one_box()가 판단 흐름(시도/배치 완료/회전 재시도)을 표준 logging
모듈로 남기는지 검증. 로봇과 결합했을 때 터미널에서 "왜 이 자리에 놨는지"를
바로 볼 수 있게 하기 위함 - print가 아니라 logging을 쓰는 이유는 로그 레벨로
켜고 끌 수 있고, 나중에 ROS2 등 다른 로거로 연결하기도 쉬워서다.

pytest의 caplog 픽스처가 이 환경(ROS2 launch_testing 플러그인)에서 동작하지
않아서(직접 확인함 - 트리비얼한 caplog 테스트조차 빈 결과), 수동으로 로깅
핸들러를 붙여서 레코드를 모으는 방식으로 검증한다.
"""
import logging
import sys, pathlib
from importlib import import_module

_m02 = import_module("cart2trunk_planning.core.trunk_space_state")
_m03 = import_module("cart2trunk_planning.core.extreme_point_candidates")
_m07 = import_module("cart2trunk_planning.core.placement_plan")

Trunk = _m02.Trunk
Box = _m03.Box
ExtremePointState = _m03.ExtremePointState
place_one_box = _m07.place_one_box


class _ListHandler(logging.Handler):
    def __init__(self):
        super().__init__()
        self.records = []

    def emit(self, record):
        self.records.append(record)


def _capture_logs(level, fn):
    """fn()을 실행하는 동안 07_placement_plan 로거의 레코드를 모아서 반환."""
    logger = logging.getLogger("cart2trunk_planning.core.placement_plan")
    handler = _ListHandler()
    handler.setLevel(level)
    old_level = logger.level
    logger.addHandler(handler)
    logger.setLevel(level)
    try:
        result = fn()
    finally:
        logger.removeHandler(handler)
        logger.setLevel(old_level)
    return result, " ".join(r.getMessage() for r in handler.records)


def test_successful_placement_logs_box_id_and_position():
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    box = Box("LogTestBox", 0.3, 0.3, 0.2)

    plan, messages = _capture_logs(
        logging.INFO, lambda: place_one_box(box, trunk, ExtremePointState(), order=1)
    )

    assert plan is not None
    assert "LogTestBox" in messages
    assert f"{plan.position[0]:.2f}" in messages


def test_rotation_retry_is_logged_at_debug_level():
    """정자세가 실패해서 회전 재시도가 실제로 일어나면, 그 사실이 DEBUG 로그로 남아야 한다."""
    trunk = Trunk(width=0.3 + 2 * 0.02, depth=0.5 + 2 * 0.02, height=0.4)
    # 폭 0.5인 박스는 정자세로는 안 들어가지만(트렁크 폭 0.34) 회전하면 들어감
    box = Box("RotateMe", 0.5, 0.3, 0.2)

    plan, messages = _capture_logs(
        logging.DEBUG, lambda: place_one_box(box, trunk, ExtremePointState(), order=1)
    )

    assert plan is not None
    assert plan.rotated is True
    assert "회전" in messages
