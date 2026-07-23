"""
test_08_generate_loading_plan_mode.py
⑧ generate_loading_plan()의 mode("large_first"/"count_first")·margin
사용자 조절 기능 + 미적재 사유 로깅 검증.
"""
import logging
import sys, pathlib
from importlib import import_module

import pytest

_m02 = import_module("cart2trunk_planning.core.trunk_space_state")
_m03 = import_module("cart2trunk_planning.core.extreme_point_candidates")
_m08 = import_module("cart2trunk_planning.core.unloadable_reason")

Trunk = _m02.Trunk
Box = _m03.Box
generate_loading_plan = _m08.generate_loading_plan


class _ListHandler(logging.Handler):
    def __init__(self):
        super().__init__()
        self.records = []

    def emit(self, record):
        self.records.append(record)


def _capture_logs(level, logger_name, fn):
    logger = logging.getLogger(logger_name)
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


def test_default_mode_is_large_first_unchanged_behavior():
    trunk = Trunk(width=1.5, depth=1.5, height=0.9)
    boxes = [
        Box("Small", 0.30, 0.20, 0.15),
        Box("Medium", 0.40, 0.30, 0.25),
        Box("Large", 0.50, 0.35, 0.30),
    ]
    plans_default, unloadable_default = generate_loading_plan(boxes, trunk)
    plans_explicit, unloadable_explicit = generate_loading_plan(boxes, trunk, mode="large_first")

    assert [p.box_id for p in plans_default] == [p.box_id for p in plans_explicit]
    assert [p.position for p in plans_default] == [p.position for p in plans_explicit]


def test_count_first_mode_fits_more_in_a_tight_trunk():
    trunk = Trunk(width=0.6, depth=0.4, height=0.45)  # ⑮ 상단 여유(0.2m) 감안
    boxes = (
        [Box(f"Small{i}", 0.1, 0.1, 0.1) for i in range(6)]
        + [Box(f"Big{i}", 0.3, 0.2, 0.2) for i in range(2)]
    )
    plans_large_first, _ = generate_loading_plan(boxes, trunk, mode="large_first")
    plans_count_first, _ = generate_loading_plan(boxes, trunk, mode="count_first")

    assert len(plans_count_first) > len(plans_large_first)


def test_margin_override_is_respected():
    custom_margin = 0.05
    trunk = Trunk(width=0.3 + 2 * custom_margin, depth=0.3 + 2 * custom_margin, height=0.4)
    box = Box("A", 0.3, 0.3, 0.15)

    plans, unloadable = generate_loading_plan([box], trunk, margin=custom_margin)

    assert len(unloadable) == 0
    assert plans[0].position[0] == pytest.approx(custom_margin)


def test_unloadable_reason_is_logged():
    trunk = Trunk(1.5, 1.5, 0.28)  # 높이가 너무 낮아서 큰 박스는 SIZE_EXCEEDS_TRUNK
    from importlib import import_module as im
    LARGE = im("cart2trunk_planning.core.object3d_schema").TEST_OBJECT3D["LARGE"]
    box = im("cart2trunk_planning.core.object3d_schema").object3d_to_box(LARGE)

    _, messages = _capture_logs(
        logging.INFO, "cart2trunk_planning.core.unloadable_reason", lambda: generate_loading_plan([box], trunk)
    )

    assert box.id in messages
    assert "SIZE_EXCEEDS_TRUNK" in messages
