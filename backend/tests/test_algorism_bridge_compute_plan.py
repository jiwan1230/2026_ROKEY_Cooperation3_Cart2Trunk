import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
import algorism_bridge as bridge

_TRUNK_MAP = {
    "run_id": "test_run",
    "vertices": [
        [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 1.0, 0.0],
        [0.0, 0.0, 0.5], [1.0, 0.0, 0.5], [0.0, 1.0, 0.5], [1.0, 1.0, 0.5],
    ],
    "edges": [{"v": [0, 1]}, {"v": [0, 2]}, {"v": [0, 4]}],
    "obstacles": [],
}


def test_compute_plan_places_boxes_and_returns_full_payload():
    boxes = [{"id": "A", "width": 0.3, "depth": 0.2, "height": 0.15}]

    result = bridge.compute_plan(_TRUNK_MAP, boxes, box_source_label="테스트")

    assert result["trunk"] == {"width": 1.0, "depth": 1.0, "height": 0.5}
    assert len(result["placed"]) == 1
    placed = result["placed"][0]
    assert placed["box_id"] == "A"
    # large_first + 기본 preference(전부 1.0)는 항상 "weighted" 공식을 쓴다 -
    # 09_rescan_replan이 mode != "count_first"일 때 이 공식만 쓰기 때문에 결정적이다.
    bd = placed["score_breakdown"]
    assert bd["formula"] == "weighted"
    reconstructed = bd["height_term"] - bd["contact_term"] - bd["wall_a_term"] - bd["wall_bc_term"]
    assert reconstructed == pytest.approx(placed["score"], abs=1e-6)
    assert result["summary"]["total"] == 1
    assert result["summary"]["placed"] == 1
    assert result["box_snapshot_id"] == "manual_input:테스트"
    assert any("PLACED A" in line for line in result["log_lines"])


def test_compute_plan_reports_unloadable_when_box_too_big():
    boxes = [{"id": "Huge", "width": 5.0, "depth": 5.0, "height": 5.0}]

    result = bridge.compute_plan(_TRUNK_MAP, boxes)

    assert result["placed"] == []
    assert len(result["unloadable"]) == 1
    assert result["unloadable"][0]["box_id"] == "Huge"
    assert result["unloadable"][0]["reason"] == "SIZE_EXCEEDS_TRUNK"


def test_compute_plan_score_breakdown_matches_actual_score_in_count_first_mode():
    """count_first 모드는 내부적으로 서로 다른 두 채점 공식(가중치 공식 vs
    밀도/개수우선 공식) 중 하나를 쓸 수 있다 - _reconstruct_score_breakdown이
    실제로 쓰인 공식을 못 맞히면(Task 4 리뷰에서 실제로 발견된 회귀), 여기서
    formula와 무관하게 "재구성한 합이 실제 score와 같아야 한다"는 불변조건이
    깨진다."""
    boxes = [
        {"id": "A", "width": 0.30, "depth": 0.20, "height": 0.15},
        {"id": "B", "width": 0.25, "depth": 0.20, "height": 0.15},
        {"id": "C", "width": 0.20, "depth": 0.15, "height": 0.10},
    ]

    result = bridge.compute_plan(_TRUNK_MAP, boxes, mode="count_first")

    assert len(result["placed"]) >= 1
    for p in result["placed"]:
        bd = p["score_breakdown"]
        if bd["formula"] == "count_first_density":
            reconstructed = bd["height_term"] + bd["footprint_growth_term"]
        else:
            reconstructed = bd["height_term"] - bd["contact_term"] - bd["wall_a_term"] - bd["wall_bc_term"]
        assert reconstructed == pytest.approx(p["score"], abs=1e-6)


def test_compute_plan_fixed_order_true_preserves_input_order():
    boxes = [
        {"id": "Large", "width": 0.50, "depth": 0.35, "height": 0.30},
        {"id": "Small", "width": 0.30, "depth": 0.20, "height": 0.15},
    ]

    result = bridge.compute_plan(_TRUNK_MAP, boxes, fixed_order=True)

    assert [p["box_id"] for p in result["placed"]] == ["Large", "Small"]
