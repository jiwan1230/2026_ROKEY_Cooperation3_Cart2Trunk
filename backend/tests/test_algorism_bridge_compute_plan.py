import pathlib
import sys

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
    assert result["placed"][0]["box_id"] == "A"
    assert set(result["placed"][0]["score_breakdown"].keys()) == {
        "height_term", "contact_term", "wall_a_term", "wall_bc_term"}
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


def test_compute_plan_fixed_order_true_preserves_input_order():
    boxes = [
        {"id": "Large", "width": 0.50, "depth": 0.35, "height": 0.30},
        {"id": "Small", "width": 0.30, "depth": 0.20, "height": 0.15},
    ]

    result = bridge.compute_plan(_TRUNK_MAP, boxes, fixed_order=True)

    assert [p["box_id"] for p in result["placed"]] == ["Large", "Small"]
