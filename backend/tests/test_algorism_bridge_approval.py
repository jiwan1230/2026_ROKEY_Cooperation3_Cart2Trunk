import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
import algorism_bridge as bridge

_PLACED = [{
    "box_id": "A", "order": 1, "position": [0.1, 0.2, 0.0], "dimensions": [0.3, 0.2, 0.15],
    "score": 0.5, "touches": 2, "rotated": False, "target_yaw": 0.0,
}]


def test_build_approved_task_marks_approved_true():
    task = bridge.build_approved_task("plan_1", "snap_1", "trunk_1", {"mode": "large_first"}, _PLACED)
    assert task["approved"] is True
    assert task["tasks"][0]["box_id"] == "A"
    assert task["tasks"][0]["sequence"] == 1


def test_send_task_writes_file(tmp_path, monkeypatch):
    monkeypatch.setattr(bridge, "_PENDING_TASKS_DIR", tmp_path)
    task = bridge.build_approved_task("plan_2", "snap_1", "trunk_1", {}, _PLACED)

    out_path = bridge.send_task(task)

    assert pathlib.Path(out_path).exists()
    assert pathlib.Path(out_path).name == "plan_2.json"


def test_send_task_rejects_unapproved():
    task = bridge.build_approved_task("plan_3", "snap_1", "trunk_1", {}, _PLACED)
    task["approved"] = False
    try:
        bridge.send_task(task)
        assert False, "예외가 발생해야 함"
    except ValueError:
        pass
