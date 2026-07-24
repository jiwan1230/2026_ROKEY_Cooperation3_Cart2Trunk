import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
from app import create_app
import algorism_bridge as bridge

_PLACED = [{
    "box_id": "A", "order": 1, "position": [0.1, 0.2, 0.0], "dimensions": [0.3, 0.2, 0.15],
    "score": 0.5, "touches": 2, "rotated": False, "target_yaw": 0.0,
}]


def test_approve_then_send_happy_path(monkeypatch, tmp_path):
    monkeypatch.setattr(bridge, "_PENDING_TASKS_DIR", tmp_path)
    client = create_app().test_client()

    approve_resp = client.post("/api/approve", json={
        "box_snapshot_id": "snap_1", "trunk_map_id": "trunk_1",
        "parameters": {"mode": "large_first"}, "placed": _PLACED,
    })
    assert approve_resp.status_code == 200
    task = approve_resp.get_json()["task"]
    assert task["approved"] is True

    send_resp = client.post("/api/send", json={"task": task})
    assert send_resp.status_code == 200
    assert pathlib.Path(send_resp.get_json()["out_path"]).exists()


def test_approve_without_placed_returns_400():
    client = create_app().test_client()
    resp = client.post("/api/approve", json={})
    assert resp.status_code == 400
    assert resp.get_json()["error_code"] == "NO_PLAN_TO_APPROVE"


def test_send_without_task_returns_400():
    client = create_app().test_client()
    resp = client.post("/api/send", json={})
    assert resp.status_code == 400
    assert resp.get_json()["error_code"] == "NO_TASK_TO_SEND"
