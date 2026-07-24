import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
from app import create_app
import algorism_bridge as bridge

_TRUNK_MAP = {
    "run_id": "test_run",
    "vertices": [
        [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 1.0, 0.0],
        [0.0, 0.0, 0.5], [1.0, 0.0, 0.5], [0.0, 1.0, 0.5], [1.0, 1.0, 0.5],
    ],
    "edges": [{"v": [0, 1]}],
    "obstacles": [],
}


def _client(monkeypatch, tmp_path):
    run_dir = tmp_path / "run_test" / "pointcloud"
    run_dir.mkdir(parents=True)
    (run_dir / "trunk_map.json").write_text(json.dumps(_TRUNK_MAP))
    monkeypatch.setattr(bridge, "_SRC_DIR", tmp_path)
    return create_app().test_client()


def test_post_plan_happy_path(monkeypatch, tmp_path):
    client = _client(monkeypatch, tmp_path)
    resp = client.post("/api/plan", json={
        "trunk_map": "run_test",
        "boxes": [{"id": "A", "width": 0.3, "depth": 0.2, "height": 0.15}],
    })
    assert resp.status_code == 200
    assert len(resp.get_json()["placed"]) == 1


def test_post_plan_missing_trunk_map_returns_400(monkeypatch, tmp_path):
    client = _client(monkeypatch, tmp_path)
    resp = client.post("/api/plan", json={"boxes": []})
    assert resp.status_code == 400
    assert resp.get_json()["error_code"] == "TRUNK_MAP_NOT_SELECTED"


def test_post_plan_invalid_boxes_returns_400(monkeypatch, tmp_path):
    client = _client(monkeypatch, tmp_path)
    resp = client.post("/api/plan", json={"trunk_map": "run_test", "boxes": "not-a-list"})
    assert resp.status_code == 400
    assert resp.get_json()["error_code"] == "BOX_JSON_INVALID"


def test_post_plan_unknown_trunk_map_returns_404(monkeypatch, tmp_path):
    client = _client(monkeypatch, tmp_path)
    resp = client.post("/api/plan", json={"trunk_map": "run_missing", "boxes": []})
    assert resp.status_code == 404
    assert resp.get_json()["error_code"] == "TRUNK_MAP_NOT_FOUND"


def test_post_plan_corrupt_trunk_map_json_returns_400(monkeypatch, tmp_path):
    # trunk_map.json 파일은 존재하지만 내용이 손상된 JSON인 경우:
    # "파일 없음"(404)이 아니라 "파일은 있지만 파싱 실패"(400)로 분류되어야 한다.
    run_dir = tmp_path / "run_corrupt" / "pointcloud"
    run_dir.mkdir(parents=True)
    (run_dir / "trunk_map.json").write_text("{ 이거 잘못된 json")
    monkeypatch.setattr(bridge, "_SRC_DIR", tmp_path)
    client = create_app().test_client()

    resp = client.post("/api/plan", json={"trunk_map": "run_corrupt", "boxes": []})
    assert resp.status_code == 400
    assert resp.get_json()["error_code"] == "TRUNK_MAP_JSON_INVALID"
