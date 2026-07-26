import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
from app import create_app
import routes.robot as robot_module


def _client():
    return create_app().test_client()


def test_cart_scan_returns_dummy_success(monkeypatch):
    monkeypatch.setattr(robot_module, "DUMMY_DELAY_SECONDS", 0)
    resp = _client().post("/api/robot/cart-scan")
    assert resp.status_code == 200
    body = resp.get_json()
    assert body["status"] == "ok"
    assert body["dummy"] is True
    assert "카트 스캔" in body["message"]


def test_trunk_scan_returns_dummy_success(monkeypatch):
    monkeypatch.setattr(robot_module, "DUMMY_DELAY_SECONDS", 0)
    resp = _client().post("/api/robot/trunk-scan")
    assert resp.status_code == 200
    body = resp.get_json()
    assert body["status"] == "ok"
    assert "트렁크 스캔" in body["message"]


def test_pick_and_place_returns_dummy_success(monkeypatch):
    monkeypatch.setattr(robot_module, "DUMMY_DELAY_SECONDS", 0)
    resp = _client().post("/api/robot/pick-and-place")
    assert resp.status_code == 200
    body = resp.get_json()
    assert body["status"] == "ok"
    assert "픽앤플레이스" in body["message"]


def test_get_method_not_allowed():
    # 실수로 GET으로 호출하는 걸 방지하는 회귀 테스트 - 반드시 POST여야 한다.
    resp = _client().get("/api/robot/cart-scan")
    assert resp.status_code == 405
