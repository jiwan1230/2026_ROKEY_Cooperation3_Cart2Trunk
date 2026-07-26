import pathlib
import sys

import requests

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
from app import create_app
import routes.robot as robot_module


def _client():
    return create_app().test_client()


class _FakeResponse:
    def __init__(self, status_code, body):
        self.status_code = status_code
        self._body = body

    def json(self):
        return self._body


def test_cart_scan_relays_bridge_success(monkeypatch):
    bridge_body = {"status": "ok", "snapshot_id": "s1", "box_count": 3, "message": "3개 박스 검출"}
    monkeypatch.setattr(
        requests, "post", lambda url, timeout: _FakeResponse(200, bridge_body))
    resp = _client().post("/api/robot/cart-scan")
    assert resp.status_code == 200
    assert resp.get_json() == bridge_body


def test_trunk_scan_relays_bridge_error_status_and_body(monkeypatch):
    bridge_body = {"status": "error", "error_code": "DEPTH_TIMEOUT", "message": "미구현"}
    monkeypatch.setattr(
        requests, "post", lambda url, timeout: _FakeResponse(422, bridge_body))
    resp = _client().post("/api/robot/trunk-scan")
    assert resp.status_code == 422
    assert resp.get_json() == bridge_body


def test_pick_and_place_relays_bridge_success(monkeypatch):
    bridge_body = {"status": "ok", "placed_count": 3, "remaining_count": 0, "message": "완료"}
    calls = []

    def fake_post(url, timeout):
        calls.append((url, timeout))
        return _FakeResponse(200, bridge_body)

    monkeypatch.setattr(requests, "post", fake_post)
    resp = _client().post("/api/robot/pick-and-place")
    assert resp.status_code == 200
    assert resp.get_json() == bridge_body
    # pick-and-place는 RunLoadingMission 전체를 기다리므로 스캔류보다 훨씬
    # 긴 타임아웃을 써야 한다(robot_module._PICK_AND_PLACE_TIMEOUT_SECONDS).
    assert calls[0][1] == robot_module._PICK_AND_PLACE_TIMEOUT_SECONDS


def test_bridge_unreachable_returns_clear_error(monkeypatch):
    def raise_connection_error(url, timeout):
        raise requests.exceptions.ConnectionError()

    monkeypatch.setattr(requests, "post", raise_connection_error)
    resp = _client().post("/api/robot/cart-scan")
    assert resp.status_code == 503
    body = resp.get_json()
    assert body["error_code"] == "HMI_BRIDGE_UNAVAILABLE"


def test_bridge_timeout_returns_clear_error(monkeypatch):
    def raise_timeout(url, timeout):
        raise requests.exceptions.Timeout()

    monkeypatch.setattr(requests, "post", raise_timeout)
    resp = _client().post("/api/robot/trunk-scan")
    assert resp.status_code == 504
    body = resp.get_json()
    assert body["error_code"] == "HMI_BRIDGE_TIMEOUT"


def test_get_method_not_allowed():
    # 실수로 GET으로 호출하는 걸 방지하는 회귀 테스트 - 반드시 POST여야 한다.
    resp = _client().get("/api/robot/cart-scan")
    assert resp.status_code == 405
