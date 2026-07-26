import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
from app import create_app


def _client():
    return create_app().test_client()


def test_delivery_truck_places_later_stops_farther_from_entrance():
    # LIFO 정책 - 나중 배송지(delivery_stop 숫자가 큰) 박스가 입구에서 더
    # 먼(x가 큰) 자리에 있어야 한다(실측: 1/2번은 x=0.56, 3/4번은 x=0.88).
    resp = _client().post("/api/scenarios/delivery_truck/plan")
    assert resp.status_code == 200
    body = resp.get_json()
    assert body["summary"]["total"] == 4
    by_id = {p["box_id"]: p for p in body["placed"]}
    assert by_id["정류장4_박스"]["position"][0] > by_id["정류장1_박스"]["position"][0]


def test_warehouse_places_6_of_8_boxes():
    # 실측: 6개(소형6+대형2 중 소형만 다 들어가고 대형 일부는 못 들어감)
    # 데모 트렁크(0.6x0.4x0.45)가 좁아서 8개 전부는 안 들어간다.
    resp = _client().post("/api/scenarios/warehouse/plan")
    assert resp.status_code == 200
    body = resp.get_json()
    assert body["summary"] == {"total": 8, "placed": 6, "unplaced": 2}


def test_cold_chain_places_all_3_boxes():
    resp = _client().post("/api/scenarios/cold_chain/plan")
    assert resp.status_code == 200
    body = resp.get_json()
    assert body["summary"] == {"total": 3, "placed": 3, "unplaced": 0}


def test_hazmat_places_oxidizer_and_flammable_apart():
    resp = _client().post("/api/scenarios/hazmat/plan")
    assert resp.status_code == 200
    body = resp.get_json()
    ids = {p["box_id"] for p in body["placed"]}
    assert {"산화제_드럼1", "인화물_드럼1", "일반박스1"} == ids


def test_unknown_scenario_returns_404():
    resp = _client().post("/api/scenarios/nonexistent/plan")
    assert resp.status_code == 404
    assert resp.get_json()["error_code"] == "SCENARIO_NOT_FOUND"
