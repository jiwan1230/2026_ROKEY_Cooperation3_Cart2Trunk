import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
from app import create_app
import algorism_bridge as bridge


def test_get_trunk_maps_returns_list(monkeypatch):
    monkeypatch.setattr(bridge, "list_trunk_maps", lambda: ["run_x"])
    client = create_app().test_client()

    resp = client.get("/api/trunk-maps")

    assert resp.status_code == 200
    assert resp.get_json() == {"trunk_maps": ["run_x"]}


def test_get_box_presets_returns_dict(monkeypatch):
    monkeypatch.setattr(bridge, "list_box_presets", lambda: {"foo": []})
    client = create_app().test_client()

    resp = client.get("/api/box-presets")

    assert resp.status_code == 200
    assert resp.get_json() == {"presets": {"foo": []}}
