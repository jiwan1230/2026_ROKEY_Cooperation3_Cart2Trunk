# isaacpjt/Cart2Trunk/web/backend/tests/test_algorism_bridge_resources.py
import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
import algorism_bridge as bridge


def test_list_trunk_maps_finds_run_folders(tmp_path, monkeypatch):
    monkeypatch.setattr(bridge, "_SRC_DIR", tmp_path)
    run_dir = tmp_path / "run_20260101_000000" / "pointcloud"
    run_dir.mkdir(parents=True)
    (run_dir / "trunk_map.json").write_text("{}")

    assert bridge.list_trunk_maps() == ["run_20260101_000000"]


def test_list_trunk_maps_empty_when_none_found(tmp_path, monkeypatch):
    monkeypatch.setattr(bridge, "_SRC_DIR", tmp_path)
    assert bridge.list_trunk_maps() == []


def test_list_box_presets_includes_default_and_example_files(tmp_path, monkeypatch):
    monkeypatch.setattr(bridge, "_LOCAL_TEST_DATA_DIR", tmp_path)
    custom = [{"id": "A", "width": 0.2, "depth": 0.2, "height": 0.2}]
    (tmp_path / "example_cart_boxes_stress.json").write_text(json.dumps(custom))

    presets = bridge.list_box_presets()

    assert presets["기본값 (Large/Medium/Small)"] == bridge._DEFAULT_CART_BOXES
    assert presets["stress"] == custom


def test_color_for_box_id_is_stable():
    assert bridge.color_for_box_id("Large") == bridge.color_for_box_id("Large")


def test_generate_random_boxes_count_and_ranges():
    boxes = bridge.generate_random_boxes(5)
    assert len(boxes) == 5
    for b in boxes:
        assert 0.15 <= b["width"] <= 0.45
