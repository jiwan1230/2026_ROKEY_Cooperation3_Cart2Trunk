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


def test_color_for_box_id_returns_valid_hex_color():
    color = bridge.color_for_box_id("Box1")
    assert len(color) == 7 and color.startswith("#")
    int(color[1:], 16)  # 유효한 16진수여야 함 (예외 안 나면 통과)


def test_color_for_box_id_spreads_many_ids_across_distinct_hues():
    # 팔레트가 7개짜리로 고정돼 있던 예전 버전은 박스가 7개를 넘으면 색이
    # 반드시 겹쳤다 - 지금은 hue를 0~360 전체에서 뽑으므로, 실전에서 흔한
    # 규모(예: 20개)의 박스 id에서는 대부분 서로 다른 색이 나와야 한다.
    ids = [f"Box{i}" for i in range(20)]
    colors = {bridge.color_for_box_id(i) for i in ids}
    assert len(colors) >= 15  # 완전한 유일성은 보장 안 하지만(hue 충돌 가능) 대부분은 달라야 함


def test_generate_random_boxes_count_and_ranges():
    boxes = bridge.generate_random_boxes(5)
    assert len(boxes) == 5
    for b in boxes:
        assert 0.15 <= b["width"] <= 0.45
