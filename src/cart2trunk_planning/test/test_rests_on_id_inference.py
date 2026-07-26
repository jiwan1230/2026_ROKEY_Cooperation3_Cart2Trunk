"""loading_planner_action_server의 Box3D -> core.Box 변환 시 rests_on_id 기하학적
추론이 실제로 연결되는지 검증.

배경: core/loading_order_decision.py::decide_loading_order()는 이미 rests_on_id를
써서 "위에 뭔가 얹힌 박스는 그게 치워지기 전엔 픽업 순서에서 제외"하도록 구현돼
있었지만, box3d_to_core_box()가 rests_on_id를 항상 None으로 넘겨서 ROS 파이프라인
에서는 이 제약이 죽어있었다 - EDU 저장소 algorism/01_object3d_schema.py의
_infer_rests_on_ids()(코너 좌표 기반 순수 기하 추론, 비전이 주는 support_candidate_id
는 트라이얼마다 재사용되는 임시 id라 안 믿음)를 연결해서 고쳤다.
"""
from geometry_msgs.msg import Point

from cart2trunk_interfaces.msg import Box3D

from cart2trunk_planning.core.object3d_schema import _infer_rests_on_ids
from cart2trunk_planning.loading_planner_action_server import _box3d_aabb, box3d_to_core_box


def _make_box3d(box_id, x0, y0, z0, w, d, h):
    """(x0,y0,z0)를 최소 코너로 하는 회전 없는 박스 하나의 Box3D 메시지."""
    box = Box3D()
    box.box_id = box_id
    box.size.x, box.size.y, box.size.z = w, d, h
    local_corners = [
        (0.0, 0.0, h), (w, 0.0, h), (w, d, h), (0.0, d, h),
        (0.0, 0.0, 0.0), (w, 0.0, 0.0), (w, d, 0.0), (0.0, d, 0.0),
    ]
    box.corners = [Point(x=x0 + cx, y=y0 + cy, z=z0 + cz) for cx, cy, cz in local_corners]
    box.detected_pose.position.x = x0 + w / 2.0
    box.detected_pose.position.y = y0 + d / 2.0
    box.detected_pose.position.z = z0 + h / 2.0
    box.detected_pose.orientation.w = 1.0
    return box


def test_infers_rests_on_id_for_stacked_boxes():
    base = _make_box3d('base', 0.0, 0.0, 0.0, 0.30, 0.30, 0.12)
    child = _make_box3d('child', 0.05, 0.05, 0.12, 0.15, 0.15, 0.10)  # base 윗면에 정확히 얹힘
    independent = _make_box3d('indep', 1.0, 1.0, 0.0, 0.20, 0.20, 0.10)  # 바닥, 멀리 떨어짐

    boxes_msg = [base, child, independent]
    aabbs = {b.box_id: _box3d_aabb(b) for b in boxes_msg}
    rests_on = _infer_rests_on_ids(aabbs)

    assert rests_on['child'] == 'base'
    assert rests_on['base'] is None
    assert rests_on['indep'] is None


def test_box3d_to_core_box_carries_rests_on_id_through():
    base = _make_box3d('base', 0.0, 0.0, 0.0, 0.30, 0.30, 0.12)
    child = _make_box3d('child', 0.05, 0.05, 0.12, 0.15, 0.15, 0.10)

    aabbs = {b.box_id: _box3d_aabb(b) for b in (base, child)}
    rests_on = _infer_rests_on_ids(aabbs)
    core_boxes = {
        b.box_id: box3d_to_core_box(b, rests_on_id=rests_on.get(b.box_id))
        for b in (base, child)
    }

    assert core_boxes['child'].rests_on_id == 'base'
    assert core_boxes['base'].rests_on_id is None


def test_default_unpopulated_corners_never_infer_a_relation():
    """dummy_scan_boxes_server처럼 corners를 채우지 않은 박스(기본값, 8개 전부
    (0,0,0))는 부피 0인 퇴화 AABB가 되어 겹침 비율이 항상 0이 되므로, 기존 동작
    (rests_on_id=None)과 자연히 동일하게 유지되어야 한다 - 별도 분기 없이도
    회귀가 안 생김을 확인."""
    a, b = Box3D(), Box3D()
    a.box_id, b.box_id = 'a', 'b'

    aabbs = {x.box_id: _box3d_aabb(x) for x in (a, b)}
    rests_on = _infer_rests_on_ids(aabbs)

    assert rests_on['a'] is None
    assert rests_on['b'] is None
