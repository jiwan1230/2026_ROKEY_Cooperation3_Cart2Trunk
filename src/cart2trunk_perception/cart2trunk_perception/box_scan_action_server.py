"""MSI1: ScanBoxes Action 서버.

cart2trunk_interfaces/action/ScanBoxes를
core/multiview_box_detector.py(EDU 저장소 perception/multiview_scan.py 이식)의
순수 검출 로직에 연결하는 ROS 2 wrapper. trunk_scan_action_server.py와 같은
설계 원칙(코어는 순수 함수, 이 파일은 메시지 변환 + goal 처리만 담당)을 따른다.

[현재 구현 범위 - 중요, trunk_scan_action_server.py와 동일한 이유]
ScanBoxes.action의 원래 의도(capture_duration 동안 여러 waypoint를 돌며 Depth를
실시간으로 누적)는 MSI2가 로봇을 스캔 자세로 옮기고 이 노드가 그 구간의 Depth+TF를
받아 point cloud를 누적하는 것을 전제로 한다 - 아직 MSI2 카메라 브리지가 없어
실시간 누적 자체를 구현/검증할 수 없다.

지금은 EDU 저장소의 실제 검증된 스탠드얼론 흐름(99.cart_scan_dual_side_holonomic.py가
scan_cache/merged_cart_scan.npy로 저장 -> run_scan_batch.py가 오프라인 처리)과 같은
"이미 병합된 point cloud 파일 하나를 읽어 처리하는" replay 모드만 구현했다 -
`fixture_input_path` 파라미터가 설정되면 그 파일(.npy 또는 .ply, m0609_base_link
좌표계로 이미 합쳐진 point cloud)을 읽어 즉시 결과를 반환한다. 실시간 Depth+TF
누적(PointCloudAccumulator 역할)은 MSI2 카메라 브리지가 준비되면 이어서 구현한다.

[좌표계 - detected_pose를 world로 즉시 변환하는 이유]
원래(구버전)는 detected_pose를 m0609_base_link 프레임 그대로 반환했다 - 이건
"ScanBoxes 직후 곧바로 같은 위치에서 ExecutePickPlace를 부른다"는 가정에서만
유효하다(스캔과 pick 사이에 섀시가 움직이면 좌표계가 어긋난다). 사용자 지시 -
capture_cart_scan.py가 스캔 후 홀로노믹 베이스를 시작 위치로 복귀시킨 채 서버
모드를 유지하고, 그 이후 들어오는 실제 요청이 "복귀된 베이스 위치" 기준으로
정확히 동작해야 한다 - 즉 스캔과 pick 사이에 섀시가 실제로 움직이는 시나리오를
정식으로 지원해야 한다. fixture_input_path 옆에 capture_cart_scan.py가 저장한
"<stem>_anchor.json"(스캔 시점 m0609_base_link의 world pos/quat_wxyz)이 있으면
그 anchor로 detected_pose/corners를 여기서 한 번에 world 좌표로 변환해서 반환한다
- 이후 섀시가 어디로 움직이든(anchor json은 이 파일이 존재하는 한 계속 그
스캔 시점 값을 가리키므로) detected_pose는 계속 유효하다. anchor json이 없으면
(구버전 fixture 등) m0609_base_link 프레임 그대로 반환하고 header.frame_id를
비워둔다 - execute_pick_place_action_server.py가 frame_id로 두 경우를 구분해서
처리한다.
"""
import json
import os
from pathlib import Path

import numpy as np
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from geometry_msgs.msg import Point

from cart2trunk_interfaces.action import ScanBoxes
from cart2trunk_interfaces.msg import Box3D

from cart2trunk_common import error_codes
from cart2trunk_common.geometry import quaternion_from_z_yaw

from cart2trunk_perception.core import multiview_box_detector as mvd
from cart2trunk_perception.core.trunk_map_builder import quat_wxyz_to_matrix

WORLD_FRAME = "world"


def load_scan_anchor(fixture_input_path: str):
    """capture_cart_scan.py가 merged_cart_scan.npy와 나란히 저장하는
    "<stem>_anchor.json"(스캔 시점 m0609_base_link의 world pos/quat_wxyz)을 읽는다.
    없으면(구버전 fixture 등) None - box_dict_to_msg()가 base_link 프레임 그대로
    반환하는 구버전 동작으로 폴백한다."""
    anchor_path = Path(fixture_input_path).with_name(Path(fixture_input_path).stem + "_anchor.json")
    if not anchor_path.exists():
        return None
    payload = json.loads(anchor_path.read_text())
    anchor_pos = np.asarray(payload["base_link_pos"], dtype=np.float64)
    anchor_quat_wxyz = np.asarray(payload["base_link_quat_wxyz"], dtype=np.float64)
    return anchor_pos, anchor_quat_wxyz


def box_dict_to_msg(box: dict, anchor=None) -> Box3D:
    """multiview_box_detector.detect_boxes_in_base_frame()의 결과 dict 하나(corner_order:
    top_0..top_3, bottom_0..bottom_3, m0609_base_link 좌표계) -> Box3D.msg.

    anchor=(anchor_pos, anchor_quat_wxyz)가 주어지면 corners를 다른 모든 계산(중심/
    폭/깊이/yaw) 전에 먼저 world로 변환한다 - 그러면 아래 로직은 그대로 두고도
    결과가 자동으로 world 좌표계가 된다(회전이 폭/깊이 같은 길이값에는 영향을 안
    주므로 그 값들은 프레임 무관).

    detected_pose.position = 8꼭짓점의 기하 중심(코너 dict의 "top".center가 아니라
    실제 사각형 코너 평균 - fill_ratio가 낮은 치우친 클러스터일수록 top.center가
    사각형 중심과 어긋날 수 있으므로 multiview_scan._rect_center_xy()와 같은 원칙).
    yaw = 검출된 사각형 자신의 변 방향(코너 0->1) 기준, (anchor가 없으면 base_link
    +X축, 있으면 world +X축) 대비 각도 - 박스가 회전된 채 카트에 놓여도 정확히 반영된다.
    """
    corners = np.asarray(box["corners"], dtype=np.float64)  # (8,3)
    frame_id = mvd.OUTPUT_FRAME
    if anchor is not None:
        anchor_pos, anchor_quat_wxyz = anchor
        r_anchor = quat_wxyz_to_matrix(anchor_quat_wxyz)
        corners = corners @ r_anchor.T + anchor_pos[None, :]
        frame_id = WORLD_FRAME

    top_corners = corners[:4]
    bottom_corners = corners[4:]

    center = corners.mean(axis=0)
    top_z = float(top_corners[:, 2].mean())
    bottom_z = float(bottom_corners[:, 2].mean())
    height = top_z - bottom_z

    edge_u = top_corners[1] - top_corners[0]
    edge_v = top_corners[3] - top_corners[0]
    width = float(np.linalg.norm(edge_u))
    depth = float(np.linalg.norm(edge_v))
    yaw = float(np.arctan2(edge_u[1], edge_u[0]))

    msg = Box3D()
    msg.header.frame_id = frame_id
    msg.box_id = str(box["box_id"])
    msg.size.x, msg.size.y, msg.size.z = width, depth, max(height, 0.0)

    msg.detected_pose.position.x = float(center[0])
    msg.detected_pose.position.y = float(center[1])
    msg.detected_pose.position.z = float(center[2])
    qx, qy, qz, qw = quaternion_from_z_yaw(yaw)
    msg.detected_pose.orientation.x = qx
    msg.detected_pose.orientation.y = qy
    msg.detected_pose.orientation.z = qz
    msg.detected_pose.orientation.w = qw
    # target_pose는 카트 위 박스에는 의미가 없다 (HANDOFF_MSI2.md 6.4절) - 기본값(0)으로 둔다.

    msg.corners = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in corners]
    msg.yaw = yaw
    msg.confidence = float(box["top"].fill_ratio)

    return msg


class BoxScanActionServer(Node):

    def __init__(self):
        super().__init__('box_scan_action_server')
        self.declare_parameter('fixture_input_path', '')
        self._sequence = 0
        # 검출(mvd.detect_boxes_in_base_frame)이 RANSAC 150회 반복이라 수 분 걸릴 수
        # 있다(사용자 지시 - pick-place 동작 자체를 반복 테스트하는 동안 매번 이걸
        # 다시 돌릴 필요는 없음). fixture 파일이 안 바뀐 동안은 캐시를 그대로 재사용
        # 한다 - mtime으로 무효화하므로 새 스캔을 저장하면(캡처 스크립트가 매번 같은
        # 경로에 덮어쓰므로) 자동으로 다시 계산된다.
        self._cache_key = None
        self._cache_boxes = None
        self._cache_anchor = None
        self._action_server = ActionServer(
            self, ScanBoxes, '/perception/scan_boxes', self.execute_callback,
        )
        self.get_logger().info('box_scan_action_server ready')

    def execute_callback(self, goal_handle):
        feedback = ScanBoxes.Feedback()
        feedback.current_stage = 'DETECTING_BOXES'
        feedback.collected_frames = 0
        feedback.progress = 0.5
        goal_handle.publish_feedback(feedback)

        result = ScanBoxes.Result()
        fixture_input_path = self.get_parameter('fixture_input_path').value

        if not fixture_input_path:
            self.get_logger().error(
                '실시간 Depth+TF 누적은 아직 미구현 - fixture_input_path 파라미터로 replay 모드만 지원',
            )
            result.success = False
            result.error_code = error_codes.DEPTH_TIMEOUT
            result.message = (
                '실시간 스캔 미구현: fixture_input_path 파라미터에 '
                'm0609_base_link 좌표계로 병합된 point cloud(.npy/.ply) 경로를 지정하세요'
            )
            goal_handle.abort()
            return result

        try:
            cache_key = (fixture_input_path, os.path.getmtime(fixture_input_path))
            if cache_key == self._cache_key:
                boxes = self._cache_boxes
                anchor = self._cache_anchor
                self.get_logger().info(
                    f'{fixture_input_path} 변경 없음 - 캐시된 검출 결과 재사용(재검출 생략)')
            else:
                points_base = mvd.load_merged_cloud(fixture_input_path)
                boxes = mvd.detect_boxes_in_base_frame(points_base)
                anchor = load_scan_anchor(fixture_input_path)
                self._cache_key = cache_key
                self._cache_boxes = boxes
                self._cache_anchor = anchor
        except Exception as exc:
            self.get_logger().error(f'ScanBoxes 실패: {exc}')
            result.success = False
            result.error_code = error_codes.MAP_QUALITY_LOW
            result.message = str(exc)
            goal_handle.abort()
            return result

        if anchor is None:
            self.get_logger().warn(
                f'{fixture_input_path} 옆에 anchor json이 없음 - detected_pose를 '
                f'{mvd.OUTPUT_FRAME} 프레임 그대로 반환합니다(구버전 fixture 호환). '
                '섀시가 스캔 이후 움직이면 이 좌표는 더 이상 유효하지 않습니다.'
            )

        self._sequence += 1
        box_msgs = [box_dict_to_msg(b, anchor=anchor) for b in boxes]

        result.success = True
        result.snapshot_id = f'box_scan_{self._sequence:04d}'
        result.boxes = box_msgs
        result.quality_score = (
            float(np.mean([b.confidence for b in box_msgs])) if box_msgs else 0.0
        )
        result.error_code = ''
        result.message = f'{len(box_msgs)}개 박스 검출'

        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = BoxScanActionServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
