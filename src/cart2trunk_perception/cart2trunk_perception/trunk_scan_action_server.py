"""MSI1: ScanTrunk Action 서버.

cart2trunk_interfaces/action/ScanTrunk을 core/trunk_map_builder.py의 순수
알고리즘에 연결하는 ROS 2 wrapper.

[현재 구현 범위 - 중요]
ScanTrunk.action의 원래 의도(capture_duration 동안 Depth를 실시간으로 누적)는
MSI2가 로봇을 여러 스캔 자세로 옮기고 MSI1이 그 구간의 Depth를 받아 누적하는
것을 전제로 한다 (EDU 저장소 12.trunk_scan_hidden_gripper.py가 Isaac Sim에서
하던 역할) - 이건 실제 카메라/Isaac Sim이 있는 MSI2 없이는 이 노드 혼자
구현/검증할 수 없다.

지금은 그 대신 "이미 캡처된 run 폴더(trunk_pointcloud.npy + meta.json)를 읽어
trunk_map을 만드는" replay 모드만 구현했다 - `fixture_run_dir` 파라미터가
설정되면 그 폴더를 읽어 즉시 결과를 반환한다. 실시간 Depth 누적은 MSI2 카메라
브리지가 준비되면 이어서 구현한다 (depth_callback + 다중 waypoint 동기화 로직).
"""
import json
from pathlib import Path

import numpy as np
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from cart2trunk_interfaces.action import ScanTrunk
from cart2trunk_interfaces.msg import Box3D, TrunkMap

from cart2trunk_common import error_codes
from cart2trunk_common.geometry import corner_to_center, subtract_offset

from cart2trunk_perception.core.trunk_map_builder import build_trunk_map_from_scan


def trunk_map_dict_to_msg(trunk_map: dict) -> TrunkMap:
    """13.export_trunk_map.py 스타일 trunk_map dict -> TrunkMap.msg.

    trunk_pose = base 프레임에서 트렁크 로컬 원점(바닥 min 코너) 위치 (AABB라 회전 없음).
    occupied_boxes의 위치는 trunk_pose를 뺀 트렁크 로컬(trunk_frame) 좌표로 변환한다 -
    loading_planner_action_server.occupied_box3d_to_placed_box()가 기대하는 컨벤션과 일치.
    """
    msg = TrunkMap()
    msg.header.frame_id = trunk_map.get('frame', 'm0609_base_link')
    msg.map_id = trunk_map['run_id']

    v = trunk_map['vertices']
    x_min, y_min, floor_z = v[0]
    x_max, y_max, ceiling_z = v[6]

    msg.trunk_pose.position.x = x_min
    msg.trunk_pose.position.y = y_min
    msg.trunk_pose.position.z = floor_z
    msg.trunk_pose.orientation.w = 1.0

    msg.inner_size.x = x_max - x_min
    msg.inner_size.y = y_max - y_min
    msg.inner_size.z = ceiling_z - floor_z

    trunk_origin = (x_min, y_min, floor_z)
    occupied = []
    for obs in trunk_map.get('obstacles', []):
        ov = obs['vertices']
        o_min = tuple(ov[0])
        o_max = tuple(ov[6])
        size = tuple(hi - lo for hi, lo in zip(o_max, o_min))
        center_base = corner_to_center(o_min, size)
        cx, cy, cz = subtract_offset(center_base, trunk_origin)

        box = Box3D()
        box.box_id = obs['name']
        box.size.x, box.size.y, box.size.z = size
        box.detected_pose.position.x = cx
        box.detected_pose.position.y = cy
        box.detected_pose.position.z = cz
        box.detected_pose.orientation.w = 1.0
        occupied.append(box)
    msg.occupied_boxes = occupied

    return msg


def load_trunk_map_from_run_dir(run_dir: Path) -> dict:
    pc_dir = run_dir / 'pointcloud'
    pts_world = np.load(pc_dir / 'trunk_pointcloud.npy')
    meta = json.loads((pc_dir / 'trunk_pointcloud_meta.json').read_text())
    return build_trunk_map_from_scan(pts_world, meta, run_dir.name)


class TrunkScanActionServer(Node):

    def __init__(self):
        super().__init__('trunk_scan_action_server')
        self.declare_parameter('fixture_run_dir', '')
        self._action_server = ActionServer(
            self, ScanTrunk, '/perception/scan_trunk', self.execute_callback,
        )
        self.get_logger().info('trunk_scan_action_server ready')

    def execute_callback(self, goal_handle):
        feedback = ScanTrunk.Feedback()
        feedback.current_stage = 'BUILDING_TRUNK_MAP'
        feedback.collected_frames = 0
        feedback.progress = 0.5
        goal_handle.publish_feedback(feedback)

        result = ScanTrunk.Result()
        fixture_run_dir = self.get_parameter('fixture_run_dir').value

        if not fixture_run_dir:
            self.get_logger().error(
                '실시간 Depth 누적은 아직 미구현 - fixture_run_dir 파라미터로 replay 모드만 지원',
            )
            result.success = False
            result.error_code = error_codes.DEPTH_TIMEOUT
            result.message = (
                '실시간 스캔 미구현: fixture_run_dir 파라미터에 캡처된 run 폴더 경로를 지정하세요'
            )
            goal_handle.abort()
            return result

        try:
            trunk_map_dict = load_trunk_map_from_run_dir(Path(fixture_run_dir))
        except Exception as exc:
            self.get_logger().error(f'ScanTrunk 실패: {exc}')
            result.success = False
            result.error_code = error_codes.MAP_QUALITY_LOW
            result.message = str(exc)
            goal_handle.abort()
            return result

        result.success = True
        result.map_id = trunk_map_dict['run_id']
        result.trunk_map = trunk_map_dict_to_msg(trunk_map_dict)
        result.quality_score = 1.0
        result.error_code = ''
        result.message = f"obstacle {len(trunk_map_dict.get('obstacles', []))}개 검출"

        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = TrunkScanActionServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
