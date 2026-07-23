"""Lenovo: ComputeLoadPlan Action 서버.

cart2trunk_interfaces/action/ComputeLoadPlan을 core/의 순수 알고리즘
(09_rescan_replan.replan_after_rescan)에 연결하는 얇은 ROS 2 wrapper.
EDU 저장소의 trunk_map_planner_node.py가 실제로 호출하는 것과 같은 함수를
쓰므로, JSON 기반 프로토타입에서 이미 검증된 로직을 그대로 재사용한다.

NOTE: trunk_map.inner_size / occupied_boxes는 이미 trunk_frame 로컬 좌표계
(코너 (0,0,0) 기준)로 변환되어 있다고 가정한다. 원시 depth 스캔 vertex 점군에서
AABB를 뽑아내는 TrunkWorldMap.to_bounding_trunk() 변환은 perception(MSI1) 이관
시 다시 검토한다 - 지금은 perception이 이미 처리해서 보낸다고 가정.
"""
import math

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Pose

from cart2trunk_interfaces.action import ComputeLoadPlan
from cart2trunk_interfaces.msg import PickPlaceTask

from cart2trunk_common import error_codes
from cart2trunk_common.geometry import IDENTITY_QUATERNION, center_to_corner, corner_to_center, quaternion_from_z_yaw

from cart2trunk_planning.core.extreme_point_candidates import Box, PlacedBox
from cart2trunk_planning.core.trunk_space_state import Trunk
from cart2trunk_planning.core.rescan_replan import replan_after_rescan

_ROTATED_90DEG_Z_QUAT = quaternion_from_z_yaw(math.pi / 2)


def box3d_to_core_box(box3d) -> Box:
    return Box(id=box3d.box_id, width=box3d.size.x, depth=box3d.size.y, height=box3d.size.z)


def occupied_box3d_to_placed_box(box3d) -> PlacedBox:
    """occupied_boxes의 detected_pose.position은 박스 중심으로 보고, core가
    쓰는 최소 코너 좌표(x,y,z) 컨벤션으로 변환한다."""
    box = box3d_to_core_box(box3d)
    p = box3d.detected_pose.position
    x, y, z = center_to_corner((p.x, p.y, p.z), (box.width, box.depth, box.height))
    return PlacedBox(box=box, x=x, y=y, z=z)


def plan_to_task(plan, box_snapshot_id: str, trunk_map_id: str) -> PickPlaceTask:
    task = PickPlaceTask()
    task.task_id = f'{plan.box_id}_{plan.order}'
    task.box_id = plan.box_id

    cx, cy, cz = corner_to_center(plan.position, plan.dimensions)
    pose = Pose()
    pose.position.x = cx
    pose.position.y = cy
    pose.position.z = cz
    qx, qy, qz, qw = _ROTATED_90DEG_Z_QUAT if plan.rotated else IDENTITY_QUATERNION
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    task.target_pose = pose

    task.sequence = plan.order
    task.placement_score = plan.score
    task.box_snapshot_id = box_snapshot_id
    task.trunk_map_id = trunk_map_id
    return task


def compute_load_plan(boxes_msg, trunk_map_msg, mode: str = 'large_first', margin=None):
    trunk = Trunk(
        width=trunk_map_msg.inner_size.x,
        depth=trunk_map_msg.inner_size.y,
        height=trunk_map_msg.inner_size.z,
    )
    obstacles = [occupied_box3d_to_placed_box(b) for b in trunk_map_msg.occupied_boxes]
    boxes = [box3d_to_core_box(b) for b in boxes_msg.boxes]
    plans, unloadable = replan_after_rescan(boxes, trunk, obstacles, mode=mode, margin=margin)
    return plans, unloadable


class LoadingPlannerActionServer(Node):

    def __init__(self):
        super().__init__('loading_planner_action_server')
        self.declare_parameter('loading_mode', 'large_first')
        self.declare_parameter('margin', -1.0)
        self._action_server = ActionServer(
            self, ComputeLoadPlan, '/planning/compute_load_plan', self.execute_callback,
        )
        self.get_logger().info('loading_planner_action_server ready')

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        mode = self.get_parameter('loading_mode').value
        margin_param = self.get_parameter('margin').value
        margin = None if margin_param < 0 else margin_param

        feedback = ComputeLoadPlan.Feedback()
        feedback.current_stage = 'PLANNING'
        feedback.evaluated_candidates = 0
        feedback.best_score = 0.0
        feedback.progress = 0.5
        goal_handle.publish_feedback(feedback)

        result = ComputeLoadPlan.Result()
        try:
            plans, unloadable = compute_load_plan(goal.boxes, goal.trunk_map, mode=mode, margin=margin)
        except Exception as exc:
            self.get_logger().error(f'ComputeLoadPlan 실패: {exc}')
            result.success = False
            result.error_code = error_codes.INVALID_BOX_DATA
            result.message = str(exc)
            goal_handle.abort()
            return result

        tasks = [plan_to_task(p, goal.boxes.snapshot_id, goal.trunk_map.map_id) for p in plans]

        if not tasks and goal.boxes.boxes:
            result.success = False
            result.error_code = error_codes.NO_FEASIBLE_PLACEMENT
            reasons = ', '.join(f'{u.box_id}={u.reason.value}' for u in unloadable)
            result.message = f'{len(unloadable)}개 박스 모두 미적재 ({reasons})'
            goal_handle.abort()
            return result

        result.success = True
        result.plan_id = f'plan_{goal.request_id}' if goal.request_id else f'plan_{self.get_clock().now().nanoseconds}'
        result.tasks = tasks
        result.total_score = float(sum(p.score for p in plans))
        result.error_code = ''
        result.message = (
            f'{len(tasks)}개 배치, {len(unloadable)}개 미적재' if unloadable else f'{len(tasks)}개 전부 배치'
        )

        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = LoadingPlannerActionServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
