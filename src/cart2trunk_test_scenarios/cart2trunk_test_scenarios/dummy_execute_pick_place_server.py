"""ExecutePickPlace.action 더미 서버.

cart2trunk_platform/cart2trunk_motion(모바일 베이스+리프트+M0609+그리퍼 실행)은
아직 이관되지 않았다 - 이 더미는 cart2trunk_coordinator의 상태 머신을 실제
로봇/Isaac Sim 없이 end-to-end로 검증하기 위한 통신 스텁이다. 받은 task를
그대로 성공 처리하고, 실제 파이프라인이 거칠 phase 순서를 Feedback으로
흉내만 낸다.
"""
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from cart2trunk_interfaces.action import ExecutePickPlace

_PHASES = [
    'APPROACH_PICK', 'DESCEND_PICK', 'GRIP', 'LIFT', 'SAFE_RETRACT',
    'TRANSPORT', 'APPROACH_PLACE', 'DESCEND_PLACE', 'RELEASE', 'RETREAT',
]


class DummyExecutePickPlaceServer(Node):

    def __init__(self):
        super().__init__('dummy_execute_pick_place_server')
        self._execution_count = 0
        self._action_server = ActionServer(
            self, ExecutePickPlace, '/robot/execute_pick_place', self.execute_callback,
        )
        self.get_logger().info('dummy_execute_pick_place_server ready (항상 성공 처리)')

    def execute_callback(self, goal_handle):
        task = goal_handle.request.task
        self.get_logger().info(f'[dummy] {task.box_id} pick&place 시작 (task_id={task.task_id})')

        for i, phase in enumerate(_PHASES):
            feedback = ExecutePickPlace.Feedback()
            feedback.phase = phase
            feedback.progress = (i + 1) / len(_PHASES)
            feedback.current_tool_pose = task.target_pose
            goal_handle.publish_feedback(feedback)
            time.sleep(0.02)

        self._execution_count += 1
        result = ExecutePickPlace.Result()
        result.success = True
        result.execution_id = f'exec_dummy_{self._execution_count:04d}'
        result.error_code = ''
        result.message = f'{task.box_id} 배치 완료 (dummy)'

        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = DummyExecutePickPlaceServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
