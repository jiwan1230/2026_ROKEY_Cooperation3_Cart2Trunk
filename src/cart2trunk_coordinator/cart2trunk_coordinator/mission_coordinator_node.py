"""MSI2: 전체 작업 순서를 관리하는 Mission Coordinator.

/mission/run_loading_mission (RunLoadingMission.action)을 받으면:

    BOX_SCAN(1회) -> [ TRUNK_SCAN -> PLAN_REQUEST -> EXECUTE_TASK(계획의 첫 Task만) ] 반복 -> DONE

카트 박스 목록은 한 번만 스캔하고, 트렁크는 박스를 하나 놓을 때마다 다시
스캔·재계획한다 (EDU 저장소 TRUNK_MAP_ROS2_HANDOFF.md의 PER_PLACEMENT 정책).

이 노드는 스스로 Action Client(4개)이자 Action Server(1개)라서, 콜백 안에서
다른 Action의 결과를 기다리는 동안 그 Action의 응답 콜백도 처리될 수 있어야
한다 - 그래서 MultiThreadedExecutor + ReentrantCallbackGroup으로 실행해야 한다
(main()에서 강제).
"""
import threading

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from cart2trunk_interfaces.action import ComputeLoadPlan, ExecutePickPlace, RunLoadingMission, ScanBoxes, ScanTrunk
from cart2trunk_interfaces.msg import BoxArray, MissionState as MissionStateMsg
from cart2trunk_interfaces.srv import ResetMission

from cart2trunk_common import error_codes

from cart2trunk_coordinator.mission_state_machine import (
    InvalidTransition, MissionEvent, MissionState, MissionStateMachine,
)

_LATCHED_QOS = QoSProfile(
    depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class MissionAborted(Exception):
    """미션 실행 중 복구 불가능한 실패 - execute_callback에서 goal_handle.abort()로 변환."""

    def __init__(self, error_code: str, message: str):
        super().__init__(message)
        self.error_code = error_code


class MissionCoordinatorNode(Node):

    def __init__(self):
        super().__init__('mission_coordinator_node')
        cb_group = ReentrantCallbackGroup()

        self._sm = MissionStateMachine()
        self._state_pub = self.create_publisher(MissionStateMsg, '/mission/state', _LATCHED_QOS)

        self._scan_boxes_client = ActionClient(
            self, ScanBoxes, '/perception/scan_boxes', callback_group=cb_group)
        self._scan_trunk_client = ActionClient(
            self, ScanTrunk, '/perception/scan_trunk', callback_group=cb_group)
        self._compute_load_plan_client = ActionClient(
            self, ComputeLoadPlan, '/planning/compute_load_plan', callback_group=cb_group)
        self._execute_pick_place_client = ActionClient(
            self, ExecutePickPlace, '/robot/execute_pick_place', callback_group=cb_group)

        self._mission_server = ActionServer(
            self, RunLoadingMission, '/mission/run_loading_mission', self.execute_callback,
            callback_group=cb_group,
        )
        self._reset_srv = self.create_service(
            ResetMission, '/mission/reset', self.reset_callback, callback_group=cb_group)

        self.get_logger().info('mission_coordinator_node ready')

    # ------------------------------------------------------------------ #
    # 상태 전이 + 퍼블리시 헬퍼
    # ------------------------------------------------------------------ #

    def _transition(self, event: MissionEvent, detail: str = '', error_code: str = '',
                     error_message: str = '', placed_count: int = 0, remaining_count: int = 0,
                     rescan_count: int = 0) -> MissionState:
        previous = self._sm.state
        new_state = self._sm.handle(event)
        msg = MissionStateMsg()
        msg.state = new_state.value
        msg.previous_state = previous.value
        msg.detail = detail
        msg.placed_count = placed_count
        msg.remaining_count = remaining_count
        msg.rescan_count = rescan_count
        msg.error_code = error_code
        msg.error_message = error_message
        self._state_pub.publish(msg)
        self.get_logger().info(f'[state] {previous.value} -> {new_state.value} ({detail})')
        return new_state

    # ------------------------------------------------------------------ #
    # 하위 Action 호출 (블로킹 - ReentrantCallbackGroup이라 데드락 없음)
    # ------------------------------------------------------------------ #

    @staticmethod
    def _wait_for_future(future, timeout_sec: float, action_name: str):
        """rclpy Future.result()는 블로킹하지 않는다(끝났든 아니든 즉시 반환) - done_callback +
        threading.Event로 실제로 완료될 때까지 이 스레드만 블로킹한다. done_callback은
        ReentrantCallbackGroup 덕분에 다른 executor 스레드에서 실행되므로 데드락 없음."""
        event = threading.Event()
        future.add_done_callback(lambda _f: event.set())
        if not event.wait(timeout_sec):
            raise MissionAborted(error_codes.ACTION_TIMEOUT, f'{action_name} 응답 시간 초과({timeout_sec}s)')
        return future.result()

    def _call_action(self, client: ActionClient, goal, action_name: str, timeout_sec: float = 30.0):
        if not client.wait_for_server(timeout_sec=5.0):
            raise MissionAborted(error_codes.ACTION_SERVER_UNAVAILABLE, f'{action_name} 서버 없음')

        send_future = client.send_goal_async(goal)
        goal_handle = self._wait_for_future(send_future, timeout_sec, action_name)
        if goal_handle is None or not goal_handle.accepted:
            raise MissionAborted(error_codes.GOAL_REJECTED, f'{action_name} goal이 거부됨')

        result_future = goal_handle.get_result_async()
        wrapped = self._wait_for_future(result_future, timeout_sec, action_name)
        return wrapped.result

    # ------------------------------------------------------------------ #
    # 미션 실행 (RunLoadingMission Action Server 콜백)
    # ------------------------------------------------------------------ #

    def execute_callback(self, goal_handle):
        result = RunLoadingMission.Result()
        placed_count = 0
        rescan_count = 0

        try:
            self._transition(MissionEvent.START, detail='카트 박스 스캔 시작')

            scan_boxes_goal = ScanBoxes.Goal()
            scan_boxes_goal.request_id = goal_handle.request.request_id
            boxes_result = self._call_action(self._scan_boxes_client, scan_boxes_goal, '/perception/scan_boxes')
            if not boxes_result.success:
                self._transition(MissionEvent.BOX_SCAN_FAILED, error_code=boxes_result.error_code,
                                  error_message=boxes_result.message)
                raise MissionAborted(boxes_result.error_code, boxes_result.message)

            snapshot_id = boxes_result.snapshot_id
            box_by_id = {b.box_id: b for b in boxes_result.boxes}
            remaining_box_ids = set(box_by_id.keys())
            self._transition(MissionEvent.BOXES_SCANNED, detail=boxes_result.message,
                              remaining_count=len(remaining_box_ids))

            while True:
                scan_trunk_goal = ScanTrunk.Goal()
                scan_trunk_goal.request_id = goal_handle.request.request_id
                trunk_result = self._call_action(self._scan_trunk_client, scan_trunk_goal, '/perception/scan_trunk')
                if not trunk_result.success:
                    self._transition(MissionEvent.TRUNK_SCAN_FAILED, error_code=trunk_result.error_code,
                                      error_message=trunk_result.message)
                    raise MissionAborted(trunk_result.error_code, trunk_result.message)
                rescan_count += 1
                self._transition(MissionEvent.TRUNK_SCANNED, detail=trunk_result.message,
                                  placed_count=placed_count, remaining_count=len(remaining_box_ids),
                                  rescan_count=rescan_count)

                box_array = BoxArray()
                box_array.snapshot_id = snapshot_id
                box_array.boxes = [box_by_id[bid] for bid in remaining_box_ids]

                plan_goal = ComputeLoadPlan.Goal()
                plan_goal.request_id = goal_handle.request.request_id
                plan_goal.boxes = box_array
                plan_goal.trunk_map = trunk_result.trunk_map
                plan_result = self._call_action(
                    self._compute_load_plan_client, plan_goal, '/planning/compute_load_plan')

                if not plan_result.success:
                    self._transition(MissionEvent.PLAN_FAILED, error_code=plan_result.error_code,
                                      error_message=plan_result.message)
                    raise MissionAborted(plan_result.error_code, plan_result.message)

                if not plan_result.tasks:
                    self._transition(MissionEvent.PLAN_EMPTY, detail=plan_result.message,
                                      placed_count=placed_count, remaining_count=len(remaining_box_ids))
                    break

                self._transition(MissionEvent.PLAN_READY, detail=plan_result.message,
                                  placed_count=placed_count, remaining_count=len(remaining_box_ids))

                task = plan_result.tasks[0]
                pick_place_goal = ExecutePickPlace.Goal()
                pick_place_goal.task = task
                exec_result = self._call_action(
                    self._execute_pick_place_client, pick_place_goal, '/robot/execute_pick_place')

                if not exec_result.success:
                    self._transition(MissionEvent.TASK_FAILED, error_code=exec_result.error_code,
                                      error_message=exec_result.message)
                    raise MissionAborted(exec_result.error_code, exec_result.message)

                remaining_box_ids.discard(task.box_id)
                placed_count += 1
                self._transition(MissionEvent.TASK_SUCCEEDED, detail=exec_result.message,
                                  placed_count=placed_count, remaining_count=len(remaining_box_ids))

        except MissionAborted as exc:
            result.success = False
            result.placed_count = placed_count
            result.remaining_count = len(remaining_box_ids) if 'remaining_box_ids' in locals() else 0
            result.error_code = exc.error_code
            result.message = str(exc)
            goal_handle.abort()
            return result

        result.success = True
        result.placed_count = placed_count
        result.remaining_count = len(remaining_box_ids)
        result.error_code = ''
        result.message = f'{placed_count}개 배치, {len(remaining_box_ids)}개 미배치 (재스캔 {rescan_count}회)'
        goal_handle.succeed()
        return result

    # ------------------------------------------------------------------ #
    # /mission/reset 서비스
    # ------------------------------------------------------------------ #

    def reset_callback(self, request, response):
        try:
            self._transition(MissionEvent.RESET, detail=f'reset by {request.reason or "unknown"}')
            response.success = True
            response.message = 'IDLE로 복귀'
        except InvalidTransition as exc:
            response.success = False
            response.message = str(exc)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MissionCoordinatorNode()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
