"""MSI2: ExecutePickPlace Action 서버 - cart2trunk_test_scenarios의
dummy_execute_pick_place_server.py를 대체하는 실제 구현.

cart2trunk_platform/platform_controller_node.py가 노출하는 4개 저수준
인터페이스(리프트/모바일베이스/그리퍼/M0609)를 PlatformClient로 조합해서,
EDU 저장소 100.cart_to_trunk_dual_side_holonomic.py가 하던 카트->트렁크
pick&place의 첫 단순화 슬라이스를 구현한다.

[좌표계 - 반드시 읽을 것]
ExecutePickPlace.Goal의 source_box.detected_pose/task.target_pose는 각각
m0609_base/trunk_frame 기준이다(HANDOFF_MSI2.md 6.2절). 이 두 프레임은 전부
"로봇이 지금 서 있는 위치"에 고정된 상대 프레임이라, 실제 로봇을 움직이려면
/m0609/move_to_pose가 요구하는 world 좌표로 바꿔야 한다. 이 노드는 골 실행을
시작하는 순간의 섀시 world pose + 리프트 높이를 "앵커"로 한 번 캡처해서
그 기준으로 base_frame -> world 변환을 한다.

[중요한 단순화 - 이번 슬라이스가 안 하는 것]
1. 모바일 베이스 주행(cmd_vel)을 전혀 안 쓴다 - mission_coordinator_node의
   현재 루프(ScanBoxes -> {ScanTrunk -> ComputeLoadPlan -> ExecutePickPlace})는
   그 사이에 "카트로 이동"/"트렁크로 이동" 단계가 없다 - 즉 지금 시스템은 카트와
   트렁크가 모두 팔+리프트 범위 안에 들어오는 한 위치에 로봇이 고정 주차돼
   있다고 전제한다. 실제로 두 지점이 팔 범위를 벗어나 있다면 별도의
   position/scan_motion_action_server(주차 위치 자체를 잡는 단계)가 먼저
   필요한데, 아직 이관되지 않았다 - 그 전까지 TRANSPORT 단계는 "팔로 들어서
   옮기기"만 한다.
2. 리프트를 능동적으로 쓰지 않는다(lift_delta_m 파라미터는 기본 0 - 훅만
   만들어둠) - LIFT_MIN/MAX 등 실측 상수는 platform_controller_node(Isaac Sim
   프로세스) 안에만 있어서 이 프로세스에서 안전한 기본값을 추정할 근거가 없다.
   실측 후 파라미터로 채울 것.
3. 100.py의 dual-side 카트 접근, 기울기(tilt) 진입, 충돌 회피 raycast 등은
   포함하지 않는다 - 이번 최종 하드웨어(옴니휠+리프트+M0609)에서 그 로직이
   그대로 필요한지도 아직 검증 전이라 새 하드웨어로 재검증하기 전까지는
   가장 단순한 "박스 위로 접근 -> 하강 -> 흡착 -> 들기 -> 이동 -> 하강 -> 해제"
   시퀀스만 구현한다.
"""
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from cart2trunk_interfaces.action import ExecutePickPlace

from cart2trunk_common import error_codes
from cart2trunk_common.geometry import add_offset, z_yaw_from_quaternion

from cart2trunk_motion.control_loops import base_frame_to_world, down_quat_with_yaw, move_tip_to
from cart2trunk_motion.platform_client import PlatformClient

_PHASES = [
    'APPROACH_PICK', 'DESCEND_PICK', 'GRIP', 'LIFT', 'SAFE_RETRACT',
    'TRANSPORT', 'APPROACH_PLACE', 'DESCEND_PLACE', 'RELEASE', 'RETREAT',
]


class PickPlaceAborted(Exception):
    def __init__(self, error_code: str, message: str):
        super().__init__(message)
        self.error_code = error_code


class ExecutePickPlaceActionServer(Node):

    def __init__(self):
        super().__init__('execute_pick_place_action_server')
        cb_group = ReentrantCallbackGroup()

        self.declare_parameter('approach_clearance_m', 0.15)
        self.declare_parameter('pick_tolerance_m', 0.005)
        self.declare_parameter('place_tolerance_m', 0.005)
        self.declare_parameter('lift_delta_m', 0.0)  # 훅만 - 기본은 무동작(모듈 docstring 2번 참고)

        self._platform = PlatformClient(self, cb_group)
        self._execution_count = 0

        self._action_server = ActionServer(
            self, ExecutePickPlace, '/robot/execute_pick_place', self.execute_callback,
            callback_group=cb_group,
        )
        self.get_logger().info(
            'execute_pick_place_action_server ready (platform_controller_node 연결 대기)')

    # ------------------------------------------------------------------ #

    def _feedback(self, goal_handle, phase: str, progress: float) -> None:
        fb = ExecutePickPlace.Feedback()
        fb.phase = phase
        fb.progress = progress
        tip = self._platform.tip_pose
        if tip is not None:
            pos, quat = tip
            fb.current_tool_pose.position.x = pos[0]
            fb.current_tool_pose.position.y = pos[1]
            fb.current_tool_pose.position.z = pos[2]
            (fb.current_tool_pose.orientation.x, fb.current_tool_pose.orientation.y,
             fb.current_tool_pose.orientation.z, fb.current_tool_pose.orientation.w) = quat
        goal_handle.publish_feedback(fb)
        self.get_logger().info(f'[{phase}] progress={progress:.1f}')

    def _run_move(self, goal_handle, phase: str, progress: float, target_world, orientation_xyzw,
                  tolerance: float, max_seconds: float = 30.0):
        self._feedback(goal_handle, phase, progress)
        result = move_tip_to(
            self._platform, target_world, orientation_xyzw=orientation_xyzw,
            tolerance=tolerance, max_seconds=max_seconds)
        if not result.success:
            raise PickPlaceAborted(error_codes.ROBOT_TIMEOUT, f'{phase} 실패: {result.detail}')
        self.get_logger().info(f'{phase}: {result.detail}')

    def execute_callback(self, goal_handle):
        task = goal_handle.request.task
        source_box = goal_handle.request.source_box
        trunk_pose = goal_handle.request.trunk_pose
        result = ExecutePickPlace.Result()

        if not self._platform.wait_until_ready(timeout_sec=10.0):
            result.success = False
            result.error_code = error_codes.PLATFORM_UNAVAILABLE
            result.message = 'platform_controller_node 상태 토픽이 안 들어옴 - 노드가 떠 있는지 확인'
            goal_handle.abort()
            return result

        try:
            # 앵커 - 이번 goal 실행 동안 로봇이 안 움직인다는 전제(모듈 docstring 1번)로
            # 시작 시점의 섀시/리프트 상태 한 번만 캡처한다.
            base_pos, base_quat = self._platform.base_pose
            anchor = (base_pos, z_yaw_from_quaternion(base_quat), self._platform.lift_height)

            clearance = float(self.get_parameter('approach_clearance_m').value)
            pick_tol = float(self.get_parameter('pick_tolerance_m').value)
            place_tol = float(self.get_parameter('place_tolerance_m').value)

            pick_center_base = (
                source_box.detected_pose.position.x,
                source_box.detected_pose.position.y,
                source_box.detected_pose.position.z,
            )
            pick_center_world = base_frame_to_world(anchor, pick_center_base)
            pick_top = (pick_center_world[0], pick_center_world[1],
                        pick_center_world[2] + source_box.size.z / 2.0)
            pick_hover = (pick_top[0], pick_top[1], pick_top[2] + clearance)

            target_center_trunk = (
                task.target_pose.position.x,
                task.target_pose.position.y,
                task.target_pose.position.z,
            )
            trunk_origin = (trunk_pose.position.x, trunk_pose.position.y, trunk_pose.position.z)
            place_center_base = add_offset(target_center_trunk, trunk_origin)
            place_center_world = base_frame_to_world(anchor, place_center_base)
            place_top = (place_center_world[0], place_center_world[1],
                         place_center_world[2] + source_box.size.z / 2.0)
            place_hover = (place_top[0], place_top[1], place_top[2] + clearance)

            pick_quat = down_quat_with_yaw(0.0)
            target_yaw = z_yaw_from_quaternion((
                task.target_pose.orientation.x, task.target_pose.orientation.y,
                task.target_pose.orientation.z, task.target_pose.orientation.w,
            ))
            place_quat = down_quat_with_yaw(target_yaw - source_box.yaw)

            # --- PICK ---
            self._run_move(
                goal_handle, 'APPROACH_PICK', 0.05, pick_hover, pick_quat, clearance * 0.5)
            self._run_move(goal_handle, 'DESCEND_PICK', 0.20, pick_top, pick_quat, pick_tol)

            self._feedback(goal_handle, 'GRIP', 0.30)
            success, message = self._platform.gripper_activate()
            if not success:
                raise PickPlaceAborted(error_codes.GRASP_FAILED, f'흡착 실패: {message}')
            self.get_logger().info(f'GRIP: {message}')

            self._run_move(goal_handle, 'LIFT', 0.40, pick_hover, pick_quat, pick_tol)

            lift_delta = float(self.get_parameter('lift_delta_m').value)
            if lift_delta:
                self._platform.set_lift_height((self._platform.lift_height or 0.0) + lift_delta)

            # 배치 자세로 미리 돌려놓는다(도구는 아직 pick_hover 위치, 방향만 바뀜) -
            # move_tip_to는 위치 오차만 보므로, 위치가 이미 도달해 있으면 방향이 아직
            # 안 따라왔어도 즉시 반환해버린다 - 그래서 별도로 살짝 넉넉하게 재호출해
            # RMPflow가 새 방향으로 수렴할 시간을 준다.
            self._feedback(goal_handle, 'SAFE_RETRACT', 0.50)
            move_tip_to(self._platform, pick_hover, orientation_xyzw=place_quat,
                        tolerance=pick_tol, max_seconds=5.0)

            # --- TRANSPORT (팔 범위 안에서만 - 모듈 docstring 1번) ---
            self._run_move(
                goal_handle, 'TRANSPORT', 0.65, place_hover, place_quat, clearance * 0.5)

            self._run_move(goal_handle, 'APPROACH_PLACE', 0.75, place_hover, place_quat, place_tol)
            self._run_move(goal_handle, 'DESCEND_PLACE', 0.85, place_top, place_quat, place_tol)

            self._feedback(goal_handle, 'RELEASE', 0.92)
            success, message = self._platform.gripper_release()
            if not success:
                raise PickPlaceAborted(error_codes.GRASP_FAILED, f'흡착 해제 실패: {message}')
            self.get_logger().info(f'RELEASE: {message}')

            self._run_move(goal_handle, 'RETREAT', 1.0, place_hover, place_quat, place_tol)

        except PickPlaceAborted as exc:
            result.success = False
            result.execution_id = ''
            result.error_code = exc.error_code
            result.message = str(exc)
            goal_handle.abort()
            return result

        self._execution_count += 1
        result.success = True
        result.execution_id = f'exec_{self._execution_count:04d}'
        result.error_code = ''
        result.message = f'{task.box_id} 배치 완료'
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ExecutePickPlaceActionServer()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
