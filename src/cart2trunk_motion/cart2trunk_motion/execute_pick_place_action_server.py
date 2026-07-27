"""MSI2: ExecutePickPlace Action 서버 - cart2trunk_test_scenarios의
dummy_execute_pick_place_server.py를 대체하는 실제 구현.

cart2trunk_platform/platform_controller_node.py가 노출하는 4개 저수준
인터페이스(리프트/모바일베이스/그리퍼/M0609)를 PlatformClient로 조합해서,
EDU 저장소 100.cart_to_trunk_dual_side_holonomic.py가 하던 카트->트렁크
pick&place의 첫 단순화 슬라이스를 구현한다.

[좌표계 - 반드시 읽을 것]
task.target_pose(trunk_frame 기준, HANDOFF_MSI2.md 6.2절)는 "골 실행을 시작하는
순간 로봇이 서 있던 위치"에 고정된 상대 프레임이다 - 팔을 world로 움직이려면
그 시작 시점 섀시 world pose + 리프트 높이를 "앵커"로 한 번 캡처해서 그 기준으로
base_frame -> world 변환을 한다(주행 중에도 이 앵커 자체는 안 바뀜 - 섀시가
그 뒤에 실제로 움직여도 무방).

source_box.detected_pose는 두 가지 프레임을 다 지원한다 - box_scan_action_server.py가
스캔 시점 anchor(json)를 찾으면 이미 world 좌표로 변환해서 보내고
(header.frame_id="world", 사용자 지시 - 스캔 후 홀로노믹 베이스가 시작 위치로
복귀한 뒤에도 이후 요청이 정확히 동작해야 함), 없으면(구버전 fixture/더미 서버)
위와 같은 "골 시작 시점 앵커" 기준 base_link 프레임 그대로 보낸다(header.frame_id
비어있음). header.frame_id로 두 경우를 구분해서 처리한다 - world면 그대로 쓰고,
아니면 지금 앵커로 변환한다(source_box.yaw도 동일 원칙 - world면 앵커 yaw만큼
빼서 앵커-상대 값으로 재해석한다).

[모바일 베이스 주행 - APPROACH_CART/TRANSPORT]
cart2trunk_simulation 씬(1차 슬라이스)에서 카트/차량이 로봇 스폰 위치의 팔
반경 밖에 놓여 있다는 게 실측으로 확인돼서, 이번에 실제 주행을 추가했다.
control_loops.compute_standoff()로 "지금 있는 쪽에서 목표를 정면으로
distance_m만큼 떨어져서 본다"는 범용 접근 지점을 계산한다. 100.py의
CART_CLEAR_X/CART_BASE_LEFT_XY처럼 카트 손잡이 위치 등 자산별 정확한 형상을
아는 정교한 접근은 아니다(그건 실제 카트/차량 형상으로 재현되며 다듬어야 할
부분) - 다만 100.py가 반복 검증한 핵심 원칙("회전과 이동을 동시에 하면 섀시
발자국이 회전 도중 넓게 쓸고 지나가 장애물과 겹칠 수 있다")은 그대로 지킨다 -
_run_drive_to_standoff()가 제자리 회전(1단계) -> 직진 접근(2단계)으로 나눠서
drive_to()를 두 번 부른다(사용자가 GUI 실측으로 단일 동시수렴 방식의 충돌성
불안정을 확인한 뒤 수정).

박스를 든 채로 주행하는 TRANSPORT 전후, 그리고 카트로 접근하는 APPROACH_CART
직전에는 100.py STAGE0.5/STAGE1 진입부(raise_lift_and_fold 연속 호출)를 그대로
트리거하는 PlatformClient.enter_transport_pose()/exit_transport_pose()를 부른다 -
팔을 안전 접힘 자세(joint_3/5=90도)로 접고 리프트를 낮춘 뒤에야 주행하고, 도착
후 리프트를 다시 올린다(실제 관절 보간은 platform_controller_node.py 안,
LIFT_MIN/MAX 등 실측 상수도 그 프로세스 안에만 있으므로 여기서는 트리거만 한다).

카트 standoff 도착 직후(APPROACH_PICK 직전)에는 100.py pick_raise_and_aim()을
그대로 트리거하는 PlatformClient.set_pick_aim_target()+pick_raise_and_aim()을
부른다 - 리프트를 PICK_LIFT_H(도킹/LIFT_MAX와 별개인 카트 픽업 전용 높이)로
올리며 접고, FK 기반으로 joint_1(방위각)을 박스 방향으로 먼저 맞춘다(100.py는
카트 중심을 조준했지만, 여기서는 이미 world 좌표로 아는 실제 박스 위치를 써서
더 정확하다) - 순수 관절공간이라 RMPflow가 관여하지 않고, 그 다음 APPROACH_PICK의
Cartesian 이동이 큰 목표가 아니라 이미 대략 조준된 자세에서 작은 목표만 풀면 된다.

[중요한 단순화 - 이번 슬라이스가 안 하는 것]
1. 리프트를 GRIP~SAFE_RETRACT 사이(들어올린 채 유지하는 구간)에는 직접 세밀하게
   조정하지 않는다(lift_delta_m 파라미터는 기본 0 - 훅만 만들어둠).
2. 100.py의 dual-side 카트 접근(손잡이 회피), 기울기(tilt) 진입, 충돌 회피
   raycast는 아직 포함하지 않는다 - 이번 최종 하드웨어(옴니휠+리프트+M0609)에서
   그 로직이 그대로 필요한지도 아직 검증 전이라 새 하드웨어로 재검증하기 전까지는
   "안전 자세 -> 카트 옆으로 주행(회전/직진 분리) -> 리프트+조준(pick_raise_and_aim)
   -> 박스 위로 접근 -> 하강 -> 흡착 -> 들기 -> 안전 운송 자세 -> 트렁크 앞으로
   주행(회전/직진 분리) -> 리프트 재상승 -> 박스 위로 접근 -> 하강 -> 해제"
   시퀀스만 구현한다.
"""
import math
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from cart2trunk_interfaces.action import ExecutePickPlace

from cart2trunk_common import error_codes
from cart2trunk_common.geometry import add_offset, z_yaw_from_quaternion

from cart2trunk_motion.control_loops import (
    base_frame_to_world, compute_standoff, down_quat_with_yaw, drive_to, move_tip_to,
)
from cart2trunk_motion.platform_client import PlatformClient

_PHASES = [
    'APPROACH_CART', 'APPROACH_PICK', 'DESCEND_PICK', 'GRIP', 'LIFT', 'SAFE_RETRACT',
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
        self.declare_parameter('lift_delta_m', 0.0)  # 훅만 - 기본은 무동작(모듈 docstring 1번 참고)
        self.declare_parameter('trunk_standoff_m', 0.7)
        self.declare_parameter('drive_tolerance_xy_m', 0.08)
        self.declare_parameter('drive_tolerance_yaw_deg', 8.0)
        self.declare_parameter('drive_max_seconds', 120.0)
        # target(토픽) 발행 후 trigger(서비스) 호출 전 대기 시간 - 두 통신이 별도
        # 경로라 순서가 보장되지 않는다(platform_controller_node.py의
        # _on_gripper_activate() docstring 참고, 사용자 지시로 도입) - /gripper/
        # set_target+/gripper/activate와 /m0609/pick_aim_target+/m0609/pick_raise_and_aim
        # 둘 다 같은 패턴이라 이 설정을 공유한다. platform 쪽도 target freshness
        # (GRIPPER_TARGET_TIMEOUT_SEC)로 한 번 더 방어하지만, 이 대기 자체가 없으면
        # 대부분의 경우 trigger가 target보다 먼저 처리돼 매번 실패한다.
        self.declare_parameter('target_settle_sec', 0.15)

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

    def _drive_leg(self, phase: str, label: str, target_x: float, target_y: float, target_yaw: float):
        tolerance_xy_m = float(self.get_parameter('drive_tolerance_xy_m').value)
        tolerance_yaw_rad = math.radians(float(self.get_parameter('drive_tolerance_yaw_deg').value))
        max_seconds = float(self.get_parameter('drive_max_seconds').value)
        result = drive_to(
            self._platform, target_x, target_y, target_yaw,
            tolerance_xy=tolerance_xy_m, tolerance_yaw_rad=tolerance_yaw_rad,
            max_seconds=max_seconds,
        )
        if not result.success:
            raise PickPlaceAborted(error_codes.ROBOT_TIMEOUT, f'{phase} 실패({label}): {result.detail}')
        self.get_logger().info(f'{phase}({label}): {result.detail}')

    def _run_drive_to_standoff(
        self, goal_handle, phase: str, progress: float, target_xy, distance_m,
    ):
        """카트/차량의 정확한 형상을 모르는 일반적인 목표(현재 TRANSPORT)를 위한
        2단계 버전 - "카트와 X축만 분리된 안전지대"(CART_CLEAR_X)는 재현 못 하지만,
        100.py가 반복 검증한 핵심 원칙("회전과 이동을 동시에 하면 섀시 발자국이 회전
        도중 넓게 쓸고 지나가 장애물과 겹칠 수 있다")은 지킨다: 1) 제자리에서 최종
        yaw로 먼저 회전, 2) 그 다음 직진 접근. APPROACH_CART처럼 안전지대 X를 아는
        경우는 _run_drive_to_cart_standoff()를 대신 쓴다(더 안전함 - 아래 참고)."""
        current_pose = self._platform.base_pose
        if current_pose is None:
            raise PickPlaceAborted(error_codes.PLATFORM_UNAVAILABLE, f'{phase} 실패: base_pose 없음')
        standoff_xy, yaw = compute_standoff(current_pose[0][:2], target_xy, distance_m)
        self._feedback(goal_handle, phase, progress)
        current_xy = current_pose[0][:2]

        self._drive_leg(phase, '1/2 제자리 회전', current_xy[0], current_xy[1], yaw)
        self._drive_leg(phase, '2/2 직진 접근', standoff_xy[0], standoff_xy[1], yaw)

    def _run_drive_to_cart_standoff(self, goal_handle, phase: str, progress: float):
        """100.py의 카트 접근을 그대로 3단계로 포팅 - CART_CLEAR_X(회전 안전지대,
        2267행 부근 주석)를 경유한다: 1) X만 CART_CLEAR_X로 이동(Y/yaw 유지 - 이미
        그 값이면 사실상 대기), 2) X=CART_CLEAR_X 고정한 채 Y+yaw를 목표 standoff로
        동시 수렴(X 안전지대 유지되므로 회전 도중 카트와 안 겹침), 3) Y/yaw는 이미
        목표, X만 좁혀 최종 접근(Y축 분리로 안전).

        [사용자 지적으로 발견한 두 번째 버그 - standoff 자체가 틀렸었다] 처음엔
        compute_standoff()로 "박스로부터 distance_m"을 standoff로 썼다 - 그런데
        100.py의 실제 카트 PICK standoff(CART_BASE_LEFT_XY/RIGHT_XY, 1092행)는
        박스 위치와 무관하게 "카트 중심에서 옆(Y)으로 CART_STANDOFF_DIST만큼 고정된
        지점"이다 - 카트 길이 방향(X)으로 실제 박스까지 다가가는 건 섀시가 아니라
        팔(pick_raise_and_aim의 joint_1 조준 + RMPflow 리치)의 몫이다. "박스로부터
        거리" 근사는 박스가 카트 안쪽 깊이 있을 때(카트 가장자리에서 겨우 0.1m 밖)
        그 standoff가 카트 몸체와 겹쳐 마지막 접근 구간에서 정체(멈춤)를 냈다(사용자
        실측: "2.5s 동안 0.0011m밖에 못 움직임"). 지금은 platform_controller_node.py가
        노출하는 실제 카트 중심/CART_STANDOFF_DIST로 진짜 standoff를 계산한다 -
        항상 카트의 -Y쪽("오른쪽", capture_cart_scan.py가 스캔 때 검증한 바로 그
        면, yaw=0)에서 접근한다. 100.py의 왼쪽/오른쪽 XOR 선택(박스의 카트 A/B단
        치우침 + solution space)은 아직 포팅하지 않았다 - 지금은 박스가 어느 쪽에
        있든 항상 같은 쪽에서 접근한다(다음 개선 대상, 모듈 docstring 참고)."""
        current_pose = self._platform.base_pose
        if current_pose is None:
            raise PickPlaceAborted(error_codes.PLATFORM_UNAVAILABLE, f'{phase} 실패: base_pose 없음')
        clear_x = self._platform.cart_clear_x
        cart_center_xy = self._platform.cart_center_xy
        standoff_dist = self._platform.cart_standoff_dist
        if clear_x is None or cart_center_xy is None or standoff_dist is None:
            raise PickPlaceAborted(
                error_codes.PLATFORM_UNAVAILABLE, f'{phase} 실패: 카트 geometry 없음')
        standoff_xy = (cart_center_xy[0], cart_center_xy[1] - standoff_dist)
        yaw = 0.0
        self._feedback(goal_handle, phase, progress)
        current_xy = current_pose[0][:2]
        current_yaw = z_yaw_from_quaternion(current_pose[1])

        self._drive_leg(phase, '1/3 회전 안전지대(X만 이동)', clear_x, current_xy[1], current_yaw)
        self._drive_leg(phase, '2/3 안전지대에서 standoff Y/yaw로 회전+횡이동', clear_x, standoff_xy[1], yaw)
        self._drive_leg(phase, '3/3 안전지대 -> standoff 최종 접근', standoff_xy[0], standoff_xy[1], yaw)

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

            # box_scan_action_server.py가 스캔 시점 anchor(json)를 찾으면 detected_pose를
            # 이미 world 좌표로 변환해서 보낸다(header.frame_id="world") - 그러면 스캔과
            # 이 goal 사이에 섀시가 움직여도(사용자 지시 - 스캔 후 시작 위치로 복귀) 여기서
            # "지금(goal 시작 시점)" anchor로 다시 변환할 필요가 없다(오히려 하면 틀린다 -
            # anchor가 scan 시점이 아니라 지금 시점이므로). anchor json이 없는 구버전
            # fixture/더미 서버는 detected_pose가 여전히 base_link 프레임이라(header.
            # frame_id 비어있음) 기존 방식대로 지금 anchor로 변환한다.
            pick_center_source = (
                source_box.detected_pose.position.x,
                source_box.detected_pose.position.y,
                source_box.detected_pose.position.z,
            )
            if source_box.header.frame_id == 'world':
                pick_center_world = pick_center_source
                source_box_yaw = source_box.yaw - anchor[1]
            else:
                pick_center_world = base_frame_to_world(anchor, pick_center_source)
                source_box_yaw = source_box.yaw
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
            place_quat = down_quat_with_yaw(target_yaw - source_box_yaw)

            # 100.py가 카트 접근 주행 전에 항상 먼저 하던 것(2261-2262행) - 팔을
            # 안전 접힘 자세로 접고 리프트를 도킹 높이로 낮춘 채로 주행을 시작한다.
            # 이 goal 시작 시점의 팔/리프트 상태가 뭐였든(이전 goal이 남긴 자세 등)
            # 주행 자체는 항상 이 안전 자세에서 시작하게 만든다.
            self._feedback(goal_handle, 'APPROACH_CART', 0.0)
            success, message = self._platform.enter_transport_pose()
            if not success:
                raise PickPlaceAborted(error_codes.ROBOT_TIMEOUT, f'APPROACH_CART 진입 준비 실패: {message}')
            self.get_logger().info(f'APPROACH_CART 진입 준비(안전 자세): {message}')

            # --- APPROACH_CART: 카트 옆 standoff(카트 중심 기준 고정 지점)까지 주행 ---
            # cart_clear_x/cart_center_xy/cart_standoff_dist(실제 카트 geometry)를 아는
            # 3단계 버전을 쓴다 - TRANSPORT(차량 쪽, 아직 이 정보 없음)는 여전히 일반 2단계.
            self._run_drive_to_cart_standoff(goal_handle, 'APPROACH_CART', 0.03)

            # 100.py pick_raise_and_aim() 그대로 - 카트 standoff 도착 직후, Cartesian
            # 호버로 넘어가기 전에 리프트를 PICK_LIFT_H로 올리며 접고, 순수 관절공간
            # 조준(joint_1)으로 박스 방향을 먼저 맞춘다. RMPflow에게 처음부터 "지금
            # 접힌 자세 -> 박스 위 호버"라는 큰 목표를 한 번에 주지 않기 위함(이
            # 프로젝트에서 반복 확인된 "큰 목표를 한 번에 주면 이상한 해로 튄다"는
            # 교훈 그대로) - 이후 APPROACH_PICK의 Cartesian 이동은 이미 대략 조준된
            # 자세에서 시작해 훨씬 작은 목표만 풀면 된다.
            self._feedback(goal_handle, 'APPROACH_PICK', 0.08)
            self._platform.set_pick_aim_target(pick_center_world[:2])
            time.sleep(float(self.get_parameter('target_settle_sec').value))
            success, message = self._platform.pick_raise_and_aim()
            if not success:
                raise PickPlaceAborted(error_codes.ROBOT_TIMEOUT, f'PICK 리프트+조준 실패: {message}')
            self.get_logger().info(f'PICK 리프트+조준: {message}')

            # --- PICK ---
            self._run_move(
                goal_handle, 'APPROACH_PICK', 0.10, pick_hover, pick_quat, clearance * 0.5)
            self._run_move(goal_handle, 'DESCEND_PICK', 0.20, pick_top, pick_quat, pick_tol)

            self._feedback(goal_handle, 'GRIP', 0.30)
            # pick_top이 이미 이 goal의 anchor로 base-frame detected_pose를 world로
            # 바꾼 값이라(위 pick_center_world 계산 참고), platform_controller_node.py는
            # 이 world pose만으로(어떤 box_id/USD prim인지 몰라도) 실제 대상을 스스로
            # 찾는다 - 상세 설계는 platform_controller_node.py의 _find_nearest_box_prim()/
            # _on_gripper_set_target() 참고(사용자 지시로 설계/구현).
            self._platform.set_gripper_target(pick_top)
            time.sleep(float(self.get_parameter('target_settle_sec').value))
            success, message = self._platform.gripper_activate()
            if not success:
                raise PickPlaceAborted(error_codes.GRASP_FAILED, f'흡착 실패: {message}')
            self.get_logger().info(f'GRIP: {message}')

            self._run_move(goal_handle, 'LIFT', 0.40, pick_hover, pick_quat, pick_tol)

            lift_delta = float(self.get_parameter('lift_delta_m').value)
            if lift_delta:
                self._platform.set_lift_height((self._platform.lift_height or 0.0) + lift_delta)

            # 100.py STAGE0.5(안전 운송 자세) 그대로 - 박스를 든 채로 주행하기 전에
            # 팔을 접고 리프트를 내린다(둘 다 platform_controller_node 안에서 관절/
            # 리프트 보간으로 실행됨 - _on_enter_transport_pose 참고). 예전 버전은
            # 여기서 move_tip_to로 place_quat 방향만 미리 돌려놓고 팔은 pick_hover에
            # 편 채로 그대로 주행했다 - 100.py가 이 단계를 굳이 만든 이유(주행 중
            # 펴진 팔이 카트/장애물과 부딪히는 것 방지)를 그대로 건너뛴 셈이었고,
            # 실제로 사용자가 GUI로 pick-place 시도 중 충돌성 불안정을 확인했다.
            self._feedback(goal_handle, 'SAFE_RETRACT', 0.50)
            success, message = self._platform.enter_transport_pose()
            if not success:
                raise PickPlaceAborted(error_codes.ROBOT_TIMEOUT, f'SAFE_RETRACT 실패: {message}')
            self.get_logger().info(f'SAFE_RETRACT: {message}')

            # --- TRANSPORT: 박스를 든 채로(팔 접힘+리프트 하강 상태로) 트렁크 근처까지 주행 ---
            trunk_standoff_m = float(self.get_parameter('trunk_standoff_m').value)
            self._run_drive_to_standoff(
                goal_handle, 'TRANSPORT', 0.65, place_center_world[:2], trunk_standoff_m)

            # 100.py가 STAGE1 진입 전 리프트를 다시 올려두던 것과 동일 - 팔은 접힌
            # 채로 리프트만 재상승한다. 그 다음 APPROACH_PLACE의 move_tip_to(Cartesian)가
            # 이 접힌 자세에서 place_hover로 RMPflow가 자연스럽게 풀려나가게 한다.
            success, message = self._platform.exit_transport_pose()
            if not success:
                raise PickPlaceAborted(error_codes.ROBOT_TIMEOUT, f'TRANSPORT 후 리프트 재상승 실패: {message}')
            self.get_logger().info(f'TRANSPORT 후 리프트 재상승: {message}')

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
