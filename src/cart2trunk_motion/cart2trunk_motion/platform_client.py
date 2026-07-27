"""cart2trunk_platform(platform_controller_node)의 표준 ROS2 인터페이스를 감싸는
얇은 클라이언트 - execute_pick_place_action_server가 이걸 통해 리프트/모바일
베이스/그리퍼/M0609를 조합한다.

platform_controller_node는 Isaac Sim 번들 파이썬(3.11)에서 자체 rclpy 빌드로
도는 별도 프로세스다(HANDOFF_MSI2.md, HANDOFF_2026-07-26.md 4.3절 참고) -
이 노드(시스템 파이썬 3.10)와는 DDS로만 통신하고, 커스텀 인터페이스는 전혀
쓰지 않는다(표준 std_msgs/geometry_msgs/std_srvs만).

[스레딩] 이 클라이언트는 구독 콜백이 백그라운드에서 계속 상태를 갱신해줘야
폐루프 제어 루프(drive_to/move_link6_smooth)가 최신 값을 볼 수 있다 - 노드를
MultiThreadedExecutor(HANDOFF_MSI2.md 6.5절)로 돌리고, 제어 루프는 그 executor의
한 스레드 안에서 time.sleep()으로 도는 방식을 전제로 짰다(mission_coordinator_node와
동일한 이유 - Future.result()가 논블로킹이라 done_callback + threading.Event로
실제 완료를 기다린다).
"""
import threading

from geometry_msgs.msg import Pose, PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Trigger

from cart2trunk_common.geometry import z_yaw_from_quaternion

Point3 = tuple  # (x, y, z)


def _pose_position(pose: Pose) -> Point3:
    return (pose.position.x, pose.position.y, pose.position.z)


def _pose_quat_xyzw(pose: Pose):
    o = pose.orientation
    return (o.x, o.y, o.z, o.w)


class PlatformClient:
    """/lift, /mobile_base, /gripper, /m0609, /gripper/tip_pose 전부 감싼다."""

    def __init__(self, node: Node, callback_group):
        self._node = node
        self._lock = threading.Lock()

        self._lift_h = None
        self._base_pose = None  # (pos_xyz, quat_xyzw)
        self._gripper_closed = None
        self._m0609_pose = None  # (pos_xyz, quat_xyzw)
        self._tip_pose = None  # (pos_xyz, quat_xyzw)

        self._lift_move_pub = node.create_publisher(Float32, '/lift/move', 10)
        node.create_subscription(
            Float32, '/lift/state', self._on_lift_state, 10, callback_group=callback_group)

        self._cmd_vel_pub = node.create_publisher(Twist, '/mobile_base/cmd_vel', 10)
        node.create_subscription(
            PoseStamped, '/mobile_base/state', self._on_base_state, 10,
            callback_group=callback_group)

        self._gripper_activate_client = node.create_client(
            Trigger, '/gripper/activate', callback_group=callback_group)
        self._gripper_release_client = node.create_client(
            Trigger, '/gripper/release', callback_group=callback_group)
        node.create_subscription(
            Bool, '/gripper/state', self._on_gripper_state, 10, callback_group=callback_group)
        self._gripper_set_target_pub = node.create_publisher(
            PoseStamped, '/gripper/set_target', 10)

        self._m0609_move_pub = node.create_publisher(PoseStamped, '/m0609/move_to_pose', 10)
        node.create_subscription(
            PoseStamped, '/m0609/state', self._on_m0609_state, 10, callback_group=callback_group)
        node.create_subscription(
            PoseStamped, '/gripper/tip_pose', self._on_tip_pose, 10, callback_group=callback_group)

    # ---------------------------------------------------------------- #
    # 상태 콜백 (다른 executor 스레드에서 계속 실행됨)
    # ---------------------------------------------------------------- #

    def _on_lift_state(self, msg: Float32) -> None:
        with self._lock:
            self._lift_h = msg.data

    def _on_base_state(self, msg: PoseStamped) -> None:
        with self._lock:
            self._base_pose = (_pose_position(msg.pose), _pose_quat_xyzw(msg.pose))

    def _on_gripper_state(self, msg: Bool) -> None:
        with self._lock:
            self._gripper_closed = msg.data

    def _on_m0609_state(self, msg: PoseStamped) -> None:
        with self._lock:
            self._m0609_pose = (_pose_position(msg.pose), _pose_quat_xyzw(msg.pose))

    def _on_tip_pose(self, msg: PoseStamped) -> None:
        with self._lock:
            self._tip_pose = (_pose_position(msg.pose), _pose_quat_xyzw(msg.pose))

    # ---------------------------------------------------------------- #
    # 조회
    # ---------------------------------------------------------------- #

    @property
    def lift_height(self):
        with self._lock:
            return self._lift_h

    @property
    def base_pose(self):
        """(position_xyz, quat_xyzw) - world frame."""
        with self._lock:
            return self._base_pose

    @property
    def base_yaw(self):
        pose = self.base_pose
        return None if pose is None else z_yaw_from_quaternion(pose[1])

    @property
    def gripper_closed(self):
        with self._lock:
            return self._gripper_closed

    @property
    def m0609_pose(self):
        """(position_xyz, quat_xyzw) - link_6 world pose."""
        with self._lock:
            return self._m0609_pose

    @property
    def tip_pose(self):
        """(position_xyz, quat_xyzw) - 흡착 팁 world pose."""
        with self._lock:
            return self._tip_pose

    def wait_until_ready(self, timeout_sec: float = 10.0) -> bool:
        """네 상태 토픽이 전부 최소 한 번은 들어올 때까지 기다린다(플랫폼 노드가
        아직 안 떴거나 토픽 이름이 안 맞으면 여기서 타임아웃)."""
        import time
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            with self._lock:
                required = (
                    self._lift_h, self._base_pose, self._gripper_closed,
                    self._m0609_pose, self._tip_pose,
                )
                if None not in required:
                    return True
            time.sleep(0.05)
        return False

    # ---------------------------------------------------------------- #
    # 명령
    # ---------------------------------------------------------------- #

    def set_lift_height(self, h: float) -> None:
        msg = Float32()
        msg.data = float(h)
        self._lift_move_pub.publish(msg)

    def set_cmd_vel(self, vx: float, vy: float, wz: float) -> None:
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self._cmd_vel_pub.publish(msg)

    def move_m0609_to(self, position_xyz, quat_xyzw) -> None:
        msg = PoseStamped()
        msg.header.frame_id = 'world'
        px, py, pz = (float(v) for v in position_xyz)
        msg.pose.position.x = px
        msg.pose.position.y = py
        msg.pose.position.z = pz
        x, y, z, w = (float(v) for v in quat_xyzw)
        msg.pose.orientation.x = x
        msg.pose.orientation.y = y
        msg.pose.orientation.z = z
        msg.pose.orientation.w = w
        self._m0609_move_pub.publish(msg)

    @staticmethod
    def _wait_for_future(future, timeout_sec: float):
        """HANDOFF_MSI2.md 6.5절 - Future.result()는 논블로킹이라 done_callback +
        threading.Event로 실제 완료를 기다려야 한다."""
        event = threading.Event()
        future.add_done_callback(lambda _f: event.set())
        if not event.wait(timeout_sec):
            return None
        return future.result()

    def _call_trigger(self, client, timeout_sec: float = 5.0):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return False, f'{client.srv_name} 서비스 없음 - platform_controller_node가 떠 있는지 확인'
        future = client.call_async(Trigger.Request())
        result = self._wait_for_future(future, timeout_sec)
        if result is None:
            return False, f'{client.srv_name} 응답 시간 초과({timeout_sec}s)'
        return result.success, result.message

    def set_gripper_target(self, position_xyz) -> None:
        """/gripper/set_target(world PoseStamped) 발행 - platform_controller_node가
        이 world pose(박스 top-center)에 가장 가까운 실제 박스 prim을 찾아 다음
        gripper_activate() 호출의 흡착 대상으로 삼는다(플랫폼 쪽 상세 설계는
        platform_controller_node.py의 _find_nearest_box_prim() 참고 - 이 노드는
        Isaac Sim 시뮬레이션의 prim/USD 개념을 전혀 몰라도 된다). 방향은 대상
        식별에 쓰이지 않으므로 identity로 둔다."""
        msg = PoseStamped()
        msg.header.frame_id = 'world'
        msg.header.stamp = self._node.get_clock().now().to_msg()
        px, py, pz = (float(v) for v in position_xyz)
        msg.pose.position.x = px
        msg.pose.position.y = py
        msg.pose.position.z = pz
        msg.pose.orientation.w = 1.0
        self._gripper_set_target_pub.publish(msg)

    def gripper_activate(self, timeout_sec: float = 5.0):
        return self._call_trigger(self._gripper_activate_client, timeout_sec)

    def gripper_release(self, timeout_sec: float = 5.0):
        return self._call_trigger(self._gripper_release_client, timeout_sec)
