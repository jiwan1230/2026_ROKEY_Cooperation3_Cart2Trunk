"""EDU 저장소 100.cart_to_trunk_dual_side_holonomic.py의 drive_to()/move_link6_smooth()를
cart2trunk_platform의 ROS2 인터페이스(PlatformClient) 기준으로 이식한 폐루프
제어 루프. HANDOFF_MSI2.md/HANDOFF_2026-07-26.md가 "cart2trunk_motion이 할 일"로
남겨둔 부분이다.

100.py는 Isaac Sim 프로세스 안에서 물리 스텝(world.step())마다 직접 상태를 읽고
명령을 넣었지만, 여기서는 그 상태/명령이 전부 ROS2 토픽을 거친다 - 그래서 "스텝"
대신 고정 주기(hz)의 time.sleep() 루프로 바꿨다. 정체 감지/속도 스무딩의 원리는
그대로지만, 100.py의 상수(STALL_WINDOW=150스텝 @ 60Hz 물리 스텝 등)는 이 루프의
주기(기본 20Hz)에 맞춰 다시 스케일했다 - 실측 튜닝이 아니라 비례 환산이므로 실제
Isaac Sim 연동 후 재튜닝이 필요할 수 있다.
"""
import math
import time
from dataclasses import dataclass

import numpy as np

from cart2trunk_motion.platform_client import PlatformClient

# 100.py의 DOWN_QUAT(= euler_angles_to_quat([0, pi, 0]), Isaac wxyz 관례)를
# ROS geometry_msgs 관례(x,y,z,w)로 환산한 값 - 그리퍼(link_6)가 똑바로 아래를
# 보게 하는 고정 자세. 계산: wxyz=(cos(pi/2),0,sin(pi/2)*[0,1,0]) = (0,0,1,0).
DOWN_QUAT_XYZW = (0.0, 1.0, 0.0, 0.0)


def _quat_mul_xyzw(q1, q2):
    """q1 ⊗ q2 (x,y,z,w 관례) - q2를 먼저 적용한 뒤 q1을 적용한 것과 같다."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def base_frame_to_world(anchor, point_xyz):
    """anchor=(anchor_pos_xyz, anchor_yaw_rad, anchor_lift_h) - 골 실행 시작 시점에
    캡처한 섀시 world pose + 리프트 높이. m0609_base(팔 원점) 프레임은 섀시
    XY평면에서 anchor_yaw만큼 돌아있고, z는 섀시 z + 리프트 높이만큼 떠있다
    (platform_controller_node.set_lift_height()와 동일한 관계) - 그 프레임
    기준 점(point_xyz)을 world로 변환한다."""
    anchor_pos, anchor_yaw, anchor_lift = anchor
    x, y, z = point_xyz
    cos_y, sin_y = math.cos(anchor_yaw), math.sin(anchor_yaw)
    wx = anchor_pos[0] + x * cos_y - y * sin_y
    wy = anchor_pos[1] + x * sin_y + y * cos_y
    wz = anchor_pos[2] + anchor_lift + z
    return (wx, wy, wz)


def compute_standoff(from_xy, target_xy, distance_m: float):
    """target_xy에서 distance_m만큼 떨어진, from_xy가 있는 쪽을 향한 지점과
    그 지점에서 target_xy를 바라보는 yaw(라디안)를 계산한다.

    100.py의 CART_CLEAR_X/CART_BASE_LEFT_XY/CART_BASE_RIGHT_XY처럼 카트/차량
    메시의 정확한 형상(손잡이 위치 등)을 아는 정교한 접근은 아니다 - "지금 있는
    쪽에서 목표를 정면으로 보고 distance_m만큼 떨어져서 선다"는 범용 기본값이라,
    카트를 어느 쪽에서 접근해야 손잡이/장애물과 안 부딪히는지는 구분하지 못한다.
    실제 카트/차량 형상에 맞춘 정교한 접근이 필요해지면 그때 대체할 자리 -
    지금은 "카트/트렁크 쪽으로 실제로 다가간다"는 것 자체를 먼저 동작시킨다."""
    fx, fy = from_xy
    tx, ty = target_xy
    dx, dy = fx - tx, fy - ty
    dist = math.hypot(dx, dy)
    if dist < 1e-6:
        dx, dy, dist = 1.0, 0.0, 1.0
    ux, uy = dx / dist, dy / dist
    standoff_xy = (tx + ux * distance_m, ty + uy * distance_m)
    yaw = math.atan2(ty - standoff_xy[1], tx - standoff_xy[0])
    return standoff_xy, yaw


def down_quat_with_yaw(yaw_rad: float):
    """똑바로 아래를 보는 자세(DOWN_QUAT_XYZW)에 world Z축 기준 yaw_rad만큼 더
    돌린 자세. 픽업 시 yaw=0으로 잡고(흡착은 박스 XY 회전과 무관하게 top면
    중앙만 맞으면 됨), 배치 시 (target_yaw - source_box.yaw)만큼 돌려서
    박스가 실제로 target_yaw로 놓이게 한다(그리퍼에 강체로 붙어있으므로
    wrist를 돌리면 박스도 같이 돈다)."""
    from cart2trunk_common.geometry import quaternion_from_z_yaw
    return _quat_mul_xyzw(quaternion_from_z_yaw(yaw_rad), DOWN_QUAT_XYZW)


@dataclass
class LoopResult:
    success: bool
    detail: str


def drive_to(client: PlatformClient, target_x: float, target_y: float, target_yaw_rad: float,
             tolerance_xy: float = 0.03, tolerance_yaw_rad: float = math.radians(2.0),
             max_speed: float = 0.4, max_wz: float = 0.2, kp_xy: float = 1.8, kp_yaw: float = 0.25,
             hz: float = 20.0, max_seconds: float = 60.0,
             stall_window_sec: float = 2.5, stall_min_progress_m: float = 0.008) -> LoopResult:
    """100.py drive_to()와 동일한 비례제어+정체감지+속도 스무딩. base_pose는 world
    frame(/mobile_base/state)이라 target_x/y/yaw도 world 기준이어야 한다."""
    dt = 1.0 / hz
    smooth_alpha = 0.12
    vx_s = vy_s = wz_s = 0.0

    start = client.base_pose
    if start is None:
        return LoopResult(False, 'base_pose 없음 - platform_controller_node 연결 확인')
    last_check_pos = np.array(start[0][:2], dtype=float)
    last_check_t = time.monotonic()
    deadline = time.monotonic() + max_seconds

    while time.monotonic() < deadline:
        pose = client.base_pose
        pos, _quat = pose
        yaw = client.base_yaw
        ex_w = target_x - pos[0]
        ey_w = target_y - pos[1]
        eyaw = ((target_yaw_rad - yaw + math.pi) % (2 * math.pi)) - math.pi
        if abs(ex_w) < tolerance_xy and abs(ey_w) < tolerance_xy and abs(eyaw) < tolerance_yaw_rad:
            client.set_cmd_vel(0.0, 0.0, 0.0)
            detail = f'도달 (오차 xy=({ex_w:.3f},{ey_w:.3f}) yaw={math.degrees(eyaw):.1f}deg)'
            return LoopResult(True, detail)

        ex_l = ex_w * math.cos(yaw) + ey_w * math.sin(yaw)
        ey_l = -ex_w * math.sin(yaw) + ey_w * math.cos(yaw)
        vx_t = float(np.clip(kp_xy * ex_l, -max_speed, max_speed))
        vy_t = float(np.clip(kp_xy * ey_l, -max_speed, max_speed))
        wz_t = float(np.clip(kp_yaw * eyaw, -max_wz, max_wz))
        vx_s += smooth_alpha * (vx_t - vx_s)
        vy_s += smooth_alpha * (vy_t - vy_s)
        wz_s += smooth_alpha * (wz_t - wz_s)
        client.set_cmd_vel(vx_s, vy_s, wz_s)

        now = time.monotonic()
        if now - last_check_t >= stall_window_sec:
            cur = np.array(pos[:2], dtype=float)
            progress = float(np.linalg.norm(cur - last_check_pos))
            still_far = abs(ex_w) > tolerance_xy or abs(ey_w) > tolerance_xy
            if progress < stall_min_progress_m and still_far:
                client.set_cmd_vel(0.0, 0.0, 0.0)
                detail = f'정체 감지 - {stall_window_sec:.1f}s 동안 {progress:.4f}m밖에 못 움직임'
                return LoopResult(False, detail)
            last_check_pos = cur
            last_check_t = now
        time.sleep(dt)

    client.set_cmd_vel(0.0, 0.0, 0.0)
    return LoopResult(False, f'{max_seconds:.0f}s 안에 목표 도달 못함')


def move_tip_to(
    client: PlatformClient, target_tip_pos, orientation_xyzw=DOWN_QUAT_XYZW,
    tolerance: float = 0.004, max_speed: float = 0.01, kp: float = 3.0,
    hz: float = 30.0, max_seconds: float = 30.0, stall_window_sec: float = 3.0,
    stall_min_improvement_m: float = 0.003,
) -> LoopResult:
    """100.py move_link6_smooth()와 동일한 원리 - link_6이 아니라 흡착 팁(오차
    측정 기준)을 목표로 폐루프 제어한다. link_6 자체에는 target_tip_pos가 아니라
    '현재 link_6 위치 + 팁 오차만큼의 스텝'을 매번 새로 명령한다(100.py와 동일 -
    link_6<->팁 사이 고정 오프셋을 몰라도 되는 방식)."""
    dt = 1.0 / hz
    target = np.array(target_tip_pos, dtype=float)
    last_check_err = None
    last_check_t = time.monotonic()
    deadline = time.monotonic() + max_seconds

    while time.monotonic() < deadline:
        tip_pose = client.tip_pose
        m0609_pose = client.m0609_pose
        if tip_pose is None or m0609_pose is None:
            return LoopResult(False, 'tip_pose/m0609_pose 없음 - platform_controller_node 연결 확인')
        tip_pos = np.array(tip_pose[0], dtype=float)
        err_vec = target - tip_pos
        err = float(np.linalg.norm(err_vec))
        if err < tolerance:
            return LoopResult(True, f'도달 (오차 {err:.4f}m)')

        now = time.monotonic()
        if now - last_check_t >= stall_window_sec:
            if last_check_err is not None and (last_check_err - err) < stall_min_improvement_m:
                detail = f'정체 감지 - 오차 {last_check_err:.4f}m -> {err:.4f}m (자기충돌/한계 의심)'
                return LoopResult(False, detail)
            last_check_err = err
            last_check_t = now

        step_vec = kp * err_vec
        step_norm = float(np.linalg.norm(step_vec))
        if step_norm > max_speed:
            step_vec = step_vec / step_norm * max_speed
        link6_pos = np.array(m0609_pose[0], dtype=float)
        client.move_m0609_to(link6_pos + step_vec, orientation_xyzw)
        time.sleep(dt)

    return LoopResult(False, f'{max_seconds:.0f}s 안에 목표 도달 못함')
