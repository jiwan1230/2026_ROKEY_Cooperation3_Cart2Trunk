"""cart2trunk_simulation의 실제 카트+박스 씬에서, 이제 살아있는 depth 카메라로
진짜 병합 point cloud(m0609_base_link 좌표계, .npy)를 캡처한다 - cart2trunk_perception
box_scan_action_server.py의 fixture_input_path가 지금까지 참조할 실제 씬 캡처가
없어서(예전 EDU 캡처는 이 씬의 박스 배치와 무관함) 픽업 테스트가 GRASP_FAILED로
막혀 있었다(플랫폼 카메라 크래시 수정 후 남은 작업).

[설계 - EDU 저장소 99.cart_scan_dual_side_holonomic.py를 그대로 이식]
처음엔 EE(link_6) 목표 자세를 직접 추측(down_quat_with_yaw)해서 만들었는데,
카메라가 그리퍼 몸체(vgp20 흡착판)에 거의 다 가려졌다(실측 확인 - RGB 스크린샷
전부 그리퍼 몸체가 프레임 대부분을 채움) - 원본 저장소의 99.py를 다시 확인한
결과, 이 방식(임의의 EE 자세 추측)은 99.py 자신도 이미 시도했다가 "발산/엉뚱한
방향을 봄"으로 폐기한 접근이었다(99.py 820행 부근 주석). 대신 99.py가 검증한
방식을 그대로 가져온다 - 카메라가 그리퍼에 고정 마운트돼 있으므로 "지금(임의
관절각) link6 자세"와 "지금 카메라 world 자세"의 상대 오프셋(R_offset,
cam_local_pos_offset)은 관절각과 무관하게 항상 일정하다. 이 오프셋을 한 번만
측정해두면 "카메라가 어디서(eye) 어디를(look_at) 봐야 하는지"만으로 필요한
link6 목표를 역산할 수 있다(lookat_to_link6_target). 리프트를 LIFT_MAX로 올리는
것, EYE_HEIGHT_ABOVE_CART/SCAN_TILT_FROM_VERTICAL_DEG로 eye/look_at을 기하학적으로
배치하는 것, 베이스를 카트 옆(Y) standoff에 세운 채 카트 길이축(X)을 따라
strafe하며 5개 시점을 스캔하는 것, ROI로 카트 벽/철망을 미리 잘라내는 것까지
전부 99.py 값 그대로다 - cart2trunk_simulation의 카트 씬(cart_scene.py)이 이
99.py와 완전히 동일한 카트 스펙(CART_BOX_SPECS, CART_BOX_FRONT_SHIFT_M 등)에서
포팅됐으므로 좌표계/치수가 그대로 들어맞는다.

실행 방법(isaac_python 필요 - platform_controller_node.py를 그대로 import해서
씬 부트스트랩 전체를 재사용한다). platform_controller_node.py 자신이 colcon
설치본이 아니라 파일 상대경로로 직접 실행되는 스크립트이므로(그 파일 상단
주석 참고 - Isaac 번들 파이썬은 3.11이라 python3.10용 colcon 설치본을 못
찾는다), 이 스크립트도 똑같이 raw 스크립트로 실행한다(-m 모듈 실행 아님).
platform_controller_node.py 자신의 실행 안내(모듈 docstring)에 있는 대로
LD_LIBRARY_PATH도 셸에서 미리 export해야 한다(간헐적 rclpy dlopen 실패 회피):
    export LD_LIBRARY_PATH=/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/\
release/exts/isaacsim.ros2.bridge/humble/lib:$LD_LIBRARY_PATH
    HEADLESS=1 CART2TRUNK_ENABLE_CAMERA_BRIDGE=1 isaac_python \
        src/cart2trunk_simulation/cart2trunk_simulation/tools/capture_cart_scan.py

m0609_base_link 좌표계 변환은 cart2trunk_perception.core.trunk_map_builder의
world_to_base()/quat_wxyz_to_matrix()와 동일한 공식이다(그 모듈은 open3d
의존이라 Isaac 번들 파이썬에 없어 직접 import는 못 하므로 공식만 그대로
가져왔다 - 결과 좌표계가 어긋나면 안 되므로 두 구현이 어긋나지 않게 유지할 것).
"""
import json
import os
import sys
from pathlib import Path

os.environ.setdefault("HEADLESS", "1")
os.environ.setdefault("CART2TRUNK_ENABLE_CAMERA_BRIDGE", "1")

# platform_controller_node.py와 같은 이유(모듈 docstring 참고) - colcon 설치본이
# 아니라 소스 트리 파일 경로로 직접 잡는다.
_SRC_DIR = Path(__file__).resolve().parents[3]
_PLATFORM_SRC = _SRC_DIR / "cart2trunk_platform" / "cart2trunk_platform"
if str(_PLATFORM_SRC) not in sys.path:
    sys.path.insert(0, str(_PLATFORM_SRC))

import numpy as np  # noqa: E402

import platform_controller_node as pcn  # noqa: E402  (SimulationApp 인스턴스화가 여기서 일어남)
from isaacsim.core.utils.rotations import quat_to_euler_angles  # noqa: E402

OUT_DIR = Path(os.environ.get("CART2TRUNK_CART_SCAN_OUT_DIR", "/tmp/cart2trunk_cart_scan"))

# ---- 99.cart_scan_dual_side_holonomic.py 값 그대로(모듈 docstring 참고) ----
STANDOFF_MARGIN = 0.10
POST_CONVERGENCE_SETTLE_STEPS = 90
WAYPOINT_STEPS_FIRST = 350
WAYPOINT_STEPS_LATER = 90
CONVERGENCE_CHECK_INTERVAL_STEPS = 25
CONVERGENCE_MIN_STEPS = 75
CONVERGENCE_PLATEAU_TOLERANCE_M = 0.001
CART_BASKET_FLOOR_Z = 0.68  # cart_scene.py의 CART_BASKET_FLOOR_Z와 동일 - 두 값이 어긋나면 안 됨.
EYE_HEIGHT_ABOVE_CART = 0.55
SCAN_TILT_FROM_VERTICAL_DEG = 30.0
_SCAN_HORIZONTAL_OFFSET = EYE_HEIGHT_ABOVE_CART * np.tan(np.radians(SCAN_TILT_FROM_VERTICAL_DEG))
CART_SCAN_STRAFE_X_OFFSETS = [-0.28, -0.14, 0.0, 0.14, 0.28]
CART_SCAN_ROI_MAX_HEIGHT_M = 0.40
CART_SCAN_ROI_HALF_X_M = 0.29
CART_SCAN_ROI_HALF_Y_M = 0.22
# cart_scene.py의 CART_BOX_FRONT_SHIFT_M과 동일값 - 그쪽이 module-level 상수라
# import해서 쓸 수도 있지만(둘 다 순수 pxr/numpy라 인터프리터 문제는 없음), 99.py
# 자신도 이 값을 로컬 상수로 다시 정의해 썼던 전례를 그대로 따른다.
CART_SCAN_ROI_CENTER_X_SHIFT_M = 0.07
WORLD_UP = np.array([0.0, 0.0, 1.0])


def quat_wxyz_to_matrix(q) -> np.ndarray:
    """trunk_map_builder.quat_wxyz_to_matrix() / 99.py와 동일 공식 - 모듈 docstring 참고."""
    w, x, y, z = q
    n = w * w + x * x + y * y + z * z
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    wx, wy, wz = s * w * x, s * w * y, s * w * z
    xx, xy, xz = s * x * x, s * x * y, s * x * z
    yy, yz, zz = s * y * y, s * y * z, s * z * z
    return np.array([
        [1 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1 - (xx + yy)],
    ])


def rot_matrix_to_quat_wxyz(rot: np.ndarray) -> np.ndarray:
    """quat_wxyz_to_matrix()의 역변환(표준 Shepperd 방법) - 99.py는
    isaacsim.core.utils.numpy.rotations.rot_matrices_to_quats()를 쓰지만, 그 모듈이
    필요로 하는 warp/torch 의존까지 끌어오지 않으려고 순수 numpy로 직접 짠다."""
    m = np.asarray(rot, dtype=float)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return np.array([w, x, y, z], dtype=float)


def world_to_base(
    points_world: np.ndarray, base_pos: np.ndarray, base_quat_wxyz: np.ndarray,
) -> np.ndarray:
    """trunk_map_builder.world_to_base()와 동일 공식(모듈 docstring 참고) -
    p_base = R_base^T @ (p_world - base_pos)."""
    r_base = quat_wxyz_to_matrix(base_quat_wxyz)
    return (points_world - base_pos[None, :]) @ r_base


def make_usd_camera_rotation(
    eye: np.ndarray, look_at: np.ndarray, up_ref: np.ndarray = WORLD_UP,
) -> np.ndarray:
    """99.cart_scan_dual_side_holonomic.py의 make_usd_camera_rotation()과 동일 -
    USD 카메라 축(+Y up, -Z forward)에 맞는 world rotation matrix."""
    forward = look_at - eye
    forward = forward / np.linalg.norm(forward)
    up_ref = up_ref / np.linalg.norm(up_ref)
    if abs(float(np.dot(forward, up_ref))) > 0.97:
        up_ref = np.array([0.0, 1.0, 0.0])
        if abs(float(np.dot(forward, up_ref))) > 0.97:
            up_ref = np.array([1.0, 0.0, 0.0])
    right = np.cross(forward, up_ref)
    right = right / np.linalg.norm(right)
    backward = -forward
    camera_up = np.cross(backward, right)
    camera_up = camera_up / np.linalg.norm(camera_up)
    return np.column_stack((right, camera_up, backward))


_SMOOTH_ALPHA = 0.12  # 99.py SMOOTH_ALPHA와 동일값.
_smooth_state = {"vx": 0.0, "vy": 0.0, "wz": 0.0}


def drive_to(
    target_x: float = None, target_y: float = None, target_yaw_deg: float = None,
    tolerance_xy: float = 0.03, tolerance_yaw_deg: float = 2.0,
    max_speed: float = 0.4, max_wz: float = 0.2, kp_xy: float = 1.8, kp_yaw: float = 0.25,
    max_steps: int = 3000, label: str = "",
):
    """99.py의 drive_to()를 그대로 이식(764행) - 이전 버전(drive_base_to_xy)은 yaw
    목표를 아예 다루지 않아 99.py와 다른 함수였다. 그게 실제 버그였다 - BASE_FACE_ROT_Z=
    90도 고정이라 섀시는 항상 yaw=90도(긴 길이축이 Y를 향함)로 스폰되는데,
    SCAN_STANDOFF_XY의 Y-clearance(standoff_y)는 "도착 후 yaw=0/180(짧은 폭축이
    Y를 향함)"이라는 전제로 계산돼 있다(99.py 670행 주석) - yaw를 스폰값 그대로
    유지한 채 위치만 standoff로 옮기면, 실제 카트와의 간격이 formula가 가정한
    것보다 좁아져(긴 축이 노출되므로) 카트에 닿거나 매우 가까워진다(사용자가 GUI로
    직접 확인). 회전은 반드시 안전지대(X=CART_CLEAR_X, 카트와 X축으로만 분리된 상태)
    안에서 끝내야 한다(99.py 953행 주석, 100.py 2267행 주석과 동일 원리) - 이 함수는
    xy와 yaw를 동시에 폐루프로 수렴시키므로, 호출부가 "회전이 안전한 구간"에서만
    target_yaw_deg를 바꾸도록 단계를 나눠 호출해야 한다(아래 main() 참고)."""
    start_pos, start_quat = pcn.base_robot.get_world_pose()
    start_yaw = float(np.degrees(quat_to_euler_angles(start_quat)[2]))
    tx = target_x if target_x is not None else float(start_pos[0])
    ty = target_y if target_y is not None else float(start_pos[1])
    tyaw = target_yaw_deg if target_yaw_deg is not None else start_yaw
    print(f"[주행 시작]{' ' + label if label else ''} 목표=({tx:.3f},{ty:.3f},{tyaw:.1f}deg)", flush=True)

    stall_window, stall_min_progress = 150, 0.008
    last_check_pos = np.array([float(start_pos[0]), float(start_pos[1])])
    stalled = False
    step = 0
    for step in range(1, max_steps + 1):
        pos, quat = pcn.base_robot.get_world_pose()
        yaw_deg = float(np.degrees(quat_to_euler_angles(quat)[2]))
        ex_w, ey_w = tx - float(pos[0]), ty - float(pos[1])
        eyaw = ((tyaw - yaw_deg + 180) % 360) - 180
        if abs(ex_w) < tolerance_xy and abs(ey_w) < tolerance_xy and abs(eyaw) < tolerance_yaw_deg:
            break
        yaw_rad = np.radians(yaw_deg)
        ex_l = ex_w * np.cos(yaw_rad) + ey_w * np.sin(yaw_rad)
        ey_l = -ex_w * np.sin(yaw_rad) + ey_w * np.cos(yaw_rad)
        vx_t = float(np.clip(kp_xy * ex_l, -max_speed, max_speed))
        vy_t = float(np.clip(kp_xy * ey_l, -max_speed, max_speed))
        wz_t = float(np.clip(np.radians(kp_yaw * eyaw), -max_wz, max_wz))
        _smooth_state["vx"] += _SMOOTH_ALPHA * (vx_t - _smooth_state["vx"])
        _smooth_state["vy"] += _SMOOTH_ALPHA * (vy_t - _smooth_state["vy"])
        _smooth_state["wz"] += _SMOOTH_ALPHA * (wz_t - _smooth_state["wz"])
        pcn.base_robot.apply_action(
            pcn.holo_forward(_smooth_state["vx"], _smooth_state["vy"], _smooth_state["wz"]))
        step_hold(1)
        if step % stall_window == 0:
            cur = np.array([float(pos[0]), float(pos[1])])
            progress = float(np.linalg.norm(cur - last_check_pos))
            if progress < stall_min_progress and (abs(ex_w) > tolerance_xy or abs(ey_w) > tolerance_xy):
                stalled = True
                print(f"  [정체 감지] {progress:.4f}m밖에 못 움직임 - 중단", flush=True)
                break
            last_check_pos = cur
    for _ in range(30):
        _smooth_state["vx"] *= 1 - _SMOOTH_ALPHA
        _smooth_state["vy"] *= 1 - _SMOOTH_ALPHA
        _smooth_state["wz"] *= 1 - _SMOOTH_ALPHA
        pcn.base_robot.apply_action(
            pcn.holo_forward(_smooth_state["vx"], _smooth_state["vy"], _smooth_state["wz"]))
        step_hold(1)
    final_pos, final_quat = pcn.base_robot.get_world_pose()
    final_yaw = float(np.degrees(quat_to_euler_angles(final_quat)[2]))
    print(f"[주행 완료]{' ' + label if label else ''} {step}스텝, 최종=({final_pos[0]:.3f},"
          f"{final_pos[1]:.3f},{final_yaw:.1f}deg) 정체={stalled}", flush=True)
    return final_pos, final_yaw, not stalled


def step_hold(n: int = 1) -> None:
    """99.py의 step_hold()와 동일 - set_lift_height()를 매 스텝 호출해야 M0609가
    (독립 articulation이라) 중력에 노출되지 않고 리프트 높이에 계속 붙들려 있다.
    실측으로 찾은 버그: 이전 버전의 move_ee_to()/capture_viewpoint()가 이 호출을
    빼먹어서 EE 이동 내내 M0609가 리프트 높이를 못 지켰다."""
    for _ in range(n):
        pcn.set_lift_height(pcn.lift_state["h"])
        pcn.world.step(render=True)


def move_link6(
    target_pos: np.ndarray, target_quat_wxyz: np.ndarray, steps: int, label: str = "",
) -> float:
    """99.py의 move_link6()과 동일 - plateau 조기종료 포함(중요, 이전에는 생략했다가
    발견한 버그: 조기종료 없이 고정 스텝을 다 채우면(특히 스텝을 90->350으로
    늘렸을 때) 오차가 줄기는커녕 오히려 더 커졌다(실측: 시점1 err 0.69m -> 0.94m,
    시점4 0.37m -> 1.12m - 스텝을 더 줄수록 단조적으로 나빠짐). 이건 "시간
    부족"이 아니라 "이미 도달한 뒤에도 controller.forward()를 계속 부르면
    서서히 발산하는 불안정성"이라는 신호였다 - 99.py가 plateau 감지 즉시
    멈추는 이유가 바로 이거였다(단순 속도 최적화가 아니라 정확성 안전장치).
    사용자가 직접 99.py를 돌려 5개 시점 전부 정상 촬영되는 걸 확인해준 뒤에야
    이 차이를 알아챘다."""
    last_check_pos = None
    steps_run = 0
    for step in range(steps):
        pcn.sync_rmp_base()
        actions = pcn.controller.forward(
            target_end_effector_position=target_pos,
            target_end_effector_orientation=target_quat_wxyz,
        )
        pcn.m0609_robot.apply_action(actions)
        step_hold(1)
        steps_run += 1

        if step + 1 < CONVERGENCE_MIN_STEPS:
            continue
        if (step + 1) % CONVERGENCE_CHECK_INTERVAL_STEPS != 0:
            continue
        current_pos, _ = pcn._ee_world_pose()
        if last_check_pos is not None:
            movement = float(np.linalg.norm(current_pos - last_check_pos))
            if movement < CONVERGENCE_PLATEAU_TOLERANCE_M:
                break
        last_check_pos = current_pos

    ee_pos, _ee_quat = pcn._ee_world_pose()
    err = float(np.linalg.norm(ee_pos - target_pos))
    print(f"[웨이포인트{' ' + label if label else ''}] {steps_run}/{steps}스텝 "
          f"target={np.round(target_pos, 3)} ee={np.round(ee_pos, 3)} err={err:.4f}m", flush=True)
    return err


ERR_RETRY_THRESHOLD_M = 0.08  # 99.py의 "5cm 넘으면 경고" 기준보다 살짝 여유 - 이 이상이면 재시도.
MAX_WAYPOINT_ATTEMPTS = 3


def fold_to_known_pose(steps: int = 150) -> None:
    """실측으로 찾은 문제(사용자가 99.py를 직접 돌려 5/5 정상 촬영 확인해준 뒤 재현
    비교로 확인) - 이 프로젝트 환경에서는 RMPflow가 특정 시점 전환에서 카오스적으로
    민감하다(실측: plateau 조기종료 코드처럼 시뮬레이션에 영향 없어야 할 코드를
    추가하기만 해도(순수 읽기 전용 _ee_world_pose() 호출 하나) 같은 물리 스텝
    시퀀스인데도 최종 수렴 결과가 달라졌다 - GPU 물리 솔버의 부동소수점 비결정성이
    이 시스템의 불안정 영역에서 증폭되는 것으로 보인다). 결정론적 "근본 원인 하나"를
    찾기보다, 실패를 감지하면 관절 공간에서 알려진 자세(joint_3=joint_5=90도)로
    리셋한 뒤 다시 시도하는 편이 실용적이다 - 100.py의 raise_lift_and_fold()와
    같은 자세를 쓰지만 여기서는 "충돌 회피용 다른 solution branch"가 아니라
    "RMPflow가 갇힌 국소 상태에서 확실히 빠져나오는 리셋" 목적으로 쓴다."""
    fold_target = np.zeros(pcn.m0609_robot.num_dof)
    if "joint_3" in pcn.m0609_robot.dof_names:
        fold_target[pcn.m0609_robot.dof_names.index("joint_3")] = np.pi / 2
    if "joint_5" in pcn.m0609_robot.dof_names:
        fold_target[pcn.m0609_robot.dof_names.index("joint_5")] = np.pi / 2
    start_joints = np.array(pcn.m0609_robot.get_joint_positions(), dtype=float)
    for i in range(steps):
        alpha = (i + 1) / steps
        j = start_joints + (fold_target - start_joints) * alpha
        pcn.m0609_robot.apply_action(pcn.ArticulationAction(joint_positions=j))
        step_hold(1)


def move_link6_with_retry(
    target_pos: np.ndarray, target_quat_wxyz: np.ndarray, steps: int, label: str = "",
) -> float:
    """move_link6()을 시도하고, 오차가 ERR_RETRY_THRESHOLD_M를 넘으면(RMPflow가 이
    시점에서 국소 상태에 갇힌 것으로 보고) fold_to_known_pose()로 리셋 후 재시도한다
    (최대 MAX_WAYPOINT_ATTEMPTS회). 시도들 중 가장 오차가 작았던 결과를 채택한다."""
    best_err = None
    for attempt in range(MAX_WAYPOINT_ATTEMPTS):
        attempt_label = f"{label} 시도{attempt + 1}" if attempt > 0 else label
        # 리셋 직후(시도 2회차부터)는 시점0과 마찬가지로 관절이 크게 바뀐 뒤라
        # 짧은 steps(later용 90)로는 부족할 수 있어, 첫 시도급 예산을 준다.
        attempt_steps = steps if attempt == 0 else WAYPOINT_STEPS_FIRST
        err = move_link6(target_pos, target_quat_wxyz, steps=attempt_steps, label=attempt_label)
        if best_err is None or err < best_err:
            best_err = err
        if err <= ERR_RETRY_THRESHOLD_M:
            return err
        if attempt + 1 < MAX_WAYPOINT_ATTEMPTS:
            print(f"[재시도]{' ' + label if label else ''} 오차 {err:.4f}m > "
                  f"{ERR_RETRY_THRESHOLD_M}m - 알려진 자세로 리셋 후 재시도", flush=True)
            fold_to_known_pose()
    return best_err


def save_debug_rgb(label: str) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        rgb = pcn.depth_camera.get_rgba()[:, :, :3]
        OUT_DIR.mkdir(parents=True, exist_ok=True)
        plt.imsave(str(OUT_DIR / f"_view_{label}.png"), rgb)
    except Exception as exc:  # noqa: BLE001 - 디버그 스크린샷은 실패해도 캡처 자체는 계속
        print(f"[CAPTURE] {label}: RGB 스크린샷 저장 실패({exc})", flush=True)


def main() -> None:
    print("[CAPTURE] 물리 정착 대기...", flush=True)
    step_hold(120)

    for prim_path, box in pcn.cart_scene.box_objects.items():
        pos, _quat = box.get_world_pose()
        print(f"[GROUND TRUTH] {prim_path}: world_pos={np.round(pos, 3)}", flush=True)

    cart_center_xy = np.asarray(pcn.cart_scene.cart_center_xy, dtype=float)
    cart_half_y = float(pcn.cart_scene.cart_half_y)
    # 99.py STANDOFF_Y와 동일 공식 - CHASSIS_HALF_WIDTH_EFFECTIVE(바퀴/롤러 돌출부
    # 포함, platform_controller_node.py의 CART_STANDOFF_DIST와 같은 상수)를 써야
    # 한다. BASE_WIDTH/2.0(차체 폭만, 롤러 돌출 미포함)를 썼던 이전 버전은 99.py
    # 보다 카트에 약 8cm 더 가깝게 섰다(사용자 지적 - 실측: BASE_WIDTH/2≈0.250m
    # vs CHASSIS_HALF_WIDTH_EFFECTIVE≈0.329m) - 메카넘 휠/롤러가 카트에 그만큼
    # 가까워지고 카메라 목표 거리/각도도 99.py 검증값과 달라진다.
    chassis_half_width = pcn.CHASSIS_HALF_WIDTH_EFFECTIVE
    standoff_y = chassis_half_width + cart_half_y + STANDOFF_MARGIN
    scan_standoff_xy = np.array([cart_center_xy[0], cart_center_xy[1] - standoff_y])
    print(f"[CAPTURE] 카트 center_xy={np.round(cart_center_xy, 3)} half_y={cart_half_y:.3f} "
          f"-> standoff_xy={np.round(scan_standoff_xy, 3)}", flush=True)

    print(f"[CAPTURE] 리프트 최고 높이로: {pcn.LIFT_MIN:.3f} -> {pcn.LIFT_MAX:.3f}", flush=True)
    for i in range(120):
        h = pcn.LIFT_MIN + (pcn.LIFT_MAX - pcn.LIFT_MIN) * (i + 1) / 120
        pcn.lift_state["h"] = h
        step_hold(1)

    # [99.py 953-966행 그대로 이식] BASE_FACE_ROT_Z=90도 고정이라 섀시는 항상
    # yaw=90도(긴 길이축이 Y를 향함)로 스폰된다. scan_standoff_xy의 Y-clearance
    # (standoff_y)는 chassis_half_width로 계산했으므로 "도착 후 yaw=0도(짧은
    # 폭축이 Y를 향함)"이 전제다 - yaw를 스폰값 그대로 둔 채 위치만 옮기면 실제
    # 카트와의 간격이 formula보다 좁아진다(사용자가 GUI로 직접 확인). 그래서 99.py는
    # 회전을 "카트와 X축으로만 분리된 안전지대(X=CART_CLEAR_X)" 안에서 끝낸다 - 3단계:
    #   1단계 - X만 CART_CLEAR_X로(Y/yaw는 스폰값 유지, 회전 없음 - 이미 그 자리라 사실상 대기)
    #   2단계 - X는 CART_CLEAR_X 고정(카트와 X축 분리 유지)한 채 Y+yaw를 목표로 동시 수렴
    #   3단계 - Y/yaw는 이미 목표값, X만 좁혀 최종 standoff로 직진(Y축 분리로 안전)
    _spawn_pos, _spawn_quat = pcn.base_robot.get_world_pose()
    _spawn_yaw = float(np.degrees(quat_to_euler_angles(_spawn_quat)[2]))
    _cart_clear_x = float(_spawn_pos[0])
    drive_to(target_x=_cart_clear_x, target_y=float(_spawn_pos[1]), target_yaw_deg=_spawn_yaw,
             label="카트 접근 1/3: 회전 안전지대(X만 이동, 회전 없음)")
    drive_to(target_x=_cart_clear_x, target_y=float(scan_standoff_xy[1]), target_yaw_deg=0.0,
             label="카트 접근 2/3: 안전지대에서 standoff Y + yaw=0도로 회전")
    drive_to(target_x=float(scan_standoff_xy[0]), target_y=float(scan_standoff_xy[1]), target_yaw_deg=0.0,
             label="카트 접근 3/3: yaw/Y 고정한 채 X만 좁혀 최종 접근")
    base_pos_reached, _ = pcn.base_robot.get_world_pose()
    print(f"[CAPTURE] 베이스 도착: pos={np.round(base_pos_reached, 3)}", flush=True)

    # ---- 카메라<->link6 고정 오프셋 1회 측정 (99.py와 동일 원리) ----
    step_hold(10)
    link6_pos0, link6_quat0 = pcn._ee_world_pose()
    cam_pos0, cam_quat0 = pcn.depth_camera.get_world_pose(camera_axes="usd")
    r_link6_0 = quat_wxyz_to_matrix(link6_quat0)
    r_cam_0 = quat_wxyz_to_matrix(cam_quat0)
    r_offset = r_link6_0.T @ r_cam_0
    cam_local_pos_offset = r_link6_0.T @ (np.asarray(cam_pos0, dtype=float) - link6_pos0)
    print(f"[오프셋] camera pos offset in link6 frame="
          f"{np.round(cam_local_pos_offset, 4)}", flush=True)

    def lookat_to_link6_target(eye, look_at):
        r_cam_target = make_usd_camera_rotation(
            np.asarray(eye, dtype=float), np.asarray(look_at, dtype=float),
        )
        r_link6_target = r_cam_target @ r_offset.T
        link6_target_pos = np.asarray(eye, dtype=float) - r_link6_target @ cam_local_pos_offset
        return link6_target_pos, rot_matrix_to_quat_wxyz(r_link6_target)

    accumulated_world_points = []
    for i, x_offset in enumerate(CART_SCAN_STRAFE_X_OFFSETS):
        strafe_x = cart_center_xy[0] + x_offset
        drive_to(target_x=float(strafe_x), target_y=float(scan_standoff_xy[1]), target_yaw_deg=0.0,
                 label=f"스캔 위치 {i}로 strafe")

        scan_eye_i = np.array([
            strafe_x, cart_center_xy[1] - _SCAN_HORIZONTAL_OFFSET,
            CART_BASKET_FLOOR_Z + EYE_HEIGHT_ABOVE_CART,
        ])
        scan_look_at_i = np.array([strafe_x, cart_center_xy[1], CART_BASKET_FLOOR_Z])
        target_pos, target_quat = lookat_to_link6_target(scan_eye_i, scan_look_at_i)
        steps = WAYPOINT_STEPS_FIRST if i == 0 else WAYPOINT_STEPS_LATER
        move_link6_with_retry(target_pos, target_quat, steps=steps, label=f"스캔 위치 {i}")
        step_hold(POST_CONVERGENCE_SETTLE_STEPS)
        save_debug_rgb(f"scan_{i}")

        pts_world_i = None
        for retry in range(3):
            candidate = np.asarray(pcn.depth_camera.get_pointcloud(world_frame=True))
            if candidate.ndim == 2 and candidate.shape[1] == 3 and len(candidate) > 0:
                pts_world_i = candidate
                break
            print(f"[경고] 스캔 위치 {i}: get_pointcloud() 결과가 비정상(shape={candidate.shape}) "
                  f"-> 재시도 {retry + 1}/3", flush=True)
            step_hold(15)
        if pts_world_i is None:
            print(f"[경고] 스캔 위치 {i}: point cloud 획득 실패 - 이 시점은 건너뜀", flush=True)
            continue

        roi_center_x = cart_center_xy[0] + CART_SCAN_ROI_CENTER_X_SHIFT_M
        keep = (
            (pts_world_i[:, 0] >= roi_center_x - CART_SCAN_ROI_HALF_X_M)
            & (pts_world_i[:, 0] <= roi_center_x + CART_SCAN_ROI_HALF_X_M)
            & (pts_world_i[:, 1] >= cart_center_xy[1] - CART_SCAN_ROI_HALF_Y_M)
            & (pts_world_i[:, 1] <= cart_center_xy[1] + CART_SCAN_ROI_HALF_Y_M)
            & (pts_world_i[:, 2] >= CART_BASKET_FLOOR_Z - 0.30)
            & (pts_world_i[:, 2] <= CART_BASKET_FLOOR_Z + CART_SCAN_ROI_MAX_HEIGHT_M)
        )
        pts_world_i = pts_world_i[keep]
        accumulated_world_points.append(pts_world_i)
        print(f"[카트 스캔 {i}] x_offset={x_offset:+.2f} world_points={len(pts_world_i)}", flush=True)

    if not accumulated_world_points:
        raise RuntimeError("모든 스캔 시점에서 point cloud 획득에 실패했습니다.")

    # ---- 기준 위치(중앙) 복귀 + base_link 기준 변환/저장 (99.py와 동일) ----
    drive_to(target_x=float(scan_standoff_xy[0]), target_y=float(scan_standoff_xy[1]), target_yaw_deg=0.0,
             label="기준 위치(중앙) 복귀")
    base_link_pos, base_link_quat_wxyz = pcn.m0609_robot.get_world_pose()
    merged_world = np.concatenate(accumulated_world_points, axis=0)
    merged_base = world_to_base(
        merged_world, np.asarray(base_link_pos, dtype=float),
        np.asarray(base_link_quat_wxyz, dtype=float),
    )

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    out_path = OUT_DIR / "merged_cart_scan.npy"
    np.save(out_path, merged_base.astype(np.float64))
    print(f"[CAPTURE] 완료: {len(CART_SCAN_STRAFE_X_OFFSETS)}개 시점 누적, "
          f"총 {len(merged_base)}점 -> {out_path}", flush=True)
    print(f"[CAPTURE] m0609_base_link world pose: pos={np.round(base_link_pos, 3)} "
          f"quat_wxyz={np.round(base_link_quat_wxyz, 3)}", flush=True)

    # 사용자 지시 - 스캔 후 홀로노믹 베이스가 시작 위치로 복귀한 채로 서버 모드를
    # 유지해야 하고, 그 이후 들어오는 실제 요청(ExecutePickPlace)이 "그 복귀된
    # 베이스 위치" 기준으로 정확히 동작해야 한다. merged_base(.npy)는 스캔 시점의
    # base_link 상대 좌표라 그 자체로는 섀시가 움직이면 무효가 된다 - box_scan_action_
    # server.py가 여기서 저장하는 "스캔 시점 anchor"(base_link_pos/quat_wxyz)로 즉시
    # world 좌표로 변환해서 반환하도록 변경했으므로, 이후 섀시가 어디로 움직이든(시작
    # 위치 복귀 포함) detected_pose는 계속 유효하다. anchor 파일이 없으면
    # box_scan_action_server.py는 구버전처럼 "그 프레임에서 요청이 왔을 때 즉시 처리"
    # 가정으로 fallback한다(하위호환, execute_pick_place_action_server.py의 anchor 재해석 참고).
    anchor_path = OUT_DIR / "merged_cart_scan_anchor.json"
    anchor_path.write_text(json.dumps({
        "base_link_pos": [float(v) for v in base_link_pos],
        "base_link_quat_wxyz": [float(v) for v in base_link_quat_wxyz],
    }, indent=2))
    print(f"[CAPTURE] 스캔 시점 anchor 저장: {anchor_path}", flush=True)

    # 스캔 마지막 시점(카트 옆으로 뻗은 자세)에 팔을 그대로 두면 안전하지 않다 - 시작
    # 자세(joint_3=joint_5=90도 접힘, 100.py raise_lift_and_fold의 접기 부분과 동일 자세)로
    # 돌아온다. fold_to_known_pose()는 순수 관절공간 보간이라(리프트/섀시 둘 다 안 건드림)
    # m0609_robot.get_world_pose()가 반환하는 arm base_link pose에 영향이 없다 - 이미
    # 위에서 point cloud를 base_link_pos/quat 기준으로 저장 완료한 뒤라, 이 접기가
    # 저장된 좌표계나 CAPTURE_THEN_SERVE의 앵커 유효성(리프트 높이/섀시 pose)에 아무
    # 영향을 주지 않는다(그 두 값은 그대로 유지된다).
    fold_to_known_pose()
    print("[CAPTURE] 팔 시작 자세(joint_3/5=90도 접힘)로 복귀 완료", flush=True)

    # 사용자 지시 - 스캔 후 홀로노믹 베이스는 항상 시작 위치로 복귀하고, 서버 모드도
    # 유지해서 그 이후 들어오는 실제 요청이 "복귀된 베이스 위치" 기준으로 정확히
    # 동작해야 한다. box_scan_action_server.py가 위에서 저장한 anchor(스캔 시점
    # 섀시 pose)로 detected_pose를 이미 world 좌표로 변환해서 반환하도록 바꿨으므로
    # (execute_pick_place_action_server.py도 header.frame_id="world"면 지금 앵커로
    # 재해석하지 않고 그대로 쓰도록 바뀜), 이제 섀시가 스캔 후 실제로 움직여도
    # (이 복귀 포함) detected_pose가 더 이상 깨지지 않는다 - 예전에는 이 안전장치가
    # 없어서 CAPTURE_THEN_SERVE일 때만 복귀를 생략했었다(이제 그 조건 분기 자체가 불필요).
    #
    # 접근의 정확히 역순(3단계 중 1단계는 스폰=CART_CLEAR_X라 원래도 없었으므로
    # 2단계만 필요) - 1) X만 CART_CLEAR_X로 물러난다(Y=standoff_y, yaw=0 유지 -
    # 이미 Y축 분리돼 있으므로 안전). 2) 안전지대(X=CART_CLEAR_X)에서 스폰 Y/yaw로
    # 회전+횡이동(X축 분리 유지돼 안전) - 이러면 정확히 스폰 위치로 돌아간다.
    drive_to(target_x=_cart_clear_x, target_y=float(scan_standoff_xy[1]), target_yaw_deg=0.0,
             label="복귀 1/2: standoff -> 회전 안전지대(X만 이동)")
    drive_to(target_x=_cart_clear_x, target_y=float(_spawn_pos[1]), target_yaw_deg=_spawn_yaw,
             label="복귀 2/2: 안전지대에서 스폰 Y/yaw로 회전+횡이동")
    base_pos_final, _ = pcn.base_robot.get_world_pose()
    print(f"[CAPTURE] 홀로노믹 베이스 시작 위치로 복귀 완료: pos={np.round(base_pos_final, 3)}", flush=True)

    if os.environ.get("CART2TRUNK_CAPTURE_THEN_SERVE", "0") == "1":
        print("[CAPTURE] CART2TRUNK_CAPTURE_THEN_SERVE=1 - 스캔 종료 후 실제 "
              "platform_controller_node로 계속 서비스한다(시작 위치로 복귀한 채로)", flush=True)
        pcn.main()
    else:
        pcn.simulation_app.close()


if __name__ == "__main__":
    main()
