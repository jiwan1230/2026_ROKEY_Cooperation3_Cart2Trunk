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

import platform_controller_node as pcn  # noqa: E402

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


def drive_base_to_xy(
    target_xy: np.ndarray, tolerance_m: float = 0.05, max_steps: int = 1500,
    smooth_alpha: float = 0.12,
) -> None:
    """holo_forward()로 모바일 베이스를 target_xy까지 몬다(99.py drive_to()와 같은
    폐루프 원리, yaw 목표 없이 현재 자세 유지). base_robot.set_world_pose()로
    순간이동시켰더니 활성 조인트 드라이브가 걸린 물리 바디를 갑자기 옮기는
    셈이라 PhysX가 "Illegal BroadPhaseUpdateData"를 내며 리지드바디 pose가
    NaN/영쿼터니언으로 깨졌다(실측 확인) - 실제로 몰아서 물리 상태를 안전하게
    유지한다.

    [실측으로 찾은 함정 1] holo_forward(vx,vy,wz)의 vx/vy는 chassis 로컬 프레임
    기준인데, chassis 자신은 BASE_PATH 부모 Xform에 BASE_FACE_ROT_Z(90도)가
    이미 적용돼 있어(build_holonomic_base) yaw=0(스폰 직후)이어도 world 프레임과
    90도 어긋나 있다 - "yaw=0이니 local==world"라고 가정하고 world 프레임 오차를
    그대로 vx/vy에 넣었더니 발산했다(실측: 6m 넘게 이탈). 99.py drive_to()와 같은
    공식(ex_l = ex_w*cos(yaw)+ey_w*sin(yaw), ey_l = -ex_w*sin(yaw)+ey_w*cos(yaw))으로
    매 스텝 실제 world 쿼터니언에서 yaw를 뽑아 오차를 로컬 프레임으로 정확히
    회전시켜야 한다.

    [실측으로 찾은 함정 2 - 훨씬 심각함] 이 함수는 원래 속도 스무딩 없이 매 스텝
    비례오차를 그대로 명령했다 - 지속적인 대각선/횡이동(strafe) 구간에서
    메카넘 롤러 접촉 솔버가 결정론적으로(같은 코드로 반복해도 항상 같은 스텝에서)
    폭발했다(실측: 각속도가 수백 rad/s에서 시작해 스텝마다 기하급수적으로
    커져 수백만 rad/s까지 발산, 섀시가 순간적으로 공중으로 튀어오르며 결국
    NaN). 99.py/cart2trunk_motion.control_loops.drive_to()가 이미 하던
    지수이동평균 속도 스무딩(SMOOTH_ALPHA=0.12)을 빼먹은 게 원인이었다 -
    추가하자 같은 구간에서 폭발이 재현되지 않았다(A/B 테스트로 확인)."""
    vx_s = vy_s = 0.0
    for _ in range(max_steps):
        pos, quat_wxyz = pcn.base_robot.get_world_pose()
        dx, dy = float(target_xy[0] - pos[0]), float(target_xy[1] - pos[1])
        if np.hypot(dx, dy) < tolerance_m:
            break
        w, x, y, z = quat_wxyz
        yaw = float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
        local_x = dx * np.cos(yaw) + dy * np.sin(yaw)
        local_y = -dx * np.sin(yaw) + dy * np.cos(yaw)
        vx_t = float(np.clip(1.0 * local_x, -0.3, 0.3))
        vy_t = float(np.clip(1.0 * local_y, -0.3, 0.3))
        vx_s += smooth_alpha * (vx_t - vx_s)
        vy_s += smooth_alpha * (vy_t - vy_s)
        pcn.base_robot.apply_action(pcn.holo_forward(vx_s, vy_s, 0.0))
        step_hold(1)
    pcn.base_robot.apply_action(pcn.holo_forward(0.0, 0.0, 0.0))
    step_hold(20)


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
    chassis_half_width = pcn.BASE_WIDTH / 2.0
    standoff_y = chassis_half_width + cart_half_y + STANDOFF_MARGIN
    scan_standoff_xy = np.array([cart_center_xy[0], cart_center_xy[1] - standoff_y])
    print(f"[CAPTURE] 카트 center_xy={np.round(cart_center_xy, 3)} half_y={cart_half_y:.3f} "
          f"-> standoff_xy={np.round(scan_standoff_xy, 3)}", flush=True)

    print(f"[CAPTURE] 리프트 최고 높이로: {pcn.LIFT_MIN:.3f} -> {pcn.LIFT_MAX:.3f}", flush=True)
    for i in range(120):
        h = pcn.LIFT_MIN + (pcn.LIFT_MAX - pcn.LIFT_MIN) * (i + 1) / 120
        pcn.lift_state["h"] = h
        step_hold(1)

    drive_base_to_xy(scan_standoff_xy)
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
        drive_base_to_xy(np.array([strafe_x, scan_standoff_xy[1]]))

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
    drive_base_to_xy(scan_standoff_xy)
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

    pcn.simulation_app.close()


if __name__ == "__main__":
    main()
