"""MSI2(Isaac Sim PC): 하드웨어 제어 계층 - 리프트 + 모바일 베이스(홀로노믹) + 그리퍼 + M0609(RMPflow).

Isaac Sim은 물리 프림/Articulation 핸들을 하나의 프로세스 안에서만 공유할 수 있어서,
mobile_base/lift/m0609/gripper 컨트롤러를 별도 `ros2 run` 프로세스로 쪼갤 수 없다 -
전부 이 노드(`isaac_python`으로 실행하는 하나의 SimulationApp 프로세스) 안에서 살아야
한다. 이걸로 platform 4종(lift/mobile_base/gripper/m0609)이 전부 이 파일 하나에
모였다.

[M0609은 link_6 목표 자세를 RMPflow IK로 계속 추종하는 방식]
100.cart_to_trunk_dual_side_holonomic.py의 move_link6()/move_link6_smooth()가 매
스텝 하던 걸 그대로 가져왔다: sync_rmp_base()(리프트/모바일베이스가 매 스텝 움직이므로
RMPflow가 아는 "로봇 베이스 위치"도 매 스텝 다시 맞춰야 함) -> controller.forward(목표
link_6 자세) -> m0609_robot.apply_action(). move_link6_smooth()의 "그리퍼 팁 자체를
폐루프로 제어"(link_6과 흡착 팁 사이 실측 13cm+ 오프셋 보정 + 정체 감지)는 옮기지
않았다 - README 원칙대로 그건 cart2trunk_motion이 이 노드의 /m0609/move_to_pose +
/m0609/state를 반복 호출해서 구현할 몫이다(모바일베이스의 drive_to()와 같은 이유).
이 노드는 "지금 이 link_6 자세로 IK를 계속 풀어라"까지만 한다.

[RMPflow가 가끔 목표에서 크게 벗어난 채 멈추는 현상 - 버그 아님, 알려진 특성]
실측 확인: 어떤 시작 자세에서 특정 목표(예: 같은 자세로 z만 +10cm)를 주면 크게
못 미친 채(수십 cm 오차) 정지하는 경우가 있었다 - 반면 훨씬 큰 목표(전혀 다른
위치/자세)나 작은 목표(z +2cm)는 수 mm 오차로 정확히 수렴했다. 이건 이 노드의
배선 문제가 아니라 RMPflow(반응형 potential-field 컨트롤러)가 특이점 근처에서
지역 최소값에 걸리는 문제로, 100.cart_to_trunk_dual_side_holonomic.py 자신이
"조인트3/5 반전으로 반대쪽 solution branch를 쓰게 강제"하는 로직을 대규모로
설계해서 대응한, 이 팔+RMPflow 조합의 이미 알려진 특성이다. 좋은 중간 웨이포인트를
거쳐가서 이런 지역 최소값을 피하는 건 이 저수준 노드가 아니라 cart2trunk_motion이
할 일이다.

[그리퍼 검증용 테스트 박스]
DynamicSuctionGripper.close()는 set_target()으로 미리 지정한 프림에 대해서만
기하 조건(수평/수직 오차가 허용치 이내)을 만족해야 흡착한다 - 아직 m0609_controller
(RMPflow)가 없어서 팔을 실제로 움직여 박스에 접근시킬 수 없으므로, 이 슬라이스는
그리퍼 팁의 "지금 위치"(스폰 직후 기본 관절 자세) 바로 아래에 작은 테스트 박스를
하나 놓고 그 박스를 target으로 미리 등록해둔다 - 팔이 움직이기 시작하면(다음
슬라이스) 이 테스트 박스 배치는 의미가 없어지고 실제 카트/박스 배치로 교체해야
한다.

[모바일 베이스는 "저수준 속도 제어"까지만 - "목표 지점으로 주행"은 여기 없음]
100.py의 drive_to()/drive_until()(비례제어+정체감지+속도 스무딩으로 목표 (x,y,yaw)까지
주행)은 일부러 옮기지 않았다 - README 설계 원칙(cart2trunk_platform=저수준 명령
수신, cart2trunk_motion=platform 컨트롤러를 조합한 작업 단위)대로, "목표 지점까지
주행"은 이 노드의 /mobile_base/cmd_vel + /mobile_base/state를 조합해 상위 계층
(cart2trunk_motion)이 구현해야 할 몫이다. 이 노드는 순수 속도 명령 수신 + 상태
발행만 한다.

[ROS2 인터페이스는 표준 메시지만 쓴다 - 중요한 설계 결정]
Isaac Sim 번들 파이썬은 3.11인데 시스템 ROS2 Humble(cart2trunk_interfaces가 빌드된
환경)은 3.10이라 rclpy C 확장(cp310)을 3.11이 못 읽는다. Isaac Sim은
`isaacsim.ros2.bridge/humble/`에 자체 3.11용 rclpy+표준 메시지 빌드를 갖고 있지만
colcon/ament_cmake 빌드 도구는 없어서 커스텀 인터페이스(Box3D 등)를 이 환경용으로
새로 만들 수 없다 - 그래서 이 노드는 std_msgs/geometry_msgs/std_srvs 같은 표준
타입만 쓴다(실측 확인: 시스템 python3.10 rclpy 프로세스와 이 노드 사이에
std_msgs/Float32 토픽으로 정상 교차 통신됨). 커스텀 타입이 필요한 상위 계층
(cart2trunk_motion 등)은 이 노드를 표준 메시지로 호출하고, 자신은 시스템
파이썬(3.10)에서 별도 프로세스로 돈다 - Isaac Sim PC와 나머지 ROS 그래프를
물리적으로 다른 PC로 분리해도 그대로 동작한다(DDS 통신이라 프로세스/머신 위치 무관).

[씬 구성 출처]
build_holonomic_base/build_mecanum_wheel/mount_m0609/add_drive_stiffness/quat_between는
EDU 저장소 100.cart_to_trunk_dual_side_holonomic.py에서 그대로 가져왔다(수십 개
커밋으로 튠된 물리 상수/조인트 설정이라 재작성하지 않고 그대로 옮김). 카트/차량/
트렁크/RMPflow/그리퍼는 이 1차 슬라이스에 포함하지 않았다 - 리프트만 검증한다.

[리프트가 물리 조인트가 아니라 키네마틱 텔레포트라는 점]
100.py의 리프트는 실제 프리즘 조인트/드라이브가 아니라 M0609 베이스 프림을 매
스텝 직접 teleport하는 방식이다(18~27번 lift_stage 스크립트에서 조인트 기반
접근을 여러 번 시도하다 폐기하고 이 방식으로 정착한 결과 - 27.lift_stage9_visual_lift.py
가 이 접근의 이름 그대로다). set_lift_height()가 그 로직이다.

인터페이스:
    /lift/move          (std_msgs/Float32, subscribe)     - 목표 높이(LIFT_MIN~LIFT_MAX로 클램프)
    /lift/state         (std_msgs/Float32, publish)        - 현재 높이
    /mobile_base/cmd_vel (geometry_msgs/Twist, subscribe)  - 로봇 로컬 프레임 속도(linear.x=vx,
                                                              linear.y=vy, angular.z=wz)
    /mobile_base/state  (geometry_msgs/PoseStamped, publish) - 현재 섀시 world pose
    /gripper/set_target (geometry_msgs/PoseStamped, subscribe) - 흡착 대상 world pose(박스
                        top-center, frame_id="world" 고정) - cart2trunk_motion이 GRIP 직전에
                        발행한다. 이 노드가 cart_scene.box_objects 중 가장 가까운 실제 박스
                        prim을 찾아 내부적으로 기억한다(box_id/prim_path는 이 노드가 다른
                        인터페이스처럼 커스텀 메시지를 못 써서(Isaac 번들 python3.11) ROS로
                        직접 안 받는다 - 상세 설계는 _find_nearest_box_prim() 참고).
    /gripper/activate   (std_srvs/Trigger, service)        - 직전 /gripper/set_target으로 찾은
                        박스에 흡착 시도(목표 없음/오래됨/기하 조건 밖이면 실패 반환 - 이번
                        시도로 목표를 소비하므로 다음 activate 전에 set_target을 다시 보내야 함)
    /gripper/release    (std_srvs/Trigger, service)        - 흡착 해제
    /gripper/state      (std_msgs/Bool, publish)           - true=흡착 중
    /m0609/move_to_pose (geometry_msgs/PoseStamped, subscribe) - link_6 목표 world pose(RMPflow IK)
    /m0609/state        (geometry_msgs/PoseStamped, publish)   - 현재 link_6 world pose
    /gripper/tip_pose   (geometry_msgs/PoseStamped, publish)   - 현재 흡착 팁 world pose(link_6과
                        고정 마운트 오프셋만큼 떨어져 있음 - cart2trunk_motion 폐루프 접근용)

실행:
    HEADLESS=1 isaac_python platform_controller_node.py

[실측 확인된 불안정성 - LD_LIBRARY_PATH를 셸에서 미리 export할 것] 아래 rclpy
sys.path/LD_LIBRARY_PATH 설정 블록이 os.environ["LD_LIBRARY_PATH"]를 코드
안에서(셸이 아니라 프로세스 안에서) 채워주지만, 이 프로세스 내부 변경이 rclpy
C 확장(_rclpy_pybind11)의 자체 의존 라이브러리(librcl_action.so 등) dlopen
시점에 항상 반영되는 건 아니었다(실측: 같은 코드로 여러 번 실행했을 때 간헐적으로
"librcl_action.so: cannot open shared object file"로 실패 - 파일 자체는 항상
존재했고 ldd로도 문제없었음). 실행 전에 셸에서 아래처럼 직접 export하면
매번 확실히 성공한다:
    export LD_LIBRARY_PATH=/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/\
release/exts/isaacsim.ros2.bridge/humble/lib:$LD_LIBRARY_PATH
"""
from __future__ import annotations

import os
import sys
import time

HEADLESS = os.environ.get("HEADLESS", "0") == "1"

from isaacsim import SimulationApp

_sim_app_config = {"headless": HEADLESS}
if not HEADLESS:
    _sim_app_config.update({"width": 640, "height": 480})
simulation_app = SimulationApp(_sim_app_config)

# ---------------------------------------------------------------------------
# Isaac Sim 번들 파이썬(3.11)에서 rclpy를 쓰려면 시스템 ROS2(3.10)가 아니라
# isaacsim.ros2.bridge가 자체 포함한 humble 빌드를 잡아야 한다.
# ---------------------------------------------------------------------------
_ROS2_BRIDGE_HUMBLE = (
    "/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/"
    "exts/isaacsim.ros2.bridge/humble"
)
if os.path.isdir(_ROS2_BRIDGE_HUMBLE):
    sys.path.insert(0, os.path.join(_ROS2_BRIDGE_HUMBLE, "rclpy"))
    lib_dir = os.path.join(_ROS2_BRIDGE_HUMBLE, "lib")
    os.environ["LD_LIBRARY_PATH"] = lib_dir + os.pathsep + os.environ.get("LD_LIBRARY_PATH", "")
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")

import json
from pathlib import Path

import numpy as np
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf

from isaacsim.core.api import World
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators.grippers.surface_gripper import SurfaceGripper

# ⚠️ 실측 확인(2026-07-27) - 카메라 ROS2 브리지는 원래 isaacsim.ros2.bridge가
# 제공하는 OGN 노드(IsaacCreateRenderProduct+ROS2CameraHelper 등)로 구현했었는데,
# 이 PC의 GPU(Blackwell, RTX 5080 Laptop)+드라이버(580.159.03) 조합에서 그 OGN
# 노드 구성 자체가 반복 재현 스크립트 기준 3/3 세그폴트를 냈다(NVIDIA 이슈
# #643/#651과 동일 시그니처) - 반면 Camera 클래스로 직접 depth/rgb를 뽑는 경로는
# 3/3 완전히 안정적이었다. 그래서 카메라는 isaacsim.ros2.bridge 익스텐션도,
# omni.graph 노드도 전혀 쓰지 않고 Camera 래퍼 + 표준 rclpy 퍼블리셔로만 발행한다
# (자세한 재현 기록은 cart2trunk_simulation/core/sensor_bridge.py 모듈 docstring).
#
# HANDOFF_MSI2.md 2.3절 - M0609 자산(RMPflow/URDF/그리퍼 USD)은 git에 없고 PC마다
# 로컬 경로가 다르다 - 하드코딩 대신 CART2TRUNK_M0609_DIR 환경변수로 받는다.
# 기본값은 이 PC에서 실제로 검증된 사본(git으로 관리되는 살아있는 작업본 -
# .../isaacpjt/M0609/.git, 그리퍼 바디 prim이 GRIPPER_BODY_NAME 기본값
# "vgp20_suction_plate"와 정확히 일치함. /home/rokey/M0609_env.tar.gz에서 뽑은
# 사본은 이름이 "vgp20"으로 달라서 한때 CART2TRUNK_GRIPPER_BODY_NAME 오버라이드가
# 필요했었는데, 이 사본을 쓰면 필요 없다 - 오버라이드 자체는 다른 PC/사본
# 대비용으로 남겨둔다).
_M0609_DIR = os.environ.get("CART2TRUNK_M0609_DIR", "/home/rokey/cobot3_ws/isaacpjt/M0609")
_RMPFLOW_DIR = str(Path(_M0609_DIR) / "rmpflow")
if _RMPFLOW_DIR not in sys.path:
    sys.path.insert(0, _RMPFLOW_DIR)
from m0609_rmpflow_controller import RMPFlowController  # noqa: E402

# cart2trunk_simulation은 colcon으로 빌드된 시스템 python3.10 site-packages에
# 설치되는데, 이 프로세스는 Isaac Sim 번들 python3.11이라 그 경로가 안 보인다
# (M0609_DIR과 다른 문제 - 이건 "PC마다 자산 위치가 다르다"가 아니라 "인터프리터
# 자체가 다르다"는 문제라 환경변수가 아니라 이 파일 기준 상대경로로 src/ 트리를
# 직접 잡는다 - cart2trunk_simulation의 core 모듈은 rclpy 없이 pxr/isaacsim/numpy만
# 쓰므로 3.10용으로 빌드된 결과물이 아니라 소스를 그대로 3.11에서 import해도 된다).
_SRC_DIR = Path(__file__).resolve().parents[2]
_CART2TRUNK_SIMULATION_SRC = _SRC_DIR / "cart2trunk_simulation"
if str(_CART2TRUNK_SIMULATION_SRC) not in sys.path:
    sys.path.insert(0, str(_CART2TRUNK_SIMULATION_SRC))
from cart2trunk_simulation.core.cart_scene import build_cart_with_boxes  # noqa: E402
from cart2trunk_simulation.core.vehicle_scene import build_vehicle  # noqa: E402
from cart2trunk_simulation.core.sensor_bridge import (  # noqa: E402
    camera_info_msg, cleanup_camera_reference_asset, depth_to_image_msg,
    find_camera_prim_path, initialize_depth_camera,
)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image, CameraInfo

# ============================================================
# 씬 구성 상수 (100.py "82~91번과 동일 홀로노믹 베이스 구성" 절과 동일값)
# ============================================================

DRIVE_STIFFNESS, DRIVE_DAMPING, DRIVE_MAX_FORCE = 0.0, 50.0, 20.0
BASE_PATH = "/World/HoloBase"
CHASSIS_PATH = f"{BASE_PATH}/chassis"
BASE_FACE_ROT_Z = 90.0

ROLLER_COUNT = 9
ROLLER_MASS = 0.02
HUB_MASS = 1.0
CHASSIS_MASS = 15.0

M0609_USD = str(f"{_M0609_DIR}/Collected_m0609_vgp20_camera/m0609_vgp20_camera.usd")
M0609_URDF_PATH = str(Path(_M0609_DIR) / "doosan-robot2" / "urdf" / "m0609_isaac_sim.urdf")
M0609_DESCRIPTION_PATH = str(Path(_M0609_DIR) / "rmpflow" / "m0609_description.yaml")
M0609_RMPFLOW_CONFIG_PATH = str(Path(_M0609_DIR) / "rmpflow" / "m0609_rmpflow_common.yaml")
M0609_MOUNT_Z_ABOVE_CHASSIS_TOP = 0.02
LIFT_COLUMN_RADIUS = 0.045

# 92.trunk_place_holonomic.py/100.py와 동일 - CSV 튜닝 결과 없으면 기본값(0.50,0.50,0.15).
BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT = 0.50, 0.50, 0.15
WHEEL_RADIUS = max(0.05, BASE_HEIGHT / 2.0)
CHASSIS_BODY_HEIGHT = min(BASE_HEIGHT, 2 * WHEEL_RADIUS) * 0.7
ROLLER_RADIUS = WHEEL_RADIUS * 0.22
ROLLER_LENGTH = (2 * np.pi * (WHEEL_RADIUS - ROLLER_RADIUS)) / ROLLER_COUNT * 1.15
HUB_RADIUS = WHEEL_RADIUS - ROLLER_RADIUS * 0.85
HUB_THICKNESS = WHEEL_RADIUS * 0.55
CHASSIS_LENGTH_EXTENDED = 1.00
WHEEL_MOUNT_HALF_L = BASE_LENGTH / 2.0 - WHEEL_RADIUS * 0.6
_wheel_half_thickness_y = HUB_THICKNESS / 2.0 + ROLLER_LENGTH * 0.5 + ROLLER_RADIUS
# 100.py와 동일 공식 - CART_CLEAR_X(회전 안전지대) 계산에 필요(아래 참고).
CHASSIS_HALF_LENGTH_EFFECTIVE = CHASSIS_LENGTH_EXTENDED / 2.0 + WHEEL_RADIUS * 0.6
CHASSIS_HALF_WIDTH_EFFECTIVE = BASE_WIDTH / 2.0 + _wheel_half_thickness_y * 1.3
CART_STANDOFF_MARGIN = 0.10  # 100.py의 CART_STANDOFF_MARGIN과 동일값.

# 100.py와 동일 - MEASURED_CHASSIS_TOP_OFFSET은 실측 상수(스크린샷 대조로 확인된 값).
MEASURED_CHASSIS_TOP_OFFSET = 0.0180
LIFT_MIN = MEASURED_CHASSIS_TOP_OFFSET + M0609_MOUNT_Z_ABOVE_CHASSIS_TOP
LIFT_MAX = LIFT_MIN + 0.35  # 100.py와 동일값 - 트렁크 STAGE 3.x가 이 이름을 직접 참조하므로 절대 바꾸지 않는다.

# ---------------- cart2trunk_simulation - 카트/차량(트렁크) 씬 ----------------
# 100.py의 CART_USD/CAR_USD와 동일한 usdz 에셋(git에 커밋 안 함 - M0609 자산과
# 같은 이유, .gitignore에 *.usdz 추가됨) - 이 PC에서 확인된 경로를 기본값으로,
# 다른 PC/사본 대비 환경변수로 덮어쓸 수 있게 한다.
_ASSET_DIR = os.environ.get("CART2TRUNK_ASSET_DIR", "/home/rokey/cobot3_ws/isaacpjt/Cart2Trunk/assets")
CART_USD = str(Path(_ASSET_DIR) / "Metal_Shopping_Cart.usdz")
CAR_USD = str(Path(_ASSET_DIR) / "Lexus_IS300_Trunk_Open_No_More_Hell_Room.usdz")
# ⚠️ 실측으로 찾은 버그(2026-07-27, 사용자가 뷰포트 스크린샷으로 직접 지적) - 이
# 두 위치는 원래 100.py의 CART_POS=(0,0,0)/CAR_POS=(5,0,0)을 그대로 안 쓰고
# "로봇 스폰 위치 근처의 비어있는 좌표"로 임의로 잡았던 값이었다(카트 1.3,
# 차량 3.6 - 겹치지 않는지 스크린샷으로 확인 안 한 채 방치돼 있었음) - 실제로는
# 차량 크기 때문에 카트와 거의 붙어 보일 만큼 가까웠다. 100.py와 정확히 같은
# 값으로 맞춘다 - 두 스크립트가 같은 씬 배치를 가정해야 카트/트렁크 접근 거리
# 튜닝(cart_standoff_m 등)도 서로 의미가 통한다.
CART_SCENE_POSITION = (0.0, 0.0, 0.0)
CART_SCENE_ROT_Z = 90.0
VEHICLE_SCENE_POSITION = (5.0, 0.0, 0.0)
VEHICLE_SCENE_ROT_Z = 0.0

# ---------------- cart2trunk_simulation - 실시간 Depth 카메라 ROS2 브리지 ----------------
# 88.cart_scan_holonomic.py/89.trunk_scan_holonomic.py와 동일값 - box_scan_action_server/
# trunk_scan_action_server가 실시간 depth_callback을 구현하게 되면(HANDOFF_2026-07-26.md
# 2.4절) 이 토픽/frame_id를 그대로 구독하면 된다. M0609 vgp20 그리퍼에 이미 마운트된
# RealSense 카메라(Camera_Pseudo_Depth 프림)를 그대로 쓴다 - 별도 카메라를 새로 붙이지 않음.
DEPTH_CAMERA_NAME_HINT = "Depth"
CAMERA_WIDTH, CAMERA_HEIGHT = 640, 480
DEPTH_TOPIC = "/camera/depth"
CAMERA_INFO_TOPIC = "/camera/camera_info"
CAMERA_FRAME_ID = "m0609_depth_camera_optical_frame"

# ---------------- 91.cart_pick_holonomic.py/100.py와 동일 - 그리퍼 ----------------
EE_LINK_NAME = "link_6"
# M0609 USD 자산마다 그리퍼 바디 prim 이름이 다르다(실측 확인 - 어떤 사본은
# "vgp20_suction_plate", 이 환경에서 실제로 받은 사본은 그냥 "vgp20") - 자산이
# git에 없어 사본마다 구조가 다를 수 있다는 HANDOFF_MSI2.md 2.3절 경고 그대로라
# 하드코딩 대신 환경변수로 덮어쓸 수 있게 한다.
GRIPPER_BODY_NAME = os.environ.get("CART2TRUNK_GRIPPER_BODY_NAME", "vgp20_suction_plate")
# activate 시점(팁이 실제로 다 왔는지) 판정 - 엄격한 값.
GRASP_HORIZONTAL_MARGIN = 0.03
GRASP_VERTICAL_TOLERANCE = 0.02
# /gripper/set_target 시점(world pose로 "이게 어떤 실제 박스인가"를 찾는) 판정 -
# GRASP_*보다 넉넉하다. cart2trunk_motion이 anchor로 world 좌표를 이미 계산해
# 넘기지만, 그 계산 자체에 검출/앵커 오차(실측 수 cm대, cart2trunk_perception
# 검출 정확도 참고)가 있을 수 있어 "어떤 박스인지 식별"하는 단계는 여유를 둔다 -
# 실제 흡착 가능 여부는 어차피 activate에서 GRASP_* 기준으로 다시, 더 엄격하게 본다.
TARGET_SEARCH_HORIZONTAL_TOLERANCE_M = 0.08
TARGET_SEARCH_VERTICAL_TOLERANCE_M = 0.08
# /gripper/set_target 수신 후 이 시간(초)이 지나도록 /gripper/activate가 안 오면
# 목표를 무효화한다 - 이전 박스의 낡은 목표가 다음 박스에 실수로 재사용되는 것을
# 막는 안전장치(사용자 지시 - "이전 target이 재사용되지 않도록 해야 함").
GRIPPER_TARGET_TIMEOUT_SEC = 1.0

_GRIPPER_RANGE_JSON = Path(_M0609_DIR) / "Collected_m0609_vgp20_camera" / "_gripper_physical_range.json"
if _GRIPPER_RANGE_JSON.exists():
    _range = json.loads(_GRIPPER_RANGE_JSON.read_text())
    TIP_LOCAL_OFFSET = tuple(_range["tip_local_offset"])
else:
    TIP_LOCAL_OFFSET = (0.0, 0.0, 0.0188)

# 리프트가 한 main-loop 반복(world.step() 1회)마다 목표를 향해 움직일 수 있는 최대량(m).
# 100.py의 move_lift_to(steps=90)이 LIFT_TRAVEL_M(0.75)을 90스텝에 나눠 움직인 것과
# 비슷한 속도감(약 0.008m/step)이 되도록 잡았다 - ROS 토픽으로 목표가 언제든 바뀔 수
# 있는 연속 추종 방식이라 move_lift_to처럼 "정해진 스텝 수" 대신 "스텝당 최대 이동량"
# 으로 표현한다.
LIFT_MAX_STEP_M = 0.008


def mecanum_wheel_speeds(vx, vy, wz, wheel_radius, k):
    """100.cart_to_trunk_dual_side_holonomic.py와 동일 - 로봇 로컬 프레임 속도(vx,vy,wz)를
    4개 허브 조인트의 목표 각속도로 변환한다."""
    vy = -vy
    return [
        (vx - vy - k * wz) / wheel_radius,
        (vx + vy + k * wz) / wheel_radius,
        (vx + vy - k * wz) / wheel_radius,
        (vx - vy + k * wz) / wheel_radius,
    ]


def quat_between(v_from, v_to):
    """100.cart_to_trunk_dual_side_holonomic.py와 동일."""
    v_from = np.array(v_from, dtype=float); v_from = v_from / np.linalg.norm(v_from)
    v_to = np.array(v_to, dtype=float); v_to = v_to / np.linalg.norm(v_to)
    dot = float(np.clip(np.dot(v_from, v_to), -1.0, 1.0))
    if dot > 0.999999:
        return Gf.Quatf(1.0, 0.0, 0.0, 0.0)
    if dot < -0.999999:
        ortho = np.array([1.0, 0.0, 0.0]) if abs(v_from[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        axis = np.cross(v_from, ortho); axis = axis / np.linalg.norm(axis)
        return Gf.Quatf(0.0, float(axis[0]), float(axis[1]), float(axis[2]))
    axis = np.cross(v_from, v_to)
    w = 1.0 + dot
    q = np.array([w, axis[0], axis[1], axis[2]])
    q = q / np.linalg.norm(q)
    return Gf.Quatf(float(q[0]), float(q[1]), float(q[2]), float(q[3]))


def build_mecanum_wheel(stage, wheel_root_path, chassis_path, local_pos, wheel_material_path, chirality, name):
    """100.cart_to_trunk_dual_side_holonomic.py와 동일."""
    wx, wy, wz = local_pos
    hub_path = f"{wheel_root_path}/hub"
    hub = UsdGeom.Cylinder.Define(stage, hub_path)
    hub.CreateRadiusAttr(HUB_RADIUS)
    hub.CreateHeightAttr(HUB_THICKNESS)
    hub.CreateAxisAttr("Y")
    hub.CreateDisplayColorAttr([Gf.Vec3f(0.2, 0.2, 0.2)])
    hub_xform = UsdGeom.Xformable(hub)
    hub_xform.ClearXformOpOrder()
    hub_xform.AddTranslateOp().Set(Gf.Vec3d(wx, wy, wz))
    hub_prim = hub.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(hub_prim)
    UsdPhysics.MassAPI.Apply(hub_prim).CreateMassAttr().Set(HUB_MASS)

    hub_joint_path = f"{wheel_root_path}/joint_hub_{name}"
    hub_joint = UsdPhysics.RevoluteJoint.Define(stage, hub_joint_path)
    hub_joint.CreateAxisAttr("Y")
    hub_joint.CreateBody0Rel().SetTargets([Sdf.Path(chassis_path)])
    hub_joint.CreateBody1Rel().SetTargets([Sdf.Path(hub_path)])
    hub_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(wx, wy, 0.0))
    hub_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    drive = UsdPhysics.DriveAPI.Apply(hub_joint.GetPrim(), "angular")
    drive.CreateTypeAttr().Set("force")
    drive.CreateStiffnessAttr().Set(DRIVE_STIFFNESS)
    drive.CreateDampingAttr().Set(DRIVE_DAMPING)
    drive.CreateMaxForceAttr().Set(DRIVE_MAX_FORCE)
    drive.CreateTargetVelocityAttr().Set(0.0)

    for i in range(ROLLER_COUNT):
        theta = 2 * np.pi * i / ROLLER_COUNT
        place_r = HUB_RADIUS + ROLLER_RADIUS * 0.7
        rpos = np.array([place_r * np.cos(theta), 0.0, place_r * np.sin(theta)])
        tangent = np.array([-np.sin(theta), 0.0, np.cos(theta)])
        y_hat = np.array([0.0, 1.0, 0.0])
        roller_axis = tangent + chirality * y_hat
        roller_axis = roller_axis / np.linalg.norm(roller_axis)

        roller_path = f"{wheel_root_path}/roller_{i}"
        roller = UsdGeom.Capsule.Define(stage, roller_path)
        roller.CreateRadiusAttr(ROLLER_RADIUS)
        roller.CreateHeightAttr(ROLLER_LENGTH)
        roller.CreateAxisAttr("X")
        roller.CreateDisplayColorAttr([Gf.Vec3f(0.85, 0.35, 0.05)])
        quat = quat_between([1.0, 0.0, 0.0], roller_axis)
        r_xform = UsdGeom.Xformable(roller)
        r_xform.ClearXformOpOrder()
        r_xform.AddTranslateOp().Set(Gf.Vec3d(wx + rpos[0], wy + rpos[1], wz + rpos[2]))
        r_xform.AddOrientOp().Set(quat)
        r_prim = roller.GetPrim()
        UsdPhysics.CollisionAPI.Apply(r_prim)
        UsdPhysics.RigidBodyAPI.Apply(r_prim)
        UsdPhysics.MassAPI.Apply(r_prim).CreateMassAttr().Set(ROLLER_MASS)
        UsdShade.MaterialBindingAPI.Apply(r_prim).Bind(
            UsdShade.Material(stage.GetPrimAtPath(wheel_material_path)), materialPurpose="physics"
        )

        roller_joint_path = f"{wheel_root_path}/joint_roller_{name}_{i}"
        rjoint = UsdPhysics.RevoluteJoint.Define(stage, roller_joint_path)
        rjoint.CreateAxisAttr("X")
        rjoint.CreateBody0Rel().SetTargets([Sdf.Path(hub_path)])
        rjoint.CreateBody1Rel().SetTargets([Sdf.Path(roller_path)])
        rjoint.CreateLocalPos0Attr().Set(Gf.Vec3f(*rpos))
        rjoint.CreateLocalRot0Attr().Set(quat)
        rjoint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        rjoint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

    return hub_joint_path


def build_holonomic_base(stage, start_xy, length, width, height):
    """100.cart_to_trunk_dual_side_holonomic.py와 동일."""
    base_xform = UsdGeom.Xform.Define(stage, BASE_PATH)
    base_xform.ClearXformOpOrder()
    base_xform.AddTranslateOp().Set(Gf.Vec3d(start_xy[0], start_xy[1], 0.0))
    base_xform.AddRotateZOp().Set(BASE_FACE_ROT_Z)

    chassis_root = UsdGeom.Xform.Define(stage, CHASSIS_PATH)
    chassis_root.ClearXformOpOrder()
    chassis_root.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, WHEEL_RADIUS))
    chassis_prim = chassis_root.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(chassis_prim)
    UsdPhysics.MassAPI.Apply(chassis_prim).CreateMassAttr().Set(CHASSIS_MASS)
    UsdPhysics.ArticulationRootAPI.Apply(chassis_prim)

    chassis_geom = UsdGeom.Cube.Define(stage, f"{CHASSIS_PATH}/geom")
    chassis_geom.CreateSizeAttr(1.0)
    chassis_geom_xform = UsdGeom.Xformable(chassis_geom)
    chassis_geom_xform.ClearXformOpOrder()
    chassis_geom_xform.AddScaleOp().Set(Gf.Vec3f(CHASSIS_LENGTH_EXTENDED, width, CHASSIS_BODY_HEIGHT))
    chassis_geom.CreateDisplayColorAttr([Gf.Vec3f(0.25, 0.30, 0.35)])
    UsdPhysics.CollisionAPI.Apply(chassis_geom.GetPrim())

    wheel_material = PhysicsMaterial(
        prim_path=f"{BASE_PATH}/roller_material",
        static_friction=1.0, dynamic_friction=0.9, restitution=0.0,
    )

    corner_signs = [(1, 1, 1), (1, -1, -1), (-1, 1, -1), (-1, -1, 1)]
    corner_names = ["FL", "FR", "RL", "RR"]
    half_l = WHEEL_MOUNT_HALF_L
    half_w = width / 2.0 + _wheel_half_thickness_y * 1.3
    hub_joint_paths = []

    for (sx, sy, chirality), name in zip(corner_signs, corner_names):
        wx, wy, wz = sx * half_l, sy * half_w, 0.0
        wheel_root_path = f"{BASE_PATH}/wheel_{name}"
        hub_joint_path = build_mecanum_wheel(stage, wheel_root_path, CHASSIS_PATH, (wx, wy, wz),
                                              wheel_material.prim_path, chirality, name)
        hub_joint_paths.append(hub_joint_path)

    k_factor = half_l + half_w
    return CHASSIS_PATH, hub_joint_paths, k_factor


def add_drive_stiffness(stage, root_path, stiffness=1e8, damping=1e4, max_force=1e8):
    """100.cart_to_trunk_dual_side_holonomic.py와 동일."""
    n = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root_path)):
        for dof_type in ["angular", "linear"]:
            drive = UsdPhysics.DriveAPI.Get(prim, dof_type)
            if drive:
                drive.GetStiffnessAttr().Set(stiffness)
                drive.GetDampingAttr().Set(damping)
                drive.GetMaxForceAttr().Set(max_force)
                n += 1
    return n


def mount_m0609(stage, initial_h):
    """100.cart_to_trunk_dual_side_holonomic.py와 동일."""
    m0609_path = f"{BASE_PATH}/M0609"
    m0609_xform = UsdGeom.Xform.Define(stage, m0609_path)
    m0609_xform.GetPrim().GetReferences().AddReference(M0609_USD, "/World/m0609")
    m0609_xform.ClearXformOpOrder()
    m0609_xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, initial_h))

    for _ in range(20):
        simulation_app.update()

    base_link_path = f"{m0609_path}/base_link"
    root_joint_path = f"{m0609_path}/root_joint"
    if stage.GetPrimAtPath(root_joint_path).IsValid():
        stage.RemovePrim(root_joint_path)

    base_link_prim = stage.GetPrimAtPath(base_link_path)
    UsdPhysics.ArticulationRootAPI.Apply(base_link_prim)

    chassis_prim = stage.GetPrimAtPath(CHASSIS_PATH)
    filt_chassis = UsdPhysics.FilteredPairsAPI.Apply(chassis_prim)
    filt_chassis.CreateFilteredPairsRel().AddTarget(Sdf.Path(base_link_path))
    filt_base = UsdPhysics.FilteredPairsAPI.Apply(base_link_prim)
    filt_base.CreateFilteredPairsRel().AddTarget(Sdf.Path(CHASSIS_PATH))

    lift_column_path = "/World/LiftColumnVisual"
    lift_column = UsdGeom.Cylinder.Define(stage, lift_column_path)
    lift_column.CreateRadiusAttr().Set(LIFT_COLUMN_RADIUS)
    lift_column.CreateHeightAttr().Set(1.0)
    lift_column.CreateAxisAttr("Z")
    lift_column.CreateDisplayColorAttr([Gf.Vec3f(0.85, 0.45, 0.1)])
    lift_column_xform = UsdGeom.Xformable(lift_column)
    lift_column_xform.ClearXformOpOrder()
    lift_translate_op = lift_column_xform.AddTranslateOp()
    lift_scale_op = lift_column_xform.AddScaleOp()
    lift_scale_op.Set(Gf.Vec3f(1.0, 1.0, 0.001))

    n = add_drive_stiffness(stage, m0609_path)
    print(f"[DRIVE] M0609={n}개 조인트 강성 적용, initial_h={initial_h:.3f}", flush=True)
    return m0609_path, base_link_path, lift_translate_op, lift_scale_op


class DynamicSuctionGripper(SurfaceGripper):
    """100.cart_to_trunk_dual_side_holonomic.py와 동일 - 원통형(수평/수직 분리) 흡착 판정.
    구형 거리 판정은 수평/수직 오차를 하나로 합쳐서 봐서, 스캔 좌표 기반 목표에 수평
    오차가 있으면 수직 여유를 통째로 잡아먹는 문제가 있었다(91.cart_pick_holonomic.py에서
    실측 확인 후 교체)."""

    def __init__(self, end_effector_prim_path, gripper_body_path, tip_local_offset=(0.0, 0.0, 0.0)):
        SurfaceGripper.__init__(self, end_effector_prim_path=end_effector_prim_path, surface_gripper_path="")
        self._gripper_body_path = gripper_body_path
        self._tip_local_offset = Gf.Vec3d(*tip_local_offset)
        self._joint_path = f"{gripper_body_path}/suction_attach_joint"
        self._attached = False

    def tip_world_pos(self) -> np.ndarray:
        stage = omni.usd.get_context().get_stage()
        gripper_prim = stage.GetPrimAtPath(self._gripper_body_path)
        gripper_mat = UsdGeom.Xformable(
            gripper_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        return np.array(gripper_mat.Transform(self._tip_local_offset), dtype=float)

    def close_on_prim(
        self, target_prim_path, half_height, horizontal_tolerance, vertical_tolerance,
    ) -> bool:
        """target_prim_path의 top-center(prim 원점 + half_height, world +Z 기준 -
        박스가 yaw 회전만 있고 눕지 않는다는 이 프로젝트의 가정과 동일)에 대해
        기하 조건을 확인하고, 만족하면 실제 FixedJoint를 만든다. /gripper/set_target
        (PlatformControllerNode._on_gripper_set_target)이 world pose로 가장 가까운
        실제 박스 prim을 미리 찾아두면, activate 시점에 이 메서드가 그 prim을 대상으로
        호출된다 - half_height/tolerance는 호출마다 넘겨받으므로(이전처럼 set_target()로
        미리 고정해두지 않음) 매 흡착 시도마다 실제 대상 박스에 맞는 값을 쓸 수 있다."""
        if self._attached:
            return False
        stage = omni.usd.get_context().get_stage()
        target_prim = stage.GetPrimAtPath(target_prim_path)
        if not target_prim.IsValid():
            return False
        gripper_prim = stage.GetPrimAtPath(self._gripper_body_path)
        gripper_mat = UsdGeom.Xformable(
            gripper_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        target_mat = UsdGeom.Xformable(
            target_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        target_pos = target_mat.ExtractTranslation()
        tip_world = gripper_mat.Transform(self._tip_local_offset)
        horiz_dist = float(np.hypot(tip_world[0] - target_pos[0], tip_world[1] - target_pos[1]))
        vert_gap = float(tip_world[2] - (target_pos[2] + half_height))
        if horiz_dist > horizontal_tolerance or abs(vert_gap) > vertical_tolerance:
            print(f"  [흡착 실패] horiz={horiz_dist:.4f}m(허용 {horizontal_tolerance:.3f}) "
                  f"vert_gap={vert_gap:+.4f}m(허용 {vertical_tolerance:.3f}) "
                  f"-> {target_prim_path}", flush=True)
            return False
        rel_local = target_mat.GetInverse().Transform(tip_world)
        gripper_rot = gripper_mat.ExtractRotationQuat()
        target_rot = target_mat.ExtractRotationQuat()
        local_rot1 = target_rot.GetInverse() * gripper_rot
        joint = UsdPhysics.FixedJoint.Define(stage, self._joint_path)
        joint.CreateBody0Rel().SetTargets([Sdf.Path(self._gripper_body_path)])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(target_prim_path)])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(self._tip_local_offset))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(rel_local))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(local_rot1))
        self._attached = True
        print(f"  [흡착] horiz={horiz_dist:.4f}m vert_gap={vert_gap:+.4f}m -> {target_prim_path}에 "
              f"{self._joint_path} 생성", flush=True)
        return True

    def open(self) -> None:
        if self._attached:
            stage = omni.usd.get_context().get_stage()
            if stage.GetPrimAtPath(self._joint_path).IsValid():
                stage.RemovePrim(self._joint_path)
        self._attached = False

    def is_closed(self) -> bool:
        return self._attached

    def is_open(self) -> bool:
        return not self._attached


def get_world_pos(prim):
    """100.cart_to_trunk_dual_side_holonomic.py와 동일."""
    mat = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return np.array(mat.ExtractTranslation())


# ============================================================
# 씬 부트스트랩
# ============================================================

omni.usd.get_context().new_stage()
stage = omni.usd.get_context().get_stage()

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

cart_scene = build_cart_with_boxes(
    stage, simulation_app, CART_USD, position=CART_SCENE_POSITION, rot_z=CART_SCENE_ROT_Z)
build_vehicle(
    stage, simulation_app, CAR_USD, position=VEHICLE_SCENE_POSITION, rot_z=VEHICLE_SCENE_ROT_Z)

# 100.py의 CART_CLEAR_X와 동일 공식(회전 안전지대) - 카트의 "길이" 축 연장선상,
# 카트 bbox 바깥에 섀시 중심이 X축만으로 이미 안전 거리 떨어지게 스폰한다(yaw가
# 스폰 직후 90도든 회전 도중 어떤 중간값이든, 심지어 45도여도 절대 카트와 안
# 겹친다 - AABB가 한 축만 분리돼도 안 겹친다는 성질). cart_scene의 실측
# bbox(cart_center_xy/cart_half_x)에 따라 매번 다시 계산된다.
_cart_center_xy = cart_scene.cart_center_xy
CART_CLEAR_X = (_cart_center_xy[0] + cart_scene.cart_half_x
                + CHASSIS_HALF_LENGTH_EFFECTIVE + CHASSIS_HALF_WIDTH_EFFECTIVE
                + CART_STANDOFF_MARGIN)
CHASSIS_SPAWN_XY = (CART_CLEAR_X, _cart_center_xy[1])

chassis_path, hub_joint_paths, k_factor = build_holonomic_base(
    stage, CHASSIS_SPAWN_XY, BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT)

m0609_path, m0609_base_link_path, lift_translate_op, lift_scale_op = mount_m0609(stage, LIFT_MIN)

for _ in range(10):
    simulation_app.update()

m0609_robot = SingleArticulation(prim_path=m0609_base_link_path, name="m0609_arm")
base_robot = SingleArticulation(prim_path=chassis_path, name="holo_base")

world.reset()
base_robot.initialize(physics_sim_view=world.physics_sim_view)
m0609_robot.initialize(physics_sim_view=world.physics_sim_view)

# 100.cart_to_trunk_dual_side_holonomic.py와 동일 - USD 기본 관절값(전부 0도)은
# 이 팔+RMPflow 조합에 특이점에 가까운 자세라, 그대로 두고 RMPflow에게 "이
# 자세를 유지해라"라고 시키면 팔이 계속 불안정하게 튄다(실측 확인 - 섀시/
# 리프트는 완벽히 정지해 있는데 팔만 흔들림). 100.py가 조인트3/5를 90도로
# 미리 꺾어 특이점을 피한 자세에서 시작하는 것과 동일하게 맞춘다.
_init_joints = np.zeros(m0609_robot.num_dof)
if "joint_3" in m0609_robot.dof_names:
    _init_joints[m0609_robot.dof_names.index("joint_3")] = np.pi / 2
if "joint_5" in m0609_robot.dof_names:
    _init_joints[m0609_robot.dof_names.index("joint_5")] = np.pi / 2
m0609_robot.set_joint_positions(_init_joints)

hub_dof_indices = [base_robot.dof_names.index(Path(p).name) for p in hub_joint_paths]


def holo_forward(vx: float, vy: float, wz: float) -> ArticulationAction:
    """100.cart_to_trunk_dual_side_holonomic.py와 동일 - 로봇 로컬 프레임 속도(vx,vy,wz)
    -> 4개 허브 조인트 목표 각속도 ArticulationAction."""
    speeds = mecanum_wheel_speeds(vx, vy, wz, WHEEL_RADIUS, k_factor)
    return ArticulationAction(joint_velocities=speeds, joint_indices=hub_dof_indices)


lift_state = {"h": LIFT_MIN}


def set_lift_height(h: float) -> None:
    """100.cart_to_trunk_dual_side_holonomic.py의 set_lift_height()와 동일 -
    물리 조인트가 아니라 M0609 베이스를 매 스텝 직접 teleport한다."""
    chassis_pos, chassis_quat = base_robot.get_world_pose()
    target_pos = np.array([float(chassis_pos[0]), float(chassis_pos[1]), float(chassis_pos[2]) + h])
    m0609_robot.set_world_pose(position=target_pos, orientation=chassis_quat)
    m0609_robot.set_linear_velocity(np.zeros(3))
    m0609_robot.set_angular_velocity(np.zeros(3))
    column_base_z = float(chassis_pos[2]) + LIFT_MIN
    column_len = max(float(h) - LIFT_MIN, 0.001)
    lift_scale_op.Set(Gf.Vec3f(1.0, 1.0, column_len))
    lift_translate_op.Set(Gf.Vec3d(float(chassis_pos[0]), float(chassis_pos[1]), column_base_z + column_len / 2.0))


# ---------------- 그리퍼 ----------------
gripper_body_path = f"{m0609_path}/{GRIPPER_BODY_NAME}"
ee_path = f"{m0609_path}/{EE_LINK_NAME}"

for _ in range(100):
    set_lift_height(lift_state["h"])
    # render=True 고정 - 카메라 ROS2 브리지 그래프가 이미 만들어진 뒤라(아래 참고),
    # 렌더 안 하는 스텝을 섞으면 omni.graph.image 플러그인이 크래시했다(실측 확인).
    world.step(render=True)

# 카메라는 world.reset() + 100 렌더 스텝을 마친 뒤(9.attach_vgp20_camera.py와
# 비슷한 순서)에 초기화한다 - 실측 결과 이 타이밍 자체가 크래시를 좌우하진
# 않았지만(sensor_bridge.py 모듈 docstring 참고 - 진짜 원인은 OGN 노드였다),
# 이미 검증된 순서라 되돌리지 않고 유지한다.
#
# ⚠️ 실측으로 찾은 버그(2026-07-27, RMPflow "도달 못함" 조사 중) - RSD455의
# IMU 센서/중첩 RigidBodyAPI 정리(cleanup_camera_reference_asset)를 예전엔
# CART2TRUNK_ENABLE_CAMERA_BRIDGE=1일 때만(카메라 데이터가 실제로 필요할 때만)
# 호출했다 - 그런데 이건 "카메라 데이터 품질" 문제가 아니라 M0609 articulation
# 자체의 물리적 정합성 문제였다(9.attach_vgp20_camera.py 원문 주석: 이 IMU가
# "병합 articulation velocity tensor shape" 에러의 원인). 카메라 브리지를 끈
# 채로 그리퍼 목표 접근처럼 팔을 크게 움직이는 시나리오를 반복 실측한 결과,
# 이 정리를 안 하면 섀시/팔 pose가 NaN으로 발산하는 게 재현됐다(카메라 브리지를
# 켠 채로 한 카트 스캔 테스트들에서는 이 정리가 항상 됐었어서 이 문제 자체가
# 안 드러났었다). 그래서 카메라 사용 여부와 무관하게 항상 정리한다.
_depth_camera_prim_path, _all_camera_candidates = find_camera_prim_path(
    stage, m0609_path, DEPTH_CAMERA_NAME_HINT)
if _depth_camera_prim_path is None:
    raise RuntimeError(f"카메라 프림을 못 찾음 - 발견된 카메라 후보: {_all_camera_candidates}")
cleanup_camera_reference_asset(stage, _depth_camera_prim_path)

depth_camera = None
if os.environ.get("CART2TRUNK_ENABLE_CAMERA_BRIDGE", "0") == "1":
    print(f"[CAMERA] depth 카메라: {_depth_camera_prim_path} "
          f"(후보 전체: {_all_camera_candidates})", flush=True)
    depth_camera = initialize_depth_camera(_depth_camera_prim_path, CAMERA_WIDTH, CAMERA_HEIGHT)
    for _ in range(10):
        world.step(render=True)
    print(f"[CAMERA] {DEPTH_TOPIC}, {CAMERA_INFO_TOPIC} 발행 준비 완료 (frame_id={CAMERA_FRAME_ID}) - "
          f"PlatformControllerNode가 매 스텝 rclpy로 직접 발행", flush=True)
else:
    print("[CAMERA] CART2TRUNK_ENABLE_CAMERA_BRIDGE=1이 아니라서 카메라 브리지 비활성화 "
          "(RSD455 IMU/RigidBodyAPI 정리는 이미 완료됨)", flush=True)

gripper = DynamicSuctionGripper(
    end_effector_prim_path=ee_path, gripper_body_path=gripper_body_path, tip_local_offset=TIP_LOCAL_OFFSET,
)
print("[그리퍼] 준비 완료 - 목표는 /gripper/set_target(world PoseStamped)으로만 설정된다 "
      "(더 이상 부트스트랩 시 고정 테스트 박스를 쓰지 않음)", flush=True)


# ---------------- M0609 팔 (RMPflow) ----------------
controller = RMPFlowController(
    name="m0609_rmpflow", robot_articulation=m0609_robot, physics_dt=1.0 / 60.0,
    urdf_path=M0609_URDF_PATH, robot_description_path=M0609_DESCRIPTION_PATH,
    rmpflow_config_path=M0609_RMPFLOW_CONFIG_PATH, end_effector_frame_name=EE_LINK_NAME,
)


def sync_rmp_base() -> None:
    """100.cart_to_trunk_dual_side_holonomic.py와 동일 - 리프트/모바일베이스가 매 스텝
    M0609 베이스를 움직이므로, RMPflow가 아는 "로봇 베이스가 어디 있는지"도 매 스텝
    다시 맞춰야 IK가 엉뚱한 기준으로 풀리지 않는다."""
    chassis_pos, chassis_quat = base_robot.get_world_pose()
    base_pos = np.array([float(chassis_pos[0]), float(chassis_pos[1]), float(chassis_pos[2]) + lift_state["h"]])
    controller._default_position = base_pos
    controller._default_orientation = chassis_quat
    controller.rmp_flow.set_robot_base_pose(robot_position=base_pos, robot_orientation=chassis_quat)


def _ee_world_pose():
    ee_prim = stage.GetPrimAtPath(ee_path)
    mat = UsdGeom.Xformable(ee_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    position = np.array(mat.ExtractTranslation(), dtype=float)
    q = mat.ExtractRotationQuat()
    imag = q.GetImaginary()
    quat_wxyz = np.array([q.GetReal(), imag[0], imag[1], imag[2]], dtype=float)
    return position, quat_wxyz


def _gripper_tip_world_pose():
    """100.py의 measure_tip_world_pos()와 동일한 계산(gripper_body_path의 실제
    world transform + TIP_LOCAL_OFFSET) - link_6과 흡착 팁 사이엔 고정 마운트
    오프셋(실측 13cm대)이 있어서 link_6 자세만으로는 팁이 어디 있는지 알 수
    없다. cart2trunk_motion이 이 오프셋 상수를 몰라도 폐루프 제어를 할 수
    있도록, 그 계산을 아는 이 노드가 직접 재고 /gripper/tip_pose로 발행한다.
    회전은 link_6과 함께 움직이는 그리퍼 바디의 회전을 그대로 쓴다(팁 자체는
    별도 관절이 없어 그리퍼 바디와 항상 같은 방향)."""
    gripper_body_prim = stage.GetPrimAtPath(gripper_body_path)
    mat = UsdGeom.Xformable(gripper_body_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    position = np.array(mat.Transform(Gf.Vec3d(*TIP_LOCAL_OFFSET)), dtype=float)
    q = mat.ExtractRotationQuat()
    imag = q.GetImaginary()
    quat_wxyz = np.array([q.GetReal(), imag[0], imag[1], imag[2]], dtype=float)
    return position, quat_wxyz


def _find_nearest_box_prim(target_world_pos: np.ndarray):
    """cart_scene.box_objects(카트 씬 부트스트랩이 이미 알고 있는 실제 박스
    prim들)에서 target_world_pos(world top-center)에 가장 가까운 박스의
    top-center를 찾는다. XY/Z 오차를 각각 TARGET_SEARCH_*_TOLERANCE_M 이내로
    따로 확인한다(사용자 지시 - 구형 거리 하나로 합치면 DynamicSuctionGripper가
    이미 겪은 것과 같은 문제, 수평 오차가 수직 여유를 잡아먹는 문제가 재발한다).
    매칭 실패 시 (None, None) 반환.

    반환값의 half_height는 cart_scene.box_known_size(스폰 시점 크기 - 실측
    수축/변형 없다고 가정)에서 가져온다 - Box3D 메시지의 size는 이 함수에서
    쓰지 않는다(cart2trunk_motion이 top-center를 이미 계산해 보내주므로, 이
    노드는 "어떤 실제 박스인지" 식별에만 크기를 쓴다)."""
    best_path, best_score = None, None
    for prim_path, box in cart_scene.box_objects.items():
        pos, _quat = box.get_world_pose()
        half_height = float(cart_scene.box_known_size[prim_path][2]) / 2.0
        top_center = np.array(
            [pos[0], pos[1], pos[2] + half_height], dtype=float)
        xy_err = float(np.hypot(
            target_world_pos[0] - top_center[0], target_world_pos[1] - top_center[1]))
        z_err = float(abs(target_world_pos[2] - top_center[2]))
        xy_ok = xy_err <= TARGET_SEARCH_HORIZONTAL_TOLERANCE_M
        z_ok = z_err <= TARGET_SEARCH_VERTICAL_TOLERANCE_M
        if not (xy_ok and z_ok):
            continue
        score = xy_err + z_err
        if best_score is None or score < best_score:
            best_score, best_path, best_half_height = score, prim_path, half_height
    if best_path is None:
        return None, None
    return best_path, best_half_height


# 시작 시점의 실제 link_6 자세를 초기 목표로 잡는다 - 그래야 노드가 뜨자마자 임의
# 위치로 팔이 튀지 않고, 실제 /m0609/move_to_pose 명령이 올 때까지 제자리를 유지한다.
_initial_ee_pos, _initial_ee_quat_wxyz = _ee_world_pose()

# ============================================================
# ROS 2 인터페이스 (표준 메시지만)
# ============================================================

# 100.py의 drive_to() 기본값(max_speed=0.4, max_wz=0.2)과 같은 안전 상한 - cmd_vel로
# 직접 뭐가 들어와도 이 이상은 못 나가게 막는다(저수준 컨트롤러의 책임).
MOBILE_BASE_MAX_LINEAR_MPS = 0.4
MOBILE_BASE_MAX_ANGULAR_RADPS = 0.2


class PlatformControllerNode(Node):

    def __init__(self):
        super().__init__("platform_controller_node")
        self._target_h = LIFT_MIN
        self.create_subscription(Float32, "/lift/move", self._on_lift_move, 10)
        self._lift_state_pub = self.create_publisher(Float32, "/lift/state", 10)

        self._cmd_vel = (0.0, 0.0, 0.0)
        self.create_subscription(Twist, "/mobile_base/cmd_vel", self._on_cmd_vel, 10)
        self._base_state_pub = self.create_publisher(PoseStamped, "/mobile_base/state", 10)

        self.create_service(Trigger, "/gripper/activate", self._on_gripper_activate)
        self.create_service(Trigger, "/gripper/release", self._on_gripper_release)
        self._gripper_state_pub = self.create_publisher(Bool, "/gripper/state", 10)

        # /gripper/set_target: world PoseStamped(박스 top-center) - cart2trunk_motion이
        # anchor로 이미 world 좌표를 계산해 보낸다. cart2trunk_interfaces 커스텀
        # 메시지는 이 노드(Isaac 번들 python3.11)에서 못 쓰므로(다른 인터페이스와
        # 동일 이유) box_id/prim_path 없이 pose만 받고, 실제로 어떤 박스 prim인지는
        # 이 노드가 스스로(cart_scene.box_objects 기준 최근접 탐색) 알아낸다 -
        # 자세한 설계 근거는 _find_nearest_box_prim()/_on_gripper_set_target() 참고.
        self._gripper_target_prim_path = None
        self._gripper_target_received_time = None
        self.create_subscription(
            PoseStamped, "/gripper/set_target", self._on_gripper_set_target, 10)

        self._m0609_target_pos = _initial_ee_pos
        self._m0609_target_quat_wxyz = _initial_ee_quat_wxyz
        self.create_subscription(PoseStamped, "/m0609/move_to_pose", self._on_m0609_move_to_pose, 10)
        self._m0609_state_pub = self.create_publisher(PoseStamped, "/m0609/state", 10)
        self._gripper_tip_pose_pub = self.create_publisher(PoseStamped, "/gripper/tip_pose", 10)

        # depth_camera가 None이면(CART2TRUNK_ENABLE_CAMERA_BRIDGE=0) 퍼블리셔 자체를
        # 안 만든다 - sensor_bridge.py 모듈 docstring 참고, OGN 그래프(isaacsim.ros2.bridge)
        # 대신 Camera 래퍼로 직접 뽑은 데이터를 여기서 표준 rclpy 퍼블리셔로 발행한다.
        self._depth_pub = None
        self._camera_info_pub = None
        if depth_camera is not None:
            self._depth_pub = self.create_publisher(Image, DEPTH_TOPIC, 10)
            self._camera_info_pub = self.create_publisher(CameraInfo, CAMERA_INFO_TOPIC, 10)

        self.get_logger().info(
            f"platform_controller_node ready (lift + mobile_base + gripper + m0609) - "
            f"LIFT_MIN={LIFT_MIN:.3f} LIFT_MAX={LIFT_MAX:.3f}"
        )

    def _on_lift_move(self, msg: Float32) -> None:
        clamped = float(np.clip(msg.data, LIFT_MIN, LIFT_MAX))
        if clamped != msg.data:
            self.get_logger().warn(
                f"/lift/move 목표({msg.data:.3f})가 범위[{LIFT_MIN:.3f},{LIFT_MAX:.3f}] 밖 - {clamped:.3f}로 클램프"
            )
        self._target_h = clamped

    def step_lift_toward_target(self) -> None:
        current = lift_state["h"]
        delta = float(np.clip(self._target_h - current, -LIFT_MAX_STEP_M, LIFT_MAX_STEP_M))
        lift_state["h"] = current + delta
        set_lift_height(lift_state["h"])

    def publish_lift_state(self) -> None:
        msg = Float32()
        msg.data = float(lift_state["h"])
        self._lift_state_pub.publish(msg)

    def _on_cmd_vel(self, msg: Twist) -> None:
        vx = float(np.clip(msg.linear.x, -MOBILE_BASE_MAX_LINEAR_MPS, MOBILE_BASE_MAX_LINEAR_MPS))
        vy = float(np.clip(msg.linear.y, -MOBILE_BASE_MAX_LINEAR_MPS, MOBILE_BASE_MAX_LINEAR_MPS))
        wz = float(np.clip(msg.angular.z, -MOBILE_BASE_MAX_ANGULAR_RADPS, MOBILE_BASE_MAX_ANGULAR_RADPS))
        self._cmd_vel = (vx, vy, wz)

    def apply_cmd_vel(self) -> None:
        base_robot.apply_action(holo_forward(*self._cmd_vel))

    def publish_base_state(self) -> None:
        pos, quat_wxyz = base_robot.get_world_pose()
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])
        # Isaac Sim 쿼터니언은 (w,x,y,z) 관례, ROS geometry_msgs는 (x,y,z,w) 관례.
        w, x, y, z = quat_wxyz
        msg.pose.orientation.x = float(x)
        msg.pose.orientation.y = float(y)
        msg.pose.orientation.z = float(z)
        msg.pose.orientation.w = float(w)
        self._base_state_pub.publish(msg)

    def _on_gripper_set_target(self, msg: PoseStamped) -> None:
        if msg.header.frame_id != "world":
            self.get_logger().warn(
                f"/gripper/set_target frame_id='{msg.header.frame_id}' - world만 지원, 무시"
            )
            return
        p = msg.pose.position
        target_world = np.array([p.x, p.y, p.z], dtype=float)
        prim_path, _half_height = _find_nearest_box_prim(target_world)
        if prim_path is None:
            self._gripper_target_prim_path = None
            self._gripper_target_received_time = None
            self.get_logger().warn(
                f"/gripper/set_target={np.round(target_world, 3)} 근처"
                f"(허용 {TARGET_SEARCH_HORIZONTAL_TOLERANCE_M:.2f}/"
                f"{TARGET_SEARCH_VERTICAL_TOLERANCE_M:.2f}m)에 실제 박스가 없음 - 목표 무효화"
            )
            return
        self._gripper_target_prim_path = prim_path
        self._gripper_target_received_time = time.monotonic()
        self.get_logger().info(
            f"/gripper/set_target -> {prim_path} (world={np.round(target_world, 3)})")

    def _on_gripper_activate(self, request, response):
        # "이전 target이 재사용되지 않도록" - 성공/실패 무관하게 이번 시도에서 소비한다
        # (사용자 지시). activate가 set_target 콜백보다 먼저 도착하는 프로세스 간
        # 순서 경쟁에 대비해, cart2trunk_motion 쪽에서 set_target 발행 후 짧게(0.1~0.2초)
        # 대기하고 activate를 부르기로 했다 - 이 노드 쪽은 그 대기가 실패해도 안전하도록
        # target_received_time으로 신선도(GRIPPER_TARGET_TIMEOUT_SEC)까지 방어한다.
        prim_path = self._gripper_target_prim_path
        received_time = self._gripper_target_received_time
        self._gripper_target_prim_path = None
        self._gripper_target_received_time = None

        if prim_path is None:
            response.success = False
            response.message = "흡착 실패: /gripper/set_target으로 유효한 목표가 설정되지 않음"
            return response
        age_sec = time.monotonic() - received_time
        if age_sec > GRIPPER_TARGET_TIMEOUT_SEC:
            response.success = False
            response.message = f"흡착 실패: 목표가 오래됨({age_sec:.2f}s > {GRIPPER_TARGET_TIMEOUT_SEC}s)"
            return response

        half_height = float(cart_scene.box_known_size[prim_path][2]) / 2.0
        response.success = gripper.close_on_prim(
            prim_path, half_height, GRASP_HORIZONTAL_MARGIN, GRASP_VERTICAL_TOLERANCE,
        )
        response.message = "흡착 성공" if response.success else "흡착 실패(대상 기하 조건 밖 - 접근 자세를 확인하세요)"
        return response

    def _on_gripper_release(self, request, response):
        gripper.open()
        response.success = True
        response.message = "흡착 해제"
        return response

    def publish_gripper_state(self) -> None:
        msg = Bool()
        msg.data = gripper.is_closed()
        self._gripper_state_pub.publish(msg)

    def _on_m0609_move_to_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        o = msg.pose.orientation
        self._m0609_target_pos = np.array([p.x, p.y, p.z], dtype=float)
        # ROS geometry_msgs 쿼터니언(x,y,z,w) -> Isaac Sim 관례(w,x,y,z).
        self._m0609_target_quat_wxyz = np.array([o.w, o.x, o.y, o.z], dtype=float)

    def apply_m0609_target(self) -> None:
        sync_rmp_base()
        actions = controller.forward(
            target_end_effector_position=self._m0609_target_pos,
            target_end_effector_orientation=self._m0609_target_quat_wxyz,
        )
        m0609_robot.apply_action(actions)

    def publish_m0609_state(self) -> None:
        pos, quat_wxyz = _ee_world_pose()
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])
        w, x, y, z = quat_wxyz
        msg.pose.orientation.x = float(x)
        msg.pose.orientation.y = float(y)
        msg.pose.orientation.z = float(z)
        msg.pose.orientation.w = float(w)
        self._m0609_state_pub.publish(msg)

    def publish_gripper_tip_pose(self) -> None:
        pos, quat_wxyz = _gripper_tip_world_pose()
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])
        w, x, y, z = quat_wxyz
        msg.pose.orientation.x = float(x)
        msg.pose.orientation.y = float(y)
        msg.pose.orientation.z = float(z)
        msg.pose.orientation.w = float(w)
        self._gripper_tip_pose_pub.publish(msg)

    def publish_camera(self) -> None:
        """depth_camera가 활성화된 경우에만 호출된다(main() 참고). Camera 래퍼가
        직접 뽑은 depth/intrinsics를 sensor_msgs 메시지로 포장해 발행한다 - OGN
        그래프(ROS2CameraHelper 등)는 세그폴트 원인이라 아예 쓰지 않는다
        (sensor_bridge.py 모듈 docstring 참고)."""
        stamp = self.get_clock().now().to_msg()
        depth_msg = depth_to_image_msg(depth_camera, CAMERA_FRAME_ID, stamp)
        if depth_msg is not None:
            self._depth_pub.publish(depth_msg)
        self._camera_info_pub.publish(camera_info_msg(depth_camera, CAMERA_FRAME_ID, stamp))


# 렌더링 빈도를 줄이는 최적화(RENDER_EVERY_N_STEPS)를 한때 시도했었으나, 당시
# 카메라 ROS2 브리지가 OGN 그래프(OnPlaybackTick+IsaacCreateRenderProduct+
# ROS2CameraHelper)로 구현돼 있어서 렌더를 건너뛰는 스텝이 섞이면 그 그래프가
# omni.graph.image 플러그인 안에서 크래시했다(실측, 2회 재현) - 그래서 매 스텝
# 렌더링으로 되돌렸다. 지금은 그 OGN 그래프 자체를 더 이상 안 쓰므로(카메라는
# Camera 래퍼 + 표준 rclpy 퍼블리셔 - sensor_bridge.py 모듈 docstring 참고)
# 이 크래시 경로는 사라졌지만, render=True 매 스텝 정책은 이미 검증된 동작이라
# 그대로 유지한다.
def main():
    rclpy.init()
    node = PlatformControllerNode()

    publish_every_n = 10
    i = 0
    try:
        while simulation_app.is_running():
            rclpy.spin_once(node, timeout_sec=0.0)
            node.step_lift_toward_target()
            node.apply_cmd_vel()
            node.apply_m0609_target()
            world.step(render=True)
            i += 1
            if i % publish_every_n == 0:
                node.publish_lift_state()
                node.publish_base_state()
                node.publish_gripper_state()
                node.publish_m0609_state()
                node.publish_gripper_tip_pose()
                if depth_camera is not None:
                    node.publish_camera()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        simulation_app.close()


if __name__ == "__main__":
    main()
