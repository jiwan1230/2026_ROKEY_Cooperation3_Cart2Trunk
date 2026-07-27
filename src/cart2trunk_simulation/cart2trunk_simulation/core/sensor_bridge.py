"""EDU 저장소 88.cart_scan_holonomic.py/89.trunk_scan_holonomic.py/
99.cart_scan_dual_side_holonomic.py의 setup_ros2_camera_bridge()/
find_camera_prim_path() 이관 - M0609에 이미 마운트된 카메라 프림(vgp20 그리퍼의
side_camera_mount/realsense_d455)에서 실시간 Depth+CameraInfo를 ROS2로
발행한다.

[찾아본 결과 - 중요] "이미 저장된 씬 USD 안에 ROS2_Cart_Scan_Camera_Graph가
있다"는 예전 인수인계 기록은 확인 결과 부정확했다 - 이 그래프는 .usd/.usda
어디에도 저장돼 있지 않고, 위 EDU 스크립트들이 매 실행마다 og.Controller.edit()
로 그때그때 새로 만드는 것이다(파일 검색으로 확인 - 모든 .usd/.usda에 해당
문자열 없음). 그래서 이 모듈이 그 역할을 대신한다 - platform_controller_node.py가
부트스트랩 시 한 번 호출하면 된다.

[크래시 원인 확정 (2026-07-27) - IsaacCreateRenderProduct/ROS2CameraHelper OGN
노드 자체, Camera 클래스가 아니다]
기존에는 og.Controller.edit()로 OnPlaybackTick -> IsaacCreateRenderProduct ->
{ROS2CameraHelper(depth), ROS2CameraInfoHelper} OmniGraph를 만들어 ROS2로
발행했다 - 이 PC의 GPU(Blackwell, RTX 5080 Laptop)+드라이버(580.159.03)
조합에서 이 그래프 구성 자체가 세그폴트를 냈다(최초 6번 중 5번, isaac_python
5.1.0-rc.19 및 NVIDIA 공식 pip 6.0.0.0 양쪽 다 재현, NVIDIA 이슈 #643/#651과
동일 시그니처). "카메라 렌더 프로덕트 생성 타이밍이 너무 이르다"는 가설로
world.reset()+100 렌더 스텝 이후로 지연시켜봤지만(9.attach_vgp20_camera.py와
동일 순서) 크래시 지점이 조금도 안 옮겨졌다 - IMU 비활성화/RigidBodyAPI 제거를
먼저 적용해도 마찬가지였다. 반복 재현 스크립트로 정확히 좁힌 결과: Camera
래퍼의 initialize()+get_depth()/get_rgb()/get_intrinsics_matrix()는 3/3 완전히
안정적으로 동작하고(9.attach_vgp20_camera.py가 바로 이 경로만 쓰길래 안전했던
것), og.Controller.edit()로 IsaacCreateRenderProduct 노드를 만드는 순간부터만
3/3 크래시한다 - 즉 크래시는 카메라/렌더링 자체가 아니라 isaacsim.ros2.bridge가
제공하는 그 OGN 노드 구현(또는 그 노드가 만드는 "두 번째" 렌더 프로덕트)에
국한된다. 그래서 이 모듈은 OGN 그래프를 완전히 버리고, Camera 래퍼로 직접 뽑은
depth/intrinsics를 platform_controller_node.py가 표준 rclpy 퍼블리셔(이미
lift/base/gripper/m0609 상태 발행에 쓰고 있어 안정성이 검증된 경로)로 발행하는
방식으로 바꿨다 - enable_extension("isaacsim.ros2.bridge")도, omni.graph.core
IsaacCreateRenderProduct/ROS2CameraHelper/ROS2CameraInfoHelper도 더 이상
필요 없다.
"""
import numpy as np
from isaacsim.sensors.camera import Camera
from pxr import UsdPhysics


def find_camera_prim_path(stage, root_path: str, name_hint: str = "Depth"):
    """root_path 서브트리에서 이름에 name_hint(대소문자 무관)가 들어간 카메라
    프림을 찾는다 - 경로를 하드코딩하지 않기 위함(M0609 asset 사본마다 정확한
    경로가 조금씩 다를 수 있음, HANDOFF_MSI2.md 2.3절과 같은 이유).
    반환값: (선택된 경로 또는 None, 발견된 카메라 후보 전체 경로 리스트)."""
    from pxr import Usd, UsdGeom

    root_prim = stage.GetPrimAtPath(root_path)
    candidates = [
        str(prim.GetPath()) for prim in Usd.PrimRange(root_prim) if prim.IsA(UsdGeom.Camera)]
    for c in candidates:
        if name_hint.lower() in c.lower():
            return c, candidates
    return (candidates[0] if candidates else None), candidates


def cleanup_camera_reference_asset(stage, camera_prim_path: str) -> None:
    """9.attach_vgp20_camera.py에서 검증된 후처리를 그대로 이관 - RealSense
    rsd455.usd 참조 에셋에는 딸려오는 IMU 센서와 중첩 RigidBodyAPI가 있는데,
    이미 리지드바디/관절을 가진 M0609 articulation(vgp20 그리퍼) 밑에 그대로
    두면 둘 다 문제를 일으킨다(9.py 실측 확인, 주석 원문):
    - RSD455/Imu_Sensor: "병합 articulation velocity tensor shape" 에러 유발
    - RSD455의 RigidBodyAPI: 이미 RigidBodyAPI가 있는 vgp20 밑에 중첩되면서
      "missing xformstack reset when child of another enabled rigid body"
      경고 + DOF/바디 개수 불일치 에러를 유발할 수 있음
    (참고: 모듈 docstring의 크래시 원인 확정 실측에 따르면 이 정리 자체는 OGN
    세그폴트의 원인은 아니었다 - 다만 물리/articulation 정합성을 위해 여전히
    필요해서 유지한다.) 카메라 프림(.../RSD455/Camera_Pseudo_Depth)의 부모가
    RSD455이므로 경로를 하드코딩하지 않고 상대적으로 찾는다."""
    rsd455_prim = stage.GetPrimAtPath(camera_prim_path).GetParent()
    if not rsd455_prim.IsValid():
        return

    imu_prim = stage.GetPrimAtPath(f"{rsd455_prim.GetPath()}/Imu_Sensor")
    if imu_prim.IsValid():
        imu_prim.SetActive(False)
        print(f"[IMU] {imu_prim.GetPath()} 비활성화", flush=True)

    if rsd455_prim.HasAPI(UsdPhysics.RigidBodyAPI):
        rsd455_prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
        print(f"[RIGID BODY] {rsd455_prim.GetPath()}에서 RigidBodyAPI 제거", flush=True)


def initialize_depth_camera(camera_prim_path: str, width: int = 640, height: int = 480) -> Camera:
    """카메라 프림을 Camera 래퍼로 초기화하고 depth/rgb 프레임 애노테이터를
    붙인다. add_distance_to_image_plane_to_frame()을 빼먹으면 depth 프레임
    자체가 안 붙어서(88.py 실측 확인 - get_pointcloud()가 매번 빈 배열 반환)
    get_depth()도 매번 None을 반환한다. 이 경로(OGN 그래프 없이 Camera 래퍼만
    직접 씀)는 반복 재현으로 안정성이 확인됐다 - 모듈 docstring 참고."""
    camera = Camera(prim_path=camera_prim_path, resolution=(width, height))
    camera.initialize()
    camera.add_distance_to_image_plane_to_frame()
    camera.add_rgb_to_frame()
    return camera


def depth_to_image_msg(camera: Camera, frame_id: str, stamp):
    """Camera.get_depth()(미터 단위 distance-to-image-plane, float32 (h,w))를
    sensor_msgs/Image(encoding="32FC1")로 직접 포장한다 - ROS2CameraHelper
    OGN 노드가 하던 일을 크래시 없는 경로로 대신한다."""
    from sensor_msgs.msg import Image

    depth = camera.get_depth()
    if depth is None:
        return None
    depth = np.asarray(depth, dtype=np.float32)
    msg = Image()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height, msg.width = depth.shape[:2]
    msg.encoding = "32FC1"
    msg.is_bigendian = 0
    msg.step = int(msg.width * 4)
    msg.data = depth.tobytes()
    return msg


def camera_info_msg(camera: Camera, frame_id: str, stamp):
    """Camera.get_intrinsics_matrix()로부터 sensor_msgs/CameraInfo를 직접
    구성한다(핀홀, 왜곡 없음으로 가정 - 시뮬레이션 카메라라 렌즈 왜곡이 없다).
    ROS2CameraInfoHelper OGN 노드가 하던 일을 크래시 없는 경로로 대신한다."""
    from sensor_msgs.msg import CameraInfo

    k = np.asarray(camera.get_intrinsics_matrix(), dtype=float)
    width, height = camera.get_resolution()
    fx, fy, cx, cy = float(k[0][0]), float(k[1][1]), float(k[0][2]), float(k[1][2])
    msg = CameraInfo()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.width = int(width)
    msg.height = int(height)
    msg.distortion_model = "plumb_bob"
    msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return msg
