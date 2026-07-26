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

[ROS2CameraHelper 관련 함정 - 32.box_table_scan_setup.py 주석에 있던 내용]
inputs:type은 "rgb"/"depth"/"depth_pcl"/"instance_segmentation" 등은 되지만
"camera_info"는 없다 - camera_info는 별도 노드(ROS2CameraInfoHelper, type
입력 자체가 없음)로 발행해야 한다. 이미 이 모듈의 그래프 구성에 반영돼 있음.
"""
import omni.graph.core as og
from isaacsim.sensors.camera import Camera


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


def initialize_depth_camera(camera_prim_path: str, width: int = 640, height: int = 480) -> Camera:
    """카메라 프림을 Camera 래퍼로 초기화하고 depth/rgb 프레임 애노테이터를
    붙인다. add_distance_to_image_plane_to_frame()을 빼먹으면 depth 프레임
    자체가 안 붙어서(88.py 실측 확인 - get_pointcloud()가 매번 빈 배열 반환)
    ROS2CameraHelper의 렌더 프로덕트에도 depth 데이터가 없다."""
    camera = Camera(prim_path=camera_prim_path, resolution=(width, height))
    camera.initialize()
    camera.add_distance_to_image_plane_to_frame()
    camera.add_rgb_to_frame()
    return camera


def setup_ros2_camera_bridge(
    camera_prim_path: str, depth_topic: str = "/camera/depth",
    camera_info_topic: str = "/camera/camera_info",
    frame_id: str = "m0609_depth_camera_optical_frame",
    width: int = 640, height: int = 480,
    graph_path: str = "/World/ROS2_Cart_Scan_Camera_Graph",
) -> None:
    """OnPlaybackTick -> IsaacCreateRenderProduct -> {ROS2CameraHelper(depth),
    ROS2CameraInfoHelper} OmniGraph를 만든다. 그래프는 world.step()이 매
    프레임 자동으로 평가하므로(OnPlaybackTick), 별도로 매 스텝 이 그래프를
    돌리라고 명시적으로 호출할 필요는 없다."""
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("DepthPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CameraInfoPublish", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "DepthPublish.inputs:execIn"),
                ("CreateRenderProduct.outputs:renderProductPath",
                 "DepthPublish.inputs:renderProductPath"),
                ("CreateRenderProduct.outputs:execOut", "CameraInfoPublish.inputs:execIn"),
                ("CreateRenderProduct.outputs:renderProductPath",
                 "CameraInfoPublish.inputs:renderProductPath"),
            ],
            keys.SET_VALUES: [
                ("CreateRenderProduct.inputs:cameraPrim", camera_prim_path),
                ("CreateRenderProduct.inputs:width", width),
                ("CreateRenderProduct.inputs:height", height),
                ("DepthPublish.inputs:type", "depth"),
                ("DepthPublish.inputs:topicName", depth_topic),
                ("DepthPublish.inputs:frameId", frame_id),
                ("DepthPublish.inputs:resetSimulationTimeOnStop", True),
                ("CameraInfoPublish.inputs:topicName", camera_info_topic),
                ("CameraInfoPublish.inputs:frameId", frame_id),
                ("CameraInfoPublish.inputs:resetSimulationTimeOnStop", True),
            ],
        },
    )
