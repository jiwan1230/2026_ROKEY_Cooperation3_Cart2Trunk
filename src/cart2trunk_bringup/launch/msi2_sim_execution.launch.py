"""MSI2: Isaac Sim, 카메라 브리지, Coordinator, Pick & Place 실행, Safety.

이관 완료 시 여기에 다음 노드를 추가한다:
- cart2trunk_sim isaac_scene_manager / isaac_camera_bridge
- cart2trunk_execution scan_motion_action_server / pick_place_action_server
- cart2trunk_coordinator mission_coordinator_node
- cart2trunk_safety safety_supervisor_node
"""
from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([])
