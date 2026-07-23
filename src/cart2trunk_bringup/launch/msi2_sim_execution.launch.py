"""MSI2: Isaac Sim, 플랫폼·모션 실행, Coordinator, Safety.

이관 완료 시 여기에 다음 노드를 추가한다:
- cart2trunk_simulation scene_manager / sensor_bridge
- cart2trunk_platform platform_manager_node
  (mobile_base_controller / lift_controller / m0609_controller / gripper_controller)
- cart2trunk_motion scan_motion_action_server / pick_from_cart_action_server /
  transport_action_server / place_in_trunk_action_server
- cart2trunk_coordinator mission_coordinator_node
- cart2trunk_safety safety_supervisor_node
"""
from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([])
