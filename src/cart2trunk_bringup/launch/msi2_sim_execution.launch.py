"""MSI2: Isaac Sim, 플랫폼·모션 실행, Coordinator, Safety.

[platform_controller_node는 이 launch에 없다 - 의도적]
cart2trunk_platform/platform_controller_node.py는 Isaac Sim 번들 파이썬(3.11 -
`isaac_python`/`~/dev_ws/isaac_sim/.../python.sh`)으로만 실행 가능하고, 시스템
ROS2(3.10)용 `ros2 run`/`colcon` 빌드 대상이 아니다(cart2trunk_platform/setup.py의
entry_points가 비어있는 이유 - platform_controller_node.py 자체 docstring 4.3절
참고). `ros2 launch`의 Node 액션은 시스템 python3.10 실행 파일을 전제로 하므로
여기 넣을 수 없다 - 별도 터미널에서 아래처럼 직접 띄워야 한다:

    export HEADLESS=1
    export CART2TRUNK_M0609_DIR=<이 PC의 M0609 자산 경로>
    export CART2TRUNK_GRIPPER_BODY_NAME=<이 M0609 asset의 그리퍼 바디 prim 이름>
    export LD_LIBRARY_PATH="<isaac_sim>/exts/isaacsim.ros2.bridge/humble/lib:$LD_LIBRARY_PATH"
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    isaac_python src/cart2trunk_platform/cart2trunk_platform/platform_controller_node.py

아래 두 값은 M0609 USD 자산 사본마다 실제로 다를 수 있다(실측 확인됨) -
자산을 새로 받으면 먼저 확인할 것.

이관 완료 시 여기에 추가할 노드:
- cart2trunk_simulation scene_manager / sensor_bridge (여전히 빈 패키지)
- cart2trunk_safety safety_supervisor_node (여전히 빈 패키지)
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cart2trunk_motion',
            executable='execute_pick_place_action_server',
            name='execute_pick_place_action_server',
            output='screen',
        ),
        Node(
            package='cart2trunk_coordinator',
            executable='mission_coordinator_node',
            name='mission_coordinator_node',
            output='screen',
        ),
    ])
