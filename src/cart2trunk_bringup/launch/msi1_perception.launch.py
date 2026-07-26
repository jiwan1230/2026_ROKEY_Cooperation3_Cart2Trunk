"""MSI1: Depth 기반 카트/트렁크 인식.

box_scan_action_server/trunk_scan_action_server 둘 다 아직 실시간 Depth+TF 누적이
없는 replay 전용 구현이다(MSI2 카메라 브리지가 준비되면 이어서 구현) - 그래서 두
노드 모두 fixture 경로가 비어 있으면 goal을 abort하고 DEPTH_TIMEOUT을 반환한다.
launch 인자로 fixture 경로를 넘기면 그 파일/폴더를 replay한다(빈 문자열이 기본값 -
직접 `ros2 run`으로 띄울 때와 동일한 동작).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    box_fixture_arg = DeclareLaunchArgument(
        'box_fixture_input_path', default_value='',
        description='box_scan_action_server replay 입력 (.npy/.ply, m0609_base_link 좌표계로 병합된 point cloud)',
    )
    trunk_fixture_arg = DeclareLaunchArgument(
        'trunk_fixture_run_dir', default_value='',
        description='trunk_scan_action_server replay 입력 (pointcloud/trunk_pointcloud.npy + meta.json이 있는 run 폴더)',
    )

    return LaunchDescription([
        box_fixture_arg,
        trunk_fixture_arg,
        Node(
            package='cart2trunk_perception',
            executable='box_scan_action_server',
            name='box_scan_action_server',
            output='screen',
            parameters=[{'fixture_input_path': LaunchConfiguration('box_fixture_input_path')}],
        ),
        Node(
            package='cart2trunk_perception',
            executable='trunk_scan_action_server',
            name='trunk_scan_action_server',
            output='screen',
            parameters=[{'fixture_run_dir': LaunchConfiguration('trunk_fixture_run_dir')}],
        ),
    ])
