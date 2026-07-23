"""단일 PC(개발/시뮬레이션 검증용)에서 세 역할을 모두 기동.

msi1_perception.launch.py + msi2_sim_execution.launch.py + lenovo_planning.launch.py 를 include한다.
개별 launch 파일에 실제 노드가 채워진 뒤 사용한다.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare('cart2trunk_bringup')

    def include(launch_file):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [bringup_share, '/launch/', launch_file]
            )
        )

    return LaunchDescription([
        include('msi1_perception.launch.py'),
        include('msi2_sim_execution.launch.py'),
        include('lenovo_planning.launch.py'),
    ])
