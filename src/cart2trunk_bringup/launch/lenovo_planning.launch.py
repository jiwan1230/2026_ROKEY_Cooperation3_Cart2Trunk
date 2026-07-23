"""Lenovo: 적재 순서·배치 계획."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cart2trunk_planning',
            executable='loading_planner_action_server',
            name='loading_planner_action_server',
            output='screen',
        ),
    ])
