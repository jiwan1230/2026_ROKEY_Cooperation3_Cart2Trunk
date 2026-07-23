"""MSI1: Depth 기반 카트/트렁크 인식.

이관 완료 시 여기에 다음 노드를 추가한다:
- cart2trunk_perception box_scan_action_server (카트 내부 박스 스캔)
- cart2trunk_perception trunk_scan_action_server (트렁크 다중 시점 스캔 · 재스캔 포함)
"""
from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([])
