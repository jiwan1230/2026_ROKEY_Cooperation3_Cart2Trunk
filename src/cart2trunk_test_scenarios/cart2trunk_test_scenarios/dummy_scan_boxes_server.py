"""ScanBoxes.action 더미 서버.

box_top_extractor.py(카트 박스 검출)는 아직 이관되지 않았다 - 이 더미는
cart2trunk_coordinator의 상태 머신/오케스트레이션 로직을 실제 카메라 없이
end-to-end로 검증하기 위한 통신 스텁이다 (GPT 설계안 "2단계: Dummy 노드 통신"
단계). 고정된 박스 목록(Large/Medium/Small)을 짧은 지연 후 반환한다.
"""
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from cart2trunk_interfaces.action import ScanBoxes
from cart2trunk_interfaces.msg import Box3D

_DUMMY_BOXES = [
    ('Large', (0.50, 0.35, 0.30)),
    ('Medium', (0.40, 0.30, 0.25)),
    ('Small', (0.30, 0.20, 0.15)),
]


class DummyScanBoxesServer(Node):

    def __init__(self):
        super().__init__('dummy_scan_boxes_server')
        self._sequence = 0
        self._action_server = ActionServer(
            self, ScanBoxes, '/perception/scan_boxes', self.execute_callback,
        )
        self.get_logger().info('dummy_scan_boxes_server ready (고정 박스 3개 반환)')

    def execute_callback(self, goal_handle):
        feedback = ScanBoxes.Feedback()
        feedback.current_stage = 'DETECTING_BOXES'
        feedback.collected_frames = 0
        feedback.progress = 0.5
        goal_handle.publish_feedback(feedback)
        time.sleep(0.2)

        self._sequence += 1
        boxes = []
        for box_id, (w, d, h) in _DUMMY_BOXES:
            b = Box3D()
            b.box_id = box_id
            b.size.x, b.size.y, b.size.z = w, d, h
            b.confidence = 1.0
            boxes.append(b)

        result = ScanBoxes.Result()
        result.success = True
        result.snapshot_id = f'box_scan_dummy_{self._sequence:04d}'
        result.boxes = boxes
        result.quality_score = 1.0
        result.error_code = ''
        result.message = f'{len(boxes)}개 박스 검출 (dummy)'

        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = DummyScanBoxesServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
