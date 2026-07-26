"""cart2trunk_hmi: web/backend(Flask)가 실제 ROS2 액션을 호출할 수 있게 해주는
얇은 브릿지.

web/backend/routes/robot.py의 세 트리거(cart-scan/trunk-scan/pick-and-place)는
지금까지 실제 ROS2 호출 없이 더미 지연 후 항상 성공만 반환했다(그 파일
docstring의 "TODO(MSI2)"가 이 노드가 채우는 자리다).

[왜 Flask 프로세스 안에서 직접 rclpy를 쓰지 않는가]
web/backend/algorism_bridge.py 최상단 docstring에 이미 있는 것과 같은 이유 -
rclpy를 웹 백엔드 전용 venv에 설치하면 "ROS2 환경(시스템 python3.10)과 웹
백엔드를 분리한다"는 기존 설계 목표와 충돌한다. 대신 이 노드가 시스템
python3.10 rclpy 프로세스로 따로 떠서, 아주 단순한 내부 전용 HTTP 서버(표준
라이브러리 http.server만 사용 - 이 ROS 패키지에 Flask 의존성을 새로 추가하지
않기 위해 일부러 프레임워크를 안 씀)를 열고, web/backend가 로컬호스트로 그
서버에 POST하는 방식으로 두 프로세스를 연결한다.

[라우트]
    POST /trigger/cart-scan      -> ScanBoxes 액션 1회 호출
    POST /trigger/trunk-scan     -> ScanTrunk 액션 1회 호출
    POST /trigger/pick-and-place -> RunLoadingMission 액션 호출

    ⚠️ pick-and-place는 "박스 하나만 집어서 놓기"가 아니라 mission_state_machine이
    실제로 구현한 "카트가 빌 때까지 반복하는 전체 미션"을 그대로 노출한다 -
    지금 시스템에 그보다 더 잘게 쪼갠 API가 없다(1회 배치만 실행하고 멈추는
    기능은 코디네이터 쪽에 별도 설계가 필요 - 이번 범위 아님). 호출 시간이
    오래 걸릴 수 있으니 web/backend 쪽 HTTP 타임아웃을 넉넉히 잡아야 한다.

    GET  /health -> {"status": "ok"}

기본 포트는 5050(web/backend의 5000과 겹치지 않게) - CART2TRUNK_HMI_BRIDGE_PORT
환경변수로 바꿀 수 있다.
"""
import json
import os
import threading
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from cart2trunk_interfaces.action import RunLoadingMission, ScanBoxes, ScanTrunk

_DEFAULT_PORT = 5050


class BridgeActionError(Exception):
    def __init__(self, status_code: int, payload: dict):
        super().__init__(payload.get('message', ''))
        self.status_code = status_code
        self.payload = payload


class RosBridgeNode(Node):

    def __init__(self):
        super().__init__('hmi_ros_bridge_node')
        cb_group = ReentrantCallbackGroup()
        self._scan_boxes_client = ActionClient(
            self, ScanBoxes, '/perception/scan_boxes', callback_group=cb_group)
        self._scan_trunk_client = ActionClient(
            self, ScanTrunk, '/perception/scan_trunk', callback_group=cb_group)
        self._run_mission_client = ActionClient(
            self, RunLoadingMission, '/mission/run_loading_mission', callback_group=cb_group)
        self.get_logger().info('hmi_ros_bridge_node ready')

    @staticmethod
    def _wait_for_future(future, timeout_sec: float):
        """HANDOFF_MSI2.md 6.5절 - Future.result()는 논블로킹이라 done_callback +
        threading.Event로 실제 완료를 기다려야 한다(mission_coordinator_node와
        동일한 패턴)."""
        event = threading.Event()
        future.add_done_callback(lambda _f: event.set())
        if not event.wait(timeout_sec):
            return None
        return future.result()

    def _call_action(self, client: ActionClient, goal, action_name: str, timeout_sec: float):
        if not client.wait_for_server(timeout_sec=5.0):
            raise BridgeActionError(503, {
                'status': 'error', 'error_code': 'ACTION_SERVER_UNAVAILABLE',
                'message': f'{action_name} 서버 없음 - 해당 노드가 떠 있는지 확인하세요.',
            })
        send_future = client.send_goal_async(goal)
        goal_handle = self._wait_for_future(send_future, timeout_sec)
        if goal_handle is None:
            raise BridgeActionError(504, {
                'status': 'error', 'error_code': 'ACTION_TIMEOUT',
                'message': f'{action_name} goal 전송 응답 시간 초과',
            })
        if not goal_handle.accepted:
            raise BridgeActionError(409, {
                'status': 'error', 'error_code': 'GOAL_REJECTED',
                'message': f'{action_name} goal이 거부됨',
            })
        result_future = goal_handle.get_result_async()
        wrapped = self._wait_for_future(result_future, timeout_sec)
        if wrapped is None:
            raise BridgeActionError(504, {
                'status': 'error', 'error_code': 'ACTION_TIMEOUT',
                'message': f'{action_name} 결과 응답 시간 초과({timeout_sec}s)',
            })
        return wrapped.result

    def trigger_cart_scan(self) -> dict:
        result = self._call_action(
            self._scan_boxes_client, ScanBoxes.Goal(), 'ScanBoxes', timeout_sec=60.0)
        if not result.success:
            raise BridgeActionError(422, {
                'status': 'error', 'error_code': result.error_code, 'message': result.message,
            })
        return {
            'status': 'ok', 'snapshot_id': result.snapshot_id,
            'box_count': len(result.boxes), 'quality_score': result.quality_score,
            'message': result.message,
        }

    def trigger_trunk_scan(self) -> dict:
        result = self._call_action(
            self._scan_trunk_client, ScanTrunk.Goal(), 'ScanTrunk', timeout_sec=60.0)
        if not result.success:
            raise BridgeActionError(422, {
                'status': 'error', 'error_code': result.error_code, 'message': result.message,
            })
        return {
            'status': 'ok', 'map_id': result.map_id,
            'quality_score': result.quality_score, 'message': result.message,
        }

    def trigger_pick_and_place(self) -> dict:
        goal = RunLoadingMission.Goal()
        goal.request_id = f'hmi_{self.get_clock().now().nanoseconds}'
        goal.auto_continue = True
        result = self._call_action(
            self._run_mission_client, goal, 'RunLoadingMission', timeout_sec=600.0)
        if not result.success:
            raise BridgeActionError(422, {
                'status': 'error', 'error_code': result.error_code, 'message': result.message,
            })
        return {
            'status': 'ok', 'placed_count': result.placed_count,
            'remaining_count': result.remaining_count, 'message': result.message,
        }


class BridgeRequestHandler(BaseHTTPRequestHandler):

    def __init__(self, *args, node: RosBridgeNode, **kwargs):
        self._node = node
        super().__init__(*args, **kwargs)

    def _write_json(self, status_code: int, payload: dict) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode('utf-8')
        self.send_response(status_code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _dispatch(self, handler) -> None:
        try:
            payload = handler()
            self._write_json(200, payload)
        except BridgeActionError as exc:
            self._write_json(exc.status_code, exc.payload)
        except Exception as exc:  # noqa: BLE001 - 브릿지 경계에서는 뭐가 나든 JSON으로 감싸 반환
            self._write_json(
                500, {'status': 'error', 'error_code': 'BRIDGE_ERROR', 'message': str(exc)})

    def do_GET(self):
        if self.path == '/health':
            self._write_json(200, {'status': 'ok'})
        else:
            self._write_json(404, {'status': 'error', 'message': f"unknown route '{self.path}'"})

    def do_POST(self):
        routes = {
            '/trigger/cart-scan': self._node.trigger_cart_scan,
            '/trigger/trunk-scan': self._node.trigger_trunk_scan,
            '/trigger/pick-and-place': self._node.trigger_pick_and_place,
        }
        handler = routes.get(self.path)
        if handler is None:
            self._write_json(404, {'status': 'error', 'message': f"unknown route '{self.path}'"})
            return
        self._dispatch(handler)

    def log_message(self, fmt, *args):  # noqa: A003 - BaseHTTPRequestHandler 시그니처 준수
        self.server.node.get_logger().info('[http] ' + (fmt % args))


def main(args=None):
    rclpy.init(args=args)
    node = RosBridgeNode()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    port = int(os.environ.get('CART2TRUNK_HMI_BRIDGE_PORT', str(_DEFAULT_PORT)))
    handler_cls = partial(BridgeRequestHandler, node=node)
    httpd = ThreadingHTTPServer(('0.0.0.0', port), handler_cls)
    httpd.node = node
    node.get_logger().info(f'HTTP 브릿지 http://0.0.0.0:{port} 에서 대기 중')

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.shutdown()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
