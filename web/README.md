# Cart2Trunk 웹 플래너 - 실행 방법

세 개의 터미널(로봇 트리거까지 실제로 쓰려면)에서 각각 띄운다: ROS2 브릿지,
백엔드, 프론트엔드. 트렁크 스캔 파일/박스 프리셋 미리보기·계획 계산만 쓸
거면 1(브릿지)은 생략해도 된다 - `/api/robot/*` 호출 시 503으로 명확히 안내된다.

## 0. ROS2 브릿지 (cart2trunk_hmi - 로봇 트리거를 실제로 쓸 때만 필요)

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash   # cart2trunk_ws 루트에서
ros2 run cart2trunk_hmi ros_bridge_node
```

기본 포트 5050(`CART2TRUNK_HMI_BRIDGE_PORT`로 변경 가능) - web/backend가
여기로 `/api/robot/cart-scan`, `/trunk-scan`, `/pick-and-place`를 그대로
릴레이한다(`web/backend/routes/robot.py`). 이 브릿지는 `/perception/scan_boxes`,
`/perception/scan_trunk`, `/mission/run_loading_mission` 액션 서버가 떠 있어야
실제로 동작한다 - 최소한 `cart2trunk_coordinator`/`cart2trunk_perception`(또는
`cart2trunk_test_scenarios`의 dummy 서버들)을 먼저 띄워둘 것.

## 1. 백엔드 (Flask, API 전용 - 화면 없음)

```bash
cd web/backend
source venv/bin/activate   # 최초 1회: python3 -m venv venv && pip install -r requirements.txt
python app.py
```

`http://localhost:5000/api/health`가 `{"status":"ok"}`를 반환하면 정상.

## 2. 프론트엔드 (React+Vite - 실제로 보고 쓰는 화면)

```bash
cd web/frontend
npm install   # 최초 1회
npm run dev
```

브라우저로 `http://localhost:5173`을 열면 실제 UI가 뜬다. 백엔드가 5000번 포트에서
먼저(또는 동시에) 켜져 있어야 트렁크 스캔 파일/박스 프리셋 목록을 불러올 수 있다.

## 테스트

```bash
# 백엔드
cd web/backend && source venv/bin/activate && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -v

# 프론트엔드
cd web/frontend && npm test
```
