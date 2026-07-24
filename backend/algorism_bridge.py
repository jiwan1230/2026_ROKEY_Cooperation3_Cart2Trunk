"""
algorism_bridge.py
Cart2Trunk/algorism/ 의 기존 계산 로직을 웹 백엔드 라우트에서 쓸 수 있게 얇게
감싸는 어댑터. 계산 로직은 절대 재구현하지 않고 전부 import해서 그대로 쓴다.

trunk_map_planner_node.py를 직접 import하지 않는 이유: 그 파일은 최상단에서
`import rclpy`를 하기 때문에, ROS2 파이썬 환경과 분리하기로 한 이 백엔드
전용 venv에도 rclpy 설치를 강제하게 된다 (설계 목표와 충돌). 대신 그 파일이
쓰는 것과 똑같은 방식으로 algorism/ 안의 번호 붙은 모듈들(02, 03, 05, 09,
17, 20)을 직접 import한다 - trunk_map_planner_node.py의
plan_from_trunk_map_data()/_color_for_box_id()/_send_task_to_msi2() 같은
소규모 글루 코드만 그대로 옮겨왔고, 그 함수들이 호출하는 실제 알고리즘
(02/03/05/09/17/20)은 한 줄도 재구현하지 않았다.
"""
import json
import sys
import pathlib
from importlib import import_module
from typing import Dict, List, Optional

_HERE = pathlib.Path(__file__).resolve().parent
_CART2TRUNK_DIR = _HERE.parent.parent
_ALGORISM_DIR = _CART2TRUNK_DIR / "algorism"
_LOCAL_TEST_DATA_DIR = _ALGORISM_DIR / "local_test_data"
for p in (str(_ALGORISM_DIR), str(_LOCAL_TEST_DATA_DIR)):
    if p not in sys.path:
        sys.path.insert(0, p)

_m02 = import_module("02_trunk_space_state")
_m03 = import_module("03_extreme_point_candidates")
_m05 = import_module("05_candidate_scoring")
_m09 = import_module("09_rescan_replan")
_m17 = import_module("17_margin_check")
_m20 = import_module("20_task_export")

load_trunk_from_world_map = _m02.load_trunk_from_world_map
load_obstacles_from_world_map = _m02.load_obstacles_from_world_map
Box = _m03.Box
PlacedBox = _m03.PlacedBox
replan_after_rescan = _m09.replan_after_rescan
count_touching_faces = _m05.count_touching_faces
entrance_distance_ratio = _m05.entrance_distance_ratio
side_wall_distance_ratio = _m05.side_wall_distance_ratio
HEIGHT_WEIGHT = _m05.HEIGHT_WEIGHT
CONTACT_WEIGHT = _m05.CONTACT_WEIGHT
WALL_A_WEIGHT = _m05.WALL_A_WEIGHT
WALL_BC_WEIGHT = _m05.WALL_BC_WEIGHT
DEFAULT_MARGIN = _m17.MARGIN
build_task_json = _m20.build_task_json

# planner_gui.py의 _discover_trunk_maps()와 같은 경로 관례 - 트렁크 스캔
# run_* 폴더는 ROS2 워크스페이스 src/ 바로 밑에 쌓인다.
_SRC_DIR = pathlib.Path("/home/sunwook/cobot3_ws/src")
_PENDING_TASKS_DIR = _ALGORISM_DIR / "local_test_data" / "pending_tasks"

_BOX_COLOR_PALETTE = ["#3498db", "#e67e22", "#2ecc71", "#9b59b6", "#e74c3c", "#1abc9c", "#f1c40f"]

_DEFAULT_CART_BOXES = [
    {"id": "Large", "width": 0.50, "depth": 0.35, "height": 0.30},
    {"id": "Medium", "width": 0.40, "depth": 0.30, "height": 0.25},
    {"id": "Small", "width": 0.30, "depth": 0.20, "height": 0.15},
]


def color_for_box_id(box_id: str) -> str:
    return _BOX_COLOR_PALETTE[hash(box_id) % len(_BOX_COLOR_PALETTE)]


def list_trunk_maps() -> List[str]:
    """run_*/pointcloud/trunk_map.json이 있는 run 폴더 이름 목록 (오래된 순)."""
    paths = sorted(_SRC_DIR.glob("run_*/pointcloud/trunk_map.json"))
    return [p.parent.parent.name for p in paths]


def _trunk_map_path(run_name: str) -> pathlib.Path:
    path = _SRC_DIR / run_name / "pointcloud" / "trunk_map.json"
    if not path.exists():
        raise ValueError(f"'{run_name}' 트렁크 스캔 파일을 찾을 수 없습니다: {path}")
    return path


def list_box_presets() -> Dict[str, list]:
    """프리셋 이름 -> 박스 배열. planner_gui.py의 _discover_box_presets()와
    달리 파일 경로가 아니라 박스 배열 자체를 값으로 담아 반환한다 - 프론트엔드가
    별도 요청 없이 바로 쓸 수 있게 하기 위함."""
    presets = {"기본값 (Large/Medium/Small)": list(_DEFAULT_CART_BOXES)}
    for f in sorted(_LOCAL_TEST_DATA_DIR.glob("example_cart_boxes_*.json")):
        name = f.stem.replace("example_cart_boxes_", "")
        presets[name] = json.loads(f.read_text())
    return presets


def generate_random_boxes(count: int) -> List[dict]:
    """planner_gui.py의 _generate_random_boxes()와 동일한 범위. 실제 무작위
    생성은 프론트엔드 JS가 서버 왕복 없이 직접 하므로(설계 문서 참고), 이
    함수는 백엔드 단독 테스트/디버깅용으로만 남겨둔다."""
    import random
    rng = random.Random()
    boxes = []
    for i in range(count):
        boxes.append({
            "id": f"Box{i + 1}",
            "width": round(rng.uniform(0.15, 0.45), 2),
            "depth": round(rng.uniform(0.15, 0.40), 2),
            "height": round(rng.uniform(0.10, 0.30), 2),
        })
    return boxes
