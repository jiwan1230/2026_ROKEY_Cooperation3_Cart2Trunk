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


def _reconstruct_score_breakdown(plans, obstacles, trunk, entrance_preference, contact_preference, height_preference):
    """plans는 order로 정렬돼 있다고 가정. place_one_box가 매 박스를 놓을 때
    실제로 봤던 상태(그 이전까지 놓인 박스+장애물)를 그대로 재현해서
    count_touching_faces 등을 다시 계산한다 - 원본 점수를 따로 캐시하지 않고
    재계산하는 이유는 05_candidate_scoring.py를 전혀 수정하지 않고 이미
    공개된 building block만으로 점수를 "설명"하기 위함."""
    placed_so_far = list(obstacles)
    breakdown_by_box_id = {}
    for p in plans:
        box = Box(id=p.box_id, width=p.dimensions[0], depth=p.dimensions[1], height=p.dimensions[2])
        x, y, z = p.position
        touches = count_touching_faces(x, y, z, box, trunk, placed_so_far)
        breakdown_by_box_id[p.box_id] = {
            "height_term": HEIGHT_WEIGHT * height_preference * (z / trunk.height),
            "contact_term": CONTACT_WEIGHT * contact_preference * (touches / 6),
            "wall_a_term": WALL_A_WEIGHT * entrance_preference * entrance_distance_ratio(x, box, trunk),
            "wall_bc_term": WALL_BC_WEIGHT * (1 - side_wall_distance_ratio(y, box, trunk)),
        }
        placed_so_far.append(PlacedBox(box=box, x=x, y=y, z=z))
    return breakdown_by_box_id


def compute_plan(
    trunk_map_data: dict, boxes_raw: List[dict], box_source_label: str = "custom",
    mode: str = "large_first", margin: Optional[float] = None,
    allow_stacking: bool = False, allow_rotation: bool = True,
    wall_margin: Optional[float] = None, obstacle_margin: Optional[float] = None,
    ceiling_margin: Optional[float] = None, entrance_margin: Optional[float] = None,
    entrance_preference: float = 1.0, contact_preference: float = 1.0, height_preference: float = 1.0,
    fixed_order: bool = False,
) -> dict:
    """POST /api/plan 하나가 필요로 하는 전체 응답 payload를 만든다.
    trunk_map_planner_node.plan_from_trunk_map_data()와 같은 순서로 02의
    파서를 직접 호출한다 (그 함수 자체를 import하지 않는 이유는 이 파일
    최상단 docstring 참고)."""
    import time

    world_map = load_trunk_from_world_map(trunk_map_data)
    trunk, offset = world_map.to_bounding_trunk()
    obstacles = load_obstacles_from_world_map(trunk_map_data, offset)
    cart_boxes = [Box(**b) for b in boxes_raw]
    fixed_order_ids = [b["id"] for b in boxes_raw] if fixed_order else None

    t0 = time.perf_counter()
    plans, unloadable = replan_after_rescan(
        cart_boxes, trunk, obstacles, mode=mode, margin=margin, allow_stacking=allow_stacking,
        allow_rotation=allow_rotation, wall_margin=wall_margin, obstacle_margin=obstacle_margin,
        ceiling_margin=ceiling_margin, entrance_margin=entrance_margin,
        entrance_preference=entrance_preference, contact_preference=contact_preference,
        height_preference=height_preference, fixed_order=fixed_order_ids,
    )
    calc_time_ms = (time.perf_counter() - t0) * 1000

    plans_by_order = sorted(plans, key=lambda p: p.order)
    breakdown = _reconstruct_score_breakdown(
        plans_by_order, obstacles, trunk, entrance_preference, contact_preference, height_preference)

    effective_margin = margin if margin is not None else DEFAULT_MARGIN
    placed_volume = sum(p.dimensions[0] * p.dimensions[1] * p.dimensions[2] for p in plans)
    trunk_volume = trunk.width * trunk.depth * trunk.height
    utilization_pct = (placed_volume / trunk_volume * 100) if trunk_volume > 1e-9 else 0.0
    avg_score = (sum(p.score for p in plans) / len(plans)) if plans else 0.0

    log_lines = [
        f"[{trunk_map_data.get('run_id', '?')}] mode={mode}, margin={effective_margin:.2f}m, "
        f"쌓기={'허용' if allow_stacking else '1층전용'}, 회전={'허용' if allow_rotation else '비허용'}, "
        f"입구/깊이축={entrance_preference:+.1f}, 접촉면가중치={contact_preference:.1f}, "
        f"바닥우선강도={height_preference:.1f}, 순서고정={'예' if fixed_order_ids else '아니오'} "
        f"-> {len(plans)}/{len(boxes_raw)}개 배치"
    ]
    for p in plans_by_order:
        log_lines.append(
            f"  PLACED {p.box_id}: pos=({p.position[0]:.2f},{p.position[1]:.2f},{p.position[2]:.2f}) "
            f"rotated={p.rotated}"
        )
    for u in unloadable:
        log_lines.append(f"  UNLOADABLE {u.box_id}: {u.reason.value}")

    return {
        "trunk": {"width": trunk.width, "depth": trunk.depth, "height": trunk.height},
        "obstacles": [
            {"id": o.box.id, "x": o.x, "y": o.y, "z": o.z,
             "width": o.box.width, "depth": o.box.depth, "height": o.box.height}
            for o in obstacles
        ],
        "placed": [
            {
                "box_id": p.box_id, "order": p.order, "position": list(p.position),
                "dimensions": list(p.dimensions), "rotated": p.rotated, "target_yaw": p.target_yaw,
                "score": p.score, "touches": p.touches, "color": color_for_box_id(p.box_id),
                "score_breakdown": breakdown[p.box_id],
            }
            for p in plans_by_order
        ],
        "unloadable": [
            {"box_id": u.box_id, "reason": u.reason.value, "detail": u.detail}
            for u in unloadable
        ],
        "summary": {
            "total": len(boxes_raw), "placed": len(plans), "unplaced": len(unloadable),
            "utilization_pct": utilization_pct, "calc_time_ms": calc_time_ms, "avg_score": avg_score,
        },
        "log_lines": log_lines,
        "trunk_map_id": trunk_map_data.get("run_id", "?"),
        "box_snapshot_id": f"manual_input:{box_source_label}",
    }
