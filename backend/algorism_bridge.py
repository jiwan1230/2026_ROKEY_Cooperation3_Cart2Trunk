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
import colorsys
import json
import sys
import pathlib
import zlib
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
COUNT_FIRST_HEIGHT_WEIGHT = _m05.COUNT_FIRST_HEIGHT_WEIGHT
COUNT_FIRST_FOOTPRINT_GROWTH_WEIGHT = _m05.COUNT_FIRST_FOOTPRINT_GROWTH_WEIGHT
DEFAULT_MARGIN = _m17.MARGIN
build_task_json = _m20.build_task_json

# planner_gui.py의 _discover_trunk_maps()와 같은 경로 관례 - 트렁크 스캔
# run_* 폴더는 ROS2 워크스페이스 src/ 바로 밑에 쌓인다.
_SRC_DIR = pathlib.Path("/home/sunwook/cobot3_ws/src")
_PENDING_TASKS_DIR = _ALGORISM_DIR / "local_test_data" / "pending_tasks"

_DEFAULT_CART_BOXES = [
    {"id": "Large", "width": 0.50, "depth": 0.35, "height": 0.30},
    {"id": "Medium", "width": 0.40, "depth": 0.30, "height": 0.25},
    {"id": "Small", "width": 0.30, "depth": 0.20, "height": 0.15},
]


def color_for_box_id(box_id: str) -> str:
    """box_id 하나당 색 하나를 결정적으로 만든다. 고정된 몇 개짜리 팔레트에서
    고르면(이전 버전) 박스가 팔레트 크기(예: 7개)보다 많아지는 순간 색이
    겹치기 시작한다 - 3D 뷰의 "대기 중" 박스까지 합치면 쉽게 넘어간다(사용자
    피드백). 대신 hue(색상)를 0~360 전체에서 뽑아 훨씬 넓은 범위에서
    구분되게 하고, 혹시 hue가 우연히 가까워도(360가지 중 겹침) 채도/명도를
    같이 흔들어 눈으로 봤을 때 더 구분되게 한다.

    zlib.crc32는 프로세스마다 값이 바뀌는 파이썬 내장 hash()와 달리 입력
    바이트에만 의존하는 결정적 해시라, 백엔드를 재시작해도 같은 box_id는
    항상 같은 색을 유지한다.

    ⚠️ 프론트엔드(web/frontend/src/utils/color.js)가 "대기 중"(아직 계산
    전) 박스에도 같은 알고리즘을 JS로 옮겨서 쓴다 - Before/After를 오갈 때
    같은 박스가 같은 색을 유지하려면 그쪽과 이 함수의 계산 방식이 맞아야
    한다. 이 함수를 고치면 그쪽도 같이 고쳐야 함."""
    h = zlib.crc32(box_id.encode())
    hue = h % 360
    sat = 55 + (h // 360) % 20      # 55~74%
    light = 45 + (h // 7200) % 20   # 45~64%
    r, g, b = colorsys.hls_to_rgb(hue / 360, light / 100, sat / 100)
    return "#{:02x}{:02x}{:02x}".format(round(r * 255), round(g * 255), round(b * 255))


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
    공개된 building block만으로 점수를 "설명"하기 위함.

    ⚠️ mode="count_first"는 두 가지 서로 다른 채점 공식 중 하나를 실제로
    쓴다 (09_rescan_replan.replan_after_rescan의 best-of-two 로직):
      - "작은 것부터" 전략이 채택되면 score_count_first(밀도/공간재사용
        기반, height_term + footprint_growth_term)
      - "큰 것부터" 전략이 채택되면(=large_first와 동일 공식) 기존
        height/contact/wall_a/wall_bc 가중 공식
    09가 어느 쪽이 채택됐는지 별도로 알려주지 않고(algorism/ 파일은
    이번 프로젝트 전체에서 수정 금지라 반환값을 늘릴 수도 없다), 대신
    두 공식을 전부 재계산해서 실제 p.score와 더 가깝게 일치하는 쪽을
    채택한다 - 두 공식은 스케일이 확연히 달라(FOOTPRINT_GROWTH_WEIGHT=5.0)
    거의 항상 명확하게 구분된다. 반환 dict에는 실제로 어느 공식이었는지
    "formula" 키로 표시한다."""
    placed_so_far = list(obstacles)
    breakdown_by_box_id = {}
    for p in plans:
        box = Box(id=p.box_id, width=p.dimensions[0], depth=p.dimensions[1], height=p.dimensions[2])
        x, y, z = p.position
        touches = count_touching_faces(x, y, z, box, trunk, placed_so_far)

        height_term = HEIGHT_WEIGHT * height_preference * (z / trunk.height)
        contact_term = CONTACT_WEIGHT * contact_preference * (touches / 6)
        wall_a_term = WALL_A_WEIGHT * entrance_preference * entrance_distance_ratio(x, box, trunk)
        wall_bc_term = WALL_BC_WEIGHT * (1 - side_wall_distance_ratio(y, box, trunk))
        weighted_score = height_term - contact_term - wall_a_term - wall_bc_term

        if placed_so_far:
            used_max_x = max(pb.x_range[1] for pb in placed_so_far)
            used_max_y = max(pb.y_range[1] for pb in placed_so_far)
        else:
            used_max_x = used_max_y = 0.0
        growth_x = max(0.0, (x + box.width) - used_max_x)
        growth_y = max(0.0, (y + box.depth) - used_max_y)
        footprint_growth_term = COUNT_FIRST_FOOTPRINT_GROWTH_WEIGHT * (growth_x + growth_y)
        count_first_height_term = COUNT_FIRST_HEIGHT_WEIGHT * (z / trunk.height)
        count_first_score = count_first_height_term + footprint_growth_term

        if abs(count_first_score - p.score) < abs(weighted_score - p.score):
            breakdown_by_box_id[p.box_id] = {
                "formula": "count_first_density",
                "height_term": count_first_height_term,
                "footprint_growth_term": footprint_growth_term,
            }
        else:
            breakdown_by_box_id[p.box_id] = {
                "formula": "weighted",
                "height_term": height_term,
                "contact_term": contact_term,
                "wall_a_term": wall_a_term,
                "wall_bc_term": wall_bc_term,
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
        "trunk": {
            "width": trunk.width, "depth": trunk.depth, "height": trunk.height,
            "entrance_near_x": trunk.entrance_near_x,
        },
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


def build_approved_task(plan_id: str, box_snapshot_id: str, trunk_map_id: str,
                         parameters: dict, placed: List[dict]) -> dict:
    """승인 단계 - 20_task_export.build_task_json을 그대로 쓰기 위해,
    프론트가 보낸 placed(POST /api/plan 응답의 placed 배열 그대로)를
    잠깐 PlacementPlan 객체로 복원한다."""
    _m07 = import_module("07_placement_plan")
    PlacementPlan = _m07.PlacementPlan

    plans = [
        PlacementPlan(
            box_id=p["box_id"], order=p["order"], position=tuple(p["position"]),
            dimensions=tuple(p["dimensions"]), score=p["score"], touches=p["touches"],
            rotated=p["rotated"], target_yaw=p["target_yaw"],
        )
        for p in placed
    ]
    return build_task_json(
        plan_id=plan_id, box_snapshot_id=box_snapshot_id, trunk_map_id=trunk_map_id,
        parameters=parameters, plans=plans, approved=True,
    )


def send_task(task_json: dict) -> str:
    """trunk_map_planner_node._send_task_to_msi2()와 동일한 로직(승인 전
    전송 금지 원칙 포함) - rclpy 의존성 없이 이 파일 안에서 재구현했다
    (이유는 이 파일 최상단 docstring 참고). 실제 MSI2 전송 경로가 정해지기
    전까지는 그쪽과 똑같이 로컬 파일로만 저장한다."""
    if not task_json.get("approved", False):
        raise ValueError("approved=False인 Task는 MSI2로 보낼 수 없음 (승인 전 전달 금지 원칙)")

    _PENDING_TASKS_DIR.mkdir(parents=True, exist_ok=True)
    out_path = _PENDING_TASKS_DIR / f"{task_json['plan_id']}.json"
    out_path.write_text(json.dumps(task_json, ensure_ascii=False, indent=2))
    return str(out_path)
