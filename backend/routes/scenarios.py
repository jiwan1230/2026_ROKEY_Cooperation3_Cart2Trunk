"""
routes/scenarios.py
POST /api/scenarios/<scenario_id>/plan - 산업 현장 시나리오 4종
(algorism/industry_scenarios/) 미리보기. 각 시나리오 파일에 이미 있는
데모 트렁크/박스 데이터로 실제 시나리오 알고리즘을 호출해서 배치 결과를
반환한다. algorism/ 파일은 이 프로젝트에서 수정 금지라 기존 함수를
import해서 호출만 한다(algorism_bridge.py와 동일한 원칙).
"""
import sys
import pathlib
from importlib import import_module

from flask import Blueprint, jsonify

from routes.plan import ApiError

_HERE = pathlib.Path(__file__).resolve().parent  # .../web/backend/routes
_CART2TRUNK_DIR = _HERE.parent.parent.parent  # routes -> backend -> web -> Cart2Trunk
_ALGORISM_DIR = _CART2TRUNK_DIR / "algorism"
_SCENARIOS_DIR = _ALGORISM_DIR / "industry_scenarios"
for _p in (str(_ALGORISM_DIR), str(_SCENARIOS_DIR)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

Trunk = import_module("02_trunk_space_state").Trunk
Box = import_module("03_extreme_point_candidates").Box
_scenario1 = import_module("scenario1_delivery_truck")
_scenario2 = import_module("scenario2_warehouse_density")
_scenario3 = import_module("scenario3_cold_chain")
_scenario4 = import_module("scenario4_hazmat")

scenarios_bp = Blueprint("scenarios", __name__)

SCENARIO_DEFS = {
    "delivery_truck": {
        "label": "택배 배송 트럭",
        "trunk_kwargs": {"width": 1.2, "depth": 0.8, "height": 0.6},
        "make_boxes": lambda: [
            Box("정류장1_박스", 0.3, 0.25, 0.2, delivery_stop=1),
            Box("정류장2_박스", 0.3, 0.25, 0.2, delivery_stop=2),
            Box("정류장3_박스", 0.3, 0.25, 0.2, delivery_stop=3),
            Box("정류장4_박스", 0.3, 0.25, 0.2, delivery_stop=4),
        ],
        "generate": _scenario1.generate_loading_plan_lifo_delivery,
    },
    "warehouse": {
        "label": "창고/물류센터",
        "trunk_kwargs": {"width": 0.6, "depth": 0.4, "height": 0.45},
        "make_boxes": lambda: (
            [Box(f"소{i}", 0.1, 0.1, 0.1) for i in range(6)]
            + [Box(f"대{i}", 0.3, 0.2, 0.2) for i in range(2)]
        ),
        "generate": _scenario2.generate_loading_plan_count_first,
    },
    "cold_chain": {
        "label": "냉동/냉장 물류",
        "trunk_kwargs": {"width": 1.2, "depth": 0.6, "height": 0.5},
        "make_boxes": lambda: [Box(f"냉동박스{i}", 0.3, 0.25, 0.2) for i in range(3)],
        "generate": _scenario3.generate_loading_plan_cold_chain,
    },
    "hazmat": {
        "label": "위험물 창고",
        "trunk_kwargs": {"width": 1.5, "depth": 1.0, "height": 0.5},
        "make_boxes": lambda: [
            Box("산화제_드럼1", 0.3, 0.3, 0.3, hazard_class="oxidizer"),
            Box("인화물_드럼1", 0.3, 0.3, 0.3, hazard_class="flammable"),
            Box("일반박스1", 0.3, 0.3, 0.3),
        ],
        "generate": _scenario4.generate_loading_plan_hazmat,
    },
}


@scenarios_bp.post("/api/scenarios/<scenario_id>/plan")
def compute_scenario_plan(scenario_id):
    scenario_def = SCENARIO_DEFS.get(scenario_id)
    if scenario_def is None:
        raise ApiError(
            404, "SCENARIO_NOT_FOUND", f"'{scenario_id}' 시나리오가 없습니다.",
            "지원하는 시나리오 id(delivery_truck/warehouse/cold_chain/hazmat)인지 확인하세요.",
        )

    trunk = Trunk(**scenario_def["trunk_kwargs"])
    boxes = scenario_def["make_boxes"]()
    plans, unloadable = scenario_def["generate"](boxes, trunk)

    total = len(boxes)
    return jsonify({
        "label": scenario_def["label"],
        "trunk": {
            "width": trunk.width, "depth": trunk.depth, "height": trunk.height,
            "entrance_near_x": trunk.entrance_near_x,
        },
        # 프론트가 Before(카트에 대기 중) 화면을 그리려면 배치 성공/실패와
        # 무관하게 "원래 실으려던 박스 전체"의 크기가 필요하다 - 시뮬레이터
        # 탭의 inputBoxes(state.boxesText)와 같은 shape({id,width,depth,height}).
        "boxes": [
            {"id": b.id, "width": b.width, "depth": b.depth, "height": b.height} for b in boxes
        ],
        "placed": [
            {"box_id": p.box_id, "position": list(p.position), "dimensions": list(p.dimensions), "order": p.order}
            for p in plans
        ],
        "unloadable": [{"box_id": u.box_id, "reason": u.reason.value} for u in unloadable],
        "summary": {"total": total, "placed": len(plans), "unplaced": total - len(plans)},
    })
