"""
20_task_export.py
⑳ MSI2로 넘길 최종 Task JSON 생성
===================================
상태: 🟡 신규 - 스키마는 문서 기반으로 확정, 실제 전송 경로(토픽/서비스/액션
이름)는 아직 미확정 (지완과 확인 필요 - trunk_map_planner_node.py의
_send_task_to_msi2 참고)

"Cart2Trunk 최종 프로젝트 시나리오 및 시스템 흐름" 문서 4.3절이 정의한
Lenovo -> MSI2 적재 Task JSON 스키마를 PlacementPlan 리스트로부터 만든다:

    {"plan_id":..., "box_snapshot_id":..., "trunk_map_id":..., "approved":...,
     "parameters": {...}, "tasks": [{"sequence":..., "box_id":...,
     "target_position":[x,y,z], "target_yaw":..., "placement_score":...}]}

⚠️ 확인 필요: "담당자별 최종 실행 가이드라인" 문서(선욱 절, Task 출력 필수
필드)는 파라미터 필드를 "selected_parameters"라고 부르는데, 같은 프로젝트의
4.3절 실제 JSON 예시는 "parameters"를 쓴다 - 문서 두 개가 서로 다른 키
이름을 쓰고 있음. 여기서는 구체적인 JSON 예시(4.3절)를 따라 "parameters"로
구현했다 - 실제 연동 전에 지완과 최종 키 이름을 확인해야 한다.

approved는 이 함수를 부르는 쪽(GUI 승인 워크플로우)이 결정해서 넘겨준다 -
여기서는 그 값을 그대로 데이터에 반영할 뿐, "승인 안 됐으면 아예 호출하지
마라"같은 강제는 하지 않는다(그건 "승인 전엔 MSI2로 전달 안 함" 원칙을
지키는 호출부의 책임 - trunk_map_planner_node._send_task_to_msi2 참고).
"""

import sys, pathlib
from typing import Dict, List
from importlib import import_module

sys.path.insert(0, str(pathlib.Path(__file__).parent))
_m07 = import_module("07_placement_plan")

PlacementPlan = _m07.PlacementPlan


def build_task_json(
    plan_id: str, box_snapshot_id: str, trunk_map_id: str,
    parameters: Dict, plans: List["PlacementPlan"], approved: bool = False,
) -> dict:
    return {
        "plan_id": plan_id,
        "box_snapshot_id": box_snapshot_id,
        "trunk_map_id": trunk_map_id,
        "approved": approved,
        "parameters": dict(parameters),
        "tasks": [
            {
                "sequence": p.order,
                "box_id": p.box_id,
                "target_position": list(p.position),
                "target_yaw": p.target_yaw,
                "placement_score": p.score,
            }
            for p in plans
        ],
    }


if __name__ == "__main__":
    plan = PlacementPlan(
        box_id="BOX_03", order=1, position=(0.1, 0.2, 0.0),
        dimensions=(0.3, 0.2, 0.15), score=0.92, touches=2,
        rotated=True, target_yaw=1.57,
    )
    task = build_task_json(
        plan_id="load_plan_001", box_snapshot_id="box_scan_001", trunk_map_id="trunk_map_001",
        parameters={"box_margin": 0.02}, plans=[plan], approved=True,
    )
    import json
    print(json.dumps(task, ensure_ascii=False, indent=2))
