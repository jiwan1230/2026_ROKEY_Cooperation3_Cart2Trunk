"""
vision_adapter.py
비전(준형)이 만드는 원본 JSON 두 가지를 algorism_bridge.compute_plan()이 바로
받는 단순 박스 딕셔너리 목록({id,width,depth,height,rests_on_id})으로 바꾼다.

- box_scan.json: "Cart2Trunk 최종 프로젝트 시나리오 및 시스템 흐름" 문서
  4.2절이 정의한 "목표" 스키마 (아직 준형의 실제 최종 출력이 이 형태인지는
  미확인).
- all_boxes_corners_*.json: 준형이 실제로 넘겨준 샘플 파일의 스키마
  (~/Downloads/all_boxes_corners_20260721_174311_555644.json로 확인).

algorism/ 파일은 이 프로젝트 전체에서 수정 금지라(algorism_bridge.py 최상단
docstring, 05_candidate_scoring.py 주석 등 참고) algorism/01_object3d_schema.py의
기존 로더(load_box_snapshot_from_json, object3d_to_box)를 그대로 재사용하되,
그 파일이 아직 다루지 않는 부분은 여기서 처리한다:
  - all_boxes_corners_*.json의 support_type/support_candidate_id ->
    rests_on_id 매핑 (load_boxes_from_vision_json()은 이걸 안 함)
  - 좌표계가 안 맞을 때 "임의로 변환하지 않고 명확히 에러로 막기"
"""
import sys
import pathlib
from importlib import import_module
from typing import Dict, List, Optional, Tuple

_HERE = pathlib.Path(__file__).resolve().parent
_ALGORISM_DIR = _HERE.parent.parent / "algorism"
if str(_ALGORISM_DIR) not in sys.path:
    sys.path.insert(0, str(_ALGORISM_DIR))

_m01 = import_module("01_object3d_schema")
load_box_snapshot_from_json = _m01.load_box_snapshot_from_json
object3d_to_box = _m01.object3d_to_box
EXPECTED_BOX_FRAME = _m01.EXPECTED_BOX_FRAME


def boxes_from_box_scan(box_scan_data: dict) -> Tuple[List[dict], str]:
    """"시스템 흐름" 문서 4.2절 box_scan.json 스키마를 파싱한다. algorism/01의
    load_box_snapshot_from_json()을 그대로 쓴다 - frame_id 검증(안 맞으면
    ValueError)까지 그쪽 로직 그대로 적용된다."""
    snapshot = load_box_snapshot_from_json(box_scan_data)
    boxes = []
    for obj in snapshot.boxes:
        box = object3d_to_box(obj)
        boxes.append({
            "id": box.id, "width": box.width, "depth": box.depth, "height": box.height,
            "rests_on_id": box.rests_on_id, "initial_yaw": box.initial_yaw,
        })
    return boxes, snapshot.snapshot_id


def boxes_from_vision_corners(vision_data: dict) -> Tuple[List[dict], str]:
    """준형의 실제 박스 비전 출력(all_boxes_corners_*.json) 스키마를 파싱한다.

    [rests_on_id 매핑 - 실제 샘플로 검증한 규칙]
    각 박스는 "자기 윗면"을 top_candidate_id로 갖고, "자기가 얹혀 있는 대상의
    윗면"을 support_candidate_id로 갖는다 - support_candidate_id는 그 박스
    자신의 box_id가 아니라 다른 박스의 top_candidate_id를 가리킨다. 실제
    샘플(all_boxes_corners_20260721_174311_555644.json, 박스 3개)로 교차
    검증함: 바닥 박스(box_id=2, support_type="floor")가 top_candidate_id=0이고,
    그 위에 나란히 얹힌 두 박스(box_id=0,1)가 둘 다 support_candidate_id=0으로
    box_id=2를 가리킴. support_type=="floor"이거나 support_candidate_id==-1이면
    바닥에 직접 놓인 것(rests_on_id=None).

    [좌표계 - 미해결, 임의 변환 안 함]
    실제 샘플의 coordinate_frame은 "depth_camera_optical_frame_from_message_
    header"인데 알고리즘이 요구하는 건 "m0609_base_link"(EXPECTED_BOX_FRAME)다.
    카메라->로봇 base 외부 파라미터(extrinsic calibration) 없이 임의로 좌표를
    옮기면 박스가 엉뚱한 자리로 계산되는 게 조용히 넘어갈 위험이 있어(안전
    문제), algorism/01과 동일하게 프레임이 안 맞으면 명확한 에러로 막는다 -
    Vision(준형)/시스템통합(지완)과 좌표계를 맞춘 뒤에야 실제 데이터로 계산할
    수 있다는 뜻이다.
    """
    frame = vision_data.get("coordinate_frame")
    if frame != EXPECTED_BOX_FRAME:
        raise ValueError(
            f"박스 비전 데이터의 좌표계가 '{frame}'인데 '{EXPECTED_BOX_FRAME}'이어야 합니다 - "
            f"카메라 좌표계를 로봇 base 좌표계로 변환해서 다시 내보내달라고 "
            f"Vision(준형)/시스템통합(지완)과 확인하세요."
        )

    raw_boxes = vision_data.get("boxes", [])
    top_candidate_to_box_id: Dict[int, str] = {
        b["top_candidate_id"]: str(b["box_id"])
        for b in raw_boxes if "top_candidate_id" in b
    }

    boxes = []
    for b in raw_boxes:
        corners = b["corners_m"]
        xs = [c[0] for c in corners]
        ys = [c[1] for c in corners]
        zs = [c[2] for c in corners]
        width = max(xs) - min(xs)
        depth = max(ys) - min(ys)
        height = max(zs) - min(zs)

        support_candidate_id = b.get("support_candidate_id", -1)
        is_on_floor = b.get("support_type") == "floor" or support_candidate_id == -1
        rests_on_id: Optional[str] = None if is_on_floor else top_candidate_to_box_id.get(support_candidate_id)

        boxes.append({
            "id": str(b["box_id"]), "width": width, "depth": depth, "height": height,
            "rests_on_id": rests_on_id,
        })

    snapshot_id = vision_data.get("completed_ply_file") or f"vision_corners_{len(boxes)}boxes"
    return boxes, snapshot_id
