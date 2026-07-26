"""
14_run_full_pipeline.py
⑭ 실행 진입점 - 지완 컴퓨터에서 실제 Vision 데이터로 돌리는 용도
========================================================================
상태: 🟢 신규 (7/21)

실제 Vision 데이터(trunk_map.json + 박스 비전 JSON) 두 파일을 받아서 우리
적재 알고리즘(①~⑬) 전체를 돌리고, 각 박스가 트렁크의 어디(M0609 base 좌표계
기준)에 놓이는지 알려주는 최종 진입점. 로봇은 아직 동작 전이라 결과는 로봇에
명령을 보내지 않고, JSON 파일 + 콘솔 출력으로만 낸다.

[입력 요구사항]
  - --trunk-map: trunk_map.json. 지완의 13.export_trunk_map.py 출력 그대로
    쓰면 됨 (frame: "m0609_base_link").
  - --boxes: 박스 비전 JSON. all_boxes_corners_*.json과 같은 구조여야 하고,
    coordinate_frame 필드가 반드시 "m0609_base_link"여야 한다.
    ⚠️ 실제로 받아본 샘플(all_boxes_corners_20260721_174311_555644.json)은
    coordinate_frame이 "depth_camera_optical_frame_from_message_header"
    (카메라 좌표계)였다 - 이 상태로 넣으면 ①(load_boxes_from_vision_json)이
    바로 에러를 낸다. 카메라→로봇 base 외부파라미터(TF)로 변환한 뒤 내보내
    달라고 요청해야 함 - trunk_map.json은 이미 그렇게 나오고 있으므로 같은
    변환 파이프라인을 박스 쪽에도 적용하면 됨.

[실행 예시]
    python 14_run_full_pipeline.py --trunk-map trunk_map.json --boxes boxes.json
    python 14_run_full_pipeline.py --trunk-map trunk_map.json --boxes boxes.json --allow-stacking

[출력]
    <boxes 파일명>_placement_result.json (또는 --out으로 경로 지정) - 각 박스의
    최종 배치 좌표(M0609 base 좌표계 + 내부 로컬 좌표), 크기, 점수, 미적재
    사유를 담는다.
"""

import sys, pathlib, json, argparse
from importlib import import_module

sys.path.insert(0, str(pathlib.Path(__file__).parent))
m01 = import_module("01_object3d_schema")
m02 = import_module("02_trunk_space_state")
m03 = import_module("03_extreme_point_candidates")
m06 = import_module("06_loading_order_decision")
m07 = import_module("07_placement_plan")
m08 = import_module("08_unloadable_reason")

load_boxes_from_vision_json = m01.load_boxes_from_vision_json
object3d_to_box = m01.object3d_to_box
load_trunk_from_world_map = m02.load_trunk_from_world_map
load_obstacles_from_world_map = m02.load_obstacles_from_world_map
local_to_base_frame = m02.local_to_base_frame
ExtremePointState = m03.ExtremePointState
decide_loading_order = m06.decide_loading_order
place_one_box = m07.place_one_box
classify_unloadable_reason = m08.classify_unloadable_reason


def run_pipeline(trunk_map_path, boxes_path, allow_stacking: bool = False) -> dict:
    """
    실제 Vision 데이터 두 파일을 받아서 전체 파이프라인(①②③④⑤⑥⑦⑧⑬)을 돌리고
    결과를 dict로 반환한다. CLI 래퍼(main())와 테스트 양쪽에서 재사용하려고
    함수로 분리해뒀다.
    """
    # [②] 트렁크 로딩 - base 좌표계 원본을 내부 계산용 로컬 좌표(0,0,0 코너 기준)로 변환
    world_map = load_trunk_from_world_map(trunk_map_path)
    trunk, offset = world_map.to_bounding_trunk()
    obstacles = load_obstacles_from_world_map(trunk_map_path, offset)

    # [①] 박스 비전 데이터 로딩 - 여기서 좌표계 검증(m0609_base_link)까지 됨
    object3ds = load_boxes_from_vision_json(boxes_path)
    boxes = [object3d_to_box(o) for o in object3ds]

    # 장애물(휠하우스 등)을 먼저 등록해서 그 주변 극점 후보가 자동으로 생기게 함
    state = ExtremePointState()
    for obs in obstacles:
        state.register_placement(obs)

    # [⑥][⑦][⑧] 부피 큰 순서로 하나씩 최적 자리를 찾아 배치, 못 찾으면 사유 분류
    order = decide_loading_order(boxes)
    placements = []
    unloadable = []
    for i, box in enumerate(order, start=1):
        plan = place_one_box(box, trunk, state, order=i, allow_stacking=allow_stacking)
        if plan is None:
            reason = classify_unloadable_reason(box, trunk, state)
            unloadable.append({"box_id": box.id, "reason": reason.value})
            continue

        # 내부 로컬 좌표를 다시 M0609 base 좌표계로 되돌림 - 로봇/지완 쪽에 넘길 최종 좌표
        bx, by, bz = local_to_base_frame(*plan.position, offset)
        placements.append({
            "box_id": plan.box_id,
            "order": plan.order,
            "position_base_frame": [bx, by, bz],
            "position_local": list(plan.position),
            "dimensions": list(plan.dimensions),
            "score": plan.score,
            "touches": plan.touches,
            "rotated": plan.rotated,
        })

    return {
        "trunk_local": {"width": trunk.width, "depth": trunk.depth, "height": trunk.height},
        "trunk_offset_base_frame": list(offset),
        "allow_stacking": allow_stacking,
        "placements": placements,
        "unloadable": unloadable,
    }


def main():
    parser = argparse.ArgumentParser(
        description="Cart2Trunk 적재 알고리즘 실행 - Vision 데이터로 박스 배치 좌표 계산")
    parser.add_argument("--trunk-map", required=True, help="trunk_map.json 경로")
    parser.add_argument("--boxes", required=True,
                         help="박스 비전 JSON 경로 (all_boxes_corners_*.json 스타일, m0609_base_link 좌표계 필수)")
    parser.add_argument("--allow-stacking", action="store_true",
                         help="2층 이상 쌓기 허용 (기본: 꺼짐 = 1층만)")
    parser.add_argument("--out", default=None,
                         help="결과 JSON 저장 경로 (기본: <boxes 파일명>_placement_result.json)")
    args = parser.parse_args()

    result = run_pipeline(args.trunk_map, args.boxes, allow_stacking=args.allow_stacking)

    out_path = args.out or (str(pathlib.Path(args.boxes).with_suffix("")) + "_placement_result.json")
    pathlib.Path(out_path).write_text(json.dumps(result, indent=2, ensure_ascii=False))

    print(f"트렁크(로컬 크기): {result['trunk_local']}")
    print(f"배치 성공: {len(result['placements'])}개 / 미적재: {len(result['unloadable'])}개\n")
    for p in result["placements"]:
        x, y, z = p["position_base_frame"]
        print(f"  [{p['order']}] {p['box_id']}: base frame ({x:.3f}, {y:.3f}, {z:.3f})  "
              f"크기={p['dimensions']}")
    for u in result["unloadable"]:
        print(f"  [미적재] {u['box_id']}: {u['reason']}")
    print(f"\n결과 저장: {out_path}")


if __name__ == "__main__":
    main()
