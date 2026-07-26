"""
19_run_full_pipeline_with_yaw.py
14_run_full_pipeline.py의 복사본 - 01~08 모듈과 14.py 자체는 전혀 건드리지 않는다.

배경 (45.crate_scan_setup_variable_box_count_with_yaw.py와 짝)
----
지금까지 테이블 박스는 항상 축 정렬(yaw=0)로만 스폰됐고, `algorism` 패키지 전체가
그 가정 위에 만들어져 있다 - `01_object3d_schema.py`의 `load_boxes_from_vision_json()`
자체 주석에 "우리 알고리즘은 회전을 다루지 않으므로(MVP 범위)... 8개 점의 min/max로
AABB를 근사"라고 명시돼 있다. 45.py부터는 박스가 실제로 yaw 회전된 채 스폰되므로, 이
AABB 근사를 그대로 쓰면 회전된 박스의 폭/깊이가 실제보다 부풀려져 적재 알고리즘에
잘못된 크기가 들어간다(예: 정사각형을 45도 돌리면 AABB 변 길이가 대각선 길이가 됨).

이 스크립트는 `m01.load_boxes_from_vision_json()`을 쓰지 않고, 여기서 자체적으로
vision JSON을 읽어 "정렬된 코너의 실제 변 길이"로 폭/깊이를 계산하고(회전과 무관하게
정확함), 각 박스의 원래 on-table yaw도 같이 뽑아 로컬 dict로 들고 있는다 - 01~08
모듈의 `Object3D`/`Box` 데이터클래스 자체는 yaw 필드를 새로 추가하지 않고 그대로 둔다
(패킹 알고리즘 자체는 여전히 "박스는 축 정렬"이라는 가정으로 폭/깊이만 가지고 0°/90°
중 어느 쪽으로 놓을지만 결정하면 충분하기 때문 - box_top_extractor.py가 이미
cv2.minAreaRect()로 회전된 사각형을 정확히 피팅하므로 Vision 쪽은 손댈 필요 없음).

그 대신 출력 JSON의 각 placement에 필드 2개를 추가한다:
  - "source_yaw_deg": 이 박스가 테이블 위에서 원래 가지고 있던 yaw(도, [0,180) 범위 -
    사각형은 180도 대칭이라 그 이상은 구분 불가).
  - "wrist_yaw_deg": 46.crate_pick_to_place_with_yaw.py가 그리퍼 손목에 그대로 적용해야
    하는 절대 yaw(도). 흡착 그리퍼는 FixedJoint로 "부착 순간의 상대 회전"을 그대로
    고정하므로(36.py의 DynamicSuctionGripper.close() 참고), 손목이 이후 얼마나 돌든
    박스는 그 회전량만큼 그대로 따라 돈다 - 즉 처음 pick(손목 yaw=0)에서는 박스가
    원래 각도(source_yaw_deg) 그대로이므로, 목표 절대각(rotated=False면 0°, True면
    90° - Box.width가 트렁크 x축에 정렬되느냐 y축에 정렬되느냐와 동일한 의미,
    03/07번 파일 참고)에 도달하려면 필요한 손목 회전량은
    "목표 절대각 - source_yaw_deg" 이다(사각형의 180도 대칭을 이용해 최단 회전으로
    정규화).

[입력/실행 방법은 14.py와 동일] - 아래 문서는 14.py 원본 설명 그대로.
========================================================================
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

[실행 예시]
    python 19_run_full_pipeline_with_yaw.py --trunk-map trunk_map.json --boxes boxes.json
    python 19_run_full_pipeline_with_yaw.py --trunk-map trunk_map.json --boxes boxes.json --allow-stacking

[출력]
    <boxes 파일명>_placement_result.json (또는 --out으로 경로 지정) - 각 박스의
    최종 배치 좌표(M0609 base 좌표계 + 내부 로컬 좌표), 크기, 점수, 미적재 사유,
    그리고 이 파일이 추가하는 source_yaw_deg/wrist_yaw_deg를 담는다.
"""

import sys, pathlib, json, argparse
from importlib import import_module

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))
m01 = import_module("01_object3d_schema")
m02 = import_module("02_trunk_space_state")
m03 = import_module("03_extreme_point_candidates")
m06 = import_module("06_loading_order_decision")
m07 = import_module("07_placement_plan")
m08 = import_module("08_unloadable_reason")

Object3D = m01.Object3D
EXPECTED_BOX_FRAME = m01.EXPECTED_BOX_FRAME
object3d_to_box = m01.object3d_to_box
load_trunk_from_world_map = m02.load_trunk_from_world_map
load_obstacles_from_world_map = m02.load_obstacles_from_world_map
local_to_base_frame = m02.local_to_base_frame
ExtremePointState = m03.ExtremePointState
decide_loading_order = m06.decide_loading_order
place_one_box = m07.place_one_box
classify_unloadable_reason = m08.classify_unloadable_reason


def _oriented_footprint(corners_m):
    """45.crate_scan_setup_variable_box_count_with_yaw.py의 동일 함수와 같은 로직 -
    서로 다른 프로세스(이쪽은 순수 파이썬, 저쪽은 Isaac Sim 안에서 실행)라 import를
    공유하지 않고 그대로 복제했다. box_top_extractor.py가 corners_m[:4](윗면)를
    order_rectangle_corners()로 일관된 회전 순서로 정렬해서 저장하므로, 인접한 두
    코너 사이의 실제 변 길이로 폭/깊이를 계산하면 yaw와 무관하게 항상 정확하다."""
    top = np.asarray(corners_m[:4], dtype=float)
    edge01 = top[1] - top[0]
    edge12 = top[2] - top[1]
    width = float(np.hypot(edge01[0], edge01[1]))
    depth = float(np.hypot(edge12[0], edge12[1]))
    yaw_deg = float(np.degrees(np.arctan2(edge01[1], edge01[0])) % 180.0)
    return width, depth, yaw_deg


def load_boxes_from_vision_json_with_yaw(path):
    """m01.load_boxes_from_vision_json()과 같은 입력 포맷/검증을 따르지만, 크기를
    axis-aligned bounding box(min/max) 대신 _oriented_footprint()의 변 길이로
    계산한다 - 회전된 박스에서도 정확한 크기를 낸다. 반환값: (Object3D 리스트,
    {box_id: source_yaw_deg} dict) - yaw는 Object3D/Box 스키마에 없는 필드라 별도로
    반환한다.

    실제로 발생한 문제: 이 함수는 m01.load_boxes_from_vision_json()을 안 쓰고 vision
    JSON을 독자적으로 파싱하는 복사본이라, m01 쪽에 나중에 추가된 rests_on_id
    자동 판정(⑥ 픽업 순서 제약용 - 카트 위에서 다른 박스에 얹힌 박스는 먼저 못 집게
    막는 로직)이 여기엔 반영이 안 돼 있었다 - 그래서 이 진입점으로 돌리면 맨 밑에
    깔린 박스가 그대로 1번으로 나오는 문제가 재현됐다. m01._infer_rests_on_ids()를
    그대로 재사용해서(로직 중복 방지 - AABB는 yaw와 무관하게 이미 정확하므로 회전
    보정과 무관하게 그대로 쓸 수 있다) 같은 판정을 여기서도 적용한다."""
    data = json.loads(pathlib.Path(path).read_text())

    frame = data.get("coordinate_frame")
    if frame != EXPECTED_BOX_FRAME:
        raise ValueError(
            f"박스 비전 데이터의 좌표계가 '{frame}'인데 '{EXPECTED_BOX_FRAME}'이어야 함 - "
            f"트렁크 데이터(trunk_map.json)와 같은 좌표계로 맞춰서 다시 내보내달라고 "
            f"요청해야 함 (카메라 좌표계 그대로 쓰면 엉뚱한 자리에 배치됨)"
        )

    parsed = []
    box_aabbs = {}
    source_yaw_by_id = {}
    for entry in data.get("boxes", []):
        corners = entry["corners_m"]
        xs = [c[0] for c in corners]
        ys = [c[1] for c in corners]
        zs = [c[2] for c in corners]
        x_min, x_max = min(xs), max(xs)
        y_min, y_max = min(ys), max(ys)
        z_min, z_max = min(zs), max(zs)
        # 중심 위치는 AABB 중점으로 계산해도 정확하다 - 회전된 직사각형도 자기 중심에
        # 대해 점대칭이라 AABB 역시 그 같은 중심에 대해 대칭이기 때문(크기만 부풀지,
        # 중심 좌표는 회전과 무관하게 그대로 맞다).
        center_xyz = ((x_min + x_max) / 2, (y_min + y_max) / 2, (z_min + z_max) / 2)
        width, depth, yaw_deg = _oriented_footprint(corners)
        height = z_max - z_min
        size_xyz = (width, depth, height)
        volume = size_xyz[0] * size_xyz[1] * size_xyz[2]

        box_id = str(entry["box_id"])
        box_aabbs[box_id] = (x_min, x_max, y_min, y_max, z_min, z_max)
        parsed.append((box_id, center_xyz, size_xyz, volume))
        source_yaw_by_id[box_id] = yaw_deg

    rests_on_by_id = m01._infer_rests_on_ids(box_aabbs)

    boxes = []
    for box_id, center_xyz, size_xyz, volume in parsed:
        boxes.append(Object3D(
            id=box_id,
            center_xyz=center_xyz,
            size_xyz=size_xyz,
            volume=volume,
            confidence=1.0,
            rests_on_id=rests_on_by_id.get(box_id),
        ))
    return boxes, source_yaw_by_id


def run_pipeline(trunk_map_path, boxes_path, allow_stacking: bool = False) -> dict:
    """
    실제 Vision 데이터 두 파일을 받아서 전체 파이프라인(①②③④⑤⑥⑦⑧⑬)을 돌리고
    결과를 dict로 반환한다. 14.py와 동일하지만 박스 로딩만 yaw-aware 버전을 쓰고,
    출력에 source_yaw_deg/wrist_yaw_deg를 추가한다.
    """
    # [②] 트렁크 로딩 - base 좌표계 원본을 내부 계산용 로컬 좌표(0,0,0 코너 기준)로 변환
    world_map = load_trunk_from_world_map(trunk_map_path)
    trunk, offset = world_map.to_bounding_trunk()
    obstacles = load_obstacles_from_world_map(trunk_map_path, offset)

    # [①, yaw-aware] 박스 비전 데이터 로딩 - 여기서 좌표계 검증(m0609_base_link)까지 됨
    object3ds, source_yaw_by_id = load_boxes_from_vision_json_with_yaw(boxes_path)
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

        # plan.rotated=True는 "박스가 보고한 (width,depth)를 서로 바꿔서(가로/세로
        # swap) 놓는다"는 뜻이고, 트렁크 자체가 축 정렬이므로 이건 절대 목표각
        # 0°(미회전) 또는 90°(회전)와 동일하다(03/07번 파일: width=트렁크 x축 정렬
        # 치수, depth=y축 정렬 치수). 흡착 그리퍼는 부착 순간의 상대 회전을 그대로
        # 고정하고(36.py) 처음 pick은 항상 손목 yaw=0에서 하므로, 필요한 손목 회전량
        # = 목표 절대각 - source_yaw_deg. 사각형은 180도 대칭이므로 최단 회전으로
        # 정규화한다((x+90)%180-90 -> [-90,90] 범위).
        source_yaw_deg = source_yaw_by_id.get(str(box.id), 0.0)
        target_deg = 90.0 if plan.rotated else 0.0
        wrist_yaw_deg = ((target_deg - source_yaw_deg + 90.0) % 180.0) - 90.0

        placements.append({
            "box_id": plan.box_id,
            "order": plan.order,
            "position_base_frame": [bx, by, bz],
            "position_local": list(plan.position),
            "dimensions": list(plan.dimensions),
            "score": plan.score,
            "touches": plan.touches,
            "rotated": plan.rotated,
            "source_yaw_deg": source_yaw_deg,
            "wrist_yaw_deg": wrist_yaw_deg,
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
        description="Cart2Trunk 적재 알고리즘 실행(yaw 지원) - Vision 데이터로 박스 배치 좌표+회전 계산")
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
              f"크기={p['dimensions']}  source_yaw={p['source_yaw_deg']:.1f}°  "
              f"wrist_yaw={p['wrist_yaw_deg']:+.1f}°")
    for u in result["unloadable"]:
        print(f"  [미적재] {u['box_id']}: {u['reason']}")
    print(f"\n결과 저장: {out_path}")


if __name__ == "__main__":
    main()
