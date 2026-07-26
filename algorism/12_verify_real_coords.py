"""
12_verify_real_coords.py
지완이 준 실제 trunk_map.json으로 알고리즘 전체 파이프라인을 검증한다.

경로 구조 (git 저장소 바깥의 run_* 폴더를 직접 참조 - git 충돌 방지):
    cobot3_ws/src/
    ├── 2026_ROKEY_Cooperation3_EDU/                      ← git 저장소
    │   └── isaacpjt/Cart2Trunk/algorism/12_verify_real_coords.py  ← 이 파일
    ├── run_20260720_160153/pointcloud/trunk_map.json
    ├── run_20260720_200104/pointcloud/trunk_map.json
    └── TRUNK_MAP_ROS2_HANDOFF.md
"""
import sys, pathlib
from importlib import import_module

sys.path.insert(0, str(pathlib.Path(__file__).parent))
m02 = import_module("02_trunk_space_state")
m03 = import_module("03_extreme_point_candidates")
m04 = import_module("04_candidate_validity_check")
m06 = import_module("06_loading_order_decision")
m09 = import_module("09_rescan_replan")
m17 = import_module("17_margin_check")

# algorism -> Cart2Trunk -> isaacpjt -> 2026_ROKEY_Cooperation3_EDU -> src
_SRC_DIR = pathlib.Path(__file__).resolve().parents[4]


def run(run_folder_name, boxes):
    json_path = str(_SRC_DIR / run_folder_name / "pointcloud" / "trunk_map.json")
    print(f"\n{'='*60}\n{run_folder_name}\n{'='*60}")

    world_map = m02.load_trunk_from_world_map(json_path)
    trunk, offset = world_map.to_bounding_trunk()
    print("트렁크 크기:", trunk)
    print("오프셋:", offset)

    obstacles = m02.load_obstacles_from_world_map(json_path, offset)
    print("장애물 개수:", len(obstacles))

    plans, unloadable = m09.replan_after_rescan(boxes, trunk, obstacles)

    print("\n--- 배치 결과 ---")
    for p in plans:
        print(" ", p)
    print("--- 미적재 ---")
    for u in unloadable:
        print(" ", u)

    return plans, unloadable, trunk, obstacles


def sanity_check_with_bruteforce(trunk, obstacles, box, step=0.01):
    """
    극점 알고리즘이 놓친 자리가 실제로 있는지 격자로 전수 조사.
    ⑰(박스-벽/박스-박스 마진) 도입 후에는 마진도 같이 확인해야 공정한 비교다 -
    안 그러면 "브루트포스는 자리를 찾았다"는 게 마진을 무시한 착시일 수 있다.
    (z=0 바닥 배치만 다루므로 ⑬/⑮/⑯은 전부 자동 통과 대상이라 여기선 안 봄)
    """
    found = 0
    x = 0.0
    while x + box.width <= trunk.width:
        y = 0.0
        while y + box.depth <= trunk.depth:
            if m04.is_candidate_valid(x, y, 0.0, box, trunk, obstacles) and \
               m17.has_sufficient_margin(x, y, 0.0, box, trunk, obstacles):
                found += 1
            y += step
        x += step
    return found


if __name__ == "__main__":
    print("SRC_DIR 확인:", _SRC_DIR, "존재함?", _SRC_DIR.exists())

    boxes = [
        m03.Box("Small", 0.30, 0.20, 0.15),
        m03.Box("Medium", 0.40, 0.30, 0.25),
        m03.Box("Large", 0.50, 0.35, 0.30),
    ]

    for run_folder in ["run_20260720_160153", "run_20260720_200104"]:
        plans, unloadable, trunk, obstacles = run(run_folder, boxes)

        # 실제 알고리즘이 미적재로 판단한 시점에는 이미 앞서 성공한 다른 카트 박스들도
        # 트렁크를 같이 차지하고 있다 (예: Large가 먼저 놓인 뒤 Medium이 실패하는 경우).
        # 원래 여기서 obstacles(스캔에서 나온 원래 장애물)만 넘기고 있었는데, 그러면
        # 이미 놓인 다른 카트 박스는 브루트포스가 아예 모르는 채로 "빈 공간"이라고
        # 오판해서 거짓 경보를 낸다 - ⑥ 픽업 순서(부피 내림차순, rests_on_id 없을 때)와
        # 같은 순서로 재구성해서, 실패한 박스보다 먼저 처리됐고 실제로 성공한 박스들까지
        # placed 목록에 포함시켜야 공정한 비교다.
        pick_order = m06.decide_loading_order(boxes)
        plan_by_id = {p.box_id: p for p in plans}

        print("\n--- 브루트포스 교차 검증 ---")
        for u in unloadable:
            box = next(b for b in boxes if b.id == u.box_id)
            already_placed = list(obstacles)
            for b in pick_order:
                if b.id == u.box_id:
                    break
                if b.id in plan_by_id:
                    p = plan_by_id[b.id]
                    already_placed.append(m03.PlacedBox(box=b, x=p.position[0], y=p.position[1], z=p.position[2]))
            cnt = sanity_check_with_bruteforce(trunk, already_placed, box)
            flag = "⚠️ 실제로는 자리 있음 (알고리즘 문제)" if cnt > 0 else "진짜로 자리 없음"
            print(f"  {u.box_id}: 격자 전수조사 결과 {cnt}곳 발견 → {flag}")