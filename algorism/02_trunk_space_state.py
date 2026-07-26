"""
02_trunk_space_state.py
② 공간 상태 구성
==================
상태: 🟢 실제 데이터 연동 완료 — load_trunk_from_world_map() 구현 (7/20)

[좌표계 정리 - Q2 완전히 해소됨]
트렁크·카트·박스(Object3D) 전부 M0609(로봇팔) base 좌표계 하나로 통일해서
온다 (스케치 확인: 로봇이 원점, 트렁크/카트 점들이 그 원점 기준 벡터).
그래서 "누가 좌표를 변환해줄지" 같은 질문 자체가 필요 없어졌음 -
처음부터 끝까지 같은 좌표계.

[그런데 왜 여전히 오프셋 계산이 필요한가]
우리 극점(Extreme Point) 알고리즘(③)은 "박스가 놓일 자리를 (0,0,0)부터
계산"하는 내부 방식을 쓴다. 근데 로봇 원점 기준 트렁크 점들은 (0,0,0)이
아니라 로봇 위치에 따라 음수·양수가 섞인 값으로 온다 (예: 트렁크가
로봇보다 앞쪽·위쪽에 있으면 z는 양수, x는 로봇 기준 앞/뒤에 따라 음수일
수도 있음). 그래서 "트렁크 점들 중 가장 작은 좌표(min corner)"를 우리가
직접 계산해서, 그 지점을 (0,0,0)으로 잡고 나머지 계산은 전부 그 기준
로컬 좌표로 하는 것 - 이건 팀에게 요청할 변환이 아니라 이 파일 안에서
알아서 처리하는 내부 구현 디테일임.

나중에 배치 결과(PlacementPlan)를 로봇 제어(민결)에게 넘길 때는, 이 오프셋을
다시 더해서 M0609 base 좌표계로 되돌려줘야 함 (base_frame_to_local의 역변환).

[준형 답변 - 트렁크 스캔 데이터 형식 -> 지완 쪽 파이프라인으로 실제 도착 (7/20)]
지완의 13.export_trunk_map.py가 트렁크 포인트클라우드를 M0609 base_link
좌표계의 trunk_map.json(vertices/edges/obstacles)으로 내보낸다.
load_trunk_from_world_map()이 이 파일을 TrunkWorldMap으로 바로 파싱한다.
바닥/좌우벽/안쪽벽(4면)은 그 run의 포인트클라우드에서 실측한 값이고
(edges style="solid"), 천장(문이 닫히는 높이 한계선)은 트렁크를 연 채로
스캔해서 실제로는 존재하지 않는 면이라 설계 상수와 실측(RANSAC 평면) 중
더 낮은/보수적인 값을 쓴다(style="dashed") - to_bounding_trunk()가 만드는
Trunk.height는 이 dashed 값까지 포함한 높이이므로, 적재 계획에서 "이 높이
넘으면 트렁크 문이 안 닫힌다"는 제약으로 그대로 쓰면 된다.

[아직 막힌 지점]
run_20260720_160153을 처리해보니 바닥 위 약 0.5~0.6m 높이에 트렁크 폭
전체에 걸친 뚜렷한 수평 평면이 있었음 (RANSAC 검출, inlier 27만개) - 열린
트렁크 문(리드) 자체가 찍힌 것인지, 다른 구조물인지 아직 확실친 않지만
"그 높이에 뭔가 있었다"는 관측 자체를 신뢰해서 천장(dashed) 값에 이미
보수적으로 반영함 (13.export_trunk_map.py 참고). 휠하우스 같은 작은
돌출부는 grid+connected-component로 좌우 각각 검출해서 obstacles에 담음 -
load_obstacles_from_world_map()으로 PlacedBox로 변환 가능.

[지완 답변 - 재스캔 트리거]
박스 1개 배치할 때마다 무조건 재스캔 ("PER_PLACEMENT" 고정 주기).

[⚠️ 확인 필요 - trunk_map.json 스키마 두 버전 존재]
"Cart2Trunk 최종 프로젝트 시나리오 및 시스템 흐름" 문서 4.2절은 trunk_map.json을
{"map_id":..., "frame_id":..., "quality_score":..., "bounds":{}, "floor":{},
"walls":[], "ceiling_limit":..., "obstacles":[], "occupied_boxes":[],
"free_volume":...} 형태로 정의하는데, 지완의 실제 13.export_trunk_map.py 출력
(위에서 이미 실측 데이터로 검증 완료된 형식)은 {"schema_version":...,
"run_id":..., "vertices":[...], "edges":[...], "obstacles":[...]} 형태로
필드명이 다르다(map_id 없음, quality_score 없음, walls/floor 분리 안 됨 등).

이 파일의 load_trunk_from_world_map()은 실제로 검증된 후자(vertices/edges)
기준으로 이미 동작 중이라 그대로 둔다 - 문서가 정의한 "목표" 스키마로
섣불리 바꾸면 지완 쪽과 이미 맞춰놓은 실제 연동이 깨질 수 있다. 두 스키마 중
최종적으로 뭘 쓸지(혹은 필드명만 다른 같은 데이터인지)는 준형·지완과 확인
필요 - box_snapshot_id/trunk_map_id 정합성 검증(HMI 8절 원칙 #2)을 제대로
구현하려면 map_id에 대응하는 필드가 필요한데, 지금 스키마엔 run_id가 그
역할을 대신할 수 있어 보이지만(20_task_export.py에서 trunk_map_id로 run_id를
임시로 씀) 확정된 답은 아니다.
"""

from dataclasses import dataclass, field
from typing import List, Tuple


@dataclass
class Trunk:
    """
    내부 계산용 트렁크 표현 - 항상 (0,0,0)을 한쪽 코너로 하는 로컬 좌표계.
    실제 M0609 base 좌표계 데이터를 받으면 TrunkWorldMap.to_bounding_trunk()로
    변환해서 이 형태로 만든다.
    """
    width: float   # x축 (m)
    depth: float   # y축 (m)
    height: float  # z축 (m)

    # 입구(로봇이 박스를 넣는 쪽) 방향 힌트. True면 로컬 x=0쪽이 입구에 더 가깝고,
    # False면 반대쪽(로컬 x=width쪽)이 입구에 더 가깝다는 뜻. to_bounding_trunk()가
    # 실제 데이터로 계산해서 채워주고, ⑤ score_candidate()가 "입구에서 먼 자리"를
    # 우선하는 데 사용한다. 기본값 True는 "정보 없으면 지금까지처럼 로컬 원점을
    # 입구로 본다"는 안전한 기본값 - 합성 테스트용 Trunk(w,d,h)처럼 실제 좌표
    # 변환을 거치지 않고 직접 만드는 경우를 그대로 지원하기 위함.
    #
    # y축(depth) 필드가 없는 이유: 로봇은 M0609 base 좌표계 원점에 고정돼 있고,
    # 트렁크에는 항상 정해진 한 방향(x축)으로만 접근한다는 게 확인됐다 (실제 스캔
    # 데이터의 "x, +deep" 라벨과도 일치). 즉 y(좌우 위치)는 입구와 아예 무관해서,
    # x/y를 평균 내던 첫 버전은 "좌우 위치만 달라도 점수가 달라지는" 버그였다.
    entrance_near_x: bool = True


@dataclass
class TrunkWorldMap:
    """
    준형이 제공하는 "M0609 base 좌표계 기준, 점/선으로 표현된 트렁크 뼈대" 데이터.
    실제 포맷은 13.export_trunk_map.py의 trunk_map.json - load_trunk_from_world_map() 참고.
    """
    vertices: List[Tuple[float, float, float]] = field(default_factory=list)  # M0609 base 좌표계 기준
    edges: List[Tuple[int, int]] = field(default_factory=list)  # vertices 인덱스 쌍

    def to_bounding_trunk(self) -> Tuple[Trunk, Tuple[float, float, float]]:
        """
        M0609 base 좌표계의 vertices를 받아서:
          1. 우리 알고리즘이 쓸 로컬 Trunk(0,0,0 코너 기준)로 변환
          2. 그 변환에 쓴 오프셋(= M0609 base 좌표계에서 로컬 원점이 실제로
             어디였는지)을 같이 반환 - 나중에 로봇에게 좌표를 돌려줄 때 필요

        굴곡/오목한 부분은 지금은 무시하고 최소/최대 범위로 근사 (정밀
        형태 반영은 실제 데이터로 검증하며 다음 단계에서 확장).
        """
        if not self.vertices:
            raise ValueError("TrunkWorldMap에 vertices가 없음")

        # 모든 점의 x/y/z 값만 각각 뽑아서 리스트로 분리
        xs = [v[0] for v in self.vertices]
        ys = [v[1] for v in self.vertices]
        zs = [v[2] for v in self.vertices]

        # 점들 중 가장 작은 좌표 = 트렁크를 감싸는 직육면체(AABB)의 한쪽 코너.
        # 이 코너를 로컬 좌표계의 (0,0,0)으로 삼는다.
        offset = (min(xs), min(ys), min(zs))  # M0609 base 좌표계 기준 로컬 원점 위치
        # 폭/깊이/높이 = 최대값 - 최소값 (굴곡은 무시하고 바운딩 박스로 근사)
        trunk_width = max(xs) - min(xs)
        trunk_depth = max(ys) - min(ys)
        trunk_height = max(zs) - min(zs)

        # 입구 방향 추정: M0609 base 좌표계에서는 로봇 팔 자신이 원점(0,0,0)이고,
        # 로봇은 항상 x축 방향으로만 트렁크에 접근한다 (확인된 전제 - y축은 무관).
        # 트렁크의 로컬 x=0쪽 변과 로컬 x=width쪽 변 중, 로봇 원점에 더 가까운 쪽이
        # 곧 로봇이 실제로 손을 뻗어 넣는 "입구"에 더 가깝다고 본다.
        entrance_near_x = abs(offset[0]) <= abs(offset[0] + trunk_width)

        trunk = Trunk(width=trunk_width, depth=trunk_depth, height=trunk_height,
                      entrance_near_x=entrance_near_x)
        return trunk, offset


def local_to_base_frame(x: float, y: float, z: float, offset: Tuple[float, float, float]):
    """
    로컬 좌표(0,0,0 코너 기준) -> M0609 base 좌표계로 되돌리는 변환.
    PlacementPlan을 로봇 제어(민결)에게 넘길 때 이 함수로 되돌려서 전달해야 함.
    """
    ox, oy, oz = offset
    # to_bounding_trunk()에서 뺐던 offset을 다시 더해주면 원래 좌표계로 복귀 (역변환)
    return (x + ox, y + oy, z + oz)


# ---- 8.rescale_and_rebuild.py 시뮬레이션 씬 기준 값 (MVP fallback, 실제 World Map 데이터 예정) ----
# 참고: 이 값은 준형의 실제 M0609 base 좌표계 스캔 데이터가 아니라, 지완이 만든
# 시뮬레이션 씬에서 뽑은 값이라 임시로만 사용.
REAL_TRUNK = Trunk(
    width=3.68 - 3.11,     # 0.57
    depth=0.56 - (-0.56),  # 1.12
    height=1.28 - 1.03,    # 0.25
)
REAL_TRUNK_OFFSET = (3.11, -0.56, 1.03)  # 위 REAL_TRUNK를 만들 때 쓴 로컬 원점 오프셋 (임시값)


# ---- 재스캔 트리거 확정 (지완 답변) ----
RESCAN_TRIGGER_POLICY = "PER_PLACEMENT"  # 박스 1개 놓을 때마다 1회


def load_trunk_from_world_map(raw_scan_data) -> TrunkWorldMap:
    """
    준형이 줄 점/선 형식 데이터의 실제 포맷 확정: 지완의 13.export_trunk_map.py가
    만드는 trunk_map.json (M0609 base_link 좌표계). vertices 8개(바닥 4개 solid +
    천장 4개 dashed)와 edges를 그대로 옮기기만 하면 되고, AABB 근사는
    TrunkWorldMap.to_bounding_trunk()가 맡는다 - 여기서는 포맷 변환만 한다.

    raw_scan_data: trunk_map.json 경로(str/Path) 또는 이미 로드된 dict 둘 다 받는다.
    """
    import json
    from pathlib import Path

    data = json.loads(Path(raw_scan_data).read_text()) if isinstance(raw_scan_data, (str, Path)) else raw_scan_data
    vertices = [tuple(v) for v in data["vertices"]]
    edges = [tuple(e["v"]) for e in data["edges"]]
    return TrunkWorldMap(vertices=vertices, edges=edges)


def load_obstacles_from_world_map(raw_scan_data, offset: Tuple[float, float, float]) -> "List[PlacedBox]":
    """
    trunk_map.json의 obstacles(휠하우스처럼 바닥에서 튀어나온 고정 돌출부)를
    ExtremePointState.register_placement()로 바로 등록 가능한 PlacedBox 리스트로 변환한다.

    offset은 반드시 같은 raw_scan_data로 만든 TrunkWorldMap.to_bounding_trunk()가
    반환한 값을 그대로 넘겨야 한다 - 그래야 Trunk 로컬 좌표계(0,0,0 코너 기준)와
    어긋나지 않는다.

    지금은 열린 트렁크 문으로 추정되는 정체 불명 구조물처럼 애매한 돌출부는
    13.export_trunk_map.py가 자동 박스화를 포기하므로(위 "아직 막힌 지점" 참고)
    obstacles가 비어있을 수도 있다 - 그 경우 그냥 빈 리스트 반환.
    """
    import json
    import sys
    import pathlib
    from importlib import import_module

    data = (json.loads(pathlib.Path(raw_scan_data).read_text())
            if isinstance(raw_scan_data, (str, pathlib.Path)) else raw_scan_data)

    sys.path.insert(0, str(pathlib.Path(__file__).parent))
    _m03 = import_module("03_extreme_point_candidates")
    Box, PlacedBox = _m03.Box, _m03.PlacedBox
    ox, oy, oz = offset

    placed = []
    for i, obs in enumerate(data.get("obstacles", [])):
        xs = [p[0] for p in obs["vertices"]]
        ys = [p[1] for p in obs["vertices"]]
        zs = [p[2] for p in obs["vertices"]]
        x_min, y_min, z_min = min(xs), min(ys), min(zs)
        x_max, y_max, z_max = max(xs), max(ys), max(zs)
        box = Box(id=obs.get("name", f"obstacle_{i}"),
                   width=x_max - x_min, depth=y_max - y_min, height=z_max - z_min,
                   is_obstacle=True)
        placed.append(PlacedBox(box=box, x=x_min - ox, y=y_min - oy, z=z_min - oz))
    return placed


if __name__ == "__main__":
    # 데모: M0609 base 좌표계 기준(음수 포함) 트렁크 점들이 왔을 때 오프셋 계산 확인
    demo_map = TrunkWorldMap(vertices=[
        (-0.3, -0.5, 0.9), (0.27, -0.5, 0.9), (-0.3, 0.5, 0.9), (0.27, 0.5, 0.9),
        (-0.3, -0.5, 1.15), (0.27, -0.5, 1.15), (-0.3, 0.5, 1.15), (0.27, 0.5, 1.15),
    ])
    trunk, offset = demo_map.to_bounding_trunk()
    print("로컬 Trunk:", trunk)
    print("M0609 base 기준 오프셋:", offset)
    print("로컬 (0.1,0.1,0.1)을 base frame으로 되돌리면:", local_to_base_frame(0.1, 0.1, 0.1, offset))

    # 데모 2: 지완의 실제 trunk_map.json으로 연동 확인 (run_20260720_160153)
    import pathlib
    real_run = pathlib.Path(__file__).resolve().parent.parent / "results" / "run_20260720_160153" / "pointcloud" / "trunk_map.json"
    if real_run.exists():
        world_map = load_trunk_from_world_map(real_run)
        real_trunk, real_offset = world_map.to_bounding_trunk()
        obstacles = load_obstacles_from_world_map(real_run, real_offset)
        print("\n[실제 스캔 연동] Trunk:", real_trunk)
        print("[실제 스캔 연동] offset:", real_offset)
        print(f"[실제 스캔 연동] obstacles: {len(obstacles)}개")
        for pb in obstacles:
            print("  -", pb.box.id, "at", (pb.x, pb.y, pb.z))
