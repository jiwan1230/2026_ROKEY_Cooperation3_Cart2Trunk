"""
03_extreme_point_candidates.py
③ Extreme Point 후보 생성
===========================
상태: 🟢 완료·확정

[극점(Extreme Point) 알고리즘이란]
박스를 하나 놓을 때마다, 그 박스의 오른쪽 끝(x+width)/안쪽 끝(y+depth)/
윗쪽 끝(z+height) 세 지점을 "다음 박스를 놓을 수도 있는 후보 자리"로
새로 추가한다. 이사할 때 짐을 하나 넣으면 "이 짐 옆, 이 짐 뒤, 이 짐 위"에
다음 짐을 놓을 수 있겠다고 자연스럽게 후보를 떠올리는 것과 같다.

[왜 층(layer) 쌓기 대신 이 방식을 골랐나]
층 쌓기는 "지금까지 몇 층째인지"같은 절차적 상태를 기억해야 하는데,
극점 방식은 "지금 놓여있는 박스들 + 후보 좌표 집합"만 있으면 언제든
그 상태를 재구성할 수 있다. 이게 트렁크를 매번 재스캔해서 상태를
갱신하는 워크플로우(⑨)와 궁합이 맞는 이유다.
"""

from dataclasses import dataclass, field
from typing import List, Optional, Set, Tuple


@dataclass(frozen=True)
class Box:
    """적재 대상 박스 (기획안 9.6절 규격 기준)."""
    id: str
    width: float   # x축 (m)
    depth: float   # y축 (m)
    height: float  # z축 (m)
    mass_kg: float = 0.0
    is_fragile: bool = False

    # 카트 위에서 이 박스가 다른 어떤 박스 위에 얹혀 있는지(그 박스의 id) - 없으면
    # None(바닥에 있거나 관계 정보 없음). ⑥ decide_loading_order()가 "위에 아무것도
    # 안 얹힌 것부터 집는다"는 픽업 순서 제약을 지키는 데 씀. 비전 쪽 실제 필드명
    # (support_candidate_id 등)과의 정확한 매핑은 ①에서 별도로 확정 필요.
    rests_on_id: Optional[str] = None

    # 이 박스가 몇 번째 정류장에서 내려지는지 (1=첫 배송지) - 카트/트렁크 시나리오와
    # 무관한 범용 필드라 여기 코어에 둔다. industry_scenarios/scenario1_delivery_
    # truck.py가 "나중 정류장(숫자 큰 것)부터 트렁크 깊숙이 싣는다"(LIFO)는 정책에
    # 씀. 없으면(None) 이 정책 대상이 아니라는 뜻.
    delivery_stop: Optional[int] = None

    # 위험물 분류(예: "oxidizer", "flammable") - industry_scenarios/scenario4_
    # hazmat.py가 "이 분류끼리는 인접 배치 금지" 판단에 씀. 없으면(None) 위험물
    # 아님(어떤 것과도 같이 놓여도 됨).
    hazard_class: Optional[str] = None

    @property
    def volume(self) -> float:
        return self.width * self.depth * self.height


@dataclass
class PlacedBox:
    """트렁크 안에 실제로 배치된 박스 (좌표 포함)."""
    box: Box
    x: float
    y: float
    z: float

    @property
    def x_range(self) -> Tuple[float, float]:
        return (self.x, self.x + self.box.width)

    @property
    def y_range(self) -> Tuple[float, float]:
        return (self.y, self.y + self.box.depth)

    @property
    def z_range(self) -> Tuple[float, float]:
        return (self.z, self.z + self.box.height)


@dataclass
class ExtremePointState:
    """
    현재까지 배치된 박스들 + 다음 박스를 놓을 수 있는 후보 좌표 집합.
    이 두 가지만 있으면 트렁크 내부 상태를 완전히 재구성할 수 있다.
    """
    placed: List[PlacedBox] = field(default_factory=list)
    candidates: Set[Tuple[float, float, float]] = field(
        default_factory=lambda: {(0.0, 0.0, 0.0)}
    )

    def _slide_to_wall_or_obstacle(self, point: Tuple[float, float, float], slide_axis: int) -> float:
        """
        point를 slide_axis 방향으로 0쪽(벽 쪽)으로 밀었을 때, 가장 먼저 부딪히는
        지점을 반환한다. 그 축 위에서 다른 두 좌표를 가로막는 placed 박스가 없으면
        벽(0.0)까지, 있으면 그 중 point에 가장 가까운 박스의 먼 쪽 면에서 멈춘다.
        """
        coords = list(point)
        target = coords[slide_axis]
        other_axes = [i for i in range(3) if i != slide_axis]

        best = 0.0
        for p in self.placed:
            p_ranges = (p.x_range, p.y_range, p.z_range)
            # 미는 축이 아닌 나머지 두 축에서, point가 이 박스의 범위 안에 걸치는지 확인
            if all(p_ranges[i][0] - 1e-9 <= coords[i] <= p_ranges[i][1] + 1e-9 for i in other_axes):
                far_face = p_ranges[slide_axis][1]
                if far_face <= target + 1e-9 and far_face > best:
                    best = far_face
        return best

    def register_placement(self, placed_box: PlacedBox) -> None:
        """
        박스를 하나 배치한 뒤, 그 박스 기준으로 새 후보 3개(x/y/z축 방향)를 추가한다.
        사용된 후보 좌표는 집합에서 제거한다.

        여기에 더해, 그 3개 모서리 각각을 "자신을 만든 축이 아닌 나머지 두 축" 방향으로
        벽 또는 다른 장애물에 부딪힐 때까지 밀어서 생기는 자리도 추가로 후보에 넣는다.
        장애물 여러 개가 서로 다른 위치에 독립적으로 놓여 있으면, "각 박스 자기 모서리
        3개"만으로는 그 장애물들 사이에 생기는 틈을 못 잡는 경우가 있어서다.
        """
        b = placed_box.box
        used = (placed_box.x, placed_box.y, placed_box.z)
        self.candidates.discard(used)  # 방금 쓴 자리는 더 이상 후보가 아니므로 제거
        self.placed.append(placed_box)  # 배치 완료된 박스 목록에 추가

        # 새로 놓인 박스의 오른쪽(x+width) / 안쪽(y+depth) / 위쪽(z+height) 끝점을
        # 다음 박스가 놓일 수도 있는 새 후보로 추가한다 (③ 극점 알고리즘의 핵심 동작)
        raw_corners = [
            (placed_box.x + b.width, placed_box.y, placed_box.z),   # x축으로 만든 모서리
            (placed_box.x, placed_box.y + b.depth, placed_box.z),   # y축으로 만든 모서리
            (placed_box.x, placed_box.y, placed_box.z + b.height),  # z축으로 만든 모서리
        ]
        for corner in raw_corners:
            self.candidates.add(corner)

        # 각 모서리를 "자신을 만든 축"이 아닌 나머지 두 축 방향으로 밀어서 추가 후보 생성
        for axis_built, corner in enumerate(raw_corners):
            for slide_axis in range(3):
                if slide_axis == axis_built:
                    continue
                slid_value = self._slide_to_wall_or_obstacle(corner, slide_axis)
                if abs(slid_value - corner[slide_axis]) < 1e-9:
                    continue  # 이미 벽/장애물에 붙어 있어서 밀어도 그대로면 새 정보 없음
                slid_corner = list(corner)
                slid_corner[slide_axis] = slid_value
                self.candidates.add(tuple(slid_corner))


def fits_dims(box: Box, trunk) -> bool:
    """박스 자체 크기가 트렁크보다 큰지 여부 (회전은 고려하지 않음 - MVP 범위)."""
    return box.width <= trunk.width and box.depth <= trunk.depth and box.height <= trunk.height


def generate_wall_flush_candidates(box: Box, trunk, candidates, margin: float = 0.0) -> Set[Tuple[float, float, float]]:
    """
    "이 박스라면 벽 A/B/C에 (margin만큼 띄우고) 붙을 수 있는 자리" 후보를 추가로 만든다.

    register_placement()의 후보 생성은 박스 크기와 무관하게 이미 놓인 것들의 모서리만
    보고 후보를 만든다. 그런데 "벽에 딱 붙는 자리"는 놓으려는 박스의 폭/깊이를 알아야
    계산되는 좌표라서, 그 자리를 만들어줄 기존 모서리가 우연히 없으면 실제로는 빈
    공간인데도 후보 자체가 안 생기는 경우가 있다 (실제 발견된 사례: 폭 0.28m 박스가
    들어갈 수 있는 x=0.32 자리가, 그 근처에 아무 것도 안 놓여 있어서 후보에 없었음).

    이미 있는 후보들의 y(또는 x)를 그대로 재사용해서, 나머지 좌표만 "벽에서 margin만큼
    띄운" 값으로 바꿔치기한 변형을 추가로 만든다 - 지금 놓으려는 box 크기가 있어야 계산
    가능하므로 state.candidates에는 저장하지 않고, ⑦(place_one_box)이 매번 그 박스에
    맞게 새로 만들어서 후보 풀에 잠깐 섞어 쓴다. margin=0.0(기본값)이면 이전과 동일하게
    완전히 벽에 붙는 자리를 만든다 - ⑰(마진 확인)가 켜졌을 때만 margin>0으로 호출된다.
    """
    extra: Set[Tuple[float, float, float]] = set()
    wall_a_x = (trunk.width - box.width - margin) if trunk.entrance_near_x else margin
    wall_c_y = margin
    wall_b_y = trunk.depth - box.depth - margin

    for (x, y, z) in candidates:
        extra.add((wall_a_x, y, z))  # 벽 A(안쪽)에서 margin만큼 뗀 변형 - y/z는 기존 후보 그대로
        extra.add((x, wall_c_y, z))  # 벽 C(y=0)에서 margin만큼 뗀 변형 - x/z는 기존 후보 그대로
        extra.add((x, wall_b_y, z))  # 벽 B(y=depth쪽)에서 margin만큼 뗀 변형 - x/z는 기존 후보 그대로
        # 구석(코너) 자리 - x/y 둘 다 벽에서 margin만큼 뗀 조합. margin=0이면 wall_a_x/
        # wall_c_y/wall_b_y 조합이 이미 코너와 같은 값이라 새 정보가 없지만, margin>0일
        # 땐 위 세 변형이 한쪽 축만 margin 처리해서 나머지 축(예: y=0인 채로)이 여전히
        # 벽에 붙어있는 무효 후보가 되므로, 양쪽 다 margin 처리된 진짜 코너 후보가 따로 필요하다.
        extra.add((wall_a_x, wall_c_y, z))
        extra.add((wall_a_x, wall_b_y, z))

    return extra


def _ranges_overlap(a: Tuple[float, float], b: Tuple[float, float]) -> bool:
    """1차원 구간 두 개가 겹치는지 확인 (⑤ count_touching_faces와 같은 계산, 순환
    import를 피하려고 여기 별도로 둠 - ⑤가 이미 ③을 불러오므로 반대 방향은 불가)."""
    return a[0] < b[1] and a[1] > b[0]


def generate_box_flush_candidates(
    box: Box, trunk, candidates, placed: List["PlacedBox"], margin: float = 0.0
) -> Set[Tuple[float, float, float]]:
    """
    "이 박스라면 이미 놓인 다른 박스 옆면에 (margin만큼 띄우고) 붙을 수 있는 자리"
    후보를 추가로 만든다.

    generate_wall_flush_candidates()는 트렁크 바깥쪽 벽(A/B/C)에 딱 붙는 자리는
    커버하지만, "이미 놓인 다른 박스" 옆면에 딱 붙는 자리는 다루지 않는다. 실제
    발견된 사례: 큰 박스가 먼저 벽 A 근처에 놓이면, 그다음 박스가 그 큰 박스
    바로 앞(입구 쪽)에 딱 붙을 수 있는 자리가 물리적으로는 비어있는데도, 그 좌표를
    만들어줄 기존 모서리가 없어서 후보 자체가 안 생기는 경우가 있었다.

    이미 있는 후보들의 좌표를 재사용해서, 각 놓인 박스와 y/z(또는 x/z) 구간이
    겹치는 조합마다 그 박스의 가까운 면·먼 면에서 margin만큼 뗀 변형을 둘 다 만든다.
    margin=0.0(기본값)이면 이전과 동일하게 완전히 옆면에 붙는 자리를 만든다 -
    ⑰(마진 확인)가 켜졌을 때만 margin>0으로 호출된다. z 방향(위/아래로 쌓이는
    관계)에는 margin을 적용하지 않는다 - 그건 딱 맞닿아야(⑬ 받침) 하는 관계라서다.
    "붙였을 때 실제로 다른 것과 안 겹치는지"는 여기서 판단하지 않고 ④(유효성
    검사)에 그대로 맡긴다 - 겹치는 조합만 안 걸러내는 정도는 후보가 좀 늘어나는
    것뿐이라 문제없고, 굳이 두 번 계산할 필요가 없다. 장애물 하나를 넘어서 그
    뒤의 더 깊은 틈을 찾는 것까지는 다루지 않는다 (①과 같은 MVP 범위 - 바로
    인접한 장애물까지만 본다).
    """
    extra: Set[Tuple[float, float, float]] = set()

    for (x, y, z) in candidates:
        x0, x1 = x, x + box.width
        y0, y1 = y, y + box.depth
        z0, z1 = z, z + box.height

        for p in placed:
            px0, px1 = p.x_range
            py0, py1 = p.y_range
            pz0, pz1 = p.z_range

            # x축 방향: 이 박스의 y/z가 p와 겹치면, p의 가까운 면(입구 쪽)과
            # 먼 면(안쪽)에서 각각 margin만큼 뗀 x로 바꿔치기
            if _ranges_overlap((y0, y1), (py0, py1)) and _ranges_overlap((z0, z1), (pz0, pz1)):
                extra.add((px0 - box.width - margin, y, z))  # p 바로 앞(입구 쪽), margin만큼 띄움
                extra.add((px1 + margin, y, z))              # p를 지나 바로 뒤(안쪽), margin만큼 띄움

            # y축 방향: 이 박스의 x/z가 p와 겹치면, p의 양쪽 면에서 margin만큼 뗀 y로 바꿔치기
            if _ranges_overlap((x0, x1), (px0, px1)) and _ranges_overlap((z0, z1), (pz0, pz1)):
                extra.add((x, py0 - box.depth - margin, z))
                extra.add((x, py1 + margin, z))

    return extra


if __name__ == "__main__":
    # 간단 데모: 박스 하나 놓으면 후보가 어떻게 늘어나는지 확인
    state = ExtremePointState()
    print("초기 후보:", state.candidates)

    box = Box("B1", width=0.4, depth=0.3, height=0.25)
    placed = PlacedBox(box=box, x=0.0, y=0.0, z=0.0)
    state.register_placement(placed)
    print("박스 1개 배치 후 후보:", state.candidates)
