"""
01_object3d_schema.py
① Planner 입력 정의 (Object3D 입력 스키마)
================================================
상태: 🟢 확정 (Q1~Q4 팀 답변 반영, 7/20)

[비유]
택배 상자에 붙은 송장과 같다. 택배기사가 상자를 열어보지 않아도
송장(받는 사람/크기/무게)만 보고 처리할 수 있는 것처럼, Planner(선욱)는
Object3D라는 "정보표 한 장"만 받으면 박스를 직접 안 봐도 계산할 수 있다.

[확정된 필드] (팀 문서에 이미 명시됨, 논쟁 여지 없음)
    id, center_xyz, size_xyz, volume

[Q1~Q4 팀 답변 반영 완료]

Q1 (준형 - Vision): confidence 0.7 이하는 Vision 단에서 자동 필터링됨.
    confidence 필드 자체는 Object3D에 포함해서 넘겨주기로 함.

Q2 (지완 - 좌표계) ✅ 완전히 해소됨: 트렁크도, 카트도, 박스(Object3D)도
    전부 M0609(로봇팔) base 좌표계 하나로 통일해서 만든다고 확인함
    (스케치 참고: 로봇이 원점이고, 트렁크 점들(빨강)과 카트 점들(초록)이
    전부 그 원점 기준 벡터로 표현됨). 별도의 "트렁크 로컬 좌표계"나
    "월드 절대좌표"로의 변환 자체가 필요 없음 - 처음부터 끝까지 같은
    좌표계 하나만 쓰면 됨.
    ⚠️ 다만 우리 알고리즘(③ Extreme Point) 내부적으로는 "박스가 놓일
    자리를 (0,0,0)부터 계산"하는 방식이라, 트렁크 데이터가 로봇 원점
    기준값(예: (-0.3, 0.5, 0.1) 같은 음수 포함 값)으로 오면, 이걸 계산용
    로컬 좌표로 한 번 평행이동하는 게 필요함. 근데 이건 "팀이 해줘야 하는
    변환"이 아니라 "우리가 받은 데이터로 내부에서 알아서 처리하는 것" -
    ②에서 처리함.

Q3 (봉투): 두 가지 답변이 있었음 -
    - 지완(운영 관점): 봉투는 박스처럼 "쌓지" 않음. 박스를 먼저 다 쌓고,
      봉투는 남는 공간에 얹거나 공간이 없으면 바닥에 내려놓는 별도 방식.
    - 준형(인식 관점): 종이 bag은 박스와 비슷한 스키마(AABB)로 표현
      가능하지만, YOLO가 박스와 혼동할 위험 있어 추가 학습 필요.
    → 진행 중 (팀에서 계속 논의 중). 지금 알고리즘은 계속 박스 전용으로 진행.

Q4 (준형 - 박스 ID 지속성): 재스캔해도 같은 박스가 같은 id를 유지하는지
    물어봤더니, "지금은 보장 안 되지만 나중에 해결될 예정"이라는 답변.
    → 지금 당장 우리 쪽에서 급하게 처리할 필요는 없음. 다만 ⑨(재스캔 후
    재계획)를 실제로 완성하려면 결국 필요한 전제조건이라, "나중에"가 언제
    쯤인지는 1차 통합(7/22) 전에 한 번 더 확인하는 게 좋음. 지금은 ⑨를
    "매번 트렁크를 처음부터 새로 인식한다"는 보수적 가정으로 임시 구현.
"""

from dataclasses import dataclass
from typing import List, Optional, Tuple

# 지완 쪽 실제 박스 비전 출력(all_boxes_corners_*.json)이 이 좌표계여야 한다 -
# trunk_map.json이 이미 이 값으로 나오고 있어서(② 참고) 맞춰야 함. 다른 값이면
# ①이 카메라 좌표계 등 엉뚱한 좌표를 그대로 써버릴 위험이 있어 바로 에러를 낸다.
EXPECTED_BOX_FRAME = "m0609_base_link"


@dataclass(frozen=True)
class Object3D:
    """
    Vision(준형) → Planner(선욱) 로 넘어오는 물품 정보표 한 장.

    좌표계: center_xyz는 M0609(로봇팔) base 좌표계 기준 (Q2 확정).
    트렁크(TrunkWorldMap)도 카트도 전부 같은 좌표계를 쓰므로 별도 변환 불필요.
    """
    id: str
    center_xyz: Tuple[float, float, float]   # M0609 base 좌표계 기준
    size_xyz: Tuple[float, float, float]     # (width, depth, height) 순서
    volume: float
    confidence: float                         # Q1 확정: 0.7 초과 값만 들어옴

    # Q4 반영: 재스캔 전후 유지되는 진짜 지속 ID. 지금은 보장 안 됨 (추후 해결 예정).
    # None이면 아직 추적 로직이 없다는 뜻 (현재 상태).
    object_id: Optional[str] = None

    # ⑥ 픽업 순서 제약용 - 이 박스가 카트 위에서 다른 어떤 박스 위에 얹혀 있는지
    # (그 박스의 id). 없으면 None(바닥에 있거나 관계 정보 없음). 실제 비전 필드
    # (support_candidate_id 등)와의 정확한 매핑은 아직 미확정 - load_boxes_from_vision_json()
    # 참고.
    rests_on_id: Optional[str] = None

    # 봉투 관련 필드는 Q3 논의가 끝날 때까지 사용하지 않음 (박스 전용으로 진행)


def object3d_to_box(obj: Object3D):
    """
    Object3D(Vision 출력)를 03~08번 파일에서 쓰는 Box로 변환하는 어댑터.
    좌표계가 이미 통일돼 있으므로(Q2), 크기(size_xyz)만 그대로 옮기면 됨.
    위치(center_xyz)는 트렁크 로컬 오프셋 계산 후 ⑦에서 다시 계산하므로
    여기서는 다루지 않음. rests_on_id는 ⑥이 그대로 쓸 수 있게 끊기지 않고 넘긴다.
    """
    from importlib import import_module
    import sys, pathlib
    # 파일명이 숫자로 시작해서(예: "03_...") 일반 import문으로는 못 불러오므로
    # 폴더를 sys.path에 넣고 import_module로 문자열 이름째 불러온다.
    Box = import_module("cart2trunk_planning.core.extreme_point_candidates").Box

    w, d, h = obj.size_xyz
    return Box(id=obj.id, width=w, depth=d, height=h, rests_on_id=obj.rests_on_id)


def load_boxes_from_vision_json(path) -> List[Object3D]:
    """
    지완의 실제 박스 비전 출력(all_boxes_corners_*.json 스타일)을 Object3D
    리스트로 변환한다. ②의 load_trunk_from_world_map()과 짝을 이루는 박스용 로더.

    실제 포맷은 center_xyz/size_xyz가 아니라 박스 하나당 8개 모서리 좌표
    (corners_m, 회전된 3D 박스)로 온다. 우리 알고리즘은 회전을 다루지 않으므로
    (MVP 범위, ③ fits_dims 참고) 트렁크 변환(② to_bounding_trunk)과 같은 방식으로
    8개 점의 min/max로 AABB를 근사해서 center/size를 뽑는다.

    confidence 필드는 실제 샘플에 아예 없다 - Q1 답변대로 "0.7 이하는 Vision 단에서
    이미 필터링됨"을 전제로, 여기 도착한 박스는 전부 confidence=1.0으로 채운다.
    """
    import json
    from pathlib import Path

    data = json.loads(Path(path).read_text())

    frame = data.get("coordinate_frame")
    if frame != EXPECTED_BOX_FRAME:
        raise ValueError(
            f"박스 비전 데이터의 좌표계가 '{frame}'인데 '{EXPECTED_BOX_FRAME}'이어야 함 - "
            f"트렁크 데이터(trunk_map.json)와 같은 좌표계로 맞춰서 다시 내보내달라고 "
            f"요청해야 함 (카메라 좌표계 그대로 쓰면 엉뚱한 자리에 배치됨)"
        )

    boxes = []
    for entry in data.get("boxes", []):
        corners = entry["corners_m"]
        xs = [c[0] for c in corners]
        ys = [c[1] for c in corners]
        zs = [c[2] for c in corners]
        x_min, x_max = min(xs), max(xs)
        y_min, y_max = min(ys), max(ys)
        z_min, z_max = min(zs), max(zs)

        size_xyz = (x_max - x_min, y_max - y_min, z_max - z_min)
        center_xyz = ((x_min + x_max) / 2, (y_min + y_max) / 2, (z_min + z_max) / 2)
        volume = size_xyz[0] * size_xyz[1] * size_xyz[2]

        boxes.append(Object3D(
            id=str(entry["box_id"]),
            center_xyz=center_xyz,
            size_xyz=size_xyz,
            volume=volume,
            confidence=1.0,
        ))
    return boxes


# ---------------------------------------------------------------------------
# 지금까지 검증에 사용한 임시 테스트 데이터 (기획안 9.6절 박스 3종 규격)
# ---------------------------------------------------------------------------

TEST_OBJECT3D = {
    "SMALL": Object3D("B1", (0.0, 0.0, 0.0), (0.30, 0.20, 0.15), 0.30 * 0.20 * 0.15, confidence=0.95),
    "MEDIUM": Object3D("B2", (0.0, 0.0, 0.0), (0.40, 0.30, 0.25), 0.40 * 0.30 * 0.25, confidence=0.95),
    "LARGE": Object3D("B3", (0.0, 0.0, 0.0), (0.50, 0.35, 0.30), 0.50 * 0.35 * 0.30, confidence=0.95),
}