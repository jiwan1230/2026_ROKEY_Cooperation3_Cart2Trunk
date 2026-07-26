"""
scenario2_warehouse_density.py
산업 현장 시나리오 ② 창고/물류센터 (공간 활용 최대화 - 밀도+개수)
==============================================

[정책]
"공간을 최대한 빽빽하게 쓴다" - 입구 접근성은 무관(파레트는 쌓아두고 나중에
지게차로 다시 꺼내는 용도라, 트럭처럼 "문 열자마자 바로 손 닿아야" 하는 제약이
없다).

[코어로 승격됨 - 이 파일은 이제 얇은 래퍼]
원래 이 파일에 직접 구현했던 로직(footprint-growth 점수 + 작은 것부터 담는
순서)이 실제 로봇 결합용 "적재 모드 전환" 기능으로 코어에 정식 승격됐다:
- ⑤ 점수: 05_candidate_scoring.score_count_first
- ⑥ 순서: 06_loading_order_decision.decide_loading_order(mode="count_first")
- 통합 진입점: 08_unloadable_reason.generate_loading_plan(mode="count_first")

이 창고 시나리오는 이제 그 통합 진입점을 mode="count_first"로 호출하는 것과
완전히 같아서, 별도 로직을 유지하지 않고 그대로 위임한다 (같은 로직이 두 곳에
있으면 나중에 어긋날 위험 - 코어 쪽 변경사항을 두 번 반영할 필요가 없어짐).

[코어 mode="count_first"가 여기 흡수했던 것들]
- 원래 시나리오 6(최대 개수 적재)의 "작은 것부터" 순서
- 시행착오: 접촉면 점수는 ⑰ 마진 때문에 변별력을 잃는다는 걸 확인하고
  footprint-growth 방식으로 변경 (05_candidate_scoring.score_count_first
  docstring 참고)
- 시나리오 6의 "마진 0"은 실제 로봇 PLACE 하강 중 박스 코너가 튕겨나가는
  위험이 있어 흡수하지 않고 버림 (margin은 여전히 사용자가 원하면 조절
  가능 - generate_loading_plan_count_first의 margin 인자 참고)
"""

import sys
import pathlib
from importlib import import_module
from typing import List, Tuple

_ALGORISM_DIR = pathlib.Path(__file__).resolve().parent.parent
if str(_ALGORISM_DIR) not in sys.path:
    sys.path.insert(0, str(_ALGORISM_DIR))

_m03 = import_module("03_extreme_point_candidates")
_m08 = import_module("08_unloadable_reason")

Box = _m03.Box
generate_loading_plan = _m08.generate_loading_plan


def generate_loading_plan_count_first(boxes: List["Box"], trunk, margin=None) -> Tuple[list, list]:
    """08_unloadable_reason.generate_loading_plan(mode="count_first")의 얇은 래퍼."""
    return generate_loading_plan(boxes, trunk, mode="count_first", margin=margin)


if __name__ == "__main__":
    _m02 = import_module("02_trunk_space_state")
    Trunk = _m02.Trunk

    # "작은 것부터+공간 재사용" 정책 덕분에, 빠듯한 트렁크에 크기가 뒤섞인
    # 박스들을 넣을 때 기본 정책보다 훨씬 많이 들어간다.
    trunk = Trunk(width=0.6, depth=0.4, height=0.45)  # ⑮ 상단 여유(0.2m) 감안
    mixed_boxes = (
        [Box(f"소{i}", 0.1, 0.1, 0.1) for i in range(6)]
        + [Box(f"대{i}", 0.3, 0.2, 0.2) for i in range(2)]
    )
    default_plans, default_unloadable = generate_loading_plan(mixed_boxes, trunk)
    count_first_plans, count_first_unloadable = generate_loading_plan_count_first(mixed_boxes, trunk)
    print(f"[기본(large_first)] 배치 {len(default_plans)}/{len(mixed_boxes)}, 미적재 {len(default_unloadable)}")
    print(f"[창고(count_first)]  배치 {len(count_first_plans)}/{len(mixed_boxes)}, 미적재 {len(count_first_unloadable)}")
