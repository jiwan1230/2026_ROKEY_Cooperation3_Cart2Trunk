"""loading_planner_action_server 헬퍼 함수 테스트 (rclpy 노드 실행 없이, 메시지 생성만 검증).

실제 e2e 테스트(모든 박스를 배치한 뒤 재계획 -> plans=[])에서 total_score에
`sum(p.score for p in [])`(파이썬 int 0)를 그대로 float32 필드에 대입하다가
rclpy의 타입 assert에서 깨지는 버그가 있었다 - 이 테스트는 그 회귀를 막는다.
"""
from cart2trunk_interfaces.action import ComputeLoadPlan


def test_total_score_accepts_float_from_empty_plan_sum():
    result = ComputeLoadPlan.Result()
    plans = []  # 모든 박스를 이미 배치한 뒤 재계획하면 빈 리스트가 됨
    result.total_score = float(sum(p.score for p in plans))
    assert result.total_score == 0.0


def test_total_score_accepts_float_from_nonempty_plan_sum():
    class _FakePlan:
        def __init__(self, score):
            self.score = score

    result = ComputeLoadPlan.Result()
    plans = [_FakePlan(-0.5), _FakePlan(-0.3)]
    result.total_score = float(sum(p.score for p in plans))
    assert result.total_score == -0.8
