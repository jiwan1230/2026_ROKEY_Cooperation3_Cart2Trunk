"""
test_19_random_stress.py
⑲ 무작위 스트레스 테스트의 pytest 진입점.

전체 500회는 python3 19_random_stress_test.py로 수동 실행 (조금 걸림). 여기서는
pytest 전체 회귀에 매번 끼워 넣기 좋도록 더 적은 시행(50회, 같은 seed라 앞부분과
동일한 시나리오)만 빠르게 돌려서 회귀를 잡는다.
"""
import sys, pathlib
from importlib import import_module

_m19 = import_module("cart2trunk_planning.core.random_stress_test")

run_stress_test = _m19.run_stress_test


def test_random_stress_50_trials_no_invariant_violations():
    passed, failures = run_stress_test(num_trials=50, seed=42)
    if failures:
        details = "\n".join(
            f"trial={f['trial']} (seed={f['seed']}): {f['problems']}" for f in failures
        )
        raise AssertionError(f"{len(failures)}/50 무작위 시행이 불변조건을 위반함:\n{details}")
    assert passed == 50
