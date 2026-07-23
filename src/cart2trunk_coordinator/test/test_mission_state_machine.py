import pytest

from cart2trunk_coordinator.mission_state_machine import (
    InvalidTransition, MissionEvent, MissionState, MissionStateMachine,
)


def test_initial_state_is_idle():
    sm = MissionStateMachine()
    assert sm.state == MissionState.IDLE


def test_happy_path_single_box():
    sm = MissionStateMachine()
    assert sm.handle(MissionEvent.START) == MissionState.BOX_SCAN
    assert sm.handle(MissionEvent.BOXES_SCANNED) == MissionState.TRUNK_SCAN
    assert sm.handle(MissionEvent.TRUNK_SCANNED) == MissionState.PLAN_REQUEST
    assert sm.handle(MissionEvent.PLAN_READY) == MissionState.EXECUTE_TASK
    # 태스크 성공 -> 다음 박스를 위해 트렁크 재스캔부터 다시
    assert sm.handle(MissionEvent.TASK_SUCCEEDED) == MissionState.TRUNK_SCAN
    assert sm.handle(MissionEvent.TRUNK_SCANNED) == MissionState.PLAN_REQUEST
    # 더 이상 놓을 박스가 없음 -> 정상 종료
    assert sm.handle(MissionEvent.PLAN_EMPTY) == MissionState.DONE


@pytest.mark.parametrize('failure_state,event', [
    (MissionState.BOX_SCAN, MissionEvent.BOX_SCAN_FAILED),
    (MissionState.TRUNK_SCAN, MissionEvent.TRUNK_SCAN_FAILED),
    (MissionState.PLAN_REQUEST, MissionEvent.PLAN_FAILED),
    (MissionState.EXECUTE_TASK, MissionEvent.TASK_FAILED),
])
def test_failures_go_to_system_error(failure_state, event):
    sm = MissionStateMachine()
    sm.state = failure_state
    assert sm.handle(event) == MissionState.SYSTEM_ERROR


def test_estop_preempts_any_state():
    for state in MissionState:
        sm = MissionStateMachine()
        sm.state = state
        assert sm.handle(MissionEvent.ESTOP_TRIGGERED) == MissionState.ESTOP


def test_cancel_allowed_while_in_progress():
    for state in (MissionState.BOX_SCAN, MissionState.TRUNK_SCAN,
                  MissionState.PLAN_REQUEST, MissionState.EXECUTE_TASK):
        sm = MissionStateMachine()
        sm.state = state
        assert sm.handle(MissionEvent.CANCEL) == MissionState.CANCELING


def test_cancel_rejected_when_idle_or_terminal():
    for state in (MissionState.IDLE, MissionState.DONE,
                  MissionState.SYSTEM_ERROR, MissionState.ESTOP, MissionState.CANCELING):
        sm = MissionStateMachine()
        sm.state = state
        with pytest.raises(InvalidTransition):
            sm.handle(MissionEvent.CANCEL)


def test_reset_allowed_only_from_terminal_states():
    for state in (MissionState.DONE, MissionState.CANCELING,
                  MissionState.SYSTEM_ERROR, MissionState.ESTOP):
        sm = MissionStateMachine()
        sm.state = state
        assert sm.handle(MissionEvent.RESET) == MissionState.IDLE


def test_reset_rejected_from_in_progress_states():
    for state in (MissionState.IDLE, MissionState.BOX_SCAN, MissionState.TRUNK_SCAN,
                  MissionState.PLAN_REQUEST, MissionState.EXECUTE_TASK):
        sm = MissionStateMachine()
        sm.state = state
        with pytest.raises(InvalidTransition):
            sm.handle(MissionEvent.RESET)


def test_unexpected_event_raises():
    sm = MissionStateMachine()  # IDLE
    with pytest.raises(InvalidTransition):
        sm.handle(MissionEvent.TASK_SUCCEEDED)


def test_is_terminal_property():
    sm = MissionStateMachine()
    assert not sm.is_terminal
    sm.state = MissionState.DONE
    assert sm.is_terminal
