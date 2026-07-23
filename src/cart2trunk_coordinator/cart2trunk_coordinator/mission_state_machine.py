"""MSI2 Mission Coordinator의 상태 전이 로직 (ROS 비의존 순수 Python).

정상 플로우 (docs/EDU 저장소 TRUNK_MAP_ROS2_HANDOFF.md의 PER_PLACEMENT 재스캔
정책을 그대로 반영):

    IDLE -> BOX_SCAN -> TRUNK_SCAN -> PLAN_REQUEST -> EXECUTE_TASK
                            ^                              |
                            +------ (박스 1개 배치마다) -----+
                                                             |
                                                        (남은 박스 없음)
                                                             v
                                                            DONE

카트 박스 목록(BOX_SCAN)은 한 번만 스캔한다 - 매 배치마다 재스캔하는 것은
트렁크(TRUNK_SCAN)뿐이다. PLAN_REQUEST는 매번 "최신 트렁크 맵 + 아직 안 옮긴
카트 박스 목록"으로 전체 후보 계획을 다시 계산하지만, 코디네이터는 그 계획의
첫 번째 Task만 실행한다(설계 문서 11절: 실제 적재 위치가 계획과 어긋나거나
장애물이 새로 생기는 것을 반영하기 위함).
"""
from enum import Enum


class MissionState(str, Enum):
    IDLE = 'IDLE'
    BOX_SCAN = 'BOX_SCAN'
    TRUNK_SCAN = 'TRUNK_SCAN'
    PLAN_REQUEST = 'PLAN_REQUEST'
    EXECUTE_TASK = 'EXECUTE_TASK'
    DONE = 'DONE'
    CANCELING = 'CANCELING'
    ESTOP = 'ESTOP'
    SYSTEM_ERROR = 'SYSTEM_ERROR'


class MissionEvent(str, Enum):
    START = 'START'
    BOXES_SCANNED = 'BOXES_SCANNED'
    BOX_SCAN_FAILED = 'BOX_SCAN_FAILED'
    TRUNK_SCANNED = 'TRUNK_SCANNED'
    TRUNK_SCAN_FAILED = 'TRUNK_SCAN_FAILED'
    PLAN_READY = 'PLAN_READY'
    PLAN_EMPTY = 'PLAN_EMPTY'      # 계획 결과 남은 박스 없음 (정상 종료)
    PLAN_FAILED = 'PLAN_FAILED'    # NO_FEASIBLE_PLACEMENT 등
    TASK_SUCCEEDED = 'TASK_SUCCEEDED'
    TASK_FAILED = 'TASK_FAILED'
    CANCEL = 'CANCEL'
    ESTOP_TRIGGERED = 'ESTOP_TRIGGERED'
    RESET = 'RESET'


class InvalidTransition(Exception):
    pass


# 정상 플로우 전이표: (현재 상태, 이벤트) -> 다음 상태.
_TRANSITIONS = {
    (MissionState.IDLE, MissionEvent.START): MissionState.BOX_SCAN,

    (MissionState.BOX_SCAN, MissionEvent.BOXES_SCANNED): MissionState.TRUNK_SCAN,
    (MissionState.BOX_SCAN, MissionEvent.BOX_SCAN_FAILED): MissionState.SYSTEM_ERROR,

    (MissionState.TRUNK_SCAN, MissionEvent.TRUNK_SCANNED): MissionState.PLAN_REQUEST,
    (MissionState.TRUNK_SCAN, MissionEvent.TRUNK_SCAN_FAILED): MissionState.SYSTEM_ERROR,

    (MissionState.PLAN_REQUEST, MissionEvent.PLAN_READY): MissionState.EXECUTE_TASK,
    (MissionState.PLAN_REQUEST, MissionEvent.PLAN_EMPTY): MissionState.DONE,
    (MissionState.PLAN_REQUEST, MissionEvent.PLAN_FAILED): MissionState.SYSTEM_ERROR,

    # 박스 1개 배치 성공 -> 트렁크 재스캔부터 다시 (PER_PLACEMENT 정책)
    (MissionState.EXECUTE_TASK, MissionEvent.TASK_SUCCEEDED): MissionState.TRUNK_SCAN,
    (MissionState.EXECUTE_TASK, MissionEvent.TASK_FAILED): MissionState.SYSTEM_ERROR,
}

# 진행 중(IDLE/터미널 상태 제외)인 모든 상태에서 취소 가능.
_CANCELABLE_STATES = {
    MissionState.BOX_SCAN, MissionState.TRUNK_SCAN,
    MissionState.PLAN_REQUEST, MissionState.EXECUTE_TASK,
}

# RESET(오류 복구)은 터미널 상태에서만 허용.
_RESETTABLE_STATES = {
    MissionState.DONE, MissionState.CANCELING, MissionState.SYSTEM_ERROR, MissionState.ESTOP,
}


class MissionStateMachine:
    """단일 미션(카트->트렁크 적재 1회 실행)의 현재 상태를 들고 있는 얇은 상태 머신.

    박스/태스크 데이터 자체(remaining_box_ids, 현재 plan 등)는 이 클래스가 아니라
    이 클래스를 사용하는 mission_coordinator_node.py가 들고 있는다 - 여기는
    "지금 어느 단계인가"만 책임진다.
    """

    def __init__(self):
        self.state = MissionState.IDLE

    def handle(self, event: MissionEvent) -> MissionState:
        # ESTOP은 어느 상태에서든 최우선으로 즉시 전이 (물리 비상정지는 언제든 눌릴 수 있음).
        if event == MissionEvent.ESTOP_TRIGGERED:
            self.state = MissionState.ESTOP
            return self.state

        if event == MissionEvent.CANCEL:
            if self.state not in _CANCELABLE_STATES:
                raise InvalidTransition(f'{self.state.value}에서는 취소할 진행 중인 작업이 없음')
            self.state = MissionState.CANCELING
            return self.state

        if event == MissionEvent.RESET:
            if self.state not in _RESETTABLE_STATES:
                raise InvalidTransition(f'{self.state.value}는 터미널 상태가 아니라 RESET 불가')
            self.state = MissionState.IDLE
            return self.state

        key = (self.state, event)
        if key not in _TRANSITIONS:
            raise InvalidTransition(f'{self.state.value}에서 {event.value} 이벤트는 허용되지 않음')
        self.state = _TRANSITIONS[key]
        return self.state

    @property
    def is_terminal(self) -> bool:
        return self.state in _RESETTABLE_STATES
