"""여러 Action/Service의 error_code 필드에서 공통으로 쓰는 문자열 상수.

패키지마다 error_code를 직접 문자열로 하드코딩하면 오타나 이름 불일치가
생기기 쉽다 - 실제로 planning/perception/coordinator에서 각자 따로 문자열을
쓰고 있던 것을 여기로 모았다. 아직 쓰이지 않는(motion/execution/safety 등
미구현 패키지의) 코드는 추가하지 않는다 - 실제로 쓰는 곳이 생길 때 같이 추가.
"""

# cart2trunk_planning
INVALID_BOX_DATA = 'INVALID_BOX_DATA'
NO_FEASIBLE_PLACEMENT = 'NO_FEASIBLE_PLACEMENT'

# cart2trunk_perception
DEPTH_TIMEOUT = 'DEPTH_TIMEOUT'
MAP_QUALITY_LOW = 'MAP_QUALITY_LOW'

# cart2trunk_coordinator (하위 Action 호출 실패 공통 사유)
ACTION_TIMEOUT = 'ACTION_TIMEOUT'
ACTION_SERVER_UNAVAILABLE = 'ACTION_SERVER_UNAVAILABLE'
GOAL_REJECTED = 'GOAL_REJECTED'

# cart2trunk_motion (ExecutePickPlace)
PLATFORM_UNAVAILABLE = 'PLATFORM_UNAVAILABLE'  # platform_controller_node의 상태 토픽이 안 들어옴
GRASP_FAILED = 'GRASP_FAILED'  # /gripper/activate가 success=False 반환(기하 조건 밖)
ROBOT_TIMEOUT = 'ROBOT_TIMEOUT'  # drive_to/move_link6_smooth 폐루프가 정체 감지로 중단
