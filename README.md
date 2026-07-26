# Cart2Trunk ROS 2 Workspace

3대 PC(MSI1 / MSI2 / Lenovo)에 배포되는 Cart2Trunk 적재 시스템의 통합 ROS 2 워크스페이스입니다.
검증된 프로토타입 코드의 출처는 [`2026_ROKEY_Cooperation3_EDU`](https://github.com/jiwan1230/2026_ROKEY_Cooperation3_EDU) 저장소(`algorism` 브랜치)입니다.

## 최종 하드웨어 플랫폼

바닥에 붙어 이동하는 **옴니휠 모바일 베이스** + **승강 리프트** + 그 위에 **M0609 로봇팔 + 흡착 그리퍼 + RealSense 카메라**.
(Isaac Sim 프로토타입에서 쓰던 Nova Carter는 시뮬레이션 전용 placeholder였고, 최종 실기체가 아님 —
`cart2trunk_platform`의 컨트롤러/어댑터 네이밍은 특정 로봇명에 종속되지 않도록 일반화한다.)

| 구성 요소 | 담당 컨트롤러(`cart2trunk_platform`) |
| --- | --- |
| 옴니휠 모바일 베이스 | `mobile_base_controller.py` |
| 승강 리프트 | `lift_controller.py` |
| M0609 로봇팔 | `m0609_controller.py` |
| 흡착 그리퍼 | `gripper_controller.py` |
| RealSense 카메라 | `camera_controller.py` |

## PC 역할

| PC | 역할 | 실행 Launch |
| --- | --- | --- |
| MSI2 | 중앙 제어 · Isaac Sim · 플랫폼/모션 실행 | `cart2trunk_bringup/launch/msi2_sim_execution.launch.py` |
| MSI1 | Depth 기반 카트/트렁크 인식 | `cart2trunk_bringup/launch/msi1_perception.launch.py` |
| Lenovo | 적재 순서·배치 계획 | `cart2trunk_bringup/launch/lenovo_planning.launch.py` |

세 PC는 동일한 저장소를 clone하고 같은 커밋/태그를 사용하며, `ROS_DOMAIN_ID`는 세 PC가 동일해야 합니다.

## 빌드

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 패키지 구조

- `cart2trunk_interfaces` — 공용 msg/srv/action 정의 (세 PC 모두 동일 버전 필요)
- `cart2trunk_common` — 좌표 변환, 공통 데이터 검증, Error Code
- `cart2trunk_simulation` — Isaac Sim Scene/Asset/센서 브리지 (MSI2)
- `cart2trunk_platform` — 모바일 베이스·리프트·M0609·그리퍼·카메라 하드웨어 제어 계층 (MSI2, Isaac Sim/실기체 어댑터 분리)
- `cart2trunk_motion` — Platform 컨트롤러를 조합한 Scan/Pick/Transport/Place Action Server (MSI2)
- `cart2trunk_perception` — Depth → 카트/박스/트렁크 인식 (MSI1)
- `cart2trunk_planning` — 적재 계획 알고리즘 (Lenovo)
- `cart2trunk_coordinator` — 전체 상태 머신 (MSI2)
- `cart2trunk_safety` — Emergency Stop / Watchdog (MSI2)
- `cart2trunk_hmi` — Flask/WebSocket 백엔드
- `cart2trunk_bringup` — Host별 launch 파일
- `cart2trunk_test_scenarios` — 축소형(테이블+크레이트) 통합 테스트 리그 — EDU 저장소 35~46번 계열의 후신
- `web/` — 웹 플래너(React+Flask). ROS2 노드가 아니라 독립 프로세스로 뜨는 사람용 UI/실험 도구 - 실행 방법은 `web/README.md` 참고. `web/backend`는 `algorism/`(아래)을 직접 import해서 적재 계획을 미리보기하고, `/api/robot/*`는 아직 더미(TODO(MSI2) - 실제 ROS2 트리거로 교체 예정)
- `algorism/` — EDU 저장소 algorism 브랜치의 번호 붙은 알고리즘 스크립트 원본을 그대로 벤더링한 참조 사본(수정 금지). `web/backend`만 이걸 직접 import한다 - ROS2 액션 서버들이 실제로 쓰는 것은 이걸 포팅한 `cart2trunk_planning/core`이며, 두 사본은 파라미터 지원 범위가 다를 수 있다(웹 쪽이 더 풍부함, 아직 미통합)

## EDU 저장소 코드의 실제 이관 대상 (2026-07-23 정정)

최초 검토에서는 EDU 저장소의 `35~37`번을 주요 이관 출발점으로 잘못 판단했다. 실제 대응 관계는 다음과 같다.

| EDU 저장소 (`isaacpjt/Cart2Trunk/`) | 실제 역할 | 이관 대상 |
| --- | --- | --- |
| `10~14번` (`trunk_depth_scan` ~ `trunk_scan_with_test_box`), `13.export_trunk_map.py`, `TRUNK_MAP_ROS2_HANDOFF.md` | 최종 트렁크 다중 시점 스캔·맵 생성 기반. **박스 하나 놓을 때마다 트렁크 전체 재스캔**하는 정책 포함 | `cart2trunk_perception` |
| `7번`, `18~27번`(개발 이력, 함수만 추출), `28.cart_to_trunk_pick_place_lift.py`, `29.carry_pose_calibration.py`, `31.collect_mobile_manipulator_lift.py`, `HANDOFF.md` | 최종 모바일 매니퓰레이터(베이스+리프트+M0609+그리퍼) 실행 기반 | `cart2trunk_platform`, `cart2trunk_motion` |
| `algorism/01~19` | 적재 계획 알고리즘 (ROS 비의존, 이미 잘 분리됨) | `cart2trunk_planning/core` |
| `35~46번` | 카트/트렁크를 테이블/크레이트로 축소한 **비전-계획-Pick&Place 통신 검증용 간이 시나리오** — 메인 코드 아님 | `cart2trunk_test_scenarios` |

`perception/box_top_extractor.py`(2,483줄)와 `trunk_map_planner_node.py`는 EDU 저장소에 이미 동작하는 `rclpy.Node`로 존재하므로, 이관 작업은 "새로 작성"이 아니라 "쪼개서 재구성"이다.

⚠️ `isaacpjt/M0609/`는 EDU 저장소 git 히스토리에 커밋된 적이 없는 broken gitlink이며, 실체 파일(RMPflow/URDF/그리퍼 USD)은 로컬 디스크에서만 존재했다. `~/cart2trunk_assets_backup/`에 백업 완료 — 새 저장소로 옮길 때 git이 아닌 수동 복사로 가져와야 한다.
