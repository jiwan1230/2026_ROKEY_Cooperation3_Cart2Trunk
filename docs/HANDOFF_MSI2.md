# Cart2Trunk MSI2 인수인계서

작성일: 2026-07-24 · 작성 환경: Lenovo (`jiwan`) · 작성자: Claude Code (Lenovo 세션)

이 문서는 대화 세션이 끊긴 채로 MSI2의 Claude Code가 이 프로젝트를 이어받는다는
전제로 쓴다. **이 문서를 읽는 사람은 이전 대화 맥락이 전혀 없다** — 그래서 "왜
이렇게 짰는지"까지 최대한 근거와 함께 적는다. 요약이 아니라 인수인계이므로 길다.
급하면 [8. MSI2가 지금 당장 해야 할 일](#8-msi2가-지금-당장-해야-할-일)부터 읽어도
되지만, 최소한 [6. 반드시 지켜야 하는 설계 결정](#6-반드시-지켜야-하는-설계-결정과-컨벤션)은
코드를 건드리기 전에 읽을 것 — 여기 나온 컨벤션을 어기면 이미 동작 검증된 3개
패키지 간 연동이 조용히 깨진다.

---

## 1. 프로젝트가 뭔가

Cart2Trunk: 카트에 실린 박스를 로봇이 집어서 차량 트렁크에 자동으로 적재하는
시스템. 3대 PC(MSI1/MSI2/레노버)에 걸친 분산 ROS 2 Humble 시스템으로 재구성 중.

**최종 하드웨어 (2026-07-23 확정, Isaac Sim 프로토타입의 Nova Carter가 아님):**

```
바닥에 붙어 이동하는 옴니휠 모바일 베이스
  └─ 승강 리프트
       └─ M0609 로봇팔 + 흡착 그리퍼 + RealSense 카메라
```

Isaac Sim에서 그동안 써온 Nova Carter는 시뮬레이션 전용 placeholder였다. **새로
짜는 `cart2trunk_platform`/`cart2trunk_motion`의 컨트롤러 이름·구조를 "nova_carter_*"
같은 특정 로봇명에 종속시키지 말 것** — `mobile_base_controller.py`처럼 일반화된
이름으로 짜고, Isaac Sim 시뮬레이션용 어댑터와 실기체 어댑터를 분리하는 구조로
가야 한다 (아래 8절 참고).

**PC 역할 분담:**

| PC | 역할 |
| --- | --- |
| MSI2 | 중앙 제어(Coordinator) · Isaac Sim · 카메라 브리지 · Pick & Place 실행 |
| MSI1 | Depth 기반 카트/트렁크 인식 |
| 레노버 | 적재 순서·배치 계획 |

**제어 흐름 원칙**: MSI1이 레노버를 직접 부르지 않는다. MSI2의 `mission_coordinator_node`가
모든 단계 호출 순서를 관리하는 중앙집중 구조다 (이미 그렇게 구현되어 있음, 6절 참고).

---

## 2. 저장소 상태

### 2.1 저장소 두 개, 역할이 다르다

```text
2026_ROKEY_Cooperation3_EDU (기존 저장소, algorism 브랜치)
  = Isaac Sim 프로토타입, 알고리즘 검증, 시행착오 보존. 여기서 "검증된 로직"만
    발췌해서 새 저장소로 이관하는 작업을 하고 있다. 새 시스템의 메인 코드가
    아니다.

2026_ROKEY_Cooperation3_Cart2Trunk (새 저장소, 이 문서가 있는 곳)
  = 3PC에서 실제 실행할 ROS 2 통합 시스템. MSI2는 이 저장소를 clone해서 작업한다.
```

```bash
git clone git@github.com:jiwan1230/2026_ROKEY_Cooperation3_Cart2Trunk.git ~/cart2trunk_ws
cd ~/cart2trunk_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

현재 `main` 브랜치 커밋 히스토리 (레노버에서 전부 push 완료):

```text
3e05b0a cart2trunk_common 실제 구현: 좌표 변환 + error_code 공통화, 3개 패키지 리팩터링
7750925 cart2trunk_coordinator 실제 구현: Mission 상태 머신 + 5노드 실전 통합
fed8fc3 cart2trunk_perception 실제 구현: trunk_map_builder 포팅 + ScanTrunk Action 서버
01b4448 cart2trunk_planning 실제 구현: algorism/01~19 포팅 + ComputeLoadPlan Action 서버
ae22c00 Merge remote initial README
748be32 README에 리프트 하드웨어 구성요소·컨트롤러 매핑 추가
797166c 이관 매핑 정정: 35~46은 테스트 시나리오, 10~14/7·18~31이 실제 이관 기반
1e251f7 Cart2Trunk ROS 2 워크스페이스 뼈대 생성
```

각 커밋 메시지가 꽤 상세하니(뭘 왜 했는지, 어떤 버그를 잡았는지) `git log -p`로
훑어보는 게 이 문서보다 정확할 수 있다.

### 2.2 EDU 저장소 코드-역할 재매핑 (중요 — 처음에 잘못 판단했던 부분)

EDU 저장소 `isaacpjt/Cart2Trunk/`의 번호 붙은 스크립트들을 처음엔 "35~37번이 새
시스템의 메인 이관 대상"이라고 잘못 판단했다가, 실제로 다시 파봤더니 아래로
정정됐다. **MSI2 작업은 이 표를 기준으로 소스를 찾아야 한다:**

| EDU 저장소 파일 | 실제 역할 | 이관 대상 | 상태 |
| --- | --- | --- | --- |
| `10.trunk_depth_scan.py` | 초기/미성숙 버전, 죽은 코드 | - | 이관 안 함 |
| `11.*` 5개 (`trunk_scan_deep_anchor_sweep`, `trunk_scan_fixed_anchor`, `trunk_scan_interactive`, `trunk_scan_link6_outside_sweep`, `view_trunk_pcd`) | 카메라 포즈 튜닝 실험, 죽은 코드. `trunk_map.json`을 안 만들고 ROS2도 안 씀 | - | 이관 안 함 |
| `12.trunk_scan_hidden_gripper.py` | **실제 트렁크 스캔 실행부**. Isaac Sim에서 그리퍼 시야 가리기, 5개 waypoint 스윕, `trunk_pointcloud.npy`+`trunk_pointcloud_meta.json` 생성 | `cart2trunk_simulation` (카메라/스캔 자세 이동 부분만, MSI2 담당) | **미이관, Isaac Sim 필요** |
| `13.export_trunk_map.py` | Isaac Sim 불필요한 순수 후처리. npy+meta → `trunk_map.json` 변환 | `cart2trunk_perception/core/trunk_map_builder.py` | **완료** (아래 5.3절) |
| `14.trunk_scan_with_test_box.py` | 12번 + 정적 테스트 박스 하나 추가한 버전(장애물 검출 검증용) | 이관 불필요(12 완성되면 자연히 커버됨) | 이관 안 함 |
| `7.build_mobile_manipulator.py`, `18~27.lift_stage*.py`(개발 이력, 함수만 추출), `28.cart_to_trunk_pick_place_lift.py`, `29.carry_pose_calibration.py`, `31.collect_mobile_manipulator_lift.py` | **실제 모바일 매니퓰레이터(베이스+리프트+M0609+그리퍼) 실행 기반** | `cart2trunk_platform` + `cart2trunk_motion` | **미이관, MSI2 최우선 작업** |
| `algorism/01~19` (11번은 md 문서, 16번은 결번, 12번은 개인 로컬 경로 하드코딩된 디버그 스크립트라 제외) | 적재 계획 알고리즘. ROS 비의존, pytest 완비 | `cart2trunk_planning/core/` | **완료** (레노버에서 끝냄) |
| `perception/box_top_extractor.py` (2,483줄) | **이미 동작하는 rclpy.Node** (`DepthTopmostBoxExtractor`) — 박스 검출 | `cart2trunk_perception` (core + action server로 재구성) | **미이관** (아래 5.4절 상세) |
| `35~46번` | 카트/트렁크를 테이블/크레이트로 축소한 **통신 흐름 검증용 간이 시나리오** — 메인 코드 아님 | `cart2trunk_test_scenarios` | 이관 안 함 (지금은 자체 제작한 dummy 서버로 그 역할 대체 중, 5.6절) |

핵심 문서 2개도 EDU 저장소에 있으니 참고할 것 (여기 요약된 내용의 원본):
`isaacpjt/Cart2Trunk/TRUNK_MAP_ROS2_HANDOFF.md`, `isaacpjt/Cart2Trunk/HANDOFF.md`.

### 2.3 ⚠️ M0609 자산 — git에 없다, 반드시 수동으로 가져와야 함

EDU 저장소의 `isaacpjt/M0609/`는 **broken git submodule**(mode 160000 gitlink인데
`.gitmodules`가 없음)이라, **RMPflow yaml/URDF/그리퍼 USD 실제 파일이 git 히스토리
어디에도 커밋된 적이 없다.** `git clone`으로는 절대 못 가져온다.

레노버 로컬 디스크에서 발견해서 백업해뒀다 (262MB, 138개 파일):

```text
~/cart2trunk_assets_backup/isaacpjt_M0609_20260723_171230/
```

내용물: `rmpflow/`(m0609_description.yaml, m0609_rmpflow_common.yaml,
m0609_rmpflow_controller.py, m0609_pick_place_controller.py), `doosan-robot2/`(URDF,
멀티 로봇 USD), `onrobot_rg2/`(그리퍼 URDF), `vgp20_gripper/`(그리퍼 USD),
`Collected_m0609_vgp20_camera/`, 번호 붙은 pick-place/카메라 실험 스크립트 10개.

**MSI2가 확인해야 할 것:**
1. MSI2 로컬에 이미 `isaacpjt/M0609/`(또는 비슷한 경로)가 있는지 먼저 확인할 것 —
   Isaac Sim 관련 개발은 GPU가 필요해서 원래 MSI2에서 진행됐을 가능성이 높다.
   MSI2 쪽 사본이 더 최신/완전할 수 있다.
2. 레노버 백업과 MSI2 사본이 다르면 어느 쪽이 최신인지 대조해서 확정할 것.
3. 최종 확정한 자산을 `cart2trunk_simulation/assets/`(작은 설정 파일)와 MSI2 로컬
   `CART2TRUNK_ASSET_ROOT` 환경변수 경로(대용량 USD)로 나눠서 배치 — **새 저장소
   git에는 커밋하지 말 것** (`.gitignore`에 `*.usd`가 이미 제외되어 있음). 레노버↔MSI2
   전송은 git이 아니라 rsync/USB 등으로.

---

## 3. 지금 빌드되는지부터 확인할 것

```bash
cd ~/cart2trunk_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 pkg list | grep cart2trunk   # 12개 패키지 다 나와야 정상
```

pytest 실행 시 주의: 이 환경에는 `anyio`의 pytest 플러그인이 시스템 pytest와
충돌해서 `-p no:anyio`를 꼭 붙여야 한다 (`ModuleNotFoundError: No module named
'_pytest.scope'` 에러가 나면 이거다):

```bash
python3 -m pytest src/cart2trunk_planning/test/ src/cart2trunk_perception/test/test_trunk_map_builder.py \
  src/cart2trunk_coordinator/test/ src/cart2trunk_common/test/test_geometry.py \
  src/cart2trunk_common/test/test_error_codes.py -p no:anyio
# 123 passed 가 나와야 정상 (2026-07-23 기준)
```

### 환경변수 관련 주의사항

레노버 환경에서 `ROS_DOMAIN_ID`/`RMW_IMPLEMENTATION` 값을 확인했는데, **`~/.bashrc`에
적힌 값(`ROS_DOMAIN_ID=141`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, cyclonedds는
주석 처리됨)과 실제 이 작업 세션의 셸에 잡힌 값(`ROS_DOMAIN_ID=140`,
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`)이 서로 다르다** — 어느 게 "진짜" 레노버
설정인지 이 세션만으로는 확정 못 했다. **3PC 실통신 테스트 전에 MSI1/MSI2/레노버
세 PC의 `ROS_DOMAIN_ID`·`RMW_IMPLEMENTATION`을 반드시 직접 확인하고 통일할 것.**
Domain ID는 0~101 범위가 공식 권장 범위(포트 계산 충돌 방지)라는 점도 참고.

---

## 4. 전체 통신 흐름 (지금 실제로 동작하는 것)

```text
MSI2 mission_coordinator_node (RunLoadingMission Action 서버)
  │
  ├─ [1회] /perception/scan_boxes (ScanBoxes)  ── 지금은 dummy 서버만 있음
  │
  └─ [반복, 박스 1개 배치마다] ────────────────────────────┐
       /perception/scan_trunk (ScanTrunk)                  │
              │                                             │
       /planning/compute_load_plan (ComputeLoadPlan)        │
              │                                             │
       /robot/execute_pick_place (ExecutePickPlace) ── 지금은 dummy 서버만 있음
              │                                             │
       (배치 성공) ──────────────────────────────────────────┘
              │
       (남은 박스 없음) → DONE
```

이 전체 루프를 **레노버에서 5개 노드(real ScanTrunk + real ComputeLoadPlan +
real Coordinator + dummy ScanBoxes + dummy ExecutePickPlace)를 동시에 띄워서
`ros2 action send_goal /mission/run_loading_mission ...`으로 실제로 끝까지
돌려봤고, 박스 3개 전부 배치·재스캔 4회까지 로그로 확인 완료했다.** 즉
**Coordinator ↔ Perception(트렁크) ↔ Planning 삼각형은 이미 실전급으로 검증됨.**
MSI2가 할 일은 이 그림에서 dummy로 되어있는 두 군데(`scan_boxes`,
`execute_pick_place`)를 실제 구현으로 갈아끼우는 것과, `scan_trunk`의 replay 모드를
실시간 모드로 바꾸는 것이다.

---

## 5. 패키지별 상세 현황

### 5.1 `cart2trunk_interfaces` — 완료

메시지/액션/서비스 전부 정의 및 빌드 검증됨 (rosidl 빌드 성공, `ros2 interface
list`로 17개 타입 확인 완료). MSI1/MSI2/레노버 전부 **같은 버전**을 써야 한다 —
바뀌면 세 PC 모두 재빌드 필요.

- **msg**: `Box3D`(`detected_pose`/`target_pose` 분리 — 인식된 현재 자세와 적재할
  목표 자세를 명확히 나눔), `BoxArray`, `TrunkMap`, `Placement`, `PickPlaceTask`,
  `LoadPlan`, `MissionState`, `SafetyState`, `NodeHeartbeat`
- **action**: `ScanBoxes`, `ScanTrunk`, `ComputeLoadPlan`, `ExecutePickPlace`,
  `RunLoadingMission`
- **srv**: `ResetSafety`, `ResetMission`, `ValidatePlan`

**아직 안 쓰이는 srv**: `ResetSafety`(safety 패키지 미구현), `ValidatePlan`(누구도
안 부름 — 필요해지면 그때 구현).

### 5.2 `cart2trunk_common` — 완료

- `error_codes.py`: 지금까지 실제로 쓰인 error_code만 상수화 (`INVALID_BOX_DATA`,
  `NO_FEASIBLE_PLACEMENT`, `DEPTH_TIMEOUT`, `MAP_QUALITY_LOW`, `ACTION_TIMEOUT`,
  `ACTION_SERVER_UNAVAILABLE`, `GOAL_REJECTED`). **motion/platform/safety 쪽에서
  새 error_code가 필요하면 여기에 추가할 것** — 원래 설계 문서에는
  `IK_FAILED`/`COLLISION_DETECTED`/`GRASP_FAILED`/`BOX_DROPPED`/`PLACE_FAILED`/
  `ROBOT_TIMEOUT` 같은 실행 계열 코드도 계획되어 있었는데, 아직 그 코드를 쓰는
  곳이 없어서 미리 안 만들어뒀다.
- `geometry.py`: `corner_to_center`/`center_to_corner`(AABB 최소코너↔중심 변환),
  `subtract_offset`/`add_offset`(좌표계 오프셋 가감), `quaternion_from_z_yaw`/
  `z_yaw_from_quaternion`(Z축 yaw↔쿼터니언). **박스는 바닥에 눕혀서만 놓는다는
  전제로 Z축 회전만 지원한다** — roll/pitch 있는 케이스는 지원 안 함. rclpy
  비의존 순수 함수라 아무 패키지에서나 가져다 쓰면 된다.

### 5.3 `cart2trunk_perception` — 트렁크 스캔만 완료, 박스 검출 미착수

**완료된 부분** (`core/trunk_map_builder.py` + `trunk_scan_action_server.py`):

EDU 저장소 `13.export_trunk_map.py`(Isaac Sim 불필요, numpy+open3d+scipy만 씀)를
그대로 포팅. 레노버 로컬에만 있던 실제 스캔 데이터(`isaacpjt/Cart2Trunk/results/
run_20260720_160153/`, **git에 없음, 64MB**)로 이중 검증했다:
1. 포팅한 코드와 원본 스크립트를 같은 입력으로 나란히 재실행 → obstacles/faces/
   edges/source_stats **완전 일치**. ceiling_z만 원본 알고리즘 자체의 RANSAC 평면
   검출 non-determinism으로 수~십수mm 차이 (포팅 버그 아님 — 원본을 두 번 돌려도
   그 정도 차이 남).
2. 커밋용 합성 포인트클라우드 pytest 10개 (occlusion을 반영해야 bump 검출이
   맞다는 걸 테스트 작성 중 직접 발견 — 실제 depth 카메라는 덩어리 아래 바닥을
   못 본다는 걸 반영 안 하면 테스트가 거짓으로 통과함).

`trunk_scan_action_server.py`는 **`fixture_run_dir` 파라미터 기반 replay 모드만
구현**되어 있다. `ScanTrunk.action`의 원래 의도(capture_duration 동안 여러
waypoint를 돌면서 Depth를 실시간 누적)는 MSI2의 카메라/Isaac Sim 없이는 이
레노버 혼자 구현/검증할 수 없어서 **의도적으로 범위를 제한**해뒀다 (파일
docstring에도 명시). MSI2가 할 일: `depth_callback` + 다중 waypoint 동기화 로직을
추가해서 진짜 실시간 누적 모드를 만드는 것. `EDU 저장소의 12.trunk_scan_hidden_gripper.py`가
Isaac Sim에서 하던 역할(5개 waypoint 스윕: `deep_center/left/right/floor/ceiling`)을
`cart2trunk_simulation`(waypoint로 이동시키는 쪽, MSI2)과 이 액션 서버(그 구간
Depth를 누적하는 쪽)로 나눠서 짜야 한다 — **한 스크립트가 로봇 이동과 인식을
전부 하지 않게** 역할을 분리하는 게 원래 설계 원칙이다.

**미착수**: 박스 검출 (`box_top_extractor.py`, 2,483줄, `DepthTopmostBoxExtractor`
클래스). 구조 분석은 끝냈다:

- **이미 실제로 동작하는 rclpy.Node다** — 새로 짜는 게 아니라 뜯어서 재구성하는
  작업.
- 순수 알고리즘(카메라 intrinsics만 파라미터로 받으면 numpy/open3d/cv2만으로
  동작, `self.` ROS 상태 의존 없음)과 ROS 글루가 이미 상당히 잘 나뉘어 있다:
  - **순수 이식 가능** (`__init__` 이후 line 734~2170 구간 대부분, 약 1,400줄):
    `preprocess_cloud`, `detect_box_top_candidates`(RANSAC 평면 반복 + DBSCAN
    클러스터링), `make_candidate`, `select_support_candidate`,
    `detect_floor_boundary`, `compute_box_corners`, `generate_completed_box_surface`,
    `sample_quad_surface`, `get_down_direction`/`order_rectangle_corners`/
    `make_plane_basis`(전부 static) → `cart2trunk_perception/core/box_detector.py`
    같은 파일로 옮기면 됨.
  - **ROS 글루** (그대로 두거나 얇게만 남겨야 함): `__init__`(구독/발행/QoS
    설정), `camera_info_callback`, `depth_callback`(카메라 콜백 오케스트레이션 +
    `cv2.imshow`/`waitKey` — **실제 디스플레이가 있어야 동작함, headless 아님,
    Isaac Sim에서 돌릴 때도 유의**), `convert_depth_message`(cv_bridge),
    `publish_*` 4종.
- 출력 스키마도 이미 확정되어 있음(`save_current_cloud`가 만드는
  `all_boxes_corners_*.json`): `box_id`, `corner_order`, `corners_m`(8×3),
  `support_type`, `completed_point_count` 등. → `Box3D.msg`의 `corners`(Point[8])
  필드에 그대로 대응시키면 됨.
- **로컬 fixture 없음** — 트렁크 쪽과 달리 레노버 디스크에 실제 박스 스캔
  결과물이 없어서, MSI2에서 이 부분을 검증하려면 (a) 실제 카메라/Isaac Sim으로
  새로 캡처하거나 (b) 합성 point cloud로 pytest를 짜야 한다.
- `requirements.txt`가 이미 있음: `numpy==1.26.4`, `opencv-python==4.11.0.86`,
  `open3d==0.19.0` (레노버는 open3d 0.19.0/scipy 1.15.3/numpy 1.24.4/opencv
  4.13.0로 실제 동작 확인함 — 정확히 이 버전이 아니어도 됨, ABI 호환만 맞으면 됨.
  cv_bridge/rclpy와 맞물려서 numpy 2.x는 피할 것 — 원본 코드 주석에 명시된 이유).
- `base_to_camera_transform.json`(카메라-베이스 고정 변환)이 스크립트와 같은
  폴더에 있어야 동작하는데, 이 값 자체가 특정 로봇 자세 전제로 측정된 값이라 —
  실기체/새 시뮬레이션 자세가 다르면 재측정 필요.

### 5.4 `cart2trunk_planning` — 완료 (레노버 담당, 재작업 불필요)

`algorism/01~19` 16개 모듈 전부 포팅, pytest 86개 + 자체 회귀 테스트 스크립트
5항목 전부 통과. `ComputeLoadPlan` 액션 서버가 EDU 저장소의
`trunk_map_planner_node.py`가 실제로 쓰던 `09_rescan_replan.replan_after_rescan()`을
그대로 재사용한다. **레노버가 계속 담당**하는 패키지라 MSI2가 손댈 필요는
기본적으로 없다 — 단, `TrunkMap.occupied_boxes`/`inner_size` 좌표계 전제(아래
6.3절)가 바뀌면 여기 `occupied_box3d_to_placed_box` 변환 로직도 같이 바뀌어야
하니 연동 지점만 알아둘 것.

### 5.5 `cart2trunk_coordinator` — 완료

`mission_state_machine.py`(순수 Python, pytest 13개) + `mission_coordinator_node.py`.
자기 자신이 Action 서버(`RunLoadingMission`)이면서 동시에 4개 Action의 Client라서
`MultiThreadedExecutor` + `ReentrantCallbackGroup`이 필수 — **일반 `rclpy.spin()` +
싱글 콜백그룹으로 바꾸면 콜백 안에서 다른 Action 응답을 기다리다가 데드락 난다.**

상태 머신: `IDLE → BOX_SCAN → (TRUNK_SCAN → PLAN_REQUEST → EXECUTE_TASK)* → DONE`,
`ESTOP`은 어느 상태에서든 최우선 전이, `CANCEL`은 진행 중 상태에서만,
`RESET`(`/mission/reset` 서비스)은 터미널 상태(DONE/CANCELING/SYSTEM_ERROR/ESTOP)에서만
IDLE로 복귀 가능. 전이표는 `mission_state_machine.py` 안의 `_TRANSITIONS` dict가
전부다 — 상태 추가/변경 시 여기부터 고칠 것.

카트 박스 목록은 **미션당 1회만 스캔**하고, 트렁크는 **박스 1개 배치할 때마다
재스캔**한다 (`EXECUTE_TASK`+`TASK_SUCCEEDED` → 무조건 `TRUNK_SCAN`으로 감, 남은
박스가 0개여도 마지막에 한 번 더 재스캔해서 빈 계획을 받아야 `DONE`으로 감 — 이건
의도된 설계다, 6.1절 참고).

### 5.6 `cart2trunk_test_scenarios` — dummy 서버만 있음

`dummy_scan_boxes_server.py`(고정 3박스 Large/Medium/Small 반환), 
`dummy_execute_pick_place_server.py`(뭘 받든 성공 처리, phase feedback만 흉내).
**Coordinator 오케스트레이션 로직을 실제 하드웨어 없이 검증하기 위한 임시
스텁이다** — 진짜 `ScanBoxes`/`ExecutePickPlace` 서버가 완성되면 이 두 dummy는
그대로 두되(회귀 테스트용으로 남겨도 됨), MSI2 launch에서는 진짜 서버를 띄우고
이 dummy들은 안 띄우면 된다.

원래 이 패키지의 목적은 EDU 저장소 35~46번(축소형 테이블+크레이트 통합 테스트
시나리오)의 후신인데, 그건 아직 이관 안 함 — 지금은 dummy 서버가 그 역할을 임시로
대신하고 있다.

### 5.7 `cart2trunk_platform` / `cart2trunk_motion` / `cart2trunk_simulation` — 빈 껍데기, MSI2 최우선 작업

`ros2 pkg create`로 만든 골격(`package.xml`/`setup.py`/`resource/`)만 있고 실제
코드 0줄. 원래 계획한 내부 구조(README에도 적어둠):

```text
cart2trunk_simulation/          # Isaac Sim Scene/Asset/센서 브리지
├── scene_manager.py            #   차량+카트+M0609 Scene 구성, SDF Collision
├── sensor_bridge.py            #   Depth/CameraInfo 발행 (isaacsim.ros2.bridge)
└── config/{scene,prim_paths,assets}.yaml

cart2trunk_platform/            # 하드웨어 제어 계층 (Isaac Sim/실기체 어댑터 분리)
├── mobile_base_controller.py   #   옴니휠 베이스 (Nova Carter 아님! 일반화된 이름으로)
├── lift_controller.py          #   승강 리프트
├── m0609_controller.py         #   RMPflow 래핑 (M0609 자산 backup의 rmpflow/ 참고)
├── gripper_controller.py       #   흡착 그리퍼 (EDU 36.py의 DynamicSuctionGripper 참고)
├── camera_controller.py        #   RealSense
└── adapters/{isaac_adapter,real_robot_adapter}.py

cart2trunk_motion/               # Platform 컨트롤러 조합 -> 작업 단위 Action Server
├── scan_motion_action_server.py         # 스캔 자세 이동 (perception과 협업)
├── pick_from_cart_action_server.py
├── transport_action_server.py
├── place_in_trunk_action_server.py
├── execute_pick_place_action_server.py  # ExecutePickPlace.action 서버 (지금 dummy로 대체됨)
└── sequences/, motion_primitives/
```

이 세 패키지는 **Isaac Sim/실기체가 있어야 개발·검증이 가능해서 레노버에서는
스캐폴딩만 만들고 손을 못 댔다.** MSI2가 실질적으로 처음 코드를 채우는 곳이다.

### 5.8 `cart2trunk_safety` / `cart2trunk_hmi` — 빈 껍데기, 하드웨어 불필요 (원하면 MSI2가 안 해도 됨)

이 둘은 실제 로봇/카메라 없이도 개발 가능한 패키지다 (레노버에서 다음 작업으로
이어가려다 이 인수인계 작업으로 넘어옴). MSI2가 platform/motion/simulation에
집중하는 동안 레노버나 다른 담당자가 병행해도 되는 영역.

- `cart2trunk_safety`: Emergency Stop / Watchdog. `SafetyState.msg`,
  `ResetSafety.srv`는 이미 정의되어 있음.
- `cart2trunk_hmi`: Flask/WebSocket 백엔드. 방금 전 논의에서 **"UI보다 백엔드
  뼈대 먼저"로 결론** — `/mission/state`가 이미 실제 데이터를 내보내고 있으니
  UI 없이 curl/websocket 클라이언트로 바로 검증 가능하다는 게 이유였다.

### 5.9 `cart2trunk_bringup` — launch 스텁, 부분적으로만 실제 연결됨

- `lenovo_planning.launch.py`: **실제로 `loading_planner_action_server` 연결됨**
- `msi1_perception.launch.py`, `msi2_sim_execution.launch.py`: 아직 빈
  `LaunchDescription([])` — 각 파일 docstring에 어떤 노드가 들어가야 할지
  주석으로 적어뒀지만 실제 `Node(...)` 엔트리는 없음. **MSI2가 자기 노드들을
  실제로 만들면 `msi2_sim_execution.launch.py`에 등록해야 함.**
- `full_system.launch.py`: 위 3개를 `IncludeLaunchDescription`으로 묶기만 함
  (개발/시뮬레이션 검증용, 단일 PC에서 세 역할 다 띄울 때).

---

## 6. 반드시 지켜야 하는 설계 결정과 컨벤션

여기 나온 걸 어기면 이미 실전 검증된 perception↔planning↔coordinator 연동이
조용히 깨진다. 코드 수정 전에 꼭 읽을 것.

### 6.1 PER_PLACEMENT 재스캔 정책

EDU 저장소 `TRUNK_MAP_ROS2_HANDOFF.md`에서 이미 확정된 정책: **박스를 하나
놓을 때마다 트렁크 전체를 처음부터 다시 스캔·재계획한다.** "초기에 한 번 스캔"이
아니다. 이유: 실제 적재 위치가 계획과 약간 어긋날 수 있고, 박스가 놓이면서 Yaw나
위치가 변할 수 있고, 인식 오차가 여러 단계 누적되는 걸 막기 위함. 코디네이터가
레노버의 전체 후보 계획(`plan.tasks[]`) 중 **첫 번째만 실행**하고 다시
트렁크스캔부터 시작하는 것도 같은 이유 — 전체 계획을 한 번에 다 실행하지 않는다.
`mission_state_machine.py`의 `EXECUTE_TASK + TASK_SUCCEEDED → TRUNK_SCAN`
전이가 이걸 강제한다. 카트 박스 목록(`ScanBoxes`)만 예외 — 카트는 재배치 중에
안 움직이므로 미션당 1회만 스캔.

### 6.2 좌표계 컨벤션 (제일 실수하기 쉬운 부분)

```text
world
└── m0609_base (로봇 팔 원점)
    ├── camera_link → camera_depth_optical_frame   (원본 Depth)
    ├── table_frame                                  (카트)
    └── trunk_frame                                  (트렁크 내부)
```

| 데이터 | 기준 Frame |
| --- | --- |
| 원본 Depth | `camera_depth_optical_frame` |
| 박스 검출 결과 (`Box3D.detected_pose`) | `m0609_base` |
| `TrunkMap.trunk_pose` | `m0609_base` (트렁크 로컬 원점 = 바닥 최소 코너의 base 프레임 좌표) |
| `TrunkMap.occupied_boxes[].detected_pose` | **trunk_pose를 뺀 trunk_frame 로컬 좌표** (base 프레임 아님!) |
| `PickPlaceTask.target_pose` (레노버 출력) | trunk_frame 기준 — MSI2가 m0609_base로 변환해서 실제 로봇 경로 생성 |
| 실제 로봇 실행 Pose | `m0609_base` |

**AABB 코너/중심 컨벤션**: 알고리즘 내부(`cart2trunk_planning/core`)는 박스를
"최소 코너 (x,y,z) + width/depth/height"로 표현한다 (`PlacedBox.x/y/z` = 최소
코너, `x_range = (x, x+width)`). 반면 ROS 메시지의 `Pose.position`은 관례상 **중심점**이다.
그래서 코너↔중심 변환이 최소 두 군데서 반드시 필요하고, 지금 `cart2trunk_common.geometry`의
`corner_to_center`/`center_to_corner`로 통일해뒀다:

- `trunk_scan_action_server.trunk_map_dict_to_msg()`: obstacle AABB(코너 두 개) → 중심 →
  trunk_pose 오프셋 뺌 → `Box3D.detected_pose`(trunk_frame 로컬 중심)
- `loading_planner_action_server.occupied_box3d_to_placed_box()`: `Box3D.detected_pose`(중심) →
  최소 코너로 역변환해서 core의 `PlacedBox`에 넣음
- `loading_planner_action_server.plan_to_task()`: core의 `PlacementPlan.position`(코너) →
  중심으로 변환해서 `PickPlaceTask.target_pose.position`에 넣음

**새로 만드는 박스 검출(`ScanBoxes`)이나 motion 쪽 코드도 이 컨벤션을 따를 것** —
안 그러면 좌표가 반 박스 크기만큼 어긋나는 미묘한 버그가 생긴다 (실제로 이런
버그를 잡은 적은 없지만, 변환 지점이 이미 3군데나 있다는 게 이런 실수가 나기
쉽다는 신호다).

### 6.3 `TrunkMap`이 "이미 처리된 데이터"라는 전제

`cart2trunk_planning`(레노버)은 `TrunkMap.inner_size`/`occupied_boxes`가 **이미
trunk_frame 로컬 좌표로 변환되어 있다고 가정**하고 짜여 있다. EDU 저장소
`trunk_space_state.py`의 원본 `TrunkWorldMap.to_bounding_trunk()`(원시 depth
스캔 vertex 점군에서 AABB를 뽑는 로직, RANSAC으로 entrance 방향까지 추정)는
**아직 이 새 시스템 어디에도 포팅되지 않았다** — 지금 `trunk_scan_action_server.py`는
이미 AABB로 정리된 `trunk_map.json`(13번 스크립트 출력)을 다루기 때문에 이 변환이
필요 없었다. **MSI2가 실시간 스캔 모드를 구현할 때 이 원시→AABB 변환 로직이 다시
필요해질 가능성이 높다** — 그때 perception 쪽에 넣을지, 원본처럼 처리하고
publish만 새 메시지로 할지 결정할 것. (원본 함수는 EDU 저장소
`isaacpjt/Cart2Trunk/algorism/02_trunk_space_state.py`의 `TrunkWorldMap`,
`to_bounding_trunk()`, `load_trunk_from_world_map()`, `load_obstacles_from_world_map()`.)

### 6.4 Box3D의 `detected_pose` vs `target_pose`

애초에 GPT 설계 검토 단계에서 "박스 초기 자세와 적재 후 자세를 분리해야 한다"고
정정한 부분이다. `detected_pose` = 인식된 현재 자세, `target_pose` = 적재할 최종
자세. **`ScanBoxes`가 반환하는 카트 위 박스들은 `target_pose`를 안 채운다** —
카트에서의 자세는 적재 계획과 무관하기 때문 (EDU 저장소 `01_object3d_schema.py`의
`object3d_to_box()`가 위치를 아예 버리는 것과 같은 이유 — 크기/id만 중요, 카트
위 위치는 계산에 안 씀). `target_pose`는 오직 `PickPlaceTask.target_pose`
쪽에서만 의미 있게 채워진다.

### 6.5 rclpy 함정 두 가지 (실제로 밟았던 버그)

1. **`rclpy.task.Future.result()`는 블로킹하지 않는다.** 끝났든 안 끝났든 즉시
   `self._result`(기본값 `None`)를 반환한다. `send_goal_async()` 직후
   `.result()`를 부르면 아직 안 끝난 상태 그대로 `None`이 넘어온다.
   **`add_done_callback` + `threading.Event`로 실제로 기다려야 한다** —
   `mission_coordinator_node.py`의 `_wait_for_future()`가 그 패턴이다. 여러
   Action을 순차 호출하는 노드를 새로 짤 때 이 패턴을 그대로 재사용할 것.
2. **Action 서버가 자기 자신도 Action/Service Client일 때는 `MultiThreadedExecutor`
   + `ReentrantCallbackGroup` 필수.** 기본 `rclpy.spin()` + 콜백그룹 없음으로
   짜면, 콜백 안에서 다른 Action의 응답을 기다리는 동안 정작 그 응답을 처리할
   콜백이 실행될 스레드가 없어서 데드락 난다. `cart2trunk_motion`의
   `execute_pick_place_action_server.py`가 만약 내부적으로 platform의 다른
   서비스/액션을 호출한다면 똑같은 문제를 겪을 것.
3. **float32 필드에 Python `int`를 대입하면 rclpy가 `AssertionError`를 던진다.**
   `sum(빈 리스트)`는 `int 0`을 반환하지, `float 0.0`이 아니다 — `loading_planner_action_server.py`에서
   실제로 이걸로 깨졌었다(`float(sum(...))`로 고침). 메시지 필드에 산술 연산
   결과를 넣을 때는 타입을 항상 의심할 것.

---

## 7. 검증 방법론 (지금까지 이렇게 확인해왔다)

새 코드를 짤 때 이 패턴을 따르는 걸 권장한다 (레노버 세션 내내 이렇게 진행함):

1. **EDU 저장소 원본 코드를 먼저 읽고 정확히 뭘 하는지 파악한다** — 짐작하지
   않는다. `git show origin/algorism:<경로>`로 브랜치 체크아웃 없이 내용만
   읽을 수 있다.
2. **가능하면 실제 데이터로 검증한다.** 로컬에 진짜 캡처 데이터가 있으면
   (`find` 등으로 찾아볼 것 — git에 없어도 디스크에 남아있는 경우가 실제로
   있었다) 포팅한 코드와 원본을 나란히 돌려서 출력을 비교한다.
3. **합성 데이터 pytest도 같이 만든다** (커밋용 회귀 테스트, 실제 fixture는
   보통 너무 커서 git에 못 넣음).
4. **노드로 감싼 뒤에는 실제로 띄워서 `ros2 action send_goal`/`ros2 service
   call`로 진짜 호출해본다.** 여러 노드가 얽히면 전부 동시에 띄우고 전체
   플로우를 끝까지 돌려본다 — 이 인수인계서에 적힌 실제 버그 2개(Future.result(),
   int/float)는 전부 이 단계에서 발견됐다. 유닛 테스트만으로는 못 잡았다.
5. **찾은 버그는 회귀 테스트로 남긴다.**

---

## 8. MSI2가 지금 당장 해야 할 일 (우선순위)

1. **저장소 clone + 빌드 확인** (3절)
2. **M0609 자산 위치 확정** — MSI2 로컬 사본과 레노버 백업(`~/cart2trunk_assets_backup/isaacpjt_M0609_20260723_171230/`)
   대조, 최종본을 `cart2trunk_simulation`/에셋 루트로 배치 (2.3절)
3. **`cart2trunk_simulation`**: Isaac Sim Scene 로드 + 카메라/Depth 브리지
   (`isaacsim.ros2.bridge`) — EDU 저장소 `7.build_mobile_manipulator.py`,
   `12.trunk_scan_hidden_gripper.py`의 Scene 구성 부분에서 발췌
4. **`cart2trunk_platform`**: 옴니휠 베이스/리프트/M0609/그리퍼/카메라 컨트롤러 —
   M0609 자산의 `rmpflow/m0609_rmpflow_controller.py`, `m0609_pick_place_controller.py`와
   EDU `36.crate_pick_to_place.py`의 `DynamicSuctionGripper` 참고 (**Nova Carter
   전용 이름/구조는 절대 그대로 가져오지 말 것** — 1절 참고)
5. **`cart2trunk_motion`**: `ExecutePickPlace.action` 서버 실제 구현 — 지금
   dummy가 하는 역할(APPROACH_PICK→...→RETREAT phase feedback)을 실제
   platform 컨트롤러 조합으로 교체. 완성되면 `cart2trunk_test_scenarios`의
   dummy 대신 이걸 띄우고 5노드 통합 테스트를 다시 돌려서 여전히 성공하는지
   확인할 것 (7절 방법론 그대로)
6. **`cart2trunk_perception` 실시간 트렁크 스캔** — `trunk_scan_action_server.py`에
   실제 depth 누적 모드 추가 (5.3절)
7. **`cart2trunk_perception` 박스 검출** — `box_top_extractor.py` 포팅 (5.3절
   상세 구조 분석 참고). MSI1이 따로 생기기 전까지는 이것도 MSI2가 개발/검증할
   수밖에 없을 것으로 보임
8. **`msi2_sim_execution.launch.py`에 실제 노드 등록**
9. 여유 있으면: `cart2trunk_safety`, `cart2trunk_hmi` (하드웨어 불필요, 5.8절)

각 단계 끝날 때마다 7절 방법론대로 실제로 띄워서 검증하고, 검증 로그·발견한
버그를 커밋 메시지에 상세히 남길 것 — 이 문서도 그런 커밋 메시지들을 바탕으로
썼다.
