# Cart2Trunk ROS 2 Workspace

3대 PC(MSI1 / MSI2 / Lenovo)에 배포되는 Cart2Trunk 적재 시스템의 통합 ROS 2 워크스페이스입니다.
검증된 프로토타입 코드의 출처는 [`2026_ROKEY_Cooperation3_EDU`](https://github.com/jiwan1230/2026_ROKEY_Cooperation3_EDU) 저장소(`algorism` 브랜치)입니다.

## PC 역할

| PC | 역할 | 실행 Launch |
| --- | --- | --- |
| MSI2 | 중앙 제어 · Isaac Sim · Pick & Place 실행 | `cart2trunk_bringup/launch/msi2_sim_execution.launch.py` |
| MSI1 | Depth 기반 박스/트렁크 인식 | `cart2trunk_bringup/launch/msi1_perception.launch.py` |
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
- `cart2trunk_perception` — Depth → Box/Trunk 인식 (MSI1)
- `cart2trunk_planning` — 적재 계획 알고리즘 (Lenovo)
- `cart2trunk_execution` — Pick & Place 실행 (MSI2)
- `cart2trunk_coordinator` — 전체 상태 머신 (MSI2)
- `cart2trunk_safety` — Emergency Stop / Watchdog (MSI2)
- `cart2trunk_sim` — Isaac Sim Scene/카메라 브리지 (MSI2)
- `cart2trunk_hmi` — Flask/WebSocket 백엔드
- `cart2trunk_bringup` — Host별 launch 파일
