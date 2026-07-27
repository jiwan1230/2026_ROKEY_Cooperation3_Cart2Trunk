"""실제 검출된 박스 + 실제 적재 계획(ComputeLoadPlan)으로 /robot/execute_pick_place를
실제로 호출하는 수동 트리거 - 예전 버전은 ScanBoxes 직후 박스 하나를 임의로 골라
target_pose를 하드코딩(0.3, 0.2, 0.1)해서 곧장 ExecutePickPlace를 불렀다(GRIP 도달
여부만 확인하는 용도) - 사용자 지적대로 "박스 스캔 -> 뭐부터 옮길지 결정" 단계
(ComputeLoadPlan)가 통째로 빠져 있었다. loading_planner_action_server.py가 이미
EDU 저장소 알고리즘(core/rescan_replan.replan_after_rescan)에 연결된 실제 서버라
(더미 아님), 지금은 정식 파이프라인 그대로 4단계를 다 부른다:
    ScanBoxes -> ScanTrunk -> ComputeLoadPlan -> (계획된 순서대로) ExecutePickPlace 반복

실행 순서(각각 별도 터미널, 1번부터 순서대로 - 2/3/4/5/6번은 fresh shell에서
LD_LIBRARY_PATH가 비어있는지(`echo $LD_LIBRARY_PATH`) 먼저 확인할 것 - 1번의
export가 섞여 들어오면 시스템 ROS2가 Isaac 번들 라이브러리와 충돌해서 못 뜬다):
    1) platform_controller_node.py(GUI로, HEADLESS=0) - 카트 스캔 -> 시작 위치로
       복귀 -> 그대로 서비스 계속:
        export LD_LIBRARY_PATH=/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/\
release/exts/isaacsim.ros2.bridge/humble/lib:$LD_LIBRARY_PATH
        export HEADLESS=0
        CART2TRUNK_ENABLE_CAMERA_BRIDGE=1 CART2TRUNK_CAPTURE_THEN_SERVE=1 isaac_python \
            src/cart2trunk_simulation/cart2trunk_simulation/tools/capture_cart_scan.py
    2) box_scan_action_server(위 스캔이 저장한 fixture로 - 검출 정확도는 아래
       튜닝값에 크게 좌우된다, EDU 저장소 perception/run_scan_batch.py 검증값과 동일):
        source /opt/ros/humble/setup.bash && source install/setup.bash
        CART2TRUNK_MAX_SUPPORT_AREA_RATIO=13 \
        CART2TRUNK_MIN_PLAUSIBLE_BOX_FOOTPRINT_SIDE_M=0.035 \
        CART2TRUNK_MAX_PLAUSIBLE_BOX_FOOTPRINT_SIDE_M=0.5 \
        CART2TRUNK_MAX_PLANES=24 \
        CART2TRUNK_RELAXED_MAX_PLANES=60 \
        CART2TRUNK_UP_FACING_NORMAL_DOT_MIN=0.78 \
        CART2TRUNK_DETECTION_TRIALS=150 \
        CART2TRUNK_DETECTION_MIN_APPEARANCE_FRACTION=0.05 \
        CART2TRUNK_DEDUP_OVERLAP_Z_TOLERANCE_M=0.03 \
        CART2TRUNK_STACKED_HEIGHT_PLAUSIBILITY_CEILING_M=0.125 \
        CART2TRUNK_FOOTPRINT_AREA_RATIO_DUPLICATE_MAX=1.8 \
        ros2 run cart2trunk_perception box_scan_action_server --ros-args \
            -p fixture_input_path:=/tmp/cart2trunk_cart_scan/merged_cart_scan.npy
    3) trunk_scan_action_server(fixture replay - 이 씬의 실제 차량을 아직 스캔하는
       코드가 없어서, 이전에 실제로 캡처된 트렁크 run 하나를 그대로 재생한다 -
       아래 [트렁크 좌표 주의] 참고):
        source /opt/ros/humble/setup.bash && source install/setup.bash
        ros2 run cart2trunk_perception trunk_scan_action_server --ros-args \
            -p fixture_run_dir:=/home/rokey/cobot3_ws/isaacpjt/Cart2Trunk/results/run_20260720_160153
    4) loading_planner_action_server(실제 EDU 알고리즘 - 더미 아님):
        source /opt/ros/humble/setup.bash && source install/setup.bash
        ros2 run cart2trunk_planning loading_planner_action_server
    5) execute_pick_place_action_server:
        source /opt/ros/humble/setup.bash && source install/setup.bash
        ros2 run cart2trunk_motion execute_pick_place_action_server
    6) 이 스크립트:
        source /opt/ros/humble/setup.bash && source install/setup.bash
        ros2 run cart2trunk_test_scenarios trigger_real_pick_place

[좌표계 - source_box] detected_pose는 world 좌표다(header.frame_id="world") -
box_scan_action_server.py가 capture_cart_scan.py의 "스캔 시점 anchor"(merged_cart_
scan_anchor.json)로 즉시 변환해서 반환하므로, 1번 단계에서 chassis가 스캔 후 시작
위치로 복귀하거나 서버가 계속 서비스하는 동안 다른 이유로 움직여도 이 좌표는
계속 유효하다.

[트렁크 좌표 주의] 3번의 fixture_run_dir은 이번 세션의 카트/로봇 스캔과 무관한,
과거에 실제로 캡처된 별도 세션의 트렁크 pointcloud다 - ComputeLoadPlan이 반환하는
task.target_pose(트렁크 로컬)와 trunk_map.trunk_pose(base-frame 오프셋) 자체는
진짜 알고리즘 결과라 유효하지만, "trunk_pose가 가리키는 실제 위치"는 지금 이
Isaac 씬의 VEHICLE_SCENE_POSITION((5,0,0))과 물리적으로 안 맞을 수 있다(이 씬의
차량을 실제로 스캔하는 코드가 아직 없다 - 다음 개선 대상). 그래서 TRANSPORT/
APPROACH_PLACE 단계가 엉뚱한 곳으로 향하거나 실패할 수 있다 - 이 스크립트로
지금 검증하려는 것은 (1) ComputeLoadPlan이 실제로 그럴듯한 계획(순서/좌표)을
만드는지, (2) 그 계획을 따라 PICK(APPROACH_CART~GRIP~LIFT)까지 실제로 도달하는지
두 가지고, PLACE 이후는 트렁크 실스캔이 붙기 전까지는 참고용이다.
"""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from cart2trunk_interfaces.action import ComputeLoadPlan, ExecutePickPlace, ScanBoxes, ScanTrunk
from cart2trunk_interfaces.msg import BoxArray

SCAN_BOXES_ACTION = "/perception/scan_boxes"
SCAN_TRUNK_ACTION = "/perception/scan_trunk"
COMPUTE_LOAD_PLAN_ACTION = "/planning/compute_load_plan"
PICK_ACTION = "/robot/execute_pick_place"


def _send_and_wait(node, client, goal, action_name, goal_timeout=15.0, result_timeout=60.0):
    """ActionClient goal 전송 + 결과 대기를 한 번에 - 4개 액션 호출 전부 같은
    패턴이라 공용화한다. 실패 시 (None, 에러메시지)를 반환한다."""
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future, timeout_sec=goal_timeout)
    goal_handle = future.result()
    if goal_handle is None or not goal_handle.accepted:
        return None, f'{action_name} goal 거부됨(또는 응답 없음)'
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=result_timeout)
    if not result_future.done():
        return None, f'{action_name} 응답 시간 초과({result_timeout}s)'
    return result_future.result().result, None


def main():
    rclpy.init()
    node = Node("trigger_real_pick_place")

    scan_boxes_client = ActionClient(node, ScanBoxes, SCAN_BOXES_ACTION)
    scan_trunk_client = ActionClient(node, ScanTrunk, SCAN_TRUNK_ACTION)
    load_plan_client = ActionClient(node, ComputeLoadPlan, COMPUTE_LOAD_PLAN_ACTION)
    pick_client = ActionClient(node, ExecutePickPlace, PICK_ACTION)

    for label, client in (
        (SCAN_BOXES_ACTION, scan_boxes_client), (SCAN_TRUNK_ACTION, scan_trunk_client),
        (COMPUTE_LOAD_PLAN_ACTION, load_plan_client), (PICK_ACTION, pick_client),
    ):
        print(f"[TRIGGER] {label} 서버 대기...", flush=True)
        if not client.wait_for_server(timeout_sec=10.0):
            print(f"[TRIGGER-FAIL] {label} 서버 없음", flush=True)
            return

    # ---- 1) ScanBoxes ----
    print(f"[TRIGGER] {SCAN_BOXES_ACTION} 호출...", flush=True)
    scan_result, err = _send_and_wait(node, scan_boxes_client, ScanBoxes.Goal(), SCAN_BOXES_ACTION)
    if err:
        print(f"[TRIGGER-FAIL] {err}", flush=True)
        return
    print(f"[TRIGGER] scan_boxes 결과 success={scan_result.success} "
          f"박스 수={len(scan_result.boxes)} quality={scan_result.quality_score:.3f}", flush=True)
    for b in scan_result.boxes:
        p = b.detected_pose.position
        print(f"  box_id={b.box_id} pos=({p.x:.3f},{p.y:.3f},{p.z:.3f}) "
              f"size=({b.size.x:.3f},{b.size.y:.3f},{b.size.z:.3f}) confidence={b.confidence:.3f}",
              flush=True)
    if not scan_result.success or not scan_result.boxes:
        print("[TRIGGER-FAIL] 검출된 박스 없음 - 중단", flush=True)
        return
    boxes_by_id = {b.box_id: b for b in scan_result.boxes}

    # ---- 2) ScanTrunk (fixture replay - 모듈 docstring [트렁크 좌표 주의] 참고) ----
    print(f"[TRIGGER] {SCAN_TRUNK_ACTION} 호출...", flush=True)
    trunk_result, err = _send_and_wait(node, scan_trunk_client, ScanTrunk.Goal(), SCAN_TRUNK_ACTION)
    if err:
        print(f"[TRIGGER-FAIL] {err}", flush=True)
        return
    if not trunk_result.success:
        print(f"[TRIGGER-FAIL] scan_trunk 실패: {trunk_result.message}", flush=True)
        return
    tp = trunk_result.trunk_map.trunk_pose.position
    sz = trunk_result.trunk_map.inner_size
    print(f"[TRIGGER] scan_trunk 결과 map_id={trunk_result.trunk_map.map_id} "
          f"trunk_pose=({tp.x:.3f},{tp.y:.3f},{tp.z:.3f}) "
          f"inner_size=({sz.x:.3f},{sz.y:.3f},{sz.z:.3f}) "
          f"occupied={len(trunk_result.trunk_map.occupied_boxes)}개", flush=True)

    # ---- 3) ComputeLoadPlan ----
    box_array = BoxArray()
    box_array.snapshot_id = scan_result.snapshot_id
    box_array.boxes = scan_result.boxes
    box_array.overall_quality = scan_result.quality_score
    plan_goal = ComputeLoadPlan.Goal()
    plan_goal.request_id = "trigger-real-pick-1"
    plan_goal.boxes = box_array
    plan_goal.trunk_map = trunk_result.trunk_map
    plan_goal.return_full_plan = True

    print(f"[TRIGGER] {COMPUTE_LOAD_PLAN_ACTION} 호출...", flush=True)
    plan_result, err = _send_and_wait(node, load_plan_client, plan_goal, COMPUTE_LOAD_PLAN_ACTION)
    if err:
        print(f"[TRIGGER-FAIL] {err}", flush=True)
        return
    if not plan_result.success or not plan_result.tasks:
        print(f"[TRIGGER-FAIL] compute_load_plan 실패/빈 계획: "
              f"{plan_result.error_code} {plan_result.message}", flush=True)
        return
    tasks = sorted(plan_result.tasks, key=lambda t: t.sequence)
    print(f"[TRIGGER] compute_load_plan 결과: {plan_result.message} "
          f"(plan_id={plan_result.plan_id} total_score={plan_result.total_score:.3f})", flush=True)
    for t in tasks:
        p = t.target_pose.position
        print(f"  순서={t.sequence} box_id={t.box_id} target_pose(트렁크 로컬)="
              f"({p.x:.3f},{p.y:.3f},{p.z:.3f}) score={t.placement_score:.3f}", flush=True)

    # ---- 4) 계획된 순서대로 ExecutePickPlace 반복 ----
    def feedback_cb(feedback_msg):
        fb = feedback_msg.feedback
        print(f"    [FEEDBACK] phase={fb.phase} progress={fb.progress:.2f}", flush=True)

    for t in tasks:
        source_box = boxes_by_id.get(t.box_id)
        if source_box is None:
            print(f"[TRIGGER-경고] task box_id={t.box_id}에 매칭되는 scan 박스 없음 - 건너뜀", flush=True)
            continue

        goal = ExecutePickPlace.Goal()
        goal.task = t
        goal.source_box = source_box
        goal.trunk_pose = trunk_result.trunk_map.trunk_pose

        print(f"[TRIGGER] {PICK_ACTION} 호출 (순서={t.sequence} box_id={t.box_id})...", flush=True)
        pick_future = pick_client.send_goal_async(goal, feedback_callback=feedback_cb)
        rclpy.spin_until_future_complete(node, pick_future, timeout_sec=15.0)
        pick_goal_handle = pick_future.result()
        if pick_goal_handle is None or not pick_goal_handle.accepted:
            print(f"[TRIGGER-FAIL] box_id={t.box_id}: ExecutePickPlace goal 거부됨", flush=True)
            continue

        result_future = pick_goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=300.0)
        if not result_future.done():
            print(f"[TRIGGER-FAIL] box_id={t.box_id}: ExecutePickPlace 응답 시간 초과(300s)", flush=True)
            continue
        result = result_future.result().result
        print(f"[TRIGGER-RESULT] box_id={t.box_id} success={result.success} "
              f"error_code={result.error_code} message={result.message}", flush=True)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
