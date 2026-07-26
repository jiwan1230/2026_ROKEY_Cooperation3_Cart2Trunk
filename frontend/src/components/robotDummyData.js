// src/components/robotDummyData.js
// 로봇 제어 탭에서 쓰는 더미 데이터. 실제 스캔/로봇 데이터와 동일한 shape로
// 맞춰뒀다 - 비전 팀 데이터가 들어오면 이 값들 대신 실제 값을 그 자리에
// 넣기만 하면 된다(shape는 그대로 유지).

// TODO(비전팀 연동 시): 트렁크 스캔 결과(algorism_bridge.load_trunk_from_world_map
// 결과와 같은 shape - {width, depth, height, entrance_near_x})로 교체.
export const DUMMY_TRUNK = {
  width: 0.65, depth: 1.10, height: 0.45, entrance_near_x: true,
};

// TODO(비전팀 연동 시): vision_adapter.boxes_from_vision_corners()가 돌려주는
// 것과 같은 shape({id, width, depth, height, rests_on_id})의 실제 카트 스캔
// 결과로 교체.
export const DUMMY_CART_BOXES = [
  { id: "Large", width: 0.30, depth: 0.30, height: 0.20 },
  { id: "Medium", width: 0.20, depth: 0.13, height: 0.15 },
  { id: "Small", width: 0.15, depth: 0.13, height: 0.10 },
];

// TODO(로봇 연동 시): ROS2에서 실제 픽앤플레이스 진행 상태를 받아오면 이
// 고정 배열 대신 그 값을 그대로 pct/label로 매핑해서 쓰면 된다.
export const PICK_PLACE_STEPS = [
  { pct: 15, label: "박스1 pick 접근" },
  { pct: 35, label: "박스1 파지" },
  { pct: 55, label: "박스1 트렁크로 이동" },
  { pct: 70, label: "박스1 place" },
  { pct: 85, label: "박스2 pick 접근" },
  { pct: 100, label: "완료" },
];
