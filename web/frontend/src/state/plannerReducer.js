export const DEFAULT_STRATEGY_PARAMS = {
  mode: "large_first",
  margin: "",
  wallMargin: "",
  obstacleMargin: "",
  ceilingMargin: "",
  entranceMargin: "",
  entrancePreference: 1.0,
  contactPreference: 1.0,
  heightPreference: 1.0,
  allowStacking: false,
  allowRotation: true,
  fixedOrder: false,
};

export const initialState = {
  trunkMaps: [],
  boxPresets: {},
  trunkMap: "",
  boxPresetName: "",
  boxesText: "[]",
  boxSourceLabel: "custom",
  // 박스 목록이 실제 비전(box_scan.json/all_boxes_corners_*.json)에서 온
  // 경우에만 진짜 snapshot_id를 갖는다 - 수동 입력/프리셋/무작위 생성은
  // null(POST /api/plan에서 생략되어 백엔드가 "manual_input:..."을 대신
  // 채운다). LOAD_VISION_BOXES에서만 값이 채워지고, 그 외 박스 목록을
  // 바꾸는 모든 액션에서 다시 null로 리셋된다.
  boxSnapshotId: null,
  params: { ...DEFAULT_STRATEGY_PARAMS },
  planState: "NOT_COMPUTED", // NOT_COMPUTED | COMPUTING | COMPUTED | APPROVED
  result: null,
  error: null,
  selectedBoxId: "",
  logLines: [],
  pendingTask: null,
};

function appendLog(state, line) {
  return { ...state, logLines: [...state.logLines, line] };
}

function invalidateIfNeeded(state) {
  if (state.planState === "NOT_COMPUTED" || state.planState === "COMPUTING") return state;
  const wasApproved = state.planState === "APPROVED";
  const next = { ...state, planState: "NOT_COMPUTED", pendingTask: null };
  return appendLog(next, `[무효화] 파라미터가 변경되어 기존 계획을 무효화했습니다${wasApproved ? " (승인도 함께 취소됨)" : ""}.`);
}

// 백엔드 algorism_bridge.list_box_presets()가 항상 이 이름으로 기본 프리셋
// (Large/Medium/Small)을 넣어준다 - 브라우저를 처음 열었을 때 알파벳/가나다
// 순으로 매겨지는 첫 번째 프리셋(예: "few_large")이 아니라 이 기본 프리셋이
// 선택되어 있어야 한다는 사용자 피드백. 백엔드 문자열이 바뀌면 여기도 같이
// 바꿔야 한다 - 못 찾으면(이름이 어긋났거나 프리셋 자체가 없으면) 안전하게
// 첫 번째 프리셋으로 폴백한다.
const DEFAULT_PRESET_NAME = "기본값 (Large/Medium/Small)";

export function plannerReducer(state, action) {
  switch (action.type) {
    case "RESOURCES_LOADED": {
      const { trunkMaps, boxPresets } = action.payload;
      const presetNames = Object.keys(boxPresets);
      const initialPresetName = presetNames.includes(DEFAULT_PRESET_NAME)
        ? DEFAULT_PRESET_NAME
        : (presetNames[0] || "");
      return {
        ...state,
        trunkMaps,
        boxPresets,
        trunkMap: trunkMaps.length ? trunkMaps[trunkMaps.length - 1] : "",
        boxPresetName: initialPresetName,
        boxesText: JSON.stringify(boxPresets[initialPresetName] || [], null, 2),
        boxSourceLabel: initialPresetName,
      };
    }
    // 트렁크 스캔이 백그라운드에서 끝나 새 run_*/pointcloud/trunk_map.json이
    // 생겨도 페이지를 새로고침하기 전엔 드롭다운에 안 보이던 문제 - 주기적으로
    // 목록만 다시 불러온다(useResourceLoader.js). RESOURCES_LOADED와 달리
    // 현재 선택된 trunkMap/박스 입력을 건드리지 않는다 - 작업 중인 화면이
    // 폴링 때문에 매번 초기화되면 안 되므로 목록 자체만 갱신한다.
    case "TRUNK_MAPS_REFRESHED":
      return { ...state, trunkMaps: action.payload.trunkMaps };
    case "SET_TRUNK_MAP":
      return invalidateIfNeeded({ ...state, trunkMap: action.payload });
    case "SELECT_PRESET": {
      const boxes = state.boxPresets[action.payload] || [];
      return invalidateIfNeeded({
        ...state, boxPresetName: action.payload, boxSnapshotId: null,
        boxesText: JSON.stringify(boxes, null, 2), boxSourceLabel: action.payload,
      });
    }
    case "SET_BOXES_TEXT":
      return invalidateIfNeeded({
        ...state, boxesText: action.payload, boxSourceLabel: "custom", boxSnapshotId: null,
      });
    case "GENERATE_RANDOM_BOXES":
      return invalidateIfNeeded({
        ...state, boxesText: JSON.stringify(action.payload, null, 2),
        boxSourceLabel: "random", boxSnapshotId: null,
      });
    // 비전(준형)이 만드는 box_scan.json/all_boxes_corners_*.json 파일을 불러와
    // vision_adapter가 변환해준 단순 박스 목록 + 진짜 snapshot_id를 그대로
    // 반영한다 - "팀원 데이터가 준비되면 그것만 넣어서 실행"하려는 목적.
    case "LOAD_VISION_BOXES":
      return invalidateIfNeeded({
        ...state,
        boxesText: JSON.stringify(action.payload.boxes, null, 2),
        boxSourceLabel: action.payload.sourceLabel,
        boxSnapshotId: action.payload.snapshotId,
        boxPresetName: "",
      });
    case "SET_PARAM":
      return invalidateIfNeeded({
        ...state, params: { ...state.params, [action.payload.key]: action.payload.value },
      });
    case "RESET_STRATEGY_DEFAULTS":
      return invalidateIfNeeded({ ...state, params: { ...DEFAULT_STRATEGY_PARAMS } });
    case "COMPUTE_START":
      return { ...state, planState: "COMPUTING", error: null };
    case "COMPUTE_SUCCESS":
      return {
        ...state, planState: "COMPUTED", result: action.payload,
        logLines: action.payload.log_lines, error: null,
        selectedBoxId: action.payload.placed.length ? action.payload.placed[0].box_id : "",
      };
    case "COMPUTE_ERROR":
      return appendLog(
        { ...state, planState: "NOT_COMPUTED", error: action.payload },
        `[오류] ${action.payload.error_code}: ${action.payload.cause}`,
      );
    case "SELECT_BOX":
      return { ...state, selectedBoxId: action.payload };
    case "APPROVE_SUCCESS":
      return appendLog(
        { ...state, planState: "APPROVED", pendingTask: action.payload.task },
        `[승인] plan_id=${action.payload.plan_id}`,
      );
    case "REJECT":
      return appendLog(
        { ...state, planState: "NOT_COMPUTED", pendingTask: null },
        "[거부] 계획을 거부했습니다 - 파라미터를 조정하고 다시 계산하세요.",
      );
    case "SEND_SUCCESS":
      return appendLog(state, `[승인 및 실행] 로컬에 저장됨: ${action.payload.out_path} (MSI2 실전송 경로 확정 대기)`);
    case "EMERGENCY_STOP":
      return appendLog(
        { ...state, planState: "NOT_COMPUTED", pendingTask: null },
        "[EMERGENCY STOP] 승인/전송을 즉시 취소했습니다. 실제 로봇 정지는 MSI2/하드웨어 E-Stop 담당입니다.",
      );
    default:
      return state;
  }
}
