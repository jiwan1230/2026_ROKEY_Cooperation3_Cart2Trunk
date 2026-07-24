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

export function plannerReducer(state, action) {
  switch (action.type) {
    case "RESOURCES_LOADED": {
      const { trunkMaps, boxPresets } = action.payload;
      const firstPresetName = Object.keys(boxPresets)[0] || "";
      return {
        ...state,
        trunkMaps,
        boxPresets,
        trunkMap: trunkMaps.length ? trunkMaps[trunkMaps.length - 1] : "",
        boxPresetName: firstPresetName,
        boxesText: JSON.stringify(boxPresets[firstPresetName] || [], null, 2),
        boxSourceLabel: firstPresetName,
      };
    }
    case "SET_TRUNK_MAP":
      return invalidateIfNeeded({ ...state, trunkMap: action.payload });
    case "SELECT_PRESET": {
      const boxes = state.boxPresets[action.payload] || [];
      return invalidateIfNeeded({
        ...state, boxPresetName: action.payload,
        boxesText: JSON.stringify(boxes, null, 2), boxSourceLabel: action.payload,
      });
    }
    case "SET_BOXES_TEXT":
      return invalidateIfNeeded({ ...state, boxesText: action.payload, boxSourceLabel: "custom" });
    case "GENERATE_RANDOM_BOXES":
      return invalidateIfNeeded({
        ...state, boxesText: JSON.stringify(action.payload, null, 2), boxSourceLabel: "random",
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
