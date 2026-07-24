import { usePlannerDispatch, usePlannerState } from "../state/PlannerContext.jsx";
import { postApprove, postSend } from "../api/client.js";
import styles from "./ControlPanel.module.css";

function generateRandomBoxes(count) {
  const boxes = [];
  for (let i = 0; i < count; i++) {
    boxes.push({
      id: `Box${i + 1}`,
      width: Math.round((0.15 + Math.random() * 0.30) * 100) / 100,
      depth: Math.round((0.15 + Math.random() * 0.25) * 100) / 100,
      height: Math.round((0.10 + Math.random() * 0.20) * 100) / 100,
    });
  }
  return boxes;
}

const MARGIN_FIELDS = [
  { key: "margin", label: "박스 간격" },
  { key: "wallMargin", label: "벽면 간격" },
  { key: "ceilingMargin", label: "천장 여유" },
  { key: "obstacleMargin", label: "장애물 간격" },
  { key: "entranceMargin", label: "입구 여유거리" },
];

const PREFERENCE_FIELDS = [
  { key: "entrancePreference", label: "입구 ↔ 깊은 위치", min: -1, max: 1, step: 0.1 },
  { key: "contactPreference", label: "공간활용 ↔ 안정성", min: 0, max: 2, step: 0.1 },
  { key: "heightPreference", label: "바닥부터 채우기 강도", min: 0, max: 2, step: 0.1 },
];

export default function ControlPanel() {
  const state = usePlannerState();
  const dispatch = usePlannerDispatch();

  const setParam = (key, value) => dispatch({ type: "SET_PARAM", payload: { key, value } });

  const handleApprove = async () => {
    if (state.planState !== "COMPUTED" || !state.result) return;
    try {
      const resp = await postApprove({
        box_snapshot_id: state.result.box_snapshot_id,
        trunk_map_id: state.result.trunk_map_id,
        parameters: state.params,
        placed: state.result.placed,
      });
      dispatch({ type: "APPROVE_SUCCESS", payload: resp });
    } catch (err) {
      dispatch({ type: "COMPUTE_ERROR", payload: { error_code: err.error_code, cause: err.cause, action: err.action } });
    }
  };

  const handleSend = async () => {
    if (state.planState !== "APPROVED" || !state.pendingTask) return;
    try {
      const resp = await postSend({ task: state.pendingTask });
      dispatch({ type: "SEND_SUCCESS", payload: resp });
    } catch (err) {
      dispatch({ type: "COMPUTE_ERROR", payload: { error_code: err.error_code, cause: err.cause, action: err.action } });
    }
  };

  const locked = state.planState === "APPROVED";

  return (
    <div className={styles.panel}>
      <section className={styles.section}>
        <label className={styles.label}>트렁크 스캔 파일</label>
        <select
          className={styles.select}
          value={state.trunkMap}
          disabled={locked}
          onChange={(e) => dispatch({ type: "SET_TRUNK_MAP", payload: e.target.value })}
        >
          {state.trunkMaps.map((name) => <option key={name} value={name}>{name}</option>)}
        </select>

        <label className={styles.label}>카트 박스 프리셋</label>
        <select
          className={styles.select}
          value={state.boxPresetName}
          disabled={locked}
          onChange={(e) => dispatch({ type: "SELECT_PRESET", payload: e.target.value })}
        >
          {Object.keys(state.boxPresets).map((name) => <option key={name} value={name}>{name}</option>)}
        </select>
      </section>

      <section className={styles.section}>
        <label className={styles.label}>적재 모드</label>
        <div className={styles.segmented}>
          {[["large_first", "큰 것 우선"], ["count_first", "개수 우선"]].map(([value, text]) => (
            <button
              key={value}
              type="button"
              disabled={locked}
              className={state.params.mode === value ? styles.segmentActive : styles.segment}
              onClick={() => setParam("mode", value)}
            >
              {text}
            </button>
          ))}
        </div>
      </section>

      <section className={styles.section}>
        <label className={styles.label}>마진 (m, 비우면 기본값)</label>
        {MARGIN_FIELDS.map(({ key, label }) => (
          <div key={key} className={styles.fieldRow}>
            <span className={styles.fieldLabel}>{label}</span>
            <input
              type="text"
              inputMode="decimal"
              className={styles.input}
              disabled={locked}
              value={state.params[key]}
              onChange={(e) => setParam(key, e.target.value)}
            />
          </div>
        ))}
      </section>

      <section className={styles.section}>
        <label className={styles.label}>우선순위</label>
        {PREFERENCE_FIELDS.map(({ key, label, min, max, step }) => (
          <div key={key} className={styles.fieldRow}>
            <span className={styles.fieldLabel}>{label} ({Number(state.params[key]).toFixed(1)})</span>
            <input
              type="range"
              min={min} max={max} step={step}
              disabled={locked}
              value={state.params[key]}
              onChange={(e) => setParam(key, Number(e.target.value))}
            />
          </div>
        ))}
      </section>

      <section className={styles.section}>
        <label className={styles.toggleRow}>
          <input type="checkbox" disabled={locked} checked={state.params.allowStacking}
                 onChange={(e) => setParam("allowStacking", e.target.checked)} />
          2층 이상 쌓기 허용
        </label>
        <label className={styles.toggleRow}>
          <input type="checkbox" disabled={locked} checked={state.params.allowRotation}
                 onChange={(e) => setParam("allowRotation", e.target.checked)} />
          90도 회전 허용
        </label>
        <label className={styles.toggleRow}>
          <input type="checkbox" disabled={locked} checked={state.params.fixedOrder}
                 onChange={(e) => setParam("fixedOrder", e.target.checked)} />
          적재 순서 고정 (박스 목록 순서 그대로)
        </label>
      </section>

      <section className={styles.section}>
        <label className={styles.label}>박스 목록 (JSON)</label>
        <div className={styles.fieldRow}>
          <button type="button" disabled={locked} onClick={() => dispatch({
            type: "GENERATE_RANDOM_BOXES", payload: generateRandomBoxes(6),
          })}>
            무작위 6개 생성
          </button>
        </div>
        <textarea
          className={styles.boxEditor}
          data-testid="box-editor"
          rows={10}
          disabled={locked}
          value={state.boxesText}
          onChange={(e) => dispatch({ type: "SET_BOXES_TEXT", payload: e.target.value })}
        />
      </section>

      <section className={styles.section}>
        <div className={styles.actions}>
          <button type="button" disabled={locked} onClick={() => dispatch({ type: "RESET_STRATEGY_DEFAULTS" })}>
            기본값으로 초기화
          </button>
          <button type="button" disabled={state.planState !== "COMPUTED"} onClick={handleApprove}>
            승인
          </button>
          <button type="button" disabled={!(state.planState === "COMPUTED" || state.planState === "APPROVED")}
                  onClick={() => dispatch({ type: "REJECT" })}>
            거부
          </button>
          <button type="button" disabled={state.planState !== "APPROVED"} onClick={handleSend}>
            MSI2로 전송
          </button>
        </div>
        {state.error && (
          <div className={styles.errorBox}>
            <strong>오류: {state.error.error_code}</strong>
            <p>{state.error.cause}</p>
            <p>권장 조치: {state.error.action}</p>
          </div>
        )}
      </section>
    </div>
  );
}
