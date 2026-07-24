import { usePlannerDispatch, usePlannerState } from "../state/PlannerContext.jsx";
import styles from "./BoxDetailPanel.module.css";

export default function BoxDetailPanel() {
  const state = usePlannerState();
  const dispatch = usePlannerDispatch();
  const placed = state.result?.placed || [];
  const selected = placed.find((p) => p.box_id === state.selectedBoxId);

  return (
    <div className={styles.panel}>
      <label className={styles.label}>박스 상세 조회</label>
      <select
        className={styles.select}
        value={state.selectedBoxId}
        onChange={(e) => dispatch({ type: "SELECT_BOX", payload: e.target.value })}
      >
        {placed.map((p) => (
          <option key={p.box_id} value={p.box_id}>{p.order}. {p.box_id}</option>
        ))}
      </select>

      {selected ? (
        <div className={styles.detail}>
          <p>
            <strong>{selected.box_id}</strong> · 적재순서 {selected.order} ·
            Target=({selected.position.map((v) => v.toFixed(2)).join(", ")})m ·
            Yaw={selected.target_yaw.toFixed(2)}rad
          </p>
          <p>접촉면 {selected.touches}/6개, {selected.rotated ? "90도 회전됨" : "정자세"}, 점수 {selected.score.toFixed(3)}(낮을수록 좋은 자리)</p>
          {/* score_breakdown.formula: "count_first" 모드는 내부적으로 서로 다른 두 채점
              공식 중 하나를 실제로 쓸 수 있어서(백엔드 algorism_bridge.compute_plan()
              참고), 두 형태를 분기해서 보여준다. */}
          {selected.score_breakdown.formula === "count_first_density" ? (
            <table className={styles.table}>
              <tbody>
                <tr><td>높이 항(불리)</td><td>{selected.score_breakdown.height_term.toFixed(3)}</td></tr>
                <tr><td>새 영역 확장 항(불리)</td><td>{selected.score_breakdown.footprint_growth_term.toFixed(3)}</td></tr>
              </tbody>
            </table>
          ) : (
            <table className={styles.table}>
              <tbody>
                <tr><td>높이 항(불리)</td><td>{selected.score_breakdown.height_term.toFixed(3)}</td></tr>
                <tr><td>접촉면 항(유리)</td><td>-{selected.score_breakdown.contact_term.toFixed(3)}</td></tr>
                <tr><td>안쪽 벽(A) 항(유리)</td><td>-{selected.score_breakdown.wall_a_term.toFixed(3)}</td></tr>
                <tr><td>측면 벽(B/C) 항(유리)</td><td>-{selected.score_breakdown.wall_bc_term.toFixed(3)}</td></tr>
              </tbody>
            </table>
          )}
        </div>
      ) : (
        <p className={styles.placeholder}>계획 계산 후 박스를 선택하면 상세정보가 표시됩니다</p>
      )}
    </div>
  );
}
