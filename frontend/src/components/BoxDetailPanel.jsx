import { usePlannerDispatch, usePlannerState } from "../state/PlannerContext.jsx";
import { gradeWeightedScore } from "../utils/scoreGrading.js";
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
          {/* "몇 점이면 좋은 건지 기준이 없다"는 피드백 - 지금 우선순위 슬라이더
              설정에서 이 공식이 낼 수 있는 최선~최악 범위 대비 이 박스의 점수가
              어디쯤인지 등급으로 보여준다. count_first_density 공식은 이론상
              상한이 트렁크 크기에 따라 달라져(밀도 공식) 등급 기준 대상이 아니다. */}
          {selected.score_breakdown.formula === "weighted" ? (
            <p className={styles.gradeLine}>
              품질 등급:{" "}
              <span className={styles.grade} data-grade={gradeWeightedScore(selected.score, state.params).label}>
                {gradeWeightedScore(selected.score, state.params).label}
              </span>{" "}
              (지금 우선순위 설정 기준 이론상 최선~최악 범위 중 상위 {gradeWeightedScore(selected.score, state.params).pct.toFixed(0)}%)
            </p>
          ) : (
            <p className={styles.gradeLine}>
              품질 등급: 개수 우선 모드에서는 값의 이론적 상한이 케이스마다 달라 등급을 매기지 않습니다 - 같은 계산 안에서 다른 박스와 상대 비교로 판단하세요.
            </p>
          )}
          {/* score_breakdown.formula: "count_first" 모드는 내부적으로 서로 다른 두 채점
              공식 중 하나를 실제로 쓸 수 있어서(백엔드 algorism_bridge.compute_plan()
              참고), 두 형태를 분기해서 보여준다. */}
          {selected.score_breakdown.formula === "count_first_density" ? (
            <div className={styles.breakdown}>
              <div className={styles.breakdownRow}><span>높이 항(불리)</span><span>{selected.score_breakdown.height_term.toFixed(3)}</span></div>
              <div className={styles.breakdownRow}><span>새 영역 확장 항(불리)</span><span>{selected.score_breakdown.footprint_growth_term.toFixed(3)}</span></div>
            </div>
          ) : (
            <div className={styles.breakdown}>
              <div className={styles.breakdownRow}><span>높이 항(불리)</span><span>{selected.score_breakdown.height_term.toFixed(3)}</span></div>
              <div className={styles.breakdownRow}><span>접촉면 항(유리)</span><span>-{selected.score_breakdown.contact_term.toFixed(3)}</span></div>
              <div className={styles.breakdownRow}><span>안쪽 벽(A) 항(유리)</span><span>-{selected.score_breakdown.wall_a_term.toFixed(3)}</span></div>
              <div className={styles.breakdownRow}><span>측면 벽(B/C) 항(유리)</span><span>-{selected.score_breakdown.wall_bc_term.toFixed(3)}</span></div>
            </div>
          )}
        </div>
      ) : (
        <p className={styles.placeholder}>계획 계산 후 박스를 선택하면 상세정보가 표시됩니다</p>
      )}
    </div>
  );
}
