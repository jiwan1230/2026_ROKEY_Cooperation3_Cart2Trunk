import { usePlannerDispatch, usePlannerState } from "../state/PlannerContext.jsx";
import { gradeBoxScore } from "../utils/scoreGrading.js";
import styles from "./BoxDetailPanel.module.css";

export default function BoxDetailPanel() {
  const state = usePlannerState();
  const dispatch = usePlannerDispatch();
  const placed = state.result?.placed || [];
  const selected = placed.find((p) => p.box_id === state.selectedBoxId);
  const grade = selected
    ? gradeBoxScore(selected.score_breakdown.formula, selected.score, {
        preferences: state.params,
        trunk: state.result?.trunk,
      })
    : null;

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
          <p>접촉면 {selected.touches}/6개, {selected.rotated ? "90도 회전됨" : "정자세"}</p>
          {/* "몇 점이면 좋은 건지 기준이 없다"는 피드백 - SummaryCard의 "평균
              점수"와 완전히 같은 모양(값 옆에 배지, 그 밑에 작은 설명 캡션)으로
              맞춘다(사용자 피드백 - 문장으로 따로 떨어져 있던 걸 값에 바로
              붙는 배지로 바꿈). formula별 등급 범위는 gradeBoxScore가 알아서
              골라준다(weighted는 preference 슬라이더 기준, count_first_density는
              trunk 크기 기준 - "개수 우선 모드에서도 등급을 보고 싶다"는
              피드백으로 두 공식 모두 지원하게 됨). */}
          <p>
            점수 {selected.score.toFixed(3)}(낮을수록 좋은 자리)
            {grade && (
              <span className={styles.grade} data-grade={grade.label}>
                {grade.label}
              </span>
            )}
          </p>
          {grade && (
            <p className={styles.gradeCaption}>
              점수 등급: {selected.score_breakdown.formula === "weighted"
                ? "지금 우선순위 설정"
                : "지금 트렁크 크기"} 기준 이론상 최선~최악 범위 중 상위 {grade.pct.toFixed(0)}%
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
