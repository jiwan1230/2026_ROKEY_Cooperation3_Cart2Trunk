// src/components/SummaryCard.jsx
import { usePlannerState } from "../state/PlannerContext.jsx";
import { gradeBoxScore, gradeUtilization } from "../utils/scoreGrading.js";
import styles from "./SummaryCard.module.css";

const STATUS_LABEL = {
  NOT_COMPUTED: "① 파라미터를 입력하면 자동으로 계산됩니다",
  COMPUTING: "계산 중...",
  COMPUTED: "계산됨 - 승인하거나 파라미터를 조정하세요",
  APPROVED: "승인됨 - 파라미터 잠김 - MSI2로 전송할 수 있습니다",
};

export default function SummaryCard() {
  const state = usePlannerState();
  const summary = state.result?.summary;
  const placed = state.result?.placed || [];
  // 평균 점수 등급은 모든 박스가 "같은" 채점 공식을 썼을 때만 의미가 있다 -
  // count_first 모드는 내부적으로 weighted/count_first_density 두 공식 중
  // 하나를 박스마다 다르게 쓸 수 있는데, 두 공식이 섞이면 스케일이 완전히
  // 달라(예: 섭씨/화씨를 같이 평균내는 것과 같음) 평균 자체가 무의미해진다.
  // 반면 "개수 우선 모드에서도 평균 점수/등급을 보고 싶다"는 피드백대로,
  // 전부 같은 공식(weighted만, 또는 count_first_density만)이면 그 공식
  // 기준으로 등급을 매길 수 있으므로 formula가 균일한 경우까지 확장한다.
  const uniformFormula = placed.length > 0 && placed.every(
    (p) => p.score_breakdown.formula === placed[0].score_breakdown.formula,
  ) ? placed[0].score_breakdown.formula : null;
  const scoreGrade = uniformFormula
    ? gradeBoxScore(uniformFormula, summary?.avg_score ?? 0, {
        preferences: state.params,
        trunk: state.result?.trunk,
      })
    : null;

  return (
    <div className={styles.card}>
      <div className={styles.row}><span>전체</span><strong>{summary ? summary.total : "-"}</strong></div>
      <div className={styles.row}><span>적재됨</span><strong>{summary ? summary.placed : "-"}</strong></div>
      <div className={styles.row}><span>미적재</span><strong>{summary ? summary.unplaced : "-"}</strong></div>
      <div className={styles.row}>
        <span>공간 활용률</span>
        <strong>
          {summary ? `${summary.utilization_pct.toFixed(1)}%` : "-"}
          {summary && (
            <span className={styles.grade} data-grade={gradeUtilization(summary.utilization_pct).label}>
              {gradeUtilization(summary.utilization_pct).label}
            </span>
          )}
        </strong>
      </div>
      <div className={styles.row}>
        <span>평균 점수</span>
        <strong>
          {summary ? summary.avg_score.toFixed(3) : "-"}
          {scoreGrade && (
            <span className={styles.grade} data-grade={scoreGrade.label}>{scoreGrade.label}</span>
          )}
        </strong>
      </div>
      {scoreGrade && (
        <div className={styles.criteria}>
          점수 등급: {uniformFormula === "weighted" ? "지금 우선순위 설정" : "지금 트렁크 크기"} 기준 이론상 최선~최악 범위 중 상위 {scoreGrade.pct.toFixed(0)}%
        </div>
      )}
      <div className={styles.row}><span>계산 시간</span><strong>{summary ? `${summary.calc_time_ms.toFixed(0)}ms` : "-"}</strong></div>
      <div className={styles.status}>상태: {STATUS_LABEL[state.planState]}</div>
      {summary && (
        <div className={styles.criteria}>
          <div>공간 활용률 기준: 22%↑ 우수 · 14~22% 양호 · 8~14% 보통 · 8%↓ 개선 필요</div>
          <div className={styles.criteriaNote}>
            (박스를 전부 실어도 트렁크 규모상 활용률이 높게 나오기 어려움 - 실측 기준)
          </div>
        </div>
      )}
    </div>
  );
}
