// src/components/SummaryCard.jsx
import { usePlannerState } from "../state/PlannerContext.jsx";
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

  return (
    <div className={styles.card}>
      <div className={styles.row}><span>전체</span><strong>{summary ? summary.total : "-"}</strong></div>
      <div className={styles.row}><span>적재됨</span><strong>{summary ? summary.placed : "-"}</strong></div>
      <div className={styles.row}><span>미적재</span><strong>{summary ? summary.unplaced : "-"}</strong></div>
      <div className={styles.row}><span>공간 활용률</span><strong>{summary ? `${summary.utilization_pct.toFixed(1)}%` : "-"}</strong></div>
      <div className={styles.row}><span>평균 점수</span><strong>{summary ? summary.avg_score.toFixed(3) : "-"}</strong></div>
      <div className={styles.row}><span>계산 시간</span><strong>{summary ? `${summary.calc_time_ms.toFixed(0)}ms` : "-"}</strong></div>
      <div className={styles.status}>상태: {STATUS_LABEL[state.planState]}</div>
    </div>
  );
}
