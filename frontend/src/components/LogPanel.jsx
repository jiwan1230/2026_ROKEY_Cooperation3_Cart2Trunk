// src/components/LogPanel.jsx
import { usePlannerState } from "../state/PlannerContext.jsx";
import styles from "./LogPanel.module.css";

export default function LogPanel() {
  const state = usePlannerState();
  return (
    <div className={styles.panel}>
      <label className={styles.label}>결과 로그</label>
      <pre className={styles.log}>{state.logLines.join("\n")}</pre>
    </div>
  );
}
