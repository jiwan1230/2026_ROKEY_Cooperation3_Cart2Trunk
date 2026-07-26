// src/components/PickPlacePanel.jsx
// Pick&Place 칸 - 진행현황 바 + 현재 진행 작업 텍스트 + 상태(Run/Stop/
// Warning). 백엔드 더미 호출(postPickAndPlace)은 지금까지와 동일하게 1회만
// 하고, 화면에 보이는 단계별 진행 애니메이션은 프론트엔드가 자체적으로
// PICK_PLACE_STEPS를 순회하며 만든다(실제 ROS2 진행 상태 스트리밍은
// 아직 없음).
import { useRef, useState } from "react";
import { postPickAndPlace } from "../api/client.js";
import { PICK_PLACE_STEPS } from "./robotDummyData.js";
import styles from "./PickPlacePanel.module.css";

const STEP_INTERVAL_MS = 700;

export default function PickPlacePanel({ onLog = () => {} }) {
  const [runState, setRunState] = useState("stopped"); // "stopped" | "running"
  const [stepIndex, setStepIndex] = useState(-1); // -1 = 아직 시작 안 함
  const timerRef = useRef(null);

  const advance = (nextIndex) => {
    if (nextIndex >= PICK_PLACE_STEPS.length) {
      setRunState("stopped");
      onLog("픽앤플레이스 완료");
      return;
    }
    setStepIndex(nextIndex);
    timerRef.current = setTimeout(() => advance(nextIndex + 1), STEP_INTERVAL_MS);
  };

  const handleStart = () => {
    setRunState("running");
    setStepIndex(-1);
    // TODO(로봇 연동 시): 여기 응답으로 실제 진행 상태가 오면, 아래 advance()의
    // 프론트 자체 타이머 시뮬레이션 대신 그 값을 그대로 반영하도록 바꾼다.
    postPickAndPlace().catch(() => {});
    onLog("픽앤플레이스 시작");
    advance(0);
  };

  const currentStep = stepIndex >= 0 ? PICK_PLACE_STEPS[stepIndex] : null;

  return (
    <div className={styles.panel}>
      <span className={styles.title}>Pick&amp;Place</span>

      <div className={styles.progressSection}>
        <label className={styles.label}>진행현황</label>
        <div className={styles.progressBar}>
          <div className={styles.progressFill} style={{ width: `${currentStep?.pct ?? 0}%` }} />
        </div>
        <span className={styles.progressPct}>{currentStep?.pct ?? 0}%</span>
      </div>

      <div className={styles.taskSection}>
        <label className={styles.label}>현재 진행 작업</label>
        <div className={styles.taskText} data-testid="current-task">
          {currentStep ? currentStep.label : "대기 중"}
        </div>
      </div>

      <div className={styles.statusSection}>
        <label className={styles.label}>현재 상태</label>
        <span className={styles.statusBadge} data-status={runState} data-testid="pick-place-status">
          {runState === "running" ? "Run" : "Stop"}
        </span>
      </div>

      <button type="button" data-testid="trigger-pickAndPlace" disabled={runState === "running"} onClick={handleStart}>
        픽앤플레이스 시작
      </button>
    </div>
  );
}
