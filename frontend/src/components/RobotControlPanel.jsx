// src/components/RobotControlPanel.jsx
// 로봇(MSI2) 동작 트리거 + 관제뷰. 시뮬레이터 탭의 PlannerContext와는 완전히
// 독립된 화면 - usePlannerState/usePlannerDispatch를 쓰지 않고 이 컴포넌트
// 안의 로컬 상태만으로 동작한다. 지금은 백엔드가 실제 ROS2를 호출하지 않고
// 더미 응답만 주므로, 버튼을 누르면 상태(대기->진행중->완료)와 로그가 실제로
// 바뀌는 것까지 눈으로 확인할 수 있다.
import { useState } from "react";
import { postCartScan, postTrunkScan, postPickAndPlace } from "../api/client.js";
import styles from "./RobotControlPanel.module.css";

const STEPS = [
  { key: "cartScan", label: "카트 스캔" },
  { key: "trunkScan", label: "트렁크 스캔" },
  { key: "pickAndPlace", label: "픽앤플레이스 시작" },
];

const STATUS_TEXT = { idle: "대기", running: "진행중", done: "완료" };

// step.trigger로 미리 캡쳐해두지 않고 호출 시점에 바로 postCartScan 등의
// 식별자를 참조한다 - 테스트에서 vi.spyOn(client, "postCartScan")으로 목을
// 걸어도, 모듈 로드 시점에 배열 리터럴 안에 원본 함수 참조를 박아두면 그
// 목이 반영되지 않는다(VisionDataLoader.jsx의 호출 시점 참조 패턴과 동일한
// 이유).
function callTrigger(key) {
  if (key === "cartScan") return postCartScan();
  if (key === "trunkScan") return postTrunkScan();
  return postPickAndPlace();
}

export default function RobotControlPanel() {
  const [statuses, setStatuses] = useState({ cartScan: "idle", trunkScan: "idle", pickAndPlace: "idle" });
  const [logs, setLogs] = useState([]);

  const appendLog = (message) => {
    const time = new Date().toLocaleTimeString("ko-KR", { hour12: false });
    setLogs((prev) => [{ time, message }, ...prev]);
  };

  const handleTrigger = async (step) => {
    setStatuses((prev) => ({ ...prev, [step.key]: "running" }));
    try {
      const resp = await callTrigger(step.key);
      setStatuses((prev) => ({ ...prev, [step.key]: "done" }));
      appendLog(resp.message);
    } catch {
      setStatuses((prev) => ({ ...prev, [step.key]: "idle" }));
      appendLog(`[오류] ${step.label} 요청 실패 - 백엔드가 실행 중인지 확인하세요`);
    }
  };

  return (
    <div className={styles.panel}>
      <div className={styles.steps}>
        {STEPS.map((step) => {
          const status = statuses[step.key];
          return (
            <div key={step.key} className={styles.step}>
              <button
                type="button"
                data-testid={`trigger-${step.key}`}
                disabled={status === "running"}
                onClick={() => handleTrigger(step)}
              >
                {step.label}
              </button>
              <span className={styles.status} data-status={status} data-testid={`status-${step.key}`}>
                {STATUS_TEXT[status]}
              </span>
            </div>
          );
        })}
      </div>

      <div className={styles.logPanel}>
        <label className={styles.logLabel}>관제 로그</label>
        <ul className={styles.logList} data-testid="robot-log-list">
          {logs.length === 0 && <li className={styles.logEmpty}>아직 실행된 동작이 없습니다.</li>}
          {logs.map((entry, i) => (
            <li key={i}>[{entry.time}] {entry.message}</li>
          ))}
        </ul>
      </div>
    </div>
  );
}
