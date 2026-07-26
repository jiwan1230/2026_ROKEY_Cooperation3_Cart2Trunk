// src/components/RobotLogPanel.jsx
// 관제 로그 - 트렁크/카트 Scan, Pick&Place가 onLog로 보고한 이벤트를
// 시간순(최신이 위, 부모가 이미 그 순서로 넘겨줌)으로 보여주기만 하는
// 순수 렌더링 컴포넌트.
import styles from "./RobotLogPanel.module.css";

export default function RobotLogPanel({ logs }) {
  return (
    <div className={styles.panel}>
      <label className={styles.label}>관제 로그</label>
      <ul className={styles.logList} data-testid="robot-log-list">
        {logs.length === 0 && <li className={styles.logEmpty}>아직 실행된 동작이 없습니다.</li>}
        {logs.map((entry, i) => (
          <li key={i}>[{entry.time}] {entry.message}</li>
        ))}
      </ul>
    </div>
  );
}
