// src/components/RobotControlPanel.jsx
// 로봇 제어 탭 - 2x2 그리드(트렁크/카트 Scan 위, 카메라+Pick&Place+관제
// 로그 아래). 관제 로그 상태를 여기서 소유하고, 자식들이 onLog로 보고하는
// 이벤트를 시간순으로 쌓는다.
import { useState } from "react";
import ScanViewerPanel from "./ScanViewerPanel.jsx";
import PickPlacePanel from "./PickPlacePanel.jsx";
import CameraPreviewPanel from "./CameraPreviewPanel.jsx";
import RobotLogPanel from "./RobotLogPanel.jsx";
import styles from "./RobotControlPanel.module.css";

export default function RobotControlPanel() {
  const [logs, setLogs] = useState([]);

  const appendLog = (message) => {
    const time = new Date().toLocaleTimeString("ko-KR", { hour12: false });
    setLogs((prev) => [{ time, message }, ...prev]);
  };

  return (
    <div className={styles.panel}>
      <div className={styles.trunk}><ScanViewerPanel kind="trunk" onLog={appendLog} /></div>
      <div className={styles.cart}><ScanViewerPanel kind="cart" onLog={appendLog} /></div>
      <div className={styles.camera}><CameraPreviewPanel /></div>
      <div className={styles.pick}><PickPlacePanel onLog={appendLog} /></div>
      <div className={styles.log}><RobotLogPanel logs={logs} /></div>
    </div>
  );
}
