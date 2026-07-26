// src/components/RobotControlPanel.jsx
// 로봇 제어 탭 - 트렁크 Scan / 카트 Scan / Pick&Place 3칸 컨테이너.
// 실제 로직은 각 자식 컴포넌트(ScanViewerPanel, PickPlacePanel)에 있고,
// 여기는 배치만 담당한다.
import ScanViewerPanel from "./ScanViewerPanel.jsx";
import PickPlacePanel from "./PickPlacePanel.jsx";
import styles from "./RobotControlPanel.module.css";

export default function RobotControlPanel() {
  return (
    <div className={styles.panel}>
      <ScanViewerPanel kind="trunk" />
      <ScanViewerPanel kind="cart" />
      <PickPlacePanel />
    </div>
  );
}
