// src/components/CameraPreviewPanel.jsx
// 로봇에 달린 비전 카메라 실시간 화면 - 아직 실제 카메라가 연결되지 않아
// 정적 플레이스홀더만 보여준다. 나중에 실제 영상 스트림이 연결되면 이 안의
// placeholder div를 <video>/이미지 스트림으로 교체하면 된다.
import styles from "./CameraPreviewPanel.module.css";

export default function CameraPreviewPanel() {
  return (
    <div className={styles.panel}>
      <span className={styles.title}>로봇 카메라 실시간</span>
      <div className={styles.placeholder}>
        <span className={styles.icon}>📷</span>
        <span>카메라 미연동 - 더미 화면</span>
      </div>
    </div>
  );
}
