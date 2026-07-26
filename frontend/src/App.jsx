// src/App.jsx
import { PlannerProvider } from "./state/PlannerContext.jsx";
import { useResourceLoader } from "./hooks/useResourceLoader.js";
import { useDebouncedPlan } from "./hooks/useDebouncedPlan.js";
import Header from "./components/Header.jsx";
import ControlPanel from "./components/ControlPanel.jsx";
import SummaryCard from "./components/SummaryCard.jsx";
import Scene3DViewer from "./components/Scene3DViewer.jsx";
import BoxDetailPanel from "./components/BoxDetailPanel.jsx";
import LogPanel from "./components/LogPanel.jsx";
import styles from "./App.module.css";

function PlannerLayout() {
  useResourceLoader();
  useDebouncedPlan();

  return (
    <div className={styles.layout}>
      <Header />
      <div className={styles.body}>
        <ControlPanel />
        <div className={styles.resultArea}>
          {/* 사용자 손그림 피드백 - 요약 카드("화면 1")는 좁게, 3D 뷰어("메인
              화면 - 2")는 넓게 나란히 배치한다. 예전엔 세로로 쌓아서 요약
              카드가 위쪽 공간을 다 차지하고 3D 뷰어(툴바+캔버스)가 스크롤을
              내려야 보였다. 요약 카드는 내용이 짧아 3D 뷰어 높이만큼의 빈
              공간이 아래에 남는데, 그 자리에 결과 로그(예전엔 페이지 맨
              아래 별도 줄)를 넣어달라는 피드백을 반영해 왼쪽 칸 안에
              같이 쌓는다. */}
          <div className={styles.topRow}>
            <div className={styles.leftColumn}>
              <SummaryCard />
              <LogPanel />
            </div>
            <Scene3DViewer />
          </div>
          <BoxDetailPanel />
        </div>
      </div>
    </div>
  );
}

export default function App() {
  return (
    <PlannerProvider>
      <PlannerLayout />
    </PlannerProvider>
  );
}
