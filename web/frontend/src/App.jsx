// src/App.jsx
import { useState } from "react";
import { PlannerProvider } from "./state/PlannerContext.jsx";
import { useResourceLoader } from "./hooks/useResourceLoader.js";
import { useDebouncedPlan } from "./hooks/useDebouncedPlan.js";
import Header from "./components/Header.jsx";
import TabBar from "./components/TabBar.jsx";
import ControlPanel from "./components/ControlPanel.jsx";
import SummaryCard from "./components/SummaryCard.jsx";
import Scene3DViewer from "./components/Scene3DViewer.jsx";
import BoxDetailPanel from "./components/BoxDetailPanel.jsx";
import LogPanel from "./components/LogPanel.jsx";
import RobotControlPanel from "./components/RobotControlPanel.jsx";
import styles from "./App.module.css";

function SimulatorBody() {
  return (
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
  );
}

function PlannerLayout() {
  // 폴링/디바운스 계산 훅은 탭과 무관하게 항상 켜둔다(SimulatorBody 안이
  // 아니라 여기서 호출) - 로봇 제어 탭을 보고 있는 동안에도 트렁크 맵 목록이
  // 계속 갱신되고, 시뮬레이터 탭으로 돌아왔을 때 골라뒀던 값들이 탭
  // 전환마다 리셋되지 않는다(이 컴포넌트가 언마운트되지 않으므로).
  useResourceLoader();
  useDebouncedPlan();
  const [activeTab, setActiveTab] = useState("simulator");

  return (
    <div className={styles.layout}>
      <Header />
      <TabBar activeTab={activeTab} onSelect={setActiveTab} />
      {activeTab === "simulator" && <SimulatorBody />}
      {activeTab === "robot" && <RobotControlPanel />}
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
