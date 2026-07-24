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
          <SummaryCard />
          <Scene3DViewer />
          <BoxDetailPanel />
        </div>
      </div>
      <LogPanel />
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
