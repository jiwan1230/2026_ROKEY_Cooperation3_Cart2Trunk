// src/components/TabBar.jsx
// 시뮬레이터/로봇 제어 탭 전환 - react-router 없이 App.jsx의 로컬 상태로
// 전환하기 위한 순수 프레젠테이션 컴포넌트.
import styles from "./TabBar.module.css";

const TABS = [
  { key: "simulator", label: "시뮬레이터" },
  { key: "robot", label: "로봇 제어" },
];

export default function TabBar({ activeTab, onSelect }) {
  return (
    <nav className={styles.tabBar}>
      {TABS.map(({ key, label }) => (
        <button
          key={key}
          type="button"
          data-testid={`tab-${key}`}
          aria-current={activeTab === key ? "page" : undefined}
          className={activeTab === key ? styles.tabActive : styles.tab}
          onClick={() => onSelect(key)}
        >
          {label}
        </button>
      ))}
    </nav>
  );
}
