import styles from "./Header.module.css";
import { usePlannerDispatch } from "../state/PlannerContext.jsx";

export default function Header() {
  const dispatch = usePlannerDispatch();

  return (
    <header className={styles.header}>
      <h1 className={styles.title}>Cart2Trunk — 적재 알고리즘 시뮬레이터</h1>
      <button
        type="button"
        className={styles.estop}
        onClick={() => dispatch({ type: "EMERGENCY_STOP" })}
      >
        EMERGENCY STOP
      </button>
    </header>
  );
}
