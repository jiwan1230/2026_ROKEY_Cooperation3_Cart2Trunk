// src/components/SummaryCard.test.jsx
import { afterEach, describe, expect, it } from "vitest";
import { cleanup, render, screen } from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import { PlannerProvider, usePlannerDispatch } from "../state/PlannerContext.jsx";
import SummaryCard from "./SummaryCard.jsx";

// vitest globals가 꺼져 있어 자동 cleanup이 동작하지 않는다 (다른 테스트 파일과 동일한 이유).
afterEach(() => { cleanup(); });

function Loader({ payload }) {
  const dispatch = usePlannerDispatch();
  return <button onClick={() => dispatch({ type: "COMPUTE_SUCCESS", payload })}>load</button>;
}

describe("SummaryCard", () => {
  it("shows placeholders before any plan is computed", () => {
    render(<PlannerProvider><SummaryCard /></PlannerProvider>);
    expect(screen.getByText(/① 파라미터를 입력하면 자동으로 계산됩니다/)).toBeInTheDocument();
  });

  it("shows a utilization grade badge once a plan is computed", async () => {
    const payload = {
      placed: [], log_lines: [],
      summary: { total: 3, placed: 2, unplaced: 1, utilization_pct: 65, avg_score: -0.5, calc_time_ms: 3 },
    };
    render(<PlannerProvider><Loader payload={payload} /><SummaryCard /></PlannerProvider>);
    await userEvent.click(screen.getByText("load"));
    expect(screen.getByText("65.0%")).toBeInTheDocument();
    expect(screen.getByText("우수")).toBeInTheDocument();
  });
});
