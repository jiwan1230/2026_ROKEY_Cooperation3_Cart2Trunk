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
      summary: { total: 3, placed: 2, unplaced: 1, utilization_pct: 25, avg_score: -0.5, calc_time_ms: 3 },
    };
    render(<PlannerProvider><Loader payload={payload} /><SummaryCard /></PlannerProvider>);
    await userEvent.click(screen.getByText("load"));
    expect(screen.getByText("25.0%")).toBeInTheDocument();
    expect(screen.getByText("우수")).toBeInTheDocument();
  });

  it("shows an average-score grade badge next to 평균 점수 when all placed boxes use the weighted formula", async () => {
    const payload = {
      log_lines: [],
      placed: [
        { box_id: "A", score_breakdown: { formula: "weighted" } },
        { box_id: "B", score_breakdown: { formula: "weighted" } },
      ],
      summary: { total: 2, placed: 2, unplaced: 0, utilization_pct: 10, avg_score: -0.5, calc_time_ms: 3 },
    };
    render(<PlannerProvider><Loader payload={payload} /><SummaryCard /></PlannerProvider>);
    await userEvent.click(screen.getByText("load"));
    expect(screen.getByText("-0.500")).toBeInTheDocument();
    expect(screen.getByText(/점수 등급: 지금 우선순위 설정 기준/)).toBeInTheDocument();
  });

  it("shows a score grade for count_first_density when all placed boxes use it and trunk size is known", async () => {
    const payload = {
      log_lines: [],
      placed: [{ box_id: "A", score_breakdown: { formula: "count_first_density" } }],
      summary: { total: 1, placed: 1, unplaced: 0, utilization_pct: 10, avg_score: 1.2, calc_time_ms: 3 },
      trunk: { width: 0.85, depth: 1.25, height: 0.5 },
    };
    render(<PlannerProvider><Loader payload={payload} /><SummaryCard /></PlannerProvider>);
    await userEvent.click(screen.getByText("load"));
    expect(screen.getByText(/점수 등급: 지금 트렁크 크기 기준/)).toBeInTheDocument();
  });

  it("hides the score grade when weighted and count_first_density formulas are mixed", async () => {
    const payload = {
      log_lines: [],
      placed: [
        { box_id: "A", score_breakdown: { formula: "weighted" } },
        { box_id: "B", score_breakdown: { formula: "count_first_density" } },
      ],
      summary: { total: 2, placed: 2, unplaced: 0, utilization_pct: 10, avg_score: 1.2, calc_time_ms: 3 },
      trunk: { width: 0.85, depth: 1.25, height: 0.5 },
    };
    render(<PlannerProvider><Loader payload={payload} /><SummaryCard /></PlannerProvider>);
    await userEvent.click(screen.getByText("load"));
    expect(screen.queryByText(/점수 등급:/)).not.toBeInTheDocument();
  });
});
