import { afterEach, describe, expect, it } from "vitest";
import { cleanup, render, screen } from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import { PlannerProvider, usePlannerDispatch } from "../state/PlannerContext.jsx";
import BoxDetailPanel from "./BoxDetailPanel.jsx";

// vitest globals가 꺼져 있어 @testing-library/react의 자동 cleanup이 동작하지 않는다.
// (ControlPanel.test.jsx, useDebouncedPlan.test.jsx와 동일한 이유로 수동 cleanup 필요)
afterEach(() => { cleanup(); });

const WEIGHTED_RESULT = {
  placed: [
    { box_id: "A", order: 1, position: [0.1, 0.2, 0.0], dimensions: [0.3, 0.2, 0.15],
      rotated: false, target_yaw: 0, score: 0.42, touches: 3,
      score_breakdown: { formula: "weighted", height_term: 0, contact_term: 0.25, wall_a_term: 0.3, wall_bc_term: 0.1 } },
  ],
  log_lines: [],
};

const COUNT_FIRST_RESULT = {
  placed: [
    { box_id: "B", order: 1, position: [0.1, 0.2, 0.0], dimensions: [0.3, 0.2, 0.15],
      rotated: false, target_yaw: 0, score: 1.5, touches: 1,
      score_breakdown: { formula: "count_first_density", height_term: 0.5, footprint_growth_term: 1.0 } },
  ],
  log_lines: [],
};

function Loader({ payload }) {
  const dispatch = usePlannerDispatch();
  return <button onClick={() => dispatch({ type: "COMPUTE_SUCCESS", payload })}>load</button>;
}

describe("BoxDetailPanel", () => {
  it("shows weighted-formula score breakdown for the selected box", async () => {
    render(
      <PlannerProvider>
        <Loader payload={WEIGHTED_RESULT} />
        <BoxDetailPanel />
      </PlannerProvider>,
    );
    await userEvent.click(screen.getByText("load"));
    expect(screen.getByText(/적재순서 1/)).toBeInTheDocument();
    expect(screen.getByText("-0.250")).toBeInTheDocument();
  });

  it("shows count-first-density formula score breakdown when that formula was used", async () => {
    render(
      <PlannerProvider>
        <Loader payload={COUNT_FIRST_RESULT} />
        <BoxDetailPanel />
      </PlannerProvider>,
    );
    await userEvent.click(screen.getByText("load"));
    expect(screen.getByText("새 영역 확장 항(불리)")).toBeInTheDocument();
    expect(screen.getByText("1.000")).toBeInTheDocument();
  });
});
