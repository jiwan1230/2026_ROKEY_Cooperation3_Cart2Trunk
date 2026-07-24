// src/hooks/useDebouncedPlan.test.jsx
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { act, render, screen } from "@testing-library/react";
import { PlannerProvider, usePlannerDispatch, usePlannerState } from "../state/PlannerContext.jsx";
import { useDebouncedPlan } from "./useDebouncedPlan.js";
import * as client from "../api/client.js";

function Harness() {
  useDebouncedPlan();
  const state = usePlannerState();
  const dispatch = usePlannerDispatch();
  return (
    <div>
      <div data-testid="plan-state">{state.planState}</div>
      <button onClick={() => dispatch({
        type: "RESOURCES_LOADED",
        payload: { trunkMaps: ["run_a"], boxPresets: { "기본값": [{ id: "A", width: 0.3, depth: 0.2, height: 0.15 }] } },
      })}>load</button>
    </div>
  );
}

describe("useDebouncedPlan", () => {
  beforeEach(() => vi.useFakeTimers());
  afterEach(() => { vi.useRealTimers(); vi.restoreAllMocks(); });

  it("fires postPlan 400ms after params settle and dispatches COMPUTE_SUCCESS", async () => {
    vi.spyOn(client, "postPlan").mockResolvedValue({ placed: [], log_lines: [] });
    render(<PlannerProvider><Harness /></PlannerProvider>);

    await act(async () => {
      screen.getByText("load").click();
    });
    await act(async () => {
      vi.advanceTimersByTime(400);
    });

    expect(client.postPlan).toHaveBeenCalledTimes(1);
    expect(screen.getByTestId("plan-state").textContent).toBe("COMPUTED");
  });
});
