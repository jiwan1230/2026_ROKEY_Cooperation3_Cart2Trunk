import { describe, expect, it } from "vitest";
import { render, screen } from "@testing-library/react";
import { PlannerProvider, usePlannerState } from "./PlannerContext.jsx";

function Probe() {
  const state = usePlannerState();
  return <div>planState:{state.planState}</div>;
}

describe("PlannerProvider", () => {
  it("provides the reducer's initial state to descendants", () => {
    render(
      <PlannerProvider>
        <Probe />
      </PlannerProvider>,
    );
    expect(screen.getByText("planState:NOT_COMPUTED")).toBeInTheDocument();
  });
});
