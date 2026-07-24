import { describe, expect, it } from "vitest";
import { render, screen } from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import { PlannerProvider, usePlannerState } from "../state/PlannerContext.jsx";
import Header from "./Header.jsx";

function StateProbe() {
  const state = usePlannerState();
  return <div data-testid="plan-state">{state.planState}</div>;
}

describe("Header", () => {
  it("emergency stop button dispatches EMERGENCY_STOP", async () => {
    render(
      <PlannerProvider>
        <Header />
        <StateProbe />
      </PlannerProvider>,
    );
    await userEvent.click(screen.getByText("EMERGENCY STOP"));
    expect(screen.getByTestId("plan-state").textContent).toBe("NOT_COMPUTED");
  });
});
