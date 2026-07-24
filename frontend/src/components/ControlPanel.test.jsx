import { afterEach, describe, expect, it } from "vitest";
import { cleanup, render, screen } from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import { PlannerProvider, usePlannerState } from "../state/PlannerContext.jsx";
import ControlPanel from "./ControlPanel.jsx";

// vitest globals가 꺼져 있어 @testing-library/react의 자동 cleanup이 동작하지 않는다.
// (useDebouncedPlan.test.jsx와 동일한 이유로 수동 cleanup 필요)
afterEach(() => { cleanup(); });

function ModeProbe() {
  const state = usePlannerState();
  return <div data-testid="mode">{state.params.mode}</div>;
}

describe("ControlPanel", () => {
  it("selecting count_first mode updates shared state", async () => {
    render(
      <PlannerProvider>
        <ControlPanel />
        <ModeProbe />
      </PlannerProvider>,
    );
    await userEvent.click(screen.getByText("개수 우선"));
    expect(screen.getByTestId("mode").textContent).toBe("count_first");
  });

  it("generating random boxes fills the box editor with 6 entries", async () => {
    render(<PlannerProvider><ControlPanel /></PlannerProvider>);
    await userEvent.click(screen.getByText("무작위 6개 생성"));
    const editor = screen.getByTestId("box-editor");
    const boxes = JSON.parse(editor.value);
    expect(boxes).toHaveLength(6);
  });
});
