// src/components/LogPanel.test.jsx
import { describe, expect, it } from "vitest";
import { render, screen } from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import { PlannerProvider, usePlannerDispatch } from "../state/PlannerContext.jsx";
import LogPanel from "./LogPanel.jsx";

function Loader() {
  const dispatch = usePlannerDispatch();
  return <button onClick={() => dispatch({ type: "EMERGENCY_STOP" })}>stop</button>;
}

describe("LogPanel", () => {
  it("renders appended log lines", async () => {
    render(<PlannerProvider><Loader /><LogPanel /></PlannerProvider>);
    await userEvent.click(screen.getByText("stop"));
    expect(screen.getByText(/EMERGENCY STOP/)).toBeInTheDocument();
  });
});
