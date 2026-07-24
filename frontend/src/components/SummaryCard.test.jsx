// src/components/SummaryCard.test.jsx
import { describe, expect, it } from "vitest";
import { render, screen } from "@testing-library/react";
import { PlannerProvider } from "../state/PlannerContext.jsx";
import SummaryCard from "./SummaryCard.jsx";

describe("SummaryCard", () => {
  it("shows placeholders before any plan is computed", () => {
    render(<PlannerProvider><SummaryCard /></PlannerProvider>);
    expect(screen.getByText(/① 파라미터를 입력하면 자동으로 계산됩니다/)).toBeInTheDocument();
  });
});
