// src/hooks/useResourceLoader.test.jsx
import { describe, expect, it, vi } from "vitest";
import { act, render, screen } from "@testing-library/react";
import { PlannerProvider, usePlannerState } from "../state/PlannerContext.jsx";
import { useResourceLoader } from "./useResourceLoader.js";
import * as client from "../api/client.js";

function Harness() {
  useResourceLoader();
  const state = usePlannerState();
  return <div data-testid="trunk-map">{state.trunkMap}</div>;
}

describe("useResourceLoader", () => {
  it("loads resources on mount and dispatches RESOURCES_LOADED", async () => {
    vi.spyOn(client, "fetchTrunkMaps").mockResolvedValue(["run_a"]);
    vi.spyOn(client, "fetchBoxPresets").mockResolvedValue({ "기본값": [] });

    render(<PlannerProvider><Harness /></PlannerProvider>);
    await act(async () => {});

    expect(screen.getByTestId("trunk-map").textContent).toBe("run_a");
  });
});
