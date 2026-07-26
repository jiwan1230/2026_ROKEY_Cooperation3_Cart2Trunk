// src/hooks/useDebouncedPlan.test.jsx
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { act, cleanup, render, screen } from "@testing-library/react";
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
  // vitest globals가 꺼져 있어 @testing-library/react의 자동 cleanup이
  // 동작하지 않으므로, 테스트 간 DOM 누적(예: "load" 버튼 중복 매칭)을
  // 막기 위해 명시적으로 unmount한다.
  afterEach(() => { cleanup(); vi.useRealTimers(); vi.restoreAllMocks(); });

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

  it("ignores a stale response that resolves after a newer request has already fired", async () => {
    // 오래된 요청(느리게 응답)과 최신 요청(빠르게 응답)이 겹치는 상황을
    // 재현한다 - 오래된 응답이 나중에 도착해도 결과를 덮어쓰면 안 된다.
    let resolveOld;
    const oldPromise = new Promise((resolve) => { resolveOld = resolve; });
    vi.spyOn(client, "postPlan")
      .mockReturnValueOnce(oldPromise)
      .mockResolvedValueOnce({ placed: [{ box_id: "NEW" }], log_lines: ["new"] });

    function TwoStepHarness() {
      useDebouncedPlan();
      const state = usePlannerState();
      const dispatch = usePlannerDispatch();
      return (
        <div>
          <div data-testid="result-log">{state.result ? state.result.log_lines[0] : "none"}</div>
          <button onClick={() => dispatch({
            type: "RESOURCES_LOADED",
            payload: { trunkMaps: ["run_a"], boxPresets: { "기본값": [{ id: "A", width: 0.3, depth: 0.2, height: 0.15 }] } },
          })}>load</button>
          <button onClick={() => dispatch({ type: "SET_PARAM", payload: { key: "mode", value: "count_first" } })}>
            change
          </button>
        </div>
      );
    }

    render(<PlannerProvider><TwoStepHarness /></PlannerProvider>);

    await act(async () => { screen.getByText("load").click(); });
    await act(async () => { vi.advanceTimersByTime(400); }); // 첫 번째(오래된) 요청 발화 - 아직 안 끝남

    await act(async () => { screen.getByText("change").click(); });
    await act(async () => { vi.advanceTimersByTime(400); }); // 두 번째(최신) 요청 발화 + 즉시 resolve
    await act(async () => { resolveOld({ placed: [{ box_id: "OLD" }], log_lines: ["old"] }); }); // 오래된 요청이 뒤늦게 도착

    expect(client.postPlan).toHaveBeenCalledTimes(2);
    expect(screen.getByTestId("result-log").textContent).toBe("new"); // 오래된 응답에 덮어써지지 않음
  });

  it("sends the real box_snapshot_id when boxes were loaded from vision data", async () => {
    vi.spyOn(client, "postPlan").mockResolvedValue({ placed: [], log_lines: [] });

    function VisionHarness() {
      useDebouncedPlan();
      const dispatch = usePlannerDispatch();
      return (
        <div>
          <button onClick={() => dispatch({
            type: "RESOURCES_LOADED",
            payload: { trunkMaps: ["run_a"], boxPresets: {} },
          })}>load</button>
          <button onClick={() => dispatch({
            type: "LOAD_VISION_BOXES",
            payload: {
              boxes: [{ id: "BOX_01", width: 0.3, depth: 0.2, height: 0.15 }],
              snapshotId: "box_scan_001", sourceLabel: "vision:box_scan_001",
            },
          })}>load-vision</button>
        </div>
      );
    }

    render(<PlannerProvider><VisionHarness /></PlannerProvider>);

    await act(async () => { screen.getByText("load").click(); });
    await act(async () => { screen.getByText("load-vision").click(); });
    await act(async () => { vi.advanceTimersByTime(400); });

    const lastCall = client.postPlan.mock.calls.at(-1)[0];
    expect(lastCall.box_snapshot_id).toBe("box_scan_001");
  });
});
