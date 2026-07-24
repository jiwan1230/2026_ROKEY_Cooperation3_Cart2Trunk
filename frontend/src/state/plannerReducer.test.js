import { describe, expect, it } from "vitest";
import { plannerReducer, initialState } from "./plannerReducer.js";

describe("plannerReducer", () => {
  it("loads resources and selects sensible defaults", () => {
    const state = plannerReducer(initialState, {
      type: "RESOURCES_LOADED",
      payload: { trunkMaps: ["run_a", "run_b"], boxPresets: { "기본값": [{ id: "A", width: 0.3, depth: 0.2, height: 0.15 }] } },
    });
    expect(state.trunkMap).toBe("run_b");
    expect(state.boxPresetName).toBe("기본값");
    expect(JSON.parse(state.boxesText)).toHaveLength(1);
  });

  it("prefers the '기본값 (Large/Medium/Small)' preset over the alphabetically-first one", () => {
    // 백엔드 GET /api/box-presets는 알파벳/가나다 순으로 정렬돼서 오므로
    // "few_large" 같은 게 항상 첫 번째 키가 된다 - 브라우저를 처음 열었을 때
    // 그게 아니라 팀의 기본 프리셋이 선택돼 있어야 한다는 피드백을 반영.
    const state = plannerReducer(initialState, {
      type: "RESOURCES_LOADED",
      payload: {
        trunkMaps: ["run_a"],
        boxPresets: {
          few_large: [{ id: "XL1", width: 0.55, depth: 0.4, height: 0.35 }],
          "기본값 (Large/Medium/Small)": [{ id: "Large", width: 0.5, depth: 0.35, height: 0.3 }],
        },
      },
    });
    expect(state.boxPresetName).toBe("기본값 (Large/Medium/Small)");
    expect(JSON.parse(state.boxesText)).toEqual([{ id: "Large", width: 0.5, depth: 0.35, height: 0.3 }]);
  });

  it("invalidates a computed plan when a param changes", () => {
    const computed = { ...initialState, planState: "COMPUTED" };
    const next = plannerReducer(computed, { type: "SET_PARAM", payload: { key: "mode", value: "count_first" } });
    expect(next.planState).toBe("NOT_COMPUTED");
    expect(next.params.mode).toBe("count_first");
    expect(next.logLines[0]).toMatch("무효화");
  });

  it("invalidation also cancels an approval and notes it in the log", () => {
    const approved = { ...initialState, planState: "APPROVED", pendingTask: { approved: true } };
    const next = plannerReducer(approved, { type: "SET_TRUNK_MAP", payload: "run_c" });
    expect(next.planState).toBe("NOT_COMPUTED");
    expect(next.pendingTask).toBeNull();
    expect(next.logLines[0]).toMatch("승인도 함께 취소됨");
  });

  it("does not invalidate a plan that has not been computed yet", () => {
    const next = plannerReducer(initialState, { type: "SET_PARAM", payload: { key: "mode", value: "count_first" } });
    expect(next.planState).toBe("NOT_COMPUTED");
    expect(next.logLines).toHaveLength(0);
  });

  it("stores the compute result and selects the first placed box", () => {
    const payload = { placed: [{ box_id: "A" }, { box_id: "B" }], log_lines: ["line1"] };
    const next = plannerReducer(initialState, { type: "COMPUTE_SUCCESS", payload });
    expect(next.planState).toBe("COMPUTED");
    expect(next.selectedBoxId).toBe("A");
    expect(next.logLines).toEqual(["line1"]);
  });

  it("emergency stop cancels approval regardless of current state", () => {
    const approved = { ...initialState, planState: "APPROVED", pendingTask: { approved: true } };
    const next = plannerReducer(approved, { type: "EMERGENCY_STOP" });
    expect(next.planState).toBe("NOT_COMPUTED");
    expect(next.pendingTask).toBeNull();
  });
});
