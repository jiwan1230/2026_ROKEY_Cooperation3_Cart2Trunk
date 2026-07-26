// src/hooks/useResourceLoader.test.jsx
import { afterEach, describe, expect, it, vi } from "vitest";
import { act, cleanup, render, screen } from "@testing-library/react";
import { PlannerProvider, usePlannerState } from "../state/PlannerContext.jsx";
import { useResourceLoader } from "./useResourceLoader.js";
import * as client from "../api/client.js";

function Harness() {
  useResourceLoader();
  const state = usePlannerState();
  return <div data-testid="trunk-map">{state.trunkMap}</div>;
}

// vitest globals가 꺼져 있어 @testing-library/react의 자동 cleanup이 동작하지
// 않는다 (ControlPanel.test.jsx와 동일한 이유) - 특히 이 파일은 폴링용
// setInterval을 쓰므로, unmount로 정리 안 하면 다음 테스트 중에도 백그라운드
// 타이머가 계속 돌아 서로 간섭할 수 있어 수동 cleanup이 더 중요하다.
afterEach(() => { cleanup(); });

describe("useResourceLoader", () => {
  it("loads resources on mount and dispatches RESOURCES_LOADED", async () => {
    vi.spyOn(client, "fetchTrunkMaps").mockResolvedValue(["run_a"]);
    vi.spyOn(client, "fetchBoxPresets").mockResolvedValue({ "기본값": [] });

    render(<PlannerProvider><Harness /></PlannerProvider>);
    await act(async () => {});

    expect(screen.getByTestId("trunk-map").textContent).toBe("run_a");
  });

  it("polls for new trunk maps without resetting the current selection", async () => {
    // 스캔이 백그라운드에서 끝나 새 run_*이 생기는 상황을 재현 - 새로고침 없이도
    // 목록에 추가돼야 하고, 이미 골라둔 trunkMap이 강제로 안 바뀌어야 한다.
    vi.useFakeTimers({ shouldAdvanceTime: true });
    vi.spyOn(client, "fetchTrunkMaps")
      .mockResolvedValueOnce(["run_a"])
      .mockResolvedValue(["run_a", "run_new"]);
    vi.spyOn(client, "fetchBoxPresets").mockResolvedValue({ "기본값": [] });

    render(<PlannerProvider><Harness /></PlannerProvider>);
    await act(async () => {});
    expect(screen.getByTestId("trunk-map").textContent).toBe("run_a");

    await act(async () => {
      await vi.advanceTimersByTimeAsync(3000);
    });

    expect(client.fetchTrunkMaps).toHaveBeenCalledTimes(2);
    // 새 run이 목록에 들어왔어도 사용자가 보고 있던 선택("run_a")은 그대로.
    expect(screen.getByTestId("trunk-map").textContent).toBe("run_a");

    vi.useRealTimers();
  });
});
