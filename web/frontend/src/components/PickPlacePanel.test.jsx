import { afterEach, describe, expect, it, vi } from "vitest";
import { act, cleanup, fireEvent, render, screen } from "@testing-library/react";
import PickPlacePanel from "./PickPlacePanel.jsx";
import * as client from "../api/client.js";

afterEach(() => { cleanup(); vi.restoreAllMocks(); });

describe("PickPlacePanel", () => {
  it("대기 중엔 Stop 상태와 '대기 중' 텍스트를 보여준다", () => {
    render(<PickPlacePanel />);
    expect(screen.getByTestId("pick-place-status").textContent).toBe("Stop");
    expect(screen.getByTestId("current-task").textContent).toBe("대기 중");
  });

  it("시작을 누르면 즉시 Run으로 바뀌고 첫 단계 텍스트가 보인다", () => {
    vi.useFakeTimers({ shouldAdvanceTime: true });
    vi.spyOn(client, "postPickAndPlace").mockResolvedValue({ status: "ok", dummy: true, message: "완료" });

    render(<PickPlacePanel />);
    fireEvent.click(screen.getByTestId("trigger-pickAndPlace"));

    expect(screen.getByTestId("pick-place-status").textContent).toBe("Run");
    expect(screen.getByTestId("current-task").textContent).toBe("박스1 pick 접근");
    expect(screen.getByTestId("trigger-pickAndPlace")).toBeDisabled();
    expect(client.postPickAndPlace).toHaveBeenCalledTimes(1);

    vi.useRealTimers();
  });

  it("모든 단계(6개, 700ms 간격)를 다 지나면 완료 텍스트와 함께 Stop으로 돌아온다", async () => {
    vi.useFakeTimers({ shouldAdvanceTime: true });
    vi.spyOn(client, "postPickAndPlace").mockResolvedValue({ status: "ok", dummy: true, message: "완료" });

    render(<PickPlacePanel />);
    fireEvent.click(screen.getByTestId("trigger-pickAndPlace"));

    await act(async () => { await vi.advanceTimersByTimeAsync(700 * 6); });

    expect(screen.getByTestId("current-task").textContent).toBe("완료");
    expect(screen.getByTestId("pick-place-status").textContent).toBe("Stop");
    expect(screen.getByTestId("trigger-pickAndPlace")).not.toBeDisabled();

    vi.useRealTimers();
  });

  it("시작하면 onLog가 시작 메시지로, 끝나면 완료 메시지로 호출된다", async () => {
    vi.useFakeTimers({ shouldAdvanceTime: true });
    vi.spyOn(client, "postPickAndPlace").mockResolvedValue({ status: "ok", dummy: true, message: "완료" });
    const onLog = vi.fn();

    render(<PickPlacePanel onLog={onLog} />);
    fireEvent.click(screen.getByTestId("trigger-pickAndPlace"));
    expect(onLog).toHaveBeenCalledWith("픽앤플레이스 시작");

    await act(async () => { await vi.advanceTimersByTimeAsync(700 * 6); });
    expect(onLog).toHaveBeenCalledWith("픽앤플레이스 완료");

    vi.useRealTimers();
  });
});
