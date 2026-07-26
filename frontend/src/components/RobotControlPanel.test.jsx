import { afterEach, describe, expect, it, vi } from "vitest";
import { cleanup, fireEvent, render, screen, waitFor } from "@testing-library/react";
import RobotControlPanel from "./RobotControlPanel.jsx";
import * as client from "../api/client.js";

afterEach(() => { cleanup(); vi.restoreAllMocks(); });

describe("RobotControlPanel", () => {
  it("클릭하면 상태가 대기->진행중->완료로 바뀌고 로그가 쌓인다", async () => {
    vi.spyOn(client, "postCartScan").mockResolvedValue({
      status: "ok", dummy: true, message: "카트 스캔 완료 (더미 - 실제 로봇 미연동)",
    });

    render(<RobotControlPanel />);
    expect(screen.getByTestId("status-cartScan").textContent).toBe("대기");

    fireEvent.click(screen.getByTestId("trigger-cartScan"));
    expect(screen.getByTestId("status-cartScan").textContent).toBe("진행중");
    expect(screen.getByTestId("trigger-cartScan")).toBeDisabled();

    await waitFor(() => expect(screen.getByTestId("status-cartScan").textContent).toBe("완료"));
    expect(screen.getByTestId("robot-log-list").textContent)
      .toContain("카트 스캔 완료 (더미 - 실제 로봇 미연동)");
  });

  it("3개 버튼은 서로 독립적으로 상태를 갖는다", async () => {
    vi.spyOn(client, "postTrunkScan").mockResolvedValue({
      status: "ok", dummy: true, message: "트렁크 스캔 완료 (더미)",
    });

    render(<RobotControlPanel />);
    fireEvent.click(screen.getByTestId("trigger-trunkScan"));
    await waitFor(() => expect(screen.getByTestId("status-trunkScan").textContent).toBe("완료"));

    expect(screen.getByTestId("status-cartScan").textContent).toBe("대기");
    expect(screen.getByTestId("status-pickAndPlace").textContent).toBe("대기");
  });

  it("요청이 실패하면 상태가 대기로 돌아가고 오류 로그가 남는다", async () => {
    vi.spyOn(client, "postPickAndPlace").mockRejectedValue(new Error("네트워크 오류"));

    render(<RobotControlPanel />);
    fireEvent.click(screen.getByTestId("trigger-pickAndPlace"));
    await waitFor(() => expect(screen.getByTestId("status-pickAndPlace").textContent).toBe("대기"));
    expect(screen.getByTestId("robot-log-list").textContent).toContain("픽앤플레이스 시작 요청 실패");
  });

  it("버튼을 다시 눌러서 반복 실행할 수 있다 (순서 강제 없음)", async () => {
    vi.spyOn(client, "postCartScan").mockResolvedValue({
      status: "ok", dummy: true, message: "카트 스캔 완료 (더미)",
    });

    render(<RobotControlPanel />);
    fireEvent.click(screen.getByTestId("trigger-cartScan"));
    await waitFor(() => expect(screen.getByTestId("status-cartScan").textContent).toBe("완료"));

    fireEvent.click(screen.getByTestId("trigger-cartScan"));
    expect(screen.getByTestId("status-cartScan").textContent).toBe("진행중");
    await waitFor(() => expect(client.postCartScan).toHaveBeenCalledTimes(2));
  });
});
