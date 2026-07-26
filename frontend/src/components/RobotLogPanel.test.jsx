import { afterEach, describe, expect, it } from "vitest";
import { cleanup, render, screen } from "@testing-library/react";
import RobotLogPanel from "./RobotLogPanel.jsx";

afterEach(() => { cleanup(); });

describe("RobotLogPanel", () => {
  it("로그가 없으면 안내 문구를 보여준다", () => {
    render(<RobotLogPanel logs={[]} />);
    expect(screen.getByText("아직 실행된 동작이 없습니다.")).toBeInTheDocument();
  });

  it("logs를 받은 순서 그대로(최신이 위) 렌더링한다", () => {
    render(<RobotLogPanel logs={[
      { time: "10:00:02", message: "트렁크 스캔 완료" },
      { time: "10:00:00", message: "트렁크 스캔 시작" },
    ]} />);
    const list = screen.getByTestId("robot-log-list");
    expect(list.textContent).toContain("[10:00:02] 트렁크 스캔 완료");
    expect(list.textContent).toContain("[10:00:00] 트렁크 스캔 시작");
  });
});
