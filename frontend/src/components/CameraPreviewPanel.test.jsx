import { afterEach, describe, expect, it } from "vitest";
import { cleanup, render, screen } from "@testing-library/react";
import CameraPreviewPanel from "./CameraPreviewPanel.jsx";

afterEach(() => { cleanup(); });

describe("CameraPreviewPanel", () => {
  it("카메라 미연동 안내 문구를 보여준다", () => {
    render(<CameraPreviewPanel />);
    expect(screen.getByText("로봇 카메라 실시간")).toBeInTheDocument();
    expect(screen.getByText("카메라 미연동 - 더미 화면")).toBeInTheDocument();
  });
});
