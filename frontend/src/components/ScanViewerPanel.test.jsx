import { afterEach, describe, expect, it, vi } from "vitest";
import { cleanup, fireEvent, render, screen, waitFor } from "@testing-library/react";
import ScanViewerPanel from "./ScanViewerPanel.jsx";
import * as client from "../api/client.js";

// <Canvas>는 실제 WebGL이 필요해서 jsdom에서 직접 렌더링하지 않는 프로젝트
// 관례(Scene3DViewer.jsx)를 따르되, 이 컴포넌트의 상태 전이/토글 로직은
// r3f 프리미티브를 간단한 div로 대체해서 검증한다.
vi.mock("@react-three/fiber", () => ({ Canvas: ({ children }) => <div data-testid="canvas">{children}</div> }));
vi.mock("@react-three/drei", () => ({ Grid: () => null, OrbitControls: () => null }));
vi.mock("./sceneMeshes.jsx", () => ({
  toThreeCenter: () => [0, 0, 0],
  TrunkWireframe: () => <div data-testid="trunk-mesh" />,
  CartWireframe: () => <div data-testid="cart-mesh" />,
  SceneBoxMesh: () => <div data-testid="box-mesh" />,
  BoundingBoxWireframe: () => <div data-testid="raw-mesh" />,
  layoutStagingBoxes: () => [{ id: "Large", position: [0, 0, 0], dimensions: [0.3, 0.3, 0.2] }],
  computeCartFootprint: () => ({ minX: 0, maxX: 1, minY: 0, maxY: 1, height: 0.5 }),
}));

afterEach(() => { cleanup(); vi.restoreAllMocks(); });

describe("ScanViewerPanel", () => {
  it("완료 전에는 3D 콘텐츠가 안 보이다가, 완료되면 기본(원본) 콘텐츠가 나타난다", async () => {
    vi.spyOn(client, "postTrunkScan").mockResolvedValue({ status: "ok", dummy: true, message: "트렁크 스캔 완료" });

    render(<ScanViewerPanel kind="trunk" />);
    expect(screen.getByTestId("status-trunk").textContent).toBe("대기");
    expect(screen.queryByTestId("raw-mesh")).toBeNull();
    expect(screen.queryByTestId("trunk-mesh")).toBeNull();

    fireEvent.click(screen.getByTestId("trigger-trunk"));
    expect(screen.getByTestId("status-trunk").textContent).toBe("진행중");
    expect(screen.getByTestId("trigger-trunk")).toBeDisabled();

    await waitFor(() => expect(screen.getByTestId("status-trunk").textContent).toBe("완료"));
    expect(screen.getByTestId("raw-mesh")).toBeInTheDocument();
    expect(screen.queryByTestId("trunk-mesh")).toBeNull();
  });

  it("전처리 토글을 누르면 완료 후 디테일 렌더링(TrunkWireframe)으로 바뀐다", async () => {
    vi.spyOn(client, "postTrunkScan").mockResolvedValue({ status: "ok", dummy: true, message: "완료" });

    render(<ScanViewerPanel kind="trunk" />);
    fireEvent.click(screen.getByTestId("trigger-trunk"));
    await waitFor(() => expect(screen.getByTestId("status-trunk").textContent).toBe("완료"));

    fireEvent.click(screen.getByTestId("trunk-viewmode-processed"));
    expect(screen.getByTestId("trunk-mesh")).toBeInTheDocument();
    expect(screen.queryByTestId("raw-mesh")).toBeNull();
  });

  it("카트 칸은 전처리 모드에서 CartWireframe와 박스 메쉬를 함께 보여준다", async () => {
    vi.spyOn(client, "postCartScan").mockResolvedValue({ status: "ok", dummy: true, message: "완료" });

    render(<ScanViewerPanel kind="cart" />);
    fireEvent.click(screen.getByTestId("trigger-cart"));
    await waitFor(() => expect(screen.getByTestId("status-cart").textContent).toBe("완료"));

    fireEvent.click(screen.getByTestId("cart-viewmode-processed"));
    expect(screen.getByTestId("cart-mesh")).toBeInTheDocument();
    expect(screen.getByTestId("box-mesh")).toBeInTheDocument();
  });

  it("요청이 실패하면 상태가 대기로 돌아간다", async () => {
    vi.spyOn(client, "postCartScan").mockRejectedValue(new Error("네트워크 오류"));

    render(<ScanViewerPanel kind="cart" />);
    fireEvent.click(screen.getByTestId("trigger-cart"));
    await waitFor(() => expect(screen.getByTestId("status-cart").textContent).toBe("대기"));
  });
});
