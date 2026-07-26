import { afterEach, describe, expect, it, vi } from "vitest";
import { cleanup, fireEvent, render, screen, waitFor } from "@testing-library/react";
import { PlannerProvider, usePlannerState } from "../state/PlannerContext.jsx";
import VisionDataLoader from "./VisionDataLoader.jsx";
import * as client from "../api/client.js";

afterEach(() => { cleanup(); vi.restoreAllMocks(); });

function StateProbe() {
  const state = usePlannerState();
  return (
    <div>
      <div data-testid="boxes-text">{state.boxesText}</div>
      <div data-testid="snapshot-id">{state.boxSnapshotId ?? ""}</div>
      <div data-testid="error">{state.error ? state.error.error_code : ""}</div>
    </div>
  );
}

function uploadFile(contentObject, name = "data.json") {
  const file = new File([JSON.stringify(contentObject)], name, { type: "application/json" });
  const input = screen.getByTestId("vision-file-input");
  fireEvent.change(input, { target: { files: [file] } });
}

describe("VisionDataLoader", () => {
  it("detects box_scan.json format and loads the parsed boxes with the real snapshot id", async () => {
    vi.spyOn(client, "postParseBoxScan").mockResolvedValue({
      boxes: [{ id: "BOX_01", width: 0.3, depth: 0.2, height: 0.15, rests_on_id: null, initial_yaw: 0 }],
      snapshot_id: "box_scan_001",
    });

    render(<PlannerProvider><VisionDataLoader /><StateProbe /></PlannerProvider>);
    uploadFile({ snapshot_id: "box_scan_001", frame_id: "m0609_base_link", boxes: [] });

    await waitFor(() => expect(screen.getByTestId("snapshot-id").textContent).toBe("box_scan_001"));
    expect(client.postParseBoxScan).toHaveBeenCalledTimes(1);
    expect(JSON.parse(screen.getByTestId("boxes-text").textContent)).toHaveLength(1);
  });

  it("detects all_boxes_corners_*.json format (corners_m field) and calls the corners parser", async () => {
    vi.spyOn(client, "postParseVisionCorners").mockResolvedValue({
      boxes: [{ id: "0", width: 0.5, depth: 0.35, height: 0.2, rests_on_id: null }],
      snapshot_id: "run_test.ply",
    });

    render(<PlannerProvider><VisionDataLoader /><StateProbe /></PlannerProvider>);
    uploadFile({
      coordinate_frame: "m0609_base_link",
      boxes: [{ box_id: 0, corners_m: [[0, 0, 0]] }],
    });

    await waitFor(() => expect(screen.getByTestId("snapshot-id").textContent).toBe("run_test.ply"));
    expect(client.postParseVisionCorners).toHaveBeenCalledTimes(1);
  });

  it("shows a clear error when the file is not valid JSON", async () => {
    render(<PlannerProvider><VisionDataLoader /><StateProbe /></PlannerProvider>);
    const input = screen.getByTestId("vision-file-input");
    const badFile = new File(["not json"], "broken.json", { type: "application/json" });
    fireEvent.change(input, { target: { files: [badFile] } });

    await waitFor(() => expect(screen.getByTestId("error").textContent).toBe("VISION_FILE_JSON_INVALID"));
  });

  it("shows a clear error when the file matches neither known schema", async () => {
    render(<PlannerProvider><VisionDataLoader /><StateProbe /></PlannerProvider>);
    uploadFile({ some: "unrelated", data: true });

    await waitFor(() => expect(screen.getByTestId("error").textContent).toBe("VISION_FORMAT_UNKNOWN"));
  });

  it("surfaces the backend's frame-mismatch error as-is (does not silently guess a transform)", async () => {
    const err = new Error("frame mismatch");
    err.error_code = "VISION_FRAME_MISMATCH";
    err.cause = "박스 비전 데이터의 좌표계가 'camera_frame'인데 'm0609_base_link'이어야 합니다";
    err.action = "Vision(준형)/시스템통합(지완)과 좌표계를 맞춘 뒤 다시 시도하세요.";
    vi.spyOn(client, "postParseBoxScan").mockRejectedValue(err);

    render(<PlannerProvider><VisionDataLoader /><StateProbe /></PlannerProvider>);
    uploadFile({ snapshot_id: "x", frame_id: "camera_frame", boxes: [] });

    await waitFor(() => expect(screen.getByTestId("error").textContent).toBe("VISION_FRAME_MISMATCH"));
  });
});
