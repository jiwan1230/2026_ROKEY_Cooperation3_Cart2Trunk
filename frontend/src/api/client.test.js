import { afterEach, describe, expect, it, vi } from "vitest";
import { fetchTrunkMaps, postPlan, postCartScan, postTrunkScan, postPickAndPlace } from "./client.js";

afterEach(() => {
  vi.unstubAllGlobals();
});

describe("api client", () => {
  it("fetchTrunkMaps returns the trunk_maps array from the response", async () => {
    vi.stubGlobal("fetch", vi.fn().mockResolvedValue({
      ok: true, json: async () => ({ trunk_maps: ["run_a"] }),
    }));
    const maps = await fetchTrunkMaps();
    expect(maps).toEqual(["run_a"]);
  });

  it("postPlan throws an error carrying error_code/cause/action on failure", async () => {
    vi.stubGlobal("fetch", vi.fn().mockResolvedValue({
      ok: false,
      json: async () => ({ error_code: "BOX_JSON_INVALID", cause: "잘못된 형식", action: "고치세요" }),
    }));
    await expect(postPlan({})).rejects.toMatchObject({ error_code: "BOX_JSON_INVALID" });
  });

  it("postCartScan resolves with the dummy success payload", async () => {
    vi.stubGlobal("fetch", vi.fn().mockResolvedValue({
      ok: true, json: async () => ({ status: "ok", dummy: true, message: "카트 스캔 완료 (더미)" }),
    }));
    const result = await postCartScan();
    expect(result).toEqual({ status: "ok", dummy: true, message: "카트 스캔 완료 (더미)" });
  });

  it("postTrunkScan posts to /api/robot/trunk-scan", async () => {
    const fetchMock = vi.fn().mockResolvedValue({
      ok: true, json: async () => ({ status: "ok", dummy: true, message: "트렁크 스캔 완료 (더미)" }),
    });
    vi.stubGlobal("fetch", fetchMock);
    await postTrunkScan();
    expect(fetchMock).toHaveBeenCalledWith("/api/robot/trunk-scan", expect.objectContaining({ method: "POST" }));
  });

  it("postPickAndPlace throws an error carrying error_code on failure", async () => {
    vi.stubGlobal("fetch", vi.fn().mockResolvedValue({
      ok: false,
      json: async () => ({ error_code: "ROBOT_TRIGGER_FAILED", cause: "실패", action: "재시도하세요" }),
    }));
    await expect(postPickAndPlace()).rejects.toMatchObject({ error_code: "ROBOT_TRIGGER_FAILED" });
  });
});
