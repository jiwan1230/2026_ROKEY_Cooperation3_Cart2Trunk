import { afterEach, describe, expect, it, vi } from "vitest";
import { fetchTrunkMaps, postPlan } from "./client.js";

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
});
