import { describe, expect, it } from "vitest";
import { colorForBoxId, crc32 } from "./color.js";

describe("colorForBoxId", () => {
  it("is stable for the same id", () => {
    expect(colorForBoxId("Large")).toBe(colorForBoxId("Large"));
  });

  it("returns a valid hex color", () => {
    const color = colorForBoxId("Box1");
    expect(color).toMatch(/^#[0-9a-f]{6}$/);
  });

  it("spreads many ids across mostly distinct colors", () => {
    const ids = Array.from({ length: 20 }, (_, i) => `Box${i}`);
    const colors = new Set(ids.map(colorForBoxId));
    expect(colors.size).toBeGreaterThanOrEqual(15);
  });

  it("matches the backend's Python implementation exactly for the same ids", () => {
    // algorism_bridge.color_for_box_id()를 실제로 실행해서 얻은 값 - 두
    // 구현이 갈라지면(예: 한쪽만 고쳤을 때) Before(프론트가 계산)와
    // After(백엔드가 계산) 색이 달라지므로, 이 테스트로 잠가둔다.
    expect(colorForBoxId("Large")).toBe("#9c54d4");
    expect(colorForBoxId("Box1")).toBe("#2db946");
    expect(colorForBoxId("XL2")).toBe("#45d34c");
    expect(colorForBoxId("A")).toBe("#d28f32");
  });
});

describe("crc32", () => {
  it("is deterministic", () => {
    expect(crc32("Large")).toBe(crc32("Large"));
  });
});
