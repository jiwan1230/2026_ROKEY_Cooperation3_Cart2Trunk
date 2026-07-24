import { describe, expect, it } from "vitest";
import { toThreeCenter } from "./Scene3DViewer.jsx";

describe("toThreeCenter", () => {
  it("maps our z-up corner coords to three.js y-up center coords", () => {
    expect(toThreeCenter(0, 0, 0, 0.4, 0.3, 0.2)).toEqual([0.2, 0.1, 0.15]);
  });

  it("keeps depth(y) mapped to three.js z axis", () => {
    const [, , threeZ] = toThreeCenter(0, 1.0, 0, 0.2, 0.2, 0.2);
    expect(threeZ).toBeCloseTo(1.1);
  });
});
