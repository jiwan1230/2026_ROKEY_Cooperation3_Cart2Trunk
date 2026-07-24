import { describe, expect, it } from "vitest";
import { layoutStagingBoxes, toThreeCenter } from "./Scene3DViewer.jsx";

describe("toThreeCenter", () => {
  it("maps our z-up corner coords to three.js y-up center coords", () => {
    expect(toThreeCenter(0, 0, 0, 0.4, 0.3, 0.2)).toEqual([0.2, 0.1, 0.15]);
  });

  it("keeps depth(y) mapped to three.js z axis", () => {
    const [, , threeZ] = toThreeCenter(0, 1.0, 0, 0.2, 0.2, 0.2);
    expect(threeZ).toBeCloseTo(1.1);
  });
});

describe("layoutStagingBoxes", () => {
  const trunk = { width: 1.0, depth: 1.0, height: 0.5, entrance_near_x: true };

  it("places boxes outside the entrance-side face (negative x) when entrance is near x=0", () => {
    const boxes = [{ id: "A", width: 0.3, depth: 0.2, height: 0.15 }];
    const [staged] = layoutStagingBoxes(boxes, trunk);
    expect(staged.position[0]).toBeLessThan(0); // 트렁크 밖(x<0)
  });

  it("places boxes beyond the far face (x > trunk.width) when entrance is near x=width", () => {
    const farTrunk = { ...trunk, entrance_near_x: false };
    const boxes = [{ id: "A", width: 0.3, depth: 0.2, height: 0.15 }];
    const [staged] = layoutStagingBoxes(boxes, farTrunk);
    expect(staged.position[0]).toBeGreaterThan(farTrunk.width);
  });

  it("lines up multiple boxes along y without overlapping", () => {
    const boxes = [
      { id: "A", width: 0.3, depth: 0.2, height: 0.15 },
      { id: "B", width: 0.3, depth: 0.25, height: 0.15 },
    ];
    const staged = layoutStagingBoxes(boxes, trunk);
    const [, aY] = staged[0].position;
    const [, bY] = staged[1].position;
    expect(bY).toBeGreaterThanOrEqual(aY + boxes[0].depth); // 겹치지 않아야 함
  });

  it("returns an empty array when there is no trunk yet", () => {
    expect(layoutStagingBoxes([{ id: "A", width: 0.3, depth: 0.2, height: 0.15 }], undefined)).toEqual([]);
  });
});
