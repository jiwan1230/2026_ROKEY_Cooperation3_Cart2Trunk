import { describe, expect, it } from "vitest";
import { scenarioBoxColor, scenarioTrunkColor, SCENARIOS } from "./scenarioTheme.js";

describe("SCENARIOS", () => {
  it("4개의 시나리오 id/label을 갖는다", () => {
    expect(SCENARIOS.map((s) => s.id)).toEqual(["delivery_truck", "warehouse", "cold_chain", "hazmat"]);
  });
});

describe("scenarioTrunkColor", () => {
  it("시나리오마다 서로 다른 고정 색을 반환한다", () => {
    const colors = SCENARIOS.map((s) => scenarioTrunkColor(s.id));
    expect(new Set(colors).size).toBe(4);
  });
});

describe("scenarioBoxColor", () => {
  it("창고 시나리오는 박스 id 접두사(소/대)로 색을 나눈다", () => {
    expect(scenarioBoxColor("warehouse", "소0")).toBe("#F2C94C");
    expect(scenarioBoxColor("warehouse", "대1")).toBe("#2F80ED");
  });

  it("위험물 시나리오는 hazard 종류별로 다른 경고색을 쓴다", () => {
    expect(scenarioBoxColor("hazmat", "산화제_드럼1")).toBe("#F2994A");
    expect(scenarioBoxColor("hazmat", "인화물_드럼1")).toBe("#EB5757");
    expect(scenarioBoxColor("hazmat", "일반박스1")).toBe("#9CA3AF");
  });

  it("택배/냉동 시나리오는 박스 id와 무관하게 단일 테마색을 쓴다", () => {
    expect(scenarioBoxColor("delivery_truck", "정류장1_박스")).toBe("#5A6472");
    expect(scenarioBoxColor("delivery_truck", "정류장4_박스")).toBe("#5A6472");
    expect(scenarioBoxColor("cold_chain", "냉동박스0")).toBe("#56CCF2");
  });
});
