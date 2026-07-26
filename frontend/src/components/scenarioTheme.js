// src/components/scenarioTheme.js
// 산업현장 시나리오 4개의 메타데이터 + 3D 뷰어 테마 색상 - 순수 함수라
// Scene3DViewer.jsx(<Canvas> 포함, jsdom 미검증)와 분리해서 여기서 직접
// 테스트한다. id는 백엔드 routes/scenarios.py의 SCENARIO_DEFS 키와 정확히
// 일치해야 한다.
export const SCENARIOS = [
  { id: "delivery_truck", label: "택배 배송 트럭" },
  { id: "warehouse", label: "창고/물류센터" },
  { id: "cold_chain", label: "냉동/냉장 물류" },
  { id: "hazmat", label: "위험물 창고" },
];

const TRUNK_COLORS = {
  delivery_truck: "#8A8F98",
  warehouse: "#F2C94C",
  cold_chain: "#2D9CDB",
  hazmat: "#F2994A",
};

export function scenarioTrunkColor(scenarioId) {
  return TRUNK_COLORS[scenarioId] || "#B8B8C4";
}

export function scenarioBoxColor(scenarioId, boxId) {
  if (scenarioId === "warehouse") {
    return boxId.startsWith("대") ? "#2F80ED" : "#F2C94C";
  }
  if (scenarioId === "hazmat") {
    if (boxId.startsWith("산화제")) return "#F2994A";
    if (boxId.startsWith("인화물")) return "#EB5757";
    return "#9CA3AF";
  }
  if (scenarioId === "cold_chain") return "#56CCF2";
  if (scenarioId === "delivery_truck") return "#5A6472";
  return "#4A90D9";
}
