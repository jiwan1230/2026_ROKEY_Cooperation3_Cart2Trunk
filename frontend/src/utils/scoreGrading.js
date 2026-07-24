// src/utils/scoreGrading.js
// "공간 활용률/점수가 몇이면 좋은 건지 기준이 없다"는 사용자 피드백에 대한
// 대응 - 두 가지 등급 기준을 만든다.

// algorism/05_candidate_scoring.py의 가중치 상수와 반드시 값이 같아야 한다 -
// 점수 등급을 계산하려면 "이 박스가 나올 수 있는 최선/최악의 점수가 뭔지"를
// 알아야 하는데, 그 계산에 이 가중치들이 그대로 쓰인다.
const HEIGHT_WEIGHT = 1.0;
const CONTACT_WEIGHT = 0.5;
const WALL_A_WEIGHT = 0.9;
const WALL_BC_WEIGHT = 0.2;

function labelForPct(pct) {
  if (pct >= 80) return "우수";
  if (pct >= 50) return "양호";
  if (pct >= 25) return "보통";
  return "개선 필요";
}

// 공간 활용률(%) 등급 - 트렁크 적재는 박스 모양·여백·장애물 때문에 100%
// 활용은 현실적으로 불가능하다. 일반적인 3D 빈패킹 실무 기준을 참고해
// "60% 이상이면 상당히 빽빽하게 채운 것"으로 본다.
export function gradeUtilization(utilizationPct) {
  const pct = Math.max(0, Math.min(100, utilizationPct));
  let label;
  if (pct >= 60) label = "우수";
  else if (pct >= 40) label = "양호";
  else if (pct >= 20) label = "보통";
  else label = "개선 필요";
  return { label, pct };
}

// 지금 우선순위 슬라이더(entrance/contact/height preference) 설정에서
// "weighted" 공식이 낼 수 있는 최선(best, 가장 낮은=가장 좋은)/최악(worst,
// 가장 높은=가장 나쁜) 점수를 계산한다. 슬라이더를 움직이면 점수의 실제
// 범위 자체가 바뀌므로, 고정된 숫자 기준이 아니라 매번 이 범위 안에서
// 상대 위치로 등급을 매긴다 - height_preference/contact_preference는
// 항상 0 이상(음수 미지원, ControlPanel 슬라이더 범위와 동일), entrance_
// preference만 음수(입구 우선)가 가능하므로 부호에 따라 최선/최악이
// 뒤집힐 수 있다는 것까지 반영한다.
export function weightedScoreRange({ entrancePreference, contactPreference, heightPreference }) {
  const kA = WALL_A_WEIGHT * entrancePreference;
  const minWallA = Math.min(kA, 0);
  const maxWallA = Math.max(kA, 0);
  const maxHeight = HEIGHT_WEIGHT * Math.max(heightPreference, 0);
  const maxContact = CONTACT_WEIGHT * Math.max(contactPreference, 0);
  const maxWallBc = WALL_BC_WEIGHT; // 이 항은 preference 배율이 없음(항상 고정)

  const best = 0 - maxContact - maxWallA - maxWallBc; // height_term 최소(0) - 나머지 항 최대
  const worst = maxHeight - minWallA; // contact_term/wall_bc_term의 최소는 항상 0이라 생략
  return { best, worst };
}

// score(낮을수록 좋음)가 [best, worst] 범위 중 어디쯤인지 0~100%로 환산하고
// 등급을 매긴다. "count_first_density" 공식(개수 우선 모드)은 이론적
// 상한(footprint_growth_term)이 트렁크 크기에 따라 달라져 이 함수의 대상이
// 아니다 - weighted 공식에만 적용한다.
export function gradeWeightedScore(score, preferences) {
  const { best, worst } = weightedScoreRange(preferences);
  if (worst <= best) return { label: "우수", pct: 100, best, worst };
  const pct = Math.max(0, Math.min(100, ((worst - score) / (worst - best)) * 100));
  return { label: labelForPct(pct), pct, best, worst };
}
