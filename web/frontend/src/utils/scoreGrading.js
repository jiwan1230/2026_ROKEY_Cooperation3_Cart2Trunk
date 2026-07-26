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
const COUNT_FIRST_HEIGHT_WEIGHT = 1.0;
const COUNT_FIRST_FOOTPRINT_GROWTH_WEIGHT = 5.0;

function labelForPct(pct) {
  if (pct >= 80) return "우수";
  if (pct >= 50) return "양호";
  if (pct >= 25) return "보통";
  return "개선 필요";
}

// 공간 활용률(%) 등급 - 처음엔 일반적인 3D 빈패킹 기준(60%/40%/20%)을
// 그대로 가져다 썼는데, 실제 트렁크 스캔 데이터 3개 + 박스 프리셋 4종 +
// 무작위 생성 30여 회를 직접 계산해보니 터무니없이 빡셌다: 박스를 100%
// 다 실어도(팀이 만든 "기본값" 프리셋조차) 활용률이 7~25% 범위였다 -
// 실제 트렁크는 박스보다 훨씬 커서(직육면체 박스가 서로 안 맞물리고,
// 벽/장애물/박스 사이 마진이 매번 들어가서) 100%에 가까운 활용률 자체가
// 이 알고리즘·이 트렁크 규모에서는 애초에 나올 수가 없다(사용자 피드백으로
// 발견, 직접 재현 후 기준을 다시 잡음). 아래 값은 그 실측 분포(최소 ~7%,
// 중앙값 ~17~20%, 최댓값 ~25~29%)를 근거로 다시 잡은 기준이다.
export function gradeUtilization(utilizationPct) {
  const pct = Math.max(0, Math.min(100, utilizationPct));
  let label;
  if (pct >= 22) label = "우수";
  else if (pct >= 14) label = "양호";
  else if (pct >= 8) label = "보통";
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
// 등급을 매긴다.
export function gradeWeightedScore(score, preferences) {
  const { best, worst } = weightedScoreRange(preferences);
  if (worst <= best) return { label: "우수", pct: 100, best, worst };
  const pct = Math.max(0, Math.min(100, ((worst - score) / (worst - best)) * 100));
  return { label: labelForPct(pct), pct, best, worst };
}

// "count_first_density" 공식(개수 우선 모드)의 최선/최악 범위. 이 공식은
// preference 슬라이더와 무관하게 height_term + footprint_growth_term로만
// 정해진다(algorism/05_candidate_scoring.py의 score_count_first 참고).
// height_term = COUNT_FIRST_HEIGHT_WEIGHT * (z/trunk.height) 는 z가
// [0, trunk.height] 범위라 [0, WEIGHT] 사이. footprint_growth_term은
// growth_x = max(0, (x+box.width)-used_max_x) 형태인데, 박스는 항상 트렁크
// 안에 들어가야 하므로(x+box.width <= trunk.width, used_max_x >= 0)
// growth_x는 최대 trunk.width, 마찬가지로 growth_y는 최대 trunk.depth로
// 상한이 걸린다 - 처음 박스를 놓을 때(이미 차지한 영역이 없을 때, used_max=0)
// 이 상한에 가장 가까워진다. 최소는 두 항 모두 0(이미 차지한 영역과 완전히
// 겹치는 자리, 높이도 바닥)이다. 즉 이 범위는 preference가 아니라 트렁크
// 크기(trunk.width/depth/height)에 따라 달라진다.
export function countFirstDensityScoreRange(trunk) {
  const best = 0;
  const worst = COUNT_FIRST_HEIGHT_WEIGHT
    + COUNT_FIRST_FOOTPRINT_GROWTH_WEIGHT * (trunk.width + trunk.depth);
  return { best, worst };
}

export function gradeCountFirstDensityScore(score, trunk) {
  const { best, worst } = countFirstDensityScoreRange(trunk);
  if (worst <= best) return { label: "우수", pct: 100, best, worst };
  const pct = Math.max(0, Math.min(100, ((worst - score) / (worst - best)) * 100));
  return { label: labelForPct(pct), pct, best, worst };
}

// 박스 하나의 score_breakdown.formula가 뭐든 상관없이 등급을 매겨주는
// 통합 진입점. formula별로 이론적 범위를 구하는 방법이 서로 달라서
// (weighted는 preference 슬라이더 기준, count_first_density는 트렁크
// 크기 기준) 내부에서 분기한다. trunk 정보가 없으면(옛 API 응답 등)
// count_first_density는 등급을 매길 수 없어 null을 반환한다.
export function gradeBoxScore(formula, score, { preferences, trunk } = {}) {
  if (formula === "count_first_density") {
    return trunk ? gradeCountFirstDensityScore(score, trunk) : null;
  }
  if (formula === "weighted") {
    return preferences ? gradeWeightedScore(score, preferences) : null;
  }
  return null;
}
