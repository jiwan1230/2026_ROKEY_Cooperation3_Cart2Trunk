import { describe, expect, it } from "vitest";
import {
  countFirstDensityScoreRange, gradeBoxScore, gradeCountFirstDensityScore,
  gradeUtilization, gradeWeightedScore, weightedScoreRange,
} from "./scoreGrading.js";

describe("gradeUtilization", () => {
  // 임계값(22/14/8)은 실제 트렁크 스캔 3종 + 박스 프리셋 4종 + 무작위 생성
  // 30여 회를 직접 계산해서 얻은 실측 분포를 근거로 잡았다 - 100% 배치
  // 성공(팀 "기본값" 프리셋)조차 활용률 17.2%였다는 게 재보정의 근거.
  it("labels high utilization as excellent", () => {
    expect(gradeUtilization(25).label).toBe("우수");
  });
  it("labels mid utilization as good", () => {
    expect(gradeUtilization(17).label).toBe("양호"); // 팀 "기본값" 프리셋 실측치(17.2%)와 같은 구간
  });
  it("labels low-mid utilization as fair", () => {
    expect(gradeUtilization(10).label).toBe("보통");
  });
  it("labels very low utilization as needs improvement", () => {
    expect(gradeUtilization(5).label).toBe("개선 필요");
  });
  it("clamps out-of-range values", () => {
    expect(gradeUtilization(150).pct).toBe(100);
    expect(gradeUtilization(-10).pct).toBe(0);
  });
});

describe("weightedScoreRange", () => {
  it("matches the known range for default preferences (1.0/1.0/1.0)", () => {
    // algorism/05_candidate_scoring.py 가중치로 손으로 계산한 값:
    // best = 0 - 0.5 - 0.9 - 0.2 = -1.6, worst = 1.0 - 0 = 1.0
    const { best, worst } = weightedScoreRange({
      entrancePreference: 1.0, contactPreference: 1.0, heightPreference: 1.0,
    });
    expect(best).toBeCloseTo(-1.6);
    expect(worst).toBeCloseTo(1.0);
  });

  it("flips the wall_a contribution when entrance preference is negative", () => {
    const { best, worst } = weightedScoreRange({
      entrancePreference: -1.0, contactPreference: 1.0, heightPreference: 1.0,
    });
    // entrancePreference<0이면 wall_a_term의 최댓값은 0(ratio=0일 때)이라
    // best 계산에서 wall_a 기여가 사라지고, worst 쪽에 -0.9가 더해진다.
    expect(best).toBeCloseTo(-0.7); // 0 - 0.5 - 0 - 0.2
    expect(worst).toBeCloseTo(1.9); // 1.0 - (-0.9)
  });
});

describe("gradeWeightedScore", () => {
  const defaultPrefs = { entrancePreference: 1.0, contactPreference: 1.0, heightPreference: 1.0 };

  it("grades the best possible score as excellent (100%)", () => {
    const result = gradeWeightedScore(-1.6, defaultPrefs);
    expect(result.pct).toBeCloseTo(100);
    expect(result.label).toBe("우수");
  });

  it("grades the worst possible score as needs improvement (0%)", () => {
    const result = gradeWeightedScore(1.0, defaultPrefs);
    expect(result.pct).toBeCloseTo(0);
    expect(result.label).toBe("개선 필요");
  });

  it("grades a real example score in the middle of the range as good", () => {
    // 실제 계산 결과 예시(XL2, 접촉면 1/6, 벽A 근접): score ≈ -0.460
    const result = gradeWeightedScore(-0.46, defaultPrefs);
    expect(result.pct).toBeCloseTo(56.15, 1);
    expect(result.label).toBe("양호");
  });
});

describe("countFirstDensityScoreRange", () => {
  it("matches the known range for a real trunk size (0.85m x 1.25m x 0.5m)", () => {
    // algorism/05_candidate_scoring.py 가중치로 손으로 계산한 값:
    // best = 0, worst = 1.0 + 5.0*(0.85+1.25) = 11.5
    const { best, worst } = countFirstDensityScoreRange({ width: 0.85, depth: 1.25, height: 0.5 });
    expect(best).toBe(0);
    expect(worst).toBeCloseTo(11.5);
  });

  it("scales the range with trunk footprint size", () => {
    const small = countFirstDensityScoreRange({ width: 0.5, depth: 0.5, height: 0.5 });
    const large = countFirstDensityScoreRange({ width: 1.0, depth: 1.5, height: 0.5 });
    expect(large.worst).toBeGreaterThan(small.worst);
  });
});

describe("gradeCountFirstDensityScore", () => {
  const trunk = { width: 0.85, depth: 1.25, height: 0.5 };

  it("grades the best possible score (0) as excellent", () => {
    expect(gradeCountFirstDensityScore(0, trunk).label).toBe("우수");
  });

  it("grades the worst possible score as needs improvement", () => {
    expect(gradeCountFirstDensityScore(11.5, trunk).label).toBe("개선 필요");
  });

  it("grades a real example score (1.5) as excellent", () => {
    // (11.5-1.5)/11.5 = 86.9% -> 우수
    const result = gradeCountFirstDensityScore(1.5, trunk);
    expect(result.pct).toBeCloseTo(86.96, 1);
    expect(result.label).toBe("우수");
  });
});

describe("gradeBoxScore", () => {
  const trunk = { width: 0.85, depth: 1.25, height: 0.5 };
  const prefs = { entrancePreference: 1.0, contactPreference: 1.0, heightPreference: 1.0 };

  it("routes weighted formula scores to gradeWeightedScore", () => {
    expect(gradeBoxScore("weighted", -1.6, { preferences: prefs, trunk }).label).toBe("우수");
  });

  it("routes count_first_density formula scores to gradeCountFirstDensityScore", () => {
    expect(gradeBoxScore("count_first_density", 1.5, { preferences: prefs, trunk }).label).toBe("우수");
  });

  it("returns null when count_first_density is graded without trunk info", () => {
    expect(gradeBoxScore("count_first_density", 1.5, { preferences: prefs })).toBeNull();
  });
});
