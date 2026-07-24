// src/hooks/useDebouncedPlan.js
import { useEffect, useRef } from "react";
import { postPlan } from "../api/client.js";
import { usePlannerDispatch, usePlannerState } from "../state/PlannerContext.jsx";

const DEBOUNCE_MS = 400;

export function useDebouncedPlan() {
  const state = usePlannerState();
  const dispatch = usePlannerDispatch();
  const timerRef = useRef(null);
  // 디바운스 만료 후에도 postPlan()이 진행 중인 상태에서 사용자가 파라미터를
  // 또 바꾸면, 새 요청이 하나 더 발화한다 - 두 요청 중 나중에 시작한(더 최신
  // 파라미터 기준) 쪽이 먼저 끝난다는 보장이 없어서, 응답이 도착한 순서대로
  // dispatch하면 오래된 결과가 최신 결과를 덮어쓸 수 있다("실시간 자동
  // 재계산"의 핵심을 깨는 경쟁 조건 - 리뷰에서 발견됨). 매 발화마다 세대
  // 번호를 하나씩 늘리고, 응답이 도착했을 때 그게 여전히 "가장 최근에 보낸
  // 요청"인지 확인해서 아니면 조용히 버린다.
  const requestIdRef = useRef(0);

  const { trunkMap, boxesText, params, boxSourceLabel } = state;

  useEffect(() => {
    if (!trunkMap) return undefined;

    let parsedBoxes;
    try {
      parsedBoxes = JSON.parse(boxesText);
    } catch {
      return undefined; // 박스 JSON이 아직 타이핑 중이라 문법이 깨진 상태 - 조용히 대기
    }
    if (!Array.isArray(parsedBoxes)) return undefined;

    if (timerRef.current) clearTimeout(timerRef.current);
    timerRef.current = setTimeout(() => {
      const requestId = ++requestIdRef.current;
      dispatch({ type: "COMPUTE_START" });
      postPlan({
        trunk_map: trunkMap,
        boxes: parsedBoxes,
        box_source_label: boxSourceLabel,
        mode: params.mode,
        margin: params.margin === "" ? null : Number(params.margin),
        wall_margin: params.wallMargin === "" ? null : Number(params.wallMargin),
        obstacle_margin: params.obstacleMargin === "" ? null : Number(params.obstacleMargin),
        ceiling_margin: params.ceilingMargin === "" ? null : Number(params.ceilingMargin),
        entrance_margin: params.entranceMargin === "" ? null : Number(params.entranceMargin),
        entrance_preference: params.entrancePreference,
        contact_preference: params.contactPreference,
        height_preference: params.heightPreference,
        allow_stacking: params.allowStacking,
        allow_rotation: params.allowRotation,
        fixed_order: params.fixedOrder,
      })
        .then((result) => {
          if (requestId === requestIdRef.current) {
            dispatch({ type: "COMPUTE_SUCCESS", payload: result });
          } // else: 더 최신 요청이 이미 발화됐으므로 이 응답은 조용히 버림
        })
        .catch((err) => {
          if (requestId === requestIdRef.current) {
            dispatch({
              type: "COMPUTE_ERROR",
              payload: { error_code: err.error_code || "UNKNOWN", cause: err.cause || err.message, action: err.action || "" },
            });
          }
        });
    }, DEBOUNCE_MS);

    return () => clearTimeout(timerRef.current);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [trunkMap, boxesText, params, boxSourceLabel]);
}
