// src/hooks/useDebouncedPlan.js
import { useEffect, useRef } from "react";
import { postPlan } from "../api/client.js";
import { usePlannerDispatch, usePlannerState } from "../state/PlannerContext.jsx";

const DEBOUNCE_MS = 400;

export function useDebouncedPlan() {
  const state = usePlannerState();
  const dispatch = usePlannerDispatch();
  const timerRef = useRef(null);

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
        .then((result) => dispatch({ type: "COMPUTE_SUCCESS", payload: result }))
        .catch((err) =>
          dispatch({
            type: "COMPUTE_ERROR",
            payload: { error_code: err.error_code || "UNKNOWN", cause: err.cause || err.message, action: err.action || "" },
          }),
        );
    }, DEBOUNCE_MS);

    return () => clearTimeout(timerRef.current);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [trunkMap, boxesText, params, boxSourceLabel]);
}
