// src/hooks/useResourceLoader.js
import { useEffect } from "react";
import { fetchBoxPresets, fetchTrunkMaps } from "../api/client.js";
import { usePlannerDispatch } from "../state/PlannerContext.jsx";

export function useResourceLoader() {
  const dispatch = usePlannerDispatch();

  useEffect(() => {
    Promise.all([fetchTrunkMaps(), fetchBoxPresets()])
      .then(([trunkMaps, boxPresets]) => {
        dispatch({ type: "RESOURCES_LOADED", payload: { trunkMaps, boxPresets } });
      })
      .catch((err) => {
        dispatch({
          type: "COMPUTE_ERROR",
          payload: {
            error_code: err.error_code || "RESOURCE_LOAD_FAILED",
            cause: err.cause || err.message,
            action: "백엔드(localhost:5000)가 실행 중인지 확인한 뒤 페이지를 새로고침하세요.",
          },
        });
      });
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);
}
