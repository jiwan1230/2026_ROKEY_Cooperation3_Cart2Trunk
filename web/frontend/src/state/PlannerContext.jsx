import { createContext, useContext, useReducer } from "react";
import { initialState, plannerReducer } from "./plannerReducer.js";

const PlannerStateContext = createContext(null);
const PlannerDispatchContext = createContext(null);

export function PlannerProvider({ children }) {
  const [state, dispatch] = useReducer(plannerReducer, initialState);
  return (
    <PlannerStateContext.Provider value={state}>
      <PlannerDispatchContext.Provider value={dispatch}>
        {children}
      </PlannerDispatchContext.Provider>
    </PlannerStateContext.Provider>
  );
}

export function usePlannerState() {
  const ctx = useContext(PlannerStateContext);
  if (ctx === null) throw new Error("usePlannerState는 PlannerProvider 안에서만 사용할 수 있습니다");
  return ctx;
}

export function usePlannerDispatch() {
  const ctx = useContext(PlannerDispatchContext);
  if (ctx === null) throw new Error("usePlannerDispatch는 PlannerProvider 안에서만 사용할 수 있습니다");
  return ctx;
}
