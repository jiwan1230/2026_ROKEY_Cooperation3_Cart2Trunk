import { describe, expect, it } from "vitest";
import { render, screen } from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import { PlannerProvider, usePlannerDispatch, usePlannerState } from "../state/PlannerContext.jsx";
import Header from "./Header.jsx";

function StateProbe() {
  const state = usePlannerState();
  return <div data-testid="plan-state">{state.planState}</div>;
}

// planState를 초기값(NOT_COMPUTED)이 아닌 다른 값으로 미리 바꿔두기 위한
// 테스트 하네스. 이게 없으면 EMERGENCY STOP 버튼이 아무 동작도 안 해도
// planState가 이미 NOT_COMPUTED라 테스트가 거짓으로 통과한다(BoxDetailPanel.test.jsx,
// LogPanel.test.jsx의 Loader 패턴과 동일).
function Loader() {
  const dispatch = usePlannerDispatch();
  return (
    <button onClick={() => dispatch({ type: "APPROVE_SUCCESS", payload: { plan_id: "x", task: {} } })}>
      approve
    </button>
  );
}

describe("Header", () => {
  it("emergency stop button dispatches EMERGENCY_STOP", async () => {
    render(
      <PlannerProvider>
        <Header />
        <Loader />
        <StateProbe />
      </PlannerProvider>,
    );

    // 먼저 planState를 NOT_COMPUTED가 아닌 상태(APPROVED)로 만든다.
    await userEvent.click(screen.getByText("approve"));
    expect(screen.getByTestId("plan-state").textContent).toBe("APPROVED");

    // EMERGENCY STOP을 누르면 실제로 NOT_COMPUTED로 바뀌어야 한다.
    await userEvent.click(screen.getByText("EMERGENCY STOP"));
    expect(screen.getByTestId("plan-state").textContent).toBe("NOT_COMPUTED");
  });
});
