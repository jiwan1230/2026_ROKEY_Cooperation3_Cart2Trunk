import { afterEach, describe, expect, it, vi } from "vitest";
import { cleanup, fireEvent, render, screen } from "@testing-library/react";
import TabBar from "./TabBar.jsx";

afterEach(() => { cleanup(); });

describe("TabBar", () => {
  it("현재 탭에 aria-current를 표시하고, 클릭하면 onSelect에 해당 키를 넘긴다", () => {
    const onSelect = vi.fn();
    render(<TabBar activeTab="simulator" onSelect={onSelect} />);

    expect(screen.getByTestId("tab-simulator").getAttribute("aria-current")).toBe("page");
    expect(screen.getByTestId("tab-robot").getAttribute("aria-current")).toBeNull();

    fireEvent.click(screen.getByTestId("tab-robot"));
    expect(onSelect).toHaveBeenCalledWith("robot");
  });

  it("activeTab이 robot이면 robot 탭에 aria-current가 표시된다", () => {
    render(<TabBar activeTab="robot" onSelect={() => {}} />);
    expect(screen.getByTestId("tab-robot").getAttribute("aria-current")).toBe("page");
    expect(screen.getByTestId("tab-simulator").getAttribute("aria-current")).toBeNull();
  });
});
