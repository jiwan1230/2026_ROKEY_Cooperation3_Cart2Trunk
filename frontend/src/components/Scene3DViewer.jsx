import { useEffect, useMemo, useRef, useState } from "react";
import { Canvas } from "@react-three/fiber";
import { Grid, OrbitControls } from "@react-three/drei";
import { usePlannerState } from "../state/PlannerContext.jsx";
import { colorForBoxId } from "../utils/color.js";
import { postScenarioPlan } from "../api/client.js";
import {
  toThreeCenter, TrunkWireframe, computeCartFootprint, CartWireframe, SceneBoxMesh, layoutStagingBoxes,
  BoundingBoxWireframe,
} from "./sceneMeshes.jsx";
import { SCENARIOS, scenarioTrunkColor, scenarioBoxColor } from "./scenarioTheme.js";
import styles from "./Scene3DViewer.module.css";

const STEP_DELAY_MS = 700; // "순서대로 재생"에서 박스 하나가 나타나고 다음 박스까지 기다리는 시간

// 브라우저 창을 DPI(픽셀 밀도)가 다른 모니터로 옮기면, react-three-fiber의
// WebGL 캔버스가 예전 모니터 기준 해상도로 고정된 채 안 바뀌어서 화면이
// 흐려지거나 깨지는 문제가 있다(사용자 리포트, 듀얼 모니터 환경) - 이건 이
// 라이브러리의 알려진 동작이다: 엘리먼트 "크기"가 바뀔 때는 ResizeObserver로
// 알아서 다시 그리지만, 크기는 그대로인데 devicePixelRatio만 바뀌는 경우는
// 별도로 감지하지 않는다. matchMedia로 devicePixelRatio 변화를 직접 감지해서
// Canvas를 강제로 새로 마운트(key로 트리거)하면 새 모니터 기준으로 WebGL
// 컨텍스트가 다시 만들어진다.
function useDevicePixelRatio() {
  const [dpr, setDpr] = useState(() => (typeof window !== "undefined" ? window.devicePixelRatio || 1 : 1));

  useEffect(() => {
    if (typeof window === "undefined" || !window.matchMedia) return undefined;
    const mediaQuery = window.matchMedia(`(resolution: ${dpr}dppx)`);
    const handleChange = () => setDpr(window.devicePixelRatio || 1);
    mediaQuery.addEventListener("change", handleChange);
    return () => mediaQuery.removeEventListener("change", handleChange);
  }, [dpr]);

  return dpr;
}

const CAMERA_PRESETS = {
  // "front"은 완전한 정면(x축 일직선)이 아니라 살짝 대각선 위에서 내려다보는
  // 각도로 잡는다 - 완전 정면이면 트렁크 입구 밖에 대기 중인 박스(음수 x
  // 쪽)가 트렁크 자체에 정확히 가려져서 안 보이는 문제가 있었다(사용자
  // 피드백으로 발견). 카메라 거리는 원래 값의 70% - 뷰어 화면을 크게 키웠더니
  // (420px 고정 -> 75vh) 모델이 화면 안에서 상대적으로 작아 보인다는 피드백을
  // 받아 좀 더 당겨 찍었다. 이후 대기 박스 영역을 감싸는 카트 모양(벽+바퀴+
  // 손잡이)까지 추가되면서 씬 전체 폭이 넓어져, 카트+트렁크가 모두 프레임
  // 안에 들어오도록 트렁크만 있을 때보다 20% 더 멀리서 잡는다.
  front: { position: [0.27, 1.5, 2.75], target: [0.1, 0, 0.4] },
  side: { position: [0.01, 1.26, 2.52], target: [0, 0, 0] },
  top: { position: [0.01, 3.36, 0.01], target: [0, 0, 0] },
};

export default function Scene3DViewer() {
  const state = usePlannerState();

  // 산업현장 시나리오 미리보기 - state.result(지금 작업 중인 계획)는 전혀
  // 건드리지 않는 완전히 별도의 로컬 상태다. 활성화되면 effectiveResult가
  // scenarioResult를 가리키게 되어, 아래 Before/After・카트 적재・순서대로
  // 재생 로직을 실시간 계획과 완전히 동일하게 재사용한다("실시간 계획하고
  // 똑같이 해줘야지"라는 사용자 피드백 반영 - 처음엔 별도의 단순 렌더링
  // 경로를 따로 만들었었는데, 카트/Before/재생이 전부 빠져서 부족했다).
  const [activeScenarioId, setActiveScenarioId] = useState(null);
  const [scenarioResult, setScenarioResult] = useState(null);
  const [scenarioError, setScenarioError] = useState(null);
  const [scenarioLoading, setScenarioLoading] = useState(false);

  const handleSelectScenario = async (id) => {
    setScenarioLoading(true);
    setScenarioError(null);
    try {
      const result = await postScenarioPlan(id);
      setActiveScenarioId(id);
      setScenarioResult(result);
    } catch (err) {
      setScenarioError(err.cause || err.message || "시나리오를 불러오지 못했습니다.");
    } finally {
      setScenarioLoading(false);
    }
  };

  const handleExitScenario = () => {
    setActiveScenarioId(null);
    setScenarioResult(null);
    setScenarioError(null);
  };

  // ControlPanel의 "무작위 N개 생성"(또는 프리셋 선택, 직접 수정)은
  // state.boxesText를 바꾸는데, 시나리오 미리보기 중엔 그게 화면에 전혀
  // 반영이 안 됐다("무작위 생성이 안 먹힌다"는 피드백) - boxesText가
  // 바뀌면 사용자가 실시간 계획 쪽을 다시 만지고 있다는 뜻이므로, 시나리오
  // 미리보기를 자동으로 해제하고 실시간 화면으로 돌아간다.
  const prevBoxesTextRef = useRef(state.boxesText);
  useEffect(() => {
    if (state.boxesText !== prevBoxesTextRef.current) {
      prevBoxesTextRef.current = state.boxesText;
      if (activeScenarioId) handleExitScenario();
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [state.boxesText]);

  // 시나리오가 활성화되어 있으면 이후 모든 파생 상태(trunk/placed/카트
  // 적재/Before-After/순서대로 재생)가 scenarioResult 기준으로 계산된다 -
  // state.result 기준 코드를 두 번 만들지 않고 그대로 재사용한다.
  const effectiveResult = activeScenarioId ? scenarioResult : state.result;
  const boxColor = (boxId) => (activeScenarioId ? scenarioBoxColor(activeScenarioId, boxId) : colorForBoxId(boxId));

  const controlsRef = useRef(null);
  const [preset, setPreset] = useState("front");
  // tkinter GUI의 Before/After 정적 이미지 대신, 같은 3D 씬을 그대로 두고
  // "적재된 박스를 보여줄지"만 토글한다 - Before는 트렁크(+ 장애물)만 빈
  // 상태로 보여준다(카트 박스는 적재 전엔 트렁크 안 실제 좌표가 없으므로).
  const [stage, setStage] = useState("after"); // "before" | "after"
  const dpr = useDevicePixelRatio();

  const placed = effectiveResult?.placed || [];
  // "순서대로 재생" - visibleCount만큼만(order 순서대로) 박스를 보여준다.
  // null이면 "재생 안 함" 상태로, 전부 다 보여준다(기본 동작).
  const [visibleCount, setVisibleCount] = useState(null);
  const [animating, setAnimating] = useState(false);

  // 새로 계산된 결과가 들어오면(실시간 계획이든 시나리오든) 재생 상태를
  // 초기화하고 전부 다 보여주는 기본 상태로 되돌린다 - 이전 재생의 중간
  // 상태가 새 결과에 남아있으면 안 됨.
  useEffect(() => {
    setAnimating(false);
    setVisibleCount(null);
  }, [effectiveResult]);

  useEffect(() => {
    if (!animating) return undefined;
    if (visibleCount >= placed.length) {
      setAnimating(false);
      return undefined;
    }
    const timer = setTimeout(() => setVisibleCount((c) => c + 1), STEP_DELAY_MS);
    return () => clearTimeout(timer);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [animating, visibleCount, placed.length]);

  const handlePlayStepByStep = () => {
    if (placed.length === 0) return;
    setVisibleCount(0);
    setAnimating(true);
  };

  const visiblePlaced = visibleCount === null ? placed : placed.slice(0, visibleCount);

  useEffect(() => {
    const controls = controlsRef.current;
    if (!controls) return;
    const { position, target } = CAMERA_PRESETS[preset];
    controls.object.position.set(...position);
    controls.target.set(...target);
    controls.update();
  }, [preset]);

  const trunk = effectiveResult?.trunk;
  const showPlaced = stage === "after";

  // 대기 박스 크기 조회용 전체 박스 목록. 실시간 계획은 state.boxesText를
  // 파싱(타이핑 도중이라 문법이 깨져 있을 수 있으므로 실패하면 조용히 빈
  // 배열로 취급 - useDebouncedPlan.js와 같은 방어 방식), 시나리오는
  // scenarioResult.boxes(백엔드가 이미 {id,width,depth,height} shape로 줌)를
  // 그대로 쓴다.
  const inputBoxes = useMemo(() => {
    if (activeScenarioId) return scenarioResult?.boxes || [];
    try {
      const parsed = JSON.parse(state.boxesText);
      return Array.isArray(parsed) ? parsed : [];
    } catch {
      return [];
    }
  }, [activeScenarioId, scenarioResult, state.boxesText]);

  // layoutStagingBoxes는 칸마다 "먼저 들어온 박스가 바닥, 나중에 들어온
  // 박스가 위"로 쌓는다 - 그래서 넘겨주는 순서가 곧 "쌓이는 순서"다.
  // [사용자 피드백] "박스 개수를 늘리니 순서대로 적재할 때 갑자기 1층(바닥)
  // 박스를 먼저 집는 물리적으로 말이 안 되는 장면"이 나왔다 - 원인은
  // fullCartLayout을 항상 입력 목록 순서(inputBoxes) 그대로 쌓았기 때문에,
  // 트렁크 적재 순서(placed[].order, 큰 것 우선 등 전략에 따라 입력 순서와
  // 무관하게 정해짐)와 카트에서 쌓인 순서가 서로 안 맞을 수 있었던 것.
  // 계획이 계산된 뒤에는, 트렁크에 먼저 실릴(order가 빠른) 박스가 각 칸의
  // 맨 위에 오도록 "적재 순서의 역순"으로 넣는다 - 역순으로 넣으면 칸마다
  // 가장 나중에 들어가는(=order가 가장 빠른) 박스가 그 칸의 맨 위에 남아서,
  // 애니메이션이 order 순서대로 박스를 지울 때 항상 "지금 칸에서 제일 위에
  // 있는 박스"부터 없어진다. 못 실은(unloadable) 박스는 절대 안 없어지므로
  // 맨 밑(다른 박스를 가리지 않는 자리)에 깔아둔다.
  const cartStackingOrder = useMemo(() => {
    if (!effectiveResult) return inputBoxes; // 계산 전엔 적재 순서 정보가 없음
    const boxById = Object.fromEntries(inputBoxes.map((b) => [b.id, b]));
    const reversedPlacedIds = [...effectiveResult.placed]
      .sort((a, b) => a.order - b.order)
      .map((p) => p.box_id)
      .reverse();
    const unloadableIds = (effectiveResult.unloadable || []).map((u) => u.box_id);
    return [...unloadableIds, ...reversedPlacedIds].map((id) => boxById[id]).filter(Boolean);
  }, [effectiveResult, inputBoxes]);

  // 카트 모양과 그 안 박스들의 자리는 항상 "박스 목록 전체"를 기준으로 한
  // 번만 계산해서 고정해 둔다(fullCartLayout) - Before/After를 오가거나
  // "순서대로 재생" 도중에 카트 크기나 남은 박스들의 자리가 흔들리지 않고,
  // 실린 박스만 자기 자리에서 그대로 사라지는 것처럼 보이게 하기 위함.
  const fullCartLayout = useMemo(
    () => layoutStagingBoxes(cartStackingOrder, trunk),
    [cartStackingOrder, trunk],
  );
  const cartFootprint = useMemo(() => computeCartFootprint(fullCartLayout), [fullCartLayout]);

  // 카트에 지금 "남아있는" 박스 id 집합.
  // Before: 계산 전이므로 전부 카트에 있음.
  // After: 이미 실린 박스는 뺀다 - 못 실은(unloadable) 박스는 항상 남고,
  // "순서대로 재생" 중에는(visibleCount!==null) 아직 트렁크에 등장하지
  // 않은(순서상 뒤인) 박스도 남겨서, 재생을 누르면 카트에서 박스가 하나씩
  // 사라지며 트렁크에 하나씩 나타나는 것처럼 보이게 한다(사용자 피드백).
  const stagedBoxIds = useMemo(() => {
    if (stage === "before") return new Set(inputBoxes.map((b) => b.id));
    if (!effectiveResult) return new Set();
    const unloadableIds = (effectiveResult.unloadable || []).map((u) => u.box_id);
    const notYetVisibleIds = visibleCount === null
      ? []
      : placed.slice(visibleCount).map((p) => p.box_id);
    return new Set([...unloadableIds, ...notYetVisibleIds]);
  }, [stage, inputBoxes, effectiveResult, visibleCount, placed]);

  const stagedBoxes = useMemo(
    () => fullCartLayout.filter((b) => stagedBoxIds.has(b.id)),
    [fullCartLayout, stagedBoxIds],
  );

  return (
    <div className={styles.wrapper}>
      <div className={styles.toolbar}>
        <div className={styles.stageBar}>
          {[["before", "Before"], ["after", "After"]].map(([value, text]) => (
            <button
              key={value}
              type="button"
              className={stage === value ? styles.stageActive : styles.stage}
              onClick={() => setStage(value)}
            >
              {text}
            </button>
          ))}
        </div>
        <div className={styles.presetBar}>
          {showPlaced && placed.length > 0 && (
            <button type="button" disabled={animating} onClick={handlePlayStepByStep}>
              {animating ? `▶ 재생 중 (${visibleCount}/${placed.length})` : "▶ 순서대로 재생"}
            </button>
          )}
          {Object.keys(CAMERA_PRESETS).map((name) => (
            <button key={name} type="button" onClick={() => setPreset(name)}>{name}</button>
          ))}
          <div className={styles.scenarioBar}>
            {SCENARIOS.map((s) => (
              <button
                key={s.id}
                type="button"
                disabled={scenarioLoading}
                className={activeScenarioId === s.id ? styles.scenarioActive : undefined}
                onClick={() => handleSelectScenario(s.id)}
              >
                {s.label}
              </button>
            ))}
            {activeScenarioId && (
              <button type="button" onClick={handleExitScenario}>실시간 계획으로 돌아가기</button>
            )}
          </div>
        </div>
      </div>
      {scenarioError && <span className={styles.scenarioError}>{scenarioError}</span>}
      <Canvas key={dpr} dpr={dpr} camera={{ position: CAMERA_PRESETS.front.position, fov: 50 }}>
        <ambientLight intensity={0.7} />
        <directionalLight position={[3, 5, 3]} intensity={0.6} />
        <OrbitControls ref={controlsRef} />
        {/* 배경이 새하얘서 밋밋하다는 피드백 - 옅은 그리드 바닥을 깔아 공간감을
            준다(참고 사진의 Isaac Sim 그리드처럼 진하지 않게, 우리 밝은
            테마에 맞춰 은은하게). 바닥은 우리 좌표계 z=0 = three.js y=0. */}
        <Grid
          position={[0, -0.001, 0]}
          args={[10, 10]}
          cellSize={0.25} cellThickness={0.5} cellColor="#D8D8DC"
          sectionSize={1} sectionThickness={1} sectionColor="#B8B8C4"
          fadeDistance={9} fadeStrength={1.2} infiniteGrid
        />
        {trunk && (
          activeScenarioId
            ? <BoundingBoxWireframe x={0} y={0} z={0} width={trunk.width} depth={trunk.depth}
                                    height={trunk.height} color={scenarioTrunkColor(activeScenarioId)} />
            : <TrunkWireframe trunk={trunk} />
        )}
        {cartFootprint && (
          <CartWireframe footprint={cartFootprint} entranceNearX={trunk.entrance_near_x !== false} />
        )}
        {(effectiveResult?.obstacles || []).map((o) => (
          <SceneBoxMesh key={o.id} position={[o.x, o.y, o.z]}
                        dimensions={[o.width, o.depth, o.height]} color="#7f8c8d" />
        ))}
        {showPlaced && visiblePlaced.map((p) => (
          <SceneBoxMesh key={p.box_id} position={p.position} dimensions={p.dimensions}
                        color={boxColor(p.box_id)} dashed={p.position[2] > 1e-6} />
        ))}
        {stagedBoxes.map((b) => (
          <SceneBoxMesh key={b.id} position={b.position} dimensions={b.dimensions}
                        color={boxColor(b.id)} />
        ))}
      </Canvas>
    </div>
  );
}
