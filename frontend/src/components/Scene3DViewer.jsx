import { useEffect, useMemo, useRef, useState } from "react";
import { Canvas } from "@react-three/fiber";
import { OrbitControls } from "@react-three/drei";
import { usePlannerState } from "../state/PlannerContext.jsx";
import { colorForBoxId } from "../utils/color.js";
import styles from "./Scene3DViewer.module.css";

const STAGING_GAP = 0.05; // 대기 박스끼리, 그리고 트렁크 입구 면과의 간격(m)
const STAGING_OFFSET = 0.3; // 트렁크 입구 면에서 대기 구역까지의 거리(m) - 트렁크
// 자체가 1m 안팎으로 작아서, 간격이 너무 좁으면(예: 0.15) 카메라 원근감 때문에
// 대기 박스가 트렁크에 거의 붙어 보이는 착시가 생겼다(사용자 확인 후 조정).

// 트렁크 입구(로봇이 접근하는 쪽) 바로 바깥에 대기 중인 박스들을 나란히
// 줄세운다 - tkinter GUI의 Before 이미지가 로봇/카트 쪽에 대기 박스를
// 보여주던 것과 같은 취지. 실제 카트 위 배치 좌표는 모르므로(그런 데이터
// 자체가 없음) 각 박스를 자기 크기 그대로, 서로 겹치지 않게 y축을 따라
// 줄세우는 것으로 근사한다.
export function layoutStagingBoxes(boxSpecs, trunk) {
  if (!trunk || boxSpecs.length === 0) return [];
  const entranceNearX = trunk.entrance_near_x !== false;
  let cursorY = 0;
  return boxSpecs.map((b) => {
    const x = entranceNearX ? -(STAGING_OFFSET + b.width) : trunk.width + STAGING_OFFSET;
    const y = cursorY;
    cursorY += b.depth + STAGING_GAP;
    return { id: b.id, position: [x, y, 0], dimensions: [b.width, b.depth, b.height] };
  });
}

// 우리 좌표계(x=width, y=depth, z=height, (0,0,0) 코너 기준)를 three.js의
// y-up 좌표계로 옮긴다: three.x=our.x, three.y=our.z(높이), three.z=our.y(깊이).
export function toThreeCenter(x, y, z, w, d, h) {
  return [x + w / 2, z + h / 2, y + d / 2];
}

function TrunkWireframe({ trunk }) {
  return (
    <mesh position={toThreeCenter(0, 0, 0, trunk.width, trunk.depth, trunk.height)}>
      <boxGeometry args={[trunk.width, trunk.height, trunk.depth]} />
      <meshBasicMaterial color="#6E6E73" wireframe />
    </mesh>
  );
}

function SceneBoxMesh({ position, dimensions, color, dashed }) {
  const [w, d, h] = dimensions;
  const [x, y, z] = position;
  return (
    <group position={toThreeCenter(x, y, z, w, d, h)}>
      <mesh>
        <boxGeometry args={[w, h, d]} />
        <meshStandardMaterial color={color} transparent opacity={dashed ? 0.55 : 0.9} />
      </mesh>
      <mesh>
        <boxGeometry args={[w, h, d]} />
        <meshBasicMaterial color="#1D1D1F" wireframe />
      </mesh>
    </group>
  );
}

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
  // 피드백으로 발견).
  front: { position: [0.3, 1.8, 3.2], target: [0.1, 0, 0.4] },
  side: { position: [0.01, 1.5, 3], target: [0, 0, 0] },
  top: { position: [0.01, 4, 0.01], target: [0, 0, 0] },
};

export default function Scene3DViewer() {
  const state = usePlannerState();
  const controlsRef = useRef(null);
  const [preset, setPreset] = useState("front");
  // tkinter GUI의 Before/After 정적 이미지 대신, 같은 3D 씬을 그대로 두고
  // "적재된 박스를 보여줄지"만 토글한다 - Before는 트렁크(+ 장애물)만 빈
  // 상태로 보여준다(카트 박스는 적재 전엔 트렁크 안 실제 좌표가 없으므로).
  const [stage, setStage] = useState("after"); // "before" | "after"
  const dpr = useDevicePixelRatio();

  const placed = state.result?.placed || [];
  // "순서대로 재생" - visibleCount만큼만(order 순서대로) 박스를 보여준다.
  // null이면 "재생 안 함" 상태로, 전부 다 보여준다(기본 동작).
  const [visibleCount, setVisibleCount] = useState(null);
  const [animating, setAnimating] = useState(false);

  // 새로 계산된 결과가 들어오면 재생 상태를 초기화하고 전부 다 보여주는
  // 기본 상태로 되돌린다 - 이전 재생의 중간 상태가 새 계획에 남아있으면 안 됨.
  useEffect(() => {
    setAnimating(false);
    setVisibleCount(null);
  }, [state.result]);

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

  const trunk = state.result?.trunk;
  const showPlaced = stage === "after";

  // 입력 박스 목록(state.boxesText)을 대기 박스 크기 조회용으로 파싱한다 -
  // 타이핑 도중이라 문법이 깨져 있을 수 있으므로 실패하면 조용히 빈 배열로
  // 취급한다(useDebouncedPlan.js와 같은 방어 방식).
  const inputBoxes = useMemo(() => {
    try {
      const parsed = JSON.parse(state.boxesText);
      return Array.isArray(parsed) ? parsed : [];
    } catch {
      return [];
    }
  }, [state.boxesText]);

  // Before: 아직 계산 전이므로 카트의 모든 박스가 대기 중.
  // After: 이미 실린 박스는 실제 위치에 놓이므로, 대기 구역에는 못 실은
  // (unloadable) 박스만 남긴다 - tkinter GUI의 Before/After 이미지와 같은 관례.
  const waitingBoxSpecs = useMemo(() => {
    if (stage === "before") return inputBoxes;
    if (!state.result) return [];
    const boxById = Object.fromEntries(inputBoxes.map((b) => [b.id, b]));
    return (state.result.unloadable || [])
      .map((u) => boxById[u.box_id])
      .filter(Boolean);
  }, [stage, inputBoxes, state.result]);

  const stagedBoxes = useMemo(() => layoutStagingBoxes(waitingBoxSpecs, trunk), [waitingBoxSpecs, trunk]);

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
        </div>
      </div>
      <Canvas key={dpr} dpr={dpr} camera={{ position: CAMERA_PRESETS.front.position, fov: 50 }}>
        <ambientLight intensity={0.7} />
        <directionalLight position={[3, 5, 3]} intensity={0.6} />
        <OrbitControls ref={controlsRef} />
        {trunk && <TrunkWireframe trunk={trunk} />}
        {state.result?.obstacles?.map((o) => (
          <SceneBoxMesh key={o.id} position={[o.x, o.y, o.z]}
                        dimensions={[o.width, o.depth, o.height]} color="#7f8c8d" />
        ))}
        {showPlaced && visiblePlaced.map((p) => (
          <SceneBoxMesh key={p.box_id} position={p.position} dimensions={p.dimensions}
                        color={colorForBoxId(p.box_id)} dashed={p.position[2] > 1e-6} />
        ))}
        {stagedBoxes.map((b) => (
          <SceneBoxMesh key={b.id} position={b.position} dimensions={b.dimensions}
                        color={colorForBoxId(b.id)} />
        ))}
      </Canvas>
    </div>
  );
}
