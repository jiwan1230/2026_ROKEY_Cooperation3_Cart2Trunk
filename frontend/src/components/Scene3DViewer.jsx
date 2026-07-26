import { useEffect, useMemo, useRef, useState } from "react";
import { Canvas } from "@react-three/fiber";
import { Grid, OrbitControls } from "@react-three/drei";
import { usePlannerState } from "../state/PlannerContext.jsx";
import { colorForBoxId } from "../utils/color.js";
import styles from "./Scene3DViewer.module.css";

const STAGING_GAP = 0.05; // 대기 박스끼리, 그리고 트렁크 입구 면과의 간격(m)
const STAGING_OFFSET = 0.3; // 트렁크 입구 면에서 대기 구역까지의 거리(m) - 트렁크
// 자체가 1m 안팎으로 작아서, 간격이 너무 좁으면(예: 0.15) 카메라 원근감 때문에
// 대기 박스가 트렁크에 거의 붙어 보이는 착시가 생겼다(사용자 확인 후 조정).
const CART_GRID_SPAN_COLUMNS = 3; // 카트 폭 방향(옆으로) 칸 수
const CART_GRID_DEPTH_ROWS = 2; // 카트 안쪽(입구 방향) 칸 수
// - "박스 10개는 담을 만큼 크게, 여러 층으로" 피드백에 맞춰 6칸(3x2) 격자를
// 만들고, 박스를 순서대로 칸에 라운드로빈으로 배정한다. 칸 수(6개)보다
// 박스가 많으면 같은 칸에 다시 배정되어 그 칸 안에서 위로 쌓인다.

// 트렁크 입구(로봇이 접근하는 쪽) 바로 바깥에 카트에 대기 중인 박스들을
// 놓는다 - tkinter GUI의 Before 이미지가 로봇/카트 쪽에 대기 박스를 보여주던
// 것과 같은 취지. 실제 카트 위 배치 좌표는 모르므로(그런 데이터 자체가
// 없음) 카트를 3x2 격자 칸으로 나누고 박스를 순서대로 칸에 채운다.
// [사용자 피드백] 처음엔 한 행(옆으로 죽 늘어놓기)이 다 차면 다음 "층" 전체를
// 그 층에서 가장 높은 박스 기준으로 새로 시작하는 선반형(shelf) 배치를
// 썼는데, 한 층 안에 키가 다른 박스가 섞이면 낮은 박스 위로 진짜 빈 공간이
// 남아 다음 층 박스가 "공중에 떠 있는 것처럼" 보였다 - 칸(격자)마다 독립적인
// 누적 높이를 쓰는 방식으로 바꿔서, 각 칸 안에서는 항상 바로 아래 박스에
// 딱 붙어 쌓이도록(간격 0) 보장한다.
export function layoutStagingBoxes(boxSpecs, trunk) {
  if (!trunk || boxSpecs.length === 0) return [];
  const entranceNearX = trunk.entrance_near_x !== false;

  const cellWidth = Math.max(...boxSpecs.map((b) => b.width));
  const cellDepth = Math.max(...boxSpecs.map((b) => b.depth));
  const numColumns = CART_GRID_SPAN_COLUMNS * CART_GRID_DEPTH_ROWS;
  const columnHeights = new Array(numColumns).fill(0);

  return boxSpecs.map((b, i) => {
    const col = i % numColumns;
    const depthIndex = Math.floor(col / CART_GRID_SPAN_COLUMNS); // 입구로부터 몇 번째 칸(깊이축)
    const spanIndex = col % CART_GRID_SPAN_COLUMNS; // 옆으로 몇 번째 칸

    const rowDepthOffset = depthIndex * (cellWidth + STAGING_GAP);
    const spanOffset = spanIndex * (cellDepth + STAGING_GAP);

    // 칸보다 작은 박스는 칸 한가운데에 오도록(폭/깊이가 서로 다른 박스가
    // 섞여도 칸 경계와 어긋나 보이지 않게) 정렬한다.
    const cellOriginX = entranceNearX
      ? -(STAGING_OFFSET + rowDepthOffset + cellWidth)
      : trunk.width + STAGING_OFFSET + rowDepthOffset;
    const x = cellOriginX + (cellWidth - b.width) / 2;
    const y = spanOffset + (cellDepth - b.depth) / 2;
    const z = columnHeights[col];
    columnHeights[col] += b.height;

    return { id: b.id, position: [x, y, z], dimensions: [b.width, b.depth, b.height] };
  });
}

// 우리 좌표계(x=width, y=depth, z=height, (0,0,0) 코너 기준)를 three.js의
// y-up 좌표계로 옮긴다: three.x=our.x, three.y=our.z(높이), three.z=our.y(깊이).
export function toThreeCenter(x, y, z, w, d, h) {
  return [x + w / 2, z + h / 2, y + d / 2];
}

const TRUNK_LID_COLOR = "#9C1F1F"; // 사용자가 준 참고 사진(빨간 세단)처럼 차체색 리드
const TRUNK_TAILLIGHT_COLOR = "#C62828";
const TRUNK_LID_TILT = (32 * Math.PI) / 180; // 열린 트렁크 리드가 수직에서 트렁크 안쪽으로 얼마나 기울어 있는지

// "실제 차 트렁크처럼 보이게 해달라"는 피드백 - 사진을 그대로 3D로 옮길 수는
// 없어서(사진은 3D 형상 데이터가 아님) 지금 있는 절차적 도형에 테일램프/
// 열린 트렁크 리드를 더해 "자동차 트렁크스러운" 실루엣을 만드는 방향으로
// 절충했다(사용자와 상의 후 결정). 처음엔 범퍼(입구 아래쪽 긴 얇은 막대)도
// 넣었는데, 카메라가 그 막대를 거의 옆에서(길이 방향으로) 보는 각도라
// 원근감 때문에 두꺼운 검은 판처럼 보이는 문제가 있어(확인 후) 뺐다.
function TrunkWireframe({ trunk }) {
  const entranceNearX = trunk.entrance_near_x !== false;
  const outwardSign = entranceNearX ? -1 : 1;
  const entranceX = entranceNearX ? 0 : trunk.width;
  const tailX = entranceNearX ? -0.015 : trunk.width - 0.015;
  const lidLength = trunk.height * 0.8;
  // [사용자 피드백] 폭을 입구 전체(trunk.depth)에서 작은 조각으로 줄였더니
  // "폭은 원래(입구 전체 폭) 그대로 두고 위치만 가운데로 보내라 - 실제
  // 트렁크를 열면 그렇게 된다"는 정정을 받았다 - 폭은 되돌리고, 아래
  // lidRotationZ(트렁크 안쪽/가운데를 향해 기울어짐)로만 "가운데로 가는"
  // 느낌을 낸다.
  // 리드는 입구 위쪽 가장자리에 경첩을 두고, unrotated 상태(로컬 +X)에서
  // 시작해 z축 회전으로 위/트렁크 안쪽을 향하도록 돌린다.
  // [사용자 피드백] 처음엔 "카트 쪽(바깥)으로 열려야 실제 차와 같다"고
  // 판단해서 반대로 만들었는데, 실제로 렌더링해서 보여주니 그러면 리드가
  // 카트 위로 붕 떠서 튀어나온 것처럼 보여 오히려 더 이상했다 - 사용자가
  // 직접 "열린 트렁크처럼 보이려면 리드가 (카트 쪽이 아니라) 가운데/트렁크
  // 안쪽으로 가야 한다"고 확인해줘서 방향을 다시 안쪽으로 바꾼다. 로컬 +X가
  // 회전 후 world (-outwardSign*sin(tilt), cos(tilt))를 향하게 하는 각도가
  // 90도 + outwardSign*tilt 이다.
  const lidRotationZ = Math.PI / 2 + outwardSign * TRUNK_LID_TILT;

  return (
    <group>
      <mesh position={toThreeCenter(0, 0, 0, trunk.width, trunk.depth, trunk.height)}>
        <boxGeometry args={[trunk.width, trunk.height, trunk.depth]} />
        <meshBasicMaterial color="#6E6E73" wireframe />
      </mesh>

      {/* 테일램프 - 입구 면 위쪽 양옆 */}
      {[0.08, Math.max(0.08, trunk.depth - 0.22)].map((yStart, i) => (
        <mesh key={i} position={toThreeCenter(tailX, yStart, trunk.height - 0.14, 0.03, 0.14, 0.1)}>
          <boxGeometry args={[0.03, 0.1, 0.14]} />
          <meshStandardMaterial color={TRUNK_TAILLIGHT_COLOR} emissive={TRUNK_TAILLIGHT_COLOR} emissiveIntensity={0.4} />
        </mesh>
      ))}

      {/* 열린 트렁크 리드 - 입구 위쪽 가장자리(폭 전체, trunk.depth)에
          경첩을 두고 위/트렁크 안쪽(가운데)으로 기울어 열려있는 판. */}
      <group position={toThreeCenter(entranceX, 0, trunk.height, 0, trunk.depth, 0)} rotation={[0, 0, lidRotationZ]}>
        <mesh position={[lidLength / 2, 0, 0]}>
          <boxGeometry args={[lidLength, 0.05, trunk.depth]} />
          <meshStandardMaterial color={TRUNK_LID_COLOR} roughness={0.5} metalness={0.2} />
        </mesh>
      </group>
    </group>
  );
}

const CART_MARGIN = 0.1; // 대기 박스 묶음과 카트 벽 사이 여유(m)
const CART_WALL_CLEARANCE = 0.12; // 가장 높은 박스보다 카트 벽을 얼마나 더 높게 그릴지(m)
const CART_WHEEL_RADIUS = 0.045;

// "쇼핑카트에서 트렁크로 옮긴다"는 실제 작업을 시각적으로 보여달라는 피드백 -
// 대기 박스 전용 좌표계(layoutStagingBoxes)를 감싸는 카트 모양(벽+바퀴+손잡이)을
// 하나 더 그린다. 카트 크기는 "지금 대기 중인 박스"가 아니라 "박스 목록
// 전체"를 기준으로 계산해서 Before/After를 오가도 카트 크기 자체는 그대로
// 유지되고(실제 카트가 그렇듯), 안에 든 박스 수만 바뀌어 보이게 한다.
export function computeCartFootprint(fullBoxLayout) {
  if (!fullBoxLayout || fullBoxLayout.length === 0) return null;
  const minX = Math.min(...fullBoxLayout.map((b) => b.position[0]));
  const maxX = Math.max(...fullBoxLayout.map((b) => b.position[0] + b.dimensions[0]));
  const maxY = Math.max(...fullBoxLayout.map((b) => b.position[1] + b.dimensions[1]));
  const maxHeight = Math.max(...fullBoxLayout.map((b) => b.dimensions[2]));
  return {
    minX: minX - CART_MARGIN,
    maxX: maxX + CART_MARGIN,
    minY: -CART_MARGIN,
    maxY: maxY + CART_MARGIN,
    height: maxHeight + CART_WALL_CLEARANCE,
  };
}

function CartWireframe({ footprint, entranceNearX }) {
  const { minX, maxX, minY, maxY, height } = footprint;
  const width = maxX - minX;
  const depth = maxY - minY;
  // 손잡이는 트렁크 입구 반대쪽(카트를 밀고 들어오는 방향의 뒤쪽)에 둔다.
  const handleX = entranceNearX ? minX : maxX;
  const wheelY = [minY + CART_WHEEL_RADIUS, maxY - CART_WHEEL_RADIUS];
  const wheelX = [minX + CART_WHEEL_RADIUS, maxX - CART_WHEEL_RADIUS];

  return (
    <group>
      <mesh position={toThreeCenter(minX, minY, 0, width, depth, height)}>
        <boxGeometry args={[width, height, depth]} />
        <meshBasicMaterial color="#A2724F" wireframe />
      </mesh>
      {wheelX.flatMap((x) => wheelY.map((y) => (
        <mesh key={`${x}-${y}`} position={toThreeCenter(x, y, CART_WHEEL_RADIUS, 0, 0, 0)}>
          <sphereGeometry args={[CART_WHEEL_RADIUS, 12, 12]} />
          <meshStandardMaterial color="#3A3A3C" />
        </mesh>
      )))}
      <mesh position={toThreeCenter(handleX, minY, height * 0.75, 0, depth, 0)}>
        <boxGeometry args={[0.03, 0.03, depth]} />
        <meshStandardMaterial color="#3A3A3C" />
      </mesh>
    </group>
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
    if (!state.result) return inputBoxes; // 계산 전엔 적재 순서 정보가 없음
    const boxById = Object.fromEntries(inputBoxes.map((b) => [b.id, b]));
    const reversedPlacedIds = [...state.result.placed]
      .sort((a, b) => a.order - b.order)
      .map((p) => p.box_id)
      .reverse();
    const unloadableIds = (state.result.unloadable || []).map((u) => u.box_id);
    return [...unloadableIds, ...reversedPlacedIds].map((id) => boxById[id]).filter(Boolean);
  }, [state.result, inputBoxes]);

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
    if (!state.result) return new Set();
    const unloadableIds = (state.result.unloadable || []).map((u) => u.box_id);
    const notYetVisibleIds = visibleCount === null
      ? []
      : placed.slice(visibleCount).map((p) => p.box_id);
    return new Set([...unloadableIds, ...notYetVisibleIds]);
  }, [stage, inputBoxes, state.result, visibleCount, placed]);

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
        </div>
      </div>
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
        {trunk && <TrunkWireframe trunk={trunk} />}
        {cartFootprint && (
          <CartWireframe footprint={cartFootprint} entranceNearX={trunk.entrance_near_x !== false} />
        )}
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
