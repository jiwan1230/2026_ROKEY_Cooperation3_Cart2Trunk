// src/components/ScanViewerPanel.jsx
// 트렁크 Scan / 카트 Scan 칸 - 뼈대(토글+상태+버튼+3D 캔버스)가 완전히
// 같아서 kind prop으로 내용만 갈아끼운다. 시뮬레이터 탭의 PlannerContext와
// 무관하게 독립적으로 동작한다.
import { useState } from "react";
import { Canvas } from "@react-three/fiber";
import { Grid, OrbitControls } from "@react-three/drei";
import { postCartScan, postTrunkScan } from "../api/client.js";
import {
  TrunkWireframe, CartWireframe, SceneBoxMesh, BoundingBoxWireframe,
  layoutStagingBoxes, computeCartFootprint,
} from "./sceneMeshes.jsx";
import { DUMMY_TRUNK, DUMMY_CART_BOXES } from "./robotDummyData.js";
import styles from "./ScanViewerPanel.module.css";

const KIND_LABELS = { trunk: "트렁크 스캔", cart: "카트 스캔" };
const STATUS_TEXT = { idle: "대기", running: "진행중", done: "완료" };

// callTrigger처럼 호출 시점에 postTrunkScan/postCartScan을 직접 참조한다 -
// 미리 객체에 캡쳐해두면 vi.spyOn 목이 반영 안 되는 문제(RobotControlPanel.jsx
// 참고)를 피하기 위함.
function callScanTrigger(kind) {
  return kind === "trunk" ? postTrunkScan() : postCartScan();
}

function RawTrunkPreview() {
  return (
    <BoundingBoxWireframe x={0} y={0} z={0}
      width={DUMMY_TRUNK.width} depth={DUMMY_TRUNK.depth} height={DUMMY_TRUNK.height} />
  );
}

function ProcessedTrunkPreview() {
  return <TrunkWireframe trunk={DUMMY_TRUNK} />;
}

function RawCartPreview() {
  const layout = layoutStagingBoxes(DUMMY_CART_BOXES, DUMMY_TRUNK);
  const footprint = computeCartFootprint(layout);
  if (!footprint) return null;
  const { minX, maxX, minY, maxY, height } = footprint;
  return <BoundingBoxWireframe x={minX} y={minY} z={0} width={maxX - minX} depth={maxY - minY} height={height} />;
}

function ProcessedCartPreview() {
  const layout = layoutStagingBoxes(DUMMY_CART_BOXES, DUMMY_TRUNK);
  const footprint = computeCartFootprint(layout);
  return (
    <>
      {footprint && <CartWireframe footprint={footprint} entranceNearX={DUMMY_TRUNK.entrance_near_x} />}
      {layout.map((b) => (
        <SceneBoxMesh key={b.id} position={b.position} dimensions={b.dimensions} color="#4A90D9" />
      ))}
    </>
  );
}

const SCENE_CONTENT = {
  trunk: { raw: RawTrunkPreview, processed: ProcessedTrunkPreview },
  cart: { raw: RawCartPreview, processed: ProcessedCartPreview },
};

export default function ScanViewerPanel({ kind }) {
  const [status, setStatus] = useState("idle");
  const [viewMode, setViewMode] = useState("raw");

  const handleTrigger = async () => {
    setStatus("running");
    try {
      // TODO(비전팀 연동 시): 여기서 실제 스캔 결과를 받으면 DUMMY_TRUNK/
      // DUMMY_CART_BOXES 대신 그 값을 써야 한다. 지금은 성공 여부만 보고
      // 더미 message 내용은 쓰지 않는다.
      await callScanTrigger(kind);
      setStatus("done");
    } catch {
      setStatus("idle");
    }
  };

  const Content = status === "done" ? SCENE_CONTENT[kind][viewMode] : null;

  return (
    <div className={styles.panel}>
      <div className={styles.header}>
        <span className={styles.title}>{KIND_LABELS[kind]}</span>
        <div className={styles.toggle}>
          <button type="button" data-testid={`${kind}-viewmode-raw`}
                  className={viewMode === "raw" ? styles.toggleActive : styles.toggleBtn}
                  onClick={() => setViewMode("raw")}>원본</button>
          <button type="button" data-testid={`${kind}-viewmode-processed`}
                  className={viewMode === "processed" ? styles.toggleActive : styles.toggleBtn}
                  onClick={() => setViewMode("processed")}>전처리</button>
        </div>
      </div>

      <div className={styles.canvasWrap}>
        <Canvas camera={{ position: [1.2, 1.0, 1.6], fov: 50 }}>
          <ambientLight intensity={0.7} />
          <directionalLight position={[3, 5, 3]} intensity={0.6} />
          <OrbitControls />
          <Grid position={[0, -0.001, 0]} args={[4, 4]} cellSize={0.25} cellThickness={0.5}
                cellColor="#D8D8DC" sectionSize={1} sectionThickness={1} sectionColor="#B8B8C4"
                fadeDistance={5} fadeStrength={1.2} infiniteGrid />
          {Content && <Content />}
        </Canvas>
      </div>

      <button type="button" data-testid={`trigger-${kind}`} disabled={status === "running"} onClick={handleTrigger}>
        {KIND_LABELS[kind]}
      </button>
      <span className={styles.status} data-status={status} data-testid={`status-${kind}`}>
        {STATUS_TEXT[status]}
      </span>
    </div>
  );
}
