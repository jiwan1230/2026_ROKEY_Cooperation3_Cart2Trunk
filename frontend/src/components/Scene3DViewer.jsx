import { useEffect, useRef, useState } from "react";
import { Canvas } from "@react-three/fiber";
import { OrbitControls } from "@react-three/drei";
import { usePlannerState } from "../state/PlannerContext.jsx";
import styles from "./Scene3DViewer.module.css";

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

const CAMERA_PRESETS = {
  front: { position: [3, 1.5, 0.01], target: [0, 0, 0] },
  side: { position: [0.01, 1.5, 3], target: [0, 0, 0] },
  top: { position: [0.01, 4, 0.01], target: [0, 0, 0] },
};

export default function Scene3DViewer() {
  const state = usePlannerState();
  const controlsRef = useRef(null);
  const [preset, setPreset] = useState("front");

  useEffect(() => {
    const controls = controlsRef.current;
    if (!controls) return;
    const { position, target } = CAMERA_PRESETS[preset];
    controls.object.position.set(...position);
    controls.target.set(...target);
    controls.update();
  }, [preset]);

  const trunk = state.result?.trunk;

  return (
    <div className={styles.wrapper}>
      <div className={styles.presetBar}>
        {Object.keys(CAMERA_PRESETS).map((name) => (
          <button key={name} type="button" onClick={() => setPreset(name)}>{name}</button>
        ))}
      </div>
      <Canvas camera={{ position: CAMERA_PRESETS.front.position, fov: 50 }}>
        <ambientLight intensity={0.7} />
        <directionalLight position={[3, 5, 3]} intensity={0.6} />
        <OrbitControls ref={controlsRef} />
        {trunk && <TrunkWireframe trunk={trunk} />}
        {state.result?.obstacles?.map((o) => (
          <SceneBoxMesh key={o.id} position={[o.x, o.y, o.z]}
                        dimensions={[o.width, o.depth, o.height]} color="#7f8c8d" />
        ))}
        {state.result?.placed?.map((p) => (
          <SceneBoxMesh key={p.box_id} position={p.position} dimensions={p.dimensions}
                        color={p.color} dashed={p.position[2] > 1e-6} />
        ))}
      </Canvas>
    </div>
  );
}
