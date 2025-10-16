// components/blocks/UrdfViewerNode.jsx
import React, { useEffect, useRef, useState } from "react";
import { Position } from "@xyflow/react";
import HandleWithLabel from "./HandleWithLabel";
import { Canvas } from "@react-three/fiber";
import { OrbitControls, Grid } from "@react-three/drei";
import * as THREE from "three";
import URDFLoader from "urdf-loader";
import "../../styles/components/_urdfViewerNode.scss";

/**
 * URDF Viewer - 3D visualization of URDF robot description
 * Renders XML received via 'xml' handle using urdf-loader and @react-three/fiber
 */
export default function UrdfViewerNode({ data }) {
  const xml = data?.xml || "";
  const [url, setUrl] = useState(null);

  useEffect(() => {
    if (!xml) {
      setUrl(null);
      return;
    }
    const blob = new Blob([xml], { type: "application/xml" });
    const objectUrl = URL.createObjectURL(blob);
    setUrl(objectUrl);
    return () => URL.revokeObjectURL(objectUrl);
  }, [xml]);

  return (
    <div className="urdf-viewer-node">
      <div className="rf-card">
        <div className="rf-card__title">URDF Viewer</div>

        <div className="urdf-canvas-container">
          {url ? (
            <UrdfCanvas url={url} />
          ) : (
            <div className="rf-hint">
              Connect XML output from Robot node to visualize
            </div>
          )}
        </div>

        <HandleWithLabel
          type="target"
          position={Position.Left}
          id="xml"
          label="xml"
          color="purple"
          top="50%"
        />
      </div>
    </div>
  );
}

function UrdfCanvas({ url }) {
  return (
    <Canvas
      style={{ width: "100%", height: "100%" }}
      camera={{ fov: 50, near: 0.0005, far: 5000 }}
      gl={{ antialias: true, logarithmicDepthBuffer: true }}
    >
      <ambientLight intensity={0.65} />
      <directionalLight position={[5, 6, 8]} intensity={0.9} />
      {/* Z-up: piso en XY => rotamos la grilla +90° en X */}
      <Grid args={[20, 40]} rotation={[Math.PI / 2, 0, 0]} position={[0, 0, 0]} />
      <RobotFromUrl url={url} />
      <OrbitControls makeDefault enableDamping dampingFactor={0.08} minDistance={0.01} maxDistance={2000} />
    </Canvas>
  );
}

function RobotFromUrl({ url }) {
  const [obj, setObj] = useState(null);
  const loaderRef = useRef(null);

  useEffect(() => {
    if (!url) return;
    const manager = new THREE.LoadingManager();
    const loader = new URDFLoader(manager);
    loaderRef.current = loader;

    loader.load(
      url,
      (urdf) => {
        urdf.scale.set(1, 1, 1);
        urdf.frustumCulled = false;
        urdf.traverse((o) => (o.frustumCulled = false));
        setObj(urdf);
      },
      undefined,
      (err) => console.error("URDF load error:", err)
    );

    return () => {
      // Limpia referencia; el objectURL se libera en el efecto del padre
      loaderRef.current = null;
      setObj(null);
    };
  }, [url]);

  return obj ? <primitive object={obj} /> : null;
}
