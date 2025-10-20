// src/levels/RvizFiberSlide.jsx
import React, { useEffect, useRef, useState } from "react";
import { Canvas, useThree } from "@react-three/fiber";
import { OrbitControls, Grid } from "@react-three/drei";
import * as THREE from "three";
import URDFLoader from "urdf-loader";
import ROSLIB from "roslib";

// >>> IP/puertos centralizados
import { getStaticBase, getRosbridgeUrl } from "../../ip";

// ---- metadata para el sistema de slides ----
export const meta = {
  id: "rf-rviz-fiber",
  title: "Visualización URDF (Rviz Fiber)",
  order: 9,
  objectiveCode: "ros-slide-rviz-fiber",
};

// Bases derivadas del único punto de verdad
const STATIC_BASE = getStaticBase();        // p.ej. http(s)://HOST:7000
const ROS_WS_URL  = getRosbridgeUrl();      // p.ej. ws(s)://HOST:9090

// --- Fuerza Z-up en la cámara/controles
function UseZUp() {
  const { camera } = useThree();
  useEffect(() => {
    camera.up.set(0, 0, 1); // Z arriba
  }, [camera]);
  return null;
}

// --- Auto-fit de cámara (sirve igual en Z-up)
function useAutofit(object3D) {
  const { camera, controls } = useThree();
  useEffect(() => {
    if (!object3D) return;
    const box = new THREE.Box3().setFromObject(object3D);
    const sphere = box.getBoundingSphere(new THREE.Sphere());
    const center = sphere.center;
    const radius = Math.max(sphere.radius, 0.001);
    const margin = 1.2;
    const dist = radius * 1000 * margin;

    camera.position.copy(center.clone().add(new THREE.Vector3(dist, dist * 0.6, dist)));
    camera.near = Math.max(radius / 500, 0.0005);
    camera.far = Math.max(dist * 20, 1000);
    camera.updateProjectionMatrix();

    if (controls?.target) {
      controls.target.copy(center);
      controls.update();
    }
  }, [object3D, camera, controls]);
}

function RobotModel() {
  const [robot, setRobot] = useState(null);
  const loadedRef = useRef(false);
  const tfRef = useRef(null);
  const rosRef = useRef(null);

  useAutofit(robot);

  useEffect(() => {
    if (loadedRef.current) return;
    loadedRef.current = true;

    // Reescribe rutas relativas hacia el server estático
    const manager = new THREE.LoadingManager();
    manager.setURLModifier((url) => {
      // Ej.: /qcar_description/... -> http(s)://HOST:7000/qcar_description/...
      if (url.startsWith("/qcar_description/")) return `${STATIC_BASE}${url}`;
      return url;
    });

    const loader = new URDFLoader(manager);
    loader.load(
      `${STATIC_BASE}/qcar_description/urdf/robot_runtime.urdf`,
      (urdf) => {
        urdf.scale.set(1, 1, 1);
        urdf.frustumCulled = false;
        urdf.traverse((o) => (o.frustumCulled = false));
        setRobot(urdf);

        // === TF en vivo vía rosbridge centralizado ===
        const ros = new ROSLIB.Ros({ url: ROS_WS_URL });
        const tfClient = new ROSLIB.TFClient({
          ros,
          fixedFrame: "base",
          angularThres: 0.001,
          transThres: 0.001,
          rate: 10,
        });
        rosRef.current = ros;
        tfRef.current = tfClient;

        urdf.traverse((child) => {
          if (!child.name) return;
          tfClient.subscribe(child.name, (tf) => {
            child.position.set(tf.translation.x, tf.translation.y, tf.translation.z);
            child.quaternion.set(tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w);
          });
        });
      },
      undefined,
      (err) => console.error("URDF error:", err),
      { packages: { qcar_description: `${STATIC_BASE}/qcar_description` } }
    );

    // Cleanup
    return () => {
      try { tfRef.current?.dispose?.(); } catch {}
      try { rosRef.current?.close?.(); } catch {}
      tfRef.current = null;
      rosRef.current = null;
    };
  }, []);

  return robot ? <primitive object={robot} /> : null;
}

function RvizFiber({ height = 600 }) {
  return (
    <Canvas
      style={{ width: "100%", height }}
      camera={{ fov: 50, near: 0.0005, far: 5000 }}
      gl={{ antialias: true, logarithmicDepthBuffer: true }}
    >
      <UseZUp />

      {/* luces */}
      <ambientLight intensity={0.65} />
      <directionalLight position={[5, 6, 8]} intensity={0.9} />

      {/* suelo en XY (Z-up): rotamos la Grid +90° en X */}
      <Grid args={[20, 40]} rotation={[Math.PI / 2, 0, 0]} position={[0, 0, 0]} />

      <RobotModel />

      <OrbitControls
        makeDefault
        enableDamping
        dampingFactor={0.08}
        minDistance={0.01}
        maxDistance={2000}
      />
    </Canvas>
  );
}

// ---- export para el sistema de slides ----
export default function RvizFiberSlide(props) {
  return <RvizFiber height={props?.height ?? 600} />;
}
