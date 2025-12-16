// =============================================================
// FILE: src/pages/LearnLevel.jsx  (optimizado)
// =============================================================
import React, { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { useParams } from "react-router-dom";
import { useProgress } from "../context/ProgressContext";
import { apiFetch } from "../context/AuthContext";
import { API_BASE } from "../config";
//------------------Levels
import TurtleSimPage from "../levels/Turtlesim";
import RosBasic from "../levels/RosBasic";
import MeshDebugPage from "../levels/rviz";
import VehicleDynamics from "../levels/VehicleDynamics";
import GazeboSim from "../levels/GazeboSim";
import IntroUIBasics from "../levels/IntroUIBasics";
import IntroGettingStarted from "../levels/IntroGettingStarted";
// Vehicle Dynamics Levels
import VdPhysicsIntro from "../levels/VdPhysicsIntro";
import VdCenterOfRotation from "../levels/VdCenterOfRotation";
import VdAckermann from "../levels/VdAckermann";
import VdBicycleModel from "../levels/VdBicycleModel";
import VdSimulations from "../levels/VdSimulations";

const DEBUG = true;

const REGISTRY = Object.freeze({
  turtlesim: TurtleSimPage,
  "ros-basic": RosBasic,
  "rviz": MeshDebugPage,
  "vehicle-dynamics": VehicleDynamics,
  "gazebo-sim": GazeboSim,
  "intro-ui-basics": IntroUIBasics,
  "intro-getting-started": IntroGettingStarted,
  // Vehicle Dynamics Unit Levels
  "vd-physics-intro": VdPhysicsIntro,
  "vd-center-of-rotation": VdCenterOfRotation,
  "vd-ackermann": VdAckermann,
  "vd-bicycle-model": VdBicycleModel,
  "vd-simulations": VdSimulations,
});

export default function LearnLevel() {
  const { unitSlug, levelSlug } = useParams();
  const wantLevel = (levelSlug || "").toLowerCase();

  const { hitObjective, completeLevel, resetLevel } = useProgress();

  const [level, setLevel] = useState(null);
  const [loading, setLoading] = useState(true);
  const [err, setErr] = useState("");


  const mountedRef = useRef(true);
  useEffect(() => {
    mountedRef.current = true;
    return () => { mountedRef.current = false; };
  }, []);


  const abortRef = useRef(null);

  const LevelCmp = REGISTRY[wantLevel] || REGISTRY[levelSlug];



 // LearnLevel.jsx
const fetchLevel = useCallback(async ({ silent = false } = {}) => {
  if (abortRef.current) abortRef.current.abort();
  const ac = new AbortController();
  abortRef.current = ac;

  const url = `${API_BASE}/levels/${encodeURIComponent(levelSlug)}/`;
  if (DEBUG) console.log("[LearnLevel] GET", url);

  if (!silent) setLoading(true);
  setErr("");

  try {
    const res = await apiFetch(url, { signal: ac.signal });
    if (!res.ok) {
      const txt = await res.text().catch(() => "");
      if (DEBUG) console.error("[LearnLevel] body:", txt);
      if (mountedRef.current) {
        setLevel(null);
        setErr(`No se pudo cargar el nivel (${res.status}).`);
      }
      return;
    }
    const data = await res.json();
    if (mountedRef.current) setLevel(data);
  } catch (e) {
    if (e?.name !== "AbortError") {
      console.error("[LearnLevel] fetchLevel error:", e);
      if (mountedRef.current) {
        setLevel(null);
        setErr("Error de red al cargar el nivel.");
      }
    }
  } finally {
    if (!silent && mountedRef.current) setLoading(false);
  }
}, [levelSlug]);


  // Carga inicial y al cambiar el slug
  useEffect(() => { fetchLevel(); }, [fetchLevel]);

  // Objetivos memorizados
  const objectives = useMemo(
    () => (Array.isArray(level?.objectives) ? level.objectives : []),
    [level]
  );

  // --------- LOGS DE DEPURACIÓN ----------
  useEffect(() => {
    if (!DEBUG) return;
    console.log("======== [LearnLevel DEBUG] ========");
    console.log("[URL] unitSlug:", unitSlug, "levelSlug:", levelSlug);
    console.log("[level] title:", level?.title, "slug:", level?.slug);
    console.log(
      "[objectives]:",
      objectives.map((o) => ({
        code: o.code,
        desc: o.description || o.Description,
        achieved: o.user_progress?.achieved,
      }))
    );
    console.log("====================================");
  }, [unitSlug, levelSlug, level, objectives]);


  const handleReset = useCallback(async () => {
    if (!window.confirm("¿Resetear el progreso de este nivel?")) return;
    try {
      await resetLevel(levelSlug);
      await fetchLevel(); // recarga desde API
    } catch (e) {
      console.error("[LearnLevel] resetLevel error:", e);
      alert("No se pudo reiniciar el nivel.");
    }
  }, [levelSlug, resetLevel, fetchLevel]);

  // tras registrar objetivo
const handleHitObjective = useCallback(async (code) => {
  if (!code) return;
  try {
    await hitObjective(code);
  } catch (e) {
    console.error("[LearnLevel] hitObjective error:", e);
    alert("No se pudo registrar el objetivo. Revisa el código.");
  } finally {
    await fetchLevel({ silent: true }); // ← sin spinner
  }
}, [hitObjective, fetchLevel]);

// al completar nivel
const handleCompleteLevel = useCallback(async () => {
  try {
    await completeLevel(levelSlug);
  } catch (e) {
    console.error("[LearnLevel] completeLevel error:", e);
    alert("No se pudo marcar el nivel como completado.");
  } finally {
    await fetchLevel({ silent: true }); // ← sin spinner
  }
}, [completeLevel, levelSlug, fetchLevel]);



  if (!LevelCmp) {
    if (DEBUG) console.log("[LearnLevel] REGISTRY miss. keys:", Object.keys(REGISTRY));
    return <div>Level "{levelSlug}" not found.</div>;
  }

  if (loading) {
    return <div className="placeholder">Cargando nivel…</div>;
  }

  if (err) {
    return <div className="error">{err}</div>;
  }

  if (!level) {
    return <div>No se pudo cargar el nivel "{levelSlug}".</div>;
  }

  return (
    <div className="level-wrap">
      <header className="level-header">
        <h1 className="level-title">{level?.title || levelSlug}</h1>

        <ul className="level-objectives">
          {objectives.length > 0 ? (
            objectives.map((o) => {
              const desc = o.description || o.Description || o.code;
              const done = !!o.user_progress?.achieved;
              return (
                <li key={o.code} className={done ? "done" : ""} title={o.code}>
                  <span className="dot" />
                  <span className="desc">{desc}</span>
                  {done && <span className="check">✔</span>}
                </li>
              );
            })
          ) : (
            <li className="empty">No objectives for this level</li>
          )}
        </ul>

        <button className="level-reset" title="Reset progress" onClick={handleReset}>
 
        </button>
      </header>

      <LevelCmp
        objectives={objectives}
        onObjectiveHit={handleHitObjective}
        onLevelCompleted={handleCompleteLevel}
      />
    </div>
  );
}
