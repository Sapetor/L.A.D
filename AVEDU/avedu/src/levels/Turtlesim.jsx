// =============================================================
// FILE: src/pages/TurtleSimPage.jsx (rewritten)
// Versión estable: suscripciones fijas, publish seguro y render optimizado
// Requisitos: tu hook useRoslib ya estable (useCallback/useMemo) o, si no,
// este archivo usa un mini-hook local `useStableRosSub` que blinda la sub.
// =============================================================
import React, { useCallback, useEffect, useMemo, useRef, useState } from "react";
import PropTypes from "prop-types";
import { useRoslib } from "../hooks/useRoslib";
import "../styles/pages/turtlesim.scss";
import { useStableRosSub } from "../hooks/useStableRosSub";

const ROS_URL = process.env.REACT_APP_ROSBRIDGE_URL || "ws://192.168.100.116:9090";
const WORLD = 11.0889;

// Objectives
const OBJ_REACH_GOAL = "TSIM_REACH_GOAL";
const OBJ_SPAWN_2ND = "TSIM_SPAWN_2ND";

// Goal bounds
const GOAL_BOUNDS = { minX: 10.0, minY: 10.0 };

/**
 * Hook local para fijar una suscripción SIN re-suscribirse en cada render.
 * Solo depende de conexión + topic + tipo. Mantiene el último onMessage via ref.
 */
export default function TurtleSimPage({ objectives = [], onObjectiveHit, onLevelCompleted }) {
  const { connected, subscribeTopic, advertise, callService } = useRoslib(ROS_URL);

  // Opciones memoizadas para la suscripción (evita deps inestables)
  const subOpts = useMemo(() => ({ throttle_rate: 50, queue_size: 1 }), []);

  // Estado de entidades (minimiza renders usando comparación de cambios)
  const [turtles, setTurtles] = useState({});
  const turtlesRef = useRef({});
  const setTurtlesSafe = useCallback((updater) => {
    setTurtles((prev) => {
      const next = typeof updater === "function" ? updater(prev) : updater;
      // Evita renders si no cambia nada a nivel superficial
      if (prev === next) return prev;
      let changed = false;
      if (Object.keys(prev).length !== Object.keys(next).length) changed = true;
      else {
        for (const k of Object.keys(next)) {
          const p = prev[k]?.pose, n = next[k]?.pose;
          if (!p || !n || Math.abs(p.x - n.x) > 1e-3 || Math.abs(p.y - n.y) > 1e-3 || Math.abs(p.theta - n.theta) > 1e-3) {
            changed = true; break;
          }
        }
      }
      return changed ? next : prev;
    });
  }, []);

  const [active, setActive] = useState("turtle1");

  const cmdRef = useRef(null);
  const holdTimerRef = useRef(null);
  const canvasRef = useRef(null);

  // Log conexión
  useEffect(() => {
    // eslint-disable-next-line no-console
    console.log(`[TSIM] ROS ${connected ? "connected" : "disconnected"} @ ${ROS_URL}`);
  }, [connected]);

  // Anti-duplicados de objetivos
  const reportedRef = useRef({ [OBJ_REACH_GOAL]: false, [OBJ_SPAWN_2ND]: false });

  // === Callbacks estables ===
  const publish = useCallback((lx, az) => {
    if (!cmdRef.current) return;
    cmdRef.current.publish({
      linear: { x: lx, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: az },
    });
  }, []);

  const startHold = useCallback((lx, az) => {
    publish(lx, az);
    clearInterval(holdTimerRef.current);
    holdTimerRef.current = setInterval(() => publish(lx, az), 150);
  }, [publish]);

  const stopHold = useCallback(() => {
    clearInterval(holdTimerRef.current);
  }, []);

  const safeHit = useCallback(async (code) => {
    if (!code) return;
    if (reportedRef.current[code]) return;
    try {
      await onObjectiveHit?.(code);
      reportedRef.current[code] = true;
    } catch (err) {
      console.error("[TSIM] safeHit ERROR ->", code, err);
    }
  }, [onObjectiveHit]);

  // Suscripción fija a /turtle1/pose (throttle para no saturar renders)
  useStableRosSub({
    connected,
    subscribeTopic,
    topic: "/turtle1/pose",
    type: "turtlesim/Pose",
    opts: subOpts,
    onMessage: (msg) => {
      turtlesRef.current = { ...turtlesRef.current, turtle1: { pose: msg } };
      setTurtlesSafe(turtlesRef.current);
    },
  });

  // (Re)advertise cmd_vel para la activa (solo cuando hay conexión o cambia active)
  useEffect(() => {
    if (!connected || !active) return undefined;
    try {
      cmdRef.current?.unadvertise?.();
      cmdRef.current = advertise(`/${active}/cmd_vel`, "geometry_msgs/Twist");
      // eslint-disable-next-line no-console
      console.log(`[TSIM] Advertise cmd_vel -> /${active}/cmd_vel`);
    } catch (e) {
      console.error("[TSIM] advertise error:", e);
    }
    return () => {
      try {
        cmdRef.current?.unadvertise?.();
      } catch (e) {
        console.error("[TSIM] unadvertise error:", e);
      }
    };
  }, [connected, active, advertise]);

  // Helper para suscribirse a pose por nombre (para spawns puntuales)
  const subscribePoseOnce = useCallback((name) => {
    return subscribeTopic(`/${name}/pose`, "turtlesim/Pose", (msg) => {
      turtlesRef.current = { ...turtlesRef.current, [name]: { pose: msg } };
      setTurtlesSafe(turtlesRef.current);
    }, { throttle_rate: 50, queue_size: 1 });
  }, [subscribeTopic, setTurtlesSafe]);

  // Spawn
  const spawnTurtle = useCallback(async (nameOpt) => {
    if (!connected) return;
    const rand = (min, max) => min + Math.random() * (max - min);
    const payload = {
      x: rand(1.0, WORLD - 1.0),
      y: rand(1.0, WORLD - 1.0),
      theta: rand(-Math.PI, Math.PI),
      name: nameOpt || "",
    };
    try {
      const res = await callService("/spawn", "turtlesim/Spawn", payload);
      const newName = res?.name || nameOpt || "turtle?";
      subscribePoseOnce(newName); // sub puntual para esta tortuga
      setActive(newName);
      if (!reportedRef.current[OBJ_SPAWN_2ND]) {
        await safeHit(OBJ_SPAWN_2ND);
      }
    } catch (err) {
      console.error("[TSIM] spawn ERROR:", err);
    }
  }, [connected, callService, subscribePoseOnce, safeHit]);

  // DPR sync (canvas HiDPI)
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return undefined;
    const ctx = canvas.getContext("2d");
    const ro = new ResizeObserver((entries) => {
      const { width, height } = entries[0].contentRect;
      const dpr = Math.max(1, window.devicePixelRatio || 1);
      const bw = Math.round(width * dpr);
      const bh = Math.round(height * dpr);
      if (canvas.width !== bw || canvas.height !== bh) {
        canvas.width = bw; canvas.height = bh;
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      }
    });
    ro.observe(canvas);
    return () => ro.disconnect();
  }, []);

  // Draw loop: usa `turtles` y `active` como fuentes de verdad
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return undefined;
    const ctx = canvas.getContext("2d");

    let rafId;
    const draw = () => {
      const rect = canvas.getBoundingClientRect();
      const W = rect.width, H = rect.height;

      ctx.clearRect(0, 0, W, H);
      ctx.fillStyle = "rgba(0,0,0,.55)";
      ctx.fillRect(0, 0, W, H);

      const scale = Math.min(W, H) / WORLD;
      const view = WORLD * scale;
      const ox = (W - view) / 2;
      const oy = (H - view) / 2;
      const mapX = (x) => ox + x * scale;
      const mapY = (y) => oy + (WORLD - y) * scale;

      // grid
      ctx.strokeStyle = "#1f2a4a";
      ctx.lineWidth = 1;
      for (let i = 0; i <= 11; i++) {
        const g = (i / WORLD) * view;
        ctx.beginPath(); ctx.moveTo(ox + g, oy); ctx.lineTo(ox + g, oy + view); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(ox, oy + g); ctx.lineTo(ox + view, oy + g); ctx.stroke();
      }

      // goal zone
      const gx0 = mapX(GOAL_BOUNDS.minX);
      const gy0 = mapY(GOAL_BOUNDS.minY);
      const gx1 = mapX(WORLD);
      const gy1 = mapY(WORLD);
      ctx.fillStyle = "rgba(125,249,255,0.08)";
      ctx.fillRect(gx0, gy1, gx1 - gx0, gy0 - gy1);
      ctx.strokeStyle = "rgba(125,249,255,0.35)";
      ctx.strokeRect(gx0, gy1, gx1 - gx0, gy0 - gy1);

      // turtles
      Object.entries(turtles).forEach(([name, data]) => {
        const p = data.pose; if (!p) return;
        const px = mapX(p.x), py = mapY(p.y);
        const k = view / 560;
        const nose = 16 * k, tail = 12 * k;

        ctx.save();
        ctx.translate(px, py);
        ctx.rotate(-p.theta);
        const isActive = name === active;
        ctx.fillStyle = isActive ? "#7df9ff" : "#10b981";
        ctx.strokeStyle = isActive ? "#7df9ffAA" : "#10b981AA";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(nose, 0);
        ctx.lineTo(-tail, 8);
        ctx.lineTo(-tail * 0.66, 0);
        ctx.lineTo(-tail, -8);
        ctx.closePath();
        ctx.fill(); ctx.stroke();
        ctx.restore();

        ctx.fillStyle = "#e6f1ff";
        ctx.font = "12px ui-monospace, monospace";
        ctx.fillText(name, px + 6, py - 6);
      });

      rafId = requestAnimationFrame(draw);
    };

    rafId = requestAnimationFrame(draw);
    return () => cancelAnimationFrame(rafId);
  }, [turtles, active]);

  // Teclado
  useEffect(() => {
    const down = (e) => {
      if (!connected || e.repeat) return;
      if (["ArrowUp","KeyW"].includes(e.code)) startHold(1.5, 0);
      if (["ArrowDown","KeyS"].includes(e.code)) startHold(-1.0, 0);
      if (["ArrowLeft","KeyA"].includes(e.code)) startHold(0, 2.0);
      if (["ArrowRight","KeyD"].includes(e.code)) startHold(0, -2.0);
      if (e.code === "Space") publish(0, 0);
    };
    const up = () => stopHold();
    window.addEventListener("keydown", down);
    window.addEventListener("keyup", up);
    return () => {
      window.removeEventListener("keydown", down);
      window.removeEventListener("keyup", up);
      stopHold();
    };
  }, [connected, startHold, stopHold, publish]);

  const turtleNames = useMemo(() => Object.keys(turtles).sort(), [turtles]);
  const activePose = turtles[active]?.pose;

  // Detección de entrada a meta
  const prevInGoalRef = useRef(false);
  const levelCompletedRef = useRef(false);
  useEffect(() => {
    const p = turtles["turtle1"]?.pose;
    if (!p) return;
    const inGoalNow = p.x >= GOAL_BOUNDS.minX && p.y >= GOAL_BOUNDS.minY;
    const wasInGoal = prevInGoalRef.current;

    if (inGoalNow && !wasInGoal) {
      if (!reportedRef.current[OBJ_REACH_GOAL]) {
        safeHit(OBJ_REACH_GOAL);
      }
      if (!levelCompletedRef.current) {
        levelCompletedRef.current = true;
        try { onLevelCompleted?.(); } catch (err) { console.error("[TSIM] onLevelCompleted ERROR:", err); }
      }
    }
    prevInGoalRef.current = inGoalNow;
  }, [turtles, onLevelCompleted, safeHit]);

  return (
    <div className="tsim-screen">
      <div className={`tsim-status ${connected ? "ok" : "down"}`}>
        {connected ? "Connected" : "Disconnected"}
      </div>

      <header className="tsim-header">
        <h1 className="tsim-title">Turtlesim</h1>
        <div className="tsim-header__spacer" />
        <ul className="tsim-objectives">
          {objectives.map((o) => {
            const desc = o.description || o.Description || o.code;
            const done = o.user_progress?.achieved;
            return (
              <li key={o.code} className={done ? "done" : ""} title={o.code}>
                <span className="dot" />
                <span className="desc">{desc}</span>
                {done && <span className="check">✔</span>}
              </li>
            );
          })}
        </ul>
        <div className="tsim-header__active">
          {activePose ? (
            <span>
              {active} · x:{activePose.x.toFixed(2)} y:{activePose.y.toFixed(2)} θ:{activePose.theta.toFixed(2)}
            </span>
          ) : "…"}
        </div>
      </header>

      <main className="tsim-layout">
        <section className="tsim-canvasCard">
          <div className="tsim-canvasWrap">
            <canvas ref={canvasRef} className="tsim-canvas" />
          </div>
        </section>

        <aside className="tsim-panel">
          <div className="tsim-row">
            <label className="tsim-label">Control</label>
            <div className="tsim-controls">
              <Btn text="↑" onDown={() => startHold(1.5, 0)} onUp={stopHold} disabled={!connected} />
              <Btn text="↓" onDown={() => startHold(-1.0, 0)} onUp={stopHold} disabled={!connected} />
              <Btn text="←" onDown={() => startHold(0, 2.0)} onUp={stopHold} disabled={!connected} />
              <Btn text="→" onDown={() => startHold(0, -2.0)} onUp={stopHold} disabled={!connected} />
              <Btn text="■" onClick={() => publish(0, 0)} disabled={!connected} />
            </div>
            <div className="tsim-hint">W/A/S/D o flechas · Space para frenar</div>
          </div>

          <div className="tsim-row tsim-row--inline">
            <label className="tsim-label">Active</label>
            <select
              className="tsim-select"
              value={active}
              onChange={(e) => setActive(e.target.value)}
              disabled={!connected || turtleNames.length === 0}
            >
              {[active, ...turtleNames.filter((n) => n !== active)].map((n) => (
                <option key={n} value={n}>{n}</option>
              ))}
            </select>
          </div>

          <div className="tsim-row tsim-row--inline">
            <label className="tsim-label">Spawn</label>
            <div className="tsim-spawn">
              <input className="tsim-input" placeholder="optional name" id="tname" />
              <button
                className="btn"
                disabled={!connected}
                onClick={() => {
                  const el = document.getElementById("tname");
                  const name = el.value.trim();
                  spawnTurtle(name || undefined);
                  el.value = "";
                }}
              >+ Add</button>
            </div>
          </div>
        </aside>
      </main>
    </div>
  );
}

function Btn({ text, onDown, onUp, onClick, disabled }) {
  return (
    <button
      className="tsim-btn"
      disabled={disabled}
      onMouseDown={onDown}
      onMouseUp={onUp}
      onMouseLeave={onUp}
      onTouchStart={(e) => { e.preventDefault(); onDown?.(); }}
      onTouchEnd={(e) => { e.preventDefault(); onUp?.(); }}
      onClick={onClick}
    >
      {text}
    </button>
  );
}

TurtleSimPage.propTypes = {
  onObjectiveHit: PropTypes.func,
  onLevelCompleted: PropTypes.func,
};

TurtleSimPage.defaultProps = {
  onObjectiveHit: undefined,
  onLevelCompleted: undefined,
};
