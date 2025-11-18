// src/levels/slidesVehicleDynamics/04-BicycleModel.jsx
import React, { useState, useRef, useEffect } from "react";
import "../../components/slides/SlideLayout.scss";

export const meta = {
  id: "vd-bicycle",
  title: "Bicycle Model (2-Wheel Model)",
  order: 4,
  objectiveCode: "vd-slide-bicycle",
};

function BicycleModelVisualization({ lf, lr, delta, beta, v, showVelocities }) {
  const canvasRef = useRef(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    const w = canvas.width;
    const h = canvas.height;

    // Clear canvas
    ctx.fillStyle = "#0a0e1a";
    ctx.fillRect(0, 0, w, h);

    const scale = 100;
    const centerX = w / 2;
    const centerY = h / 2;

    const L = lf + lr;
    const deltaRad = (delta * Math.PI) / 180;
    const betaRad = (beta * Math.PI) / 180;

    // Draw coordinate system
    ctx.strokeStyle = "rgba(255,255,255,0.2)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(0, centerY);
    ctx.lineTo(w, centerY);
    ctx.moveTo(centerX, 0);
    ctx.lineTo(centerX, h);
    ctx.stroke();

    ctx.font = "11px monospace";
    ctx.fillStyle = "rgba(255,255,255,0.4)";
    ctx.fillText("X", w - 15, centerY - 5);
    ctx.fillText("Y", centerX + 5, 15);

    // Vehicle body orientation
    ctx.save();
    ctx.translate(centerX, centerY);
    ctx.rotate(betaRad);

    // Draw vehicle body (simplified)
    ctx.strokeStyle = "#7df9ff";
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(0, -lf * scale);
    ctx.lineTo(0, lr * scale);
    ctx.stroke();

    // Front wheel (pointing forward, rotated by steering angle)
    const frontY = -lf * scale;
    ctx.strokeStyle = "#ff5cf4";
    ctx.lineWidth = 4;
    ctx.save();
    ctx.translate(0, frontY);
    ctx.rotate(deltaRad);  // Rotate by steering angle
    ctx.beginPath();
    ctx.moveTo(0, -15);  // Vertical orientation (pointing forward)
    ctx.lineTo(0, 15);
    ctx.stroke();
    ctx.restore();

    // Rear wheel (pointing forward, no rotation)
    const rearY = lr * scale;
    ctx.strokeStyle = "#7df9ff";
    ctx.lineWidth = 4;
    ctx.beginPath();
    ctx.moveTo(0, rearY - 15);  // Vertical orientation
    ctx.lineTo(0, rearY + 15);
    ctx.stroke();

    // Center of gravity (CG)
    ctx.fillStyle = "#ffd700";
    ctx.beginPath();
    ctx.arc(0, 0, 6, 0, Math.PI * 2);
    ctx.fill();
    ctx.font = "12px monospace";
    ctx.fillStyle = "#ffd700";
    ctx.fillText("CG", 10, -5);

    // Draw velocity vectors if enabled
    if (showVelocities && v > 0) {
      const vScale = v * 15;

      // Velocity at CG
      ctx.strokeStyle = "#7df9ff";
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(vScale, 0);
      ctx.stroke();

      // Arrow head for CG velocity
      ctx.fillStyle = "#7df9ff";
      ctx.beginPath();
      ctx.moveTo(vScale, 0);
      ctx.lineTo(vScale - 8, -5);
      ctx.lineTo(vScale - 8, 5);
      ctx.closePath();
      ctx.fill();

      ctx.font = "11px monospace";
      ctx.fillText("V", vScale + 5, -5);

      // Front wheel velocity (aligned with wheel orientation)
      ctx.save();
      ctx.translate(0, frontY);
      ctx.rotate(deltaRad);  // Rotate by steering angle

      ctx.strokeStyle = "#ff5cf4";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(0, -vScale * 0.8);  // Vertical, pointing forward (negative Y)
      ctx.stroke();

      ctx.fillStyle = "#ff5cf4";
      ctx.beginPath();
      ctx.moveTo(0, -vScale * 0.8);
      ctx.lineTo(-4, -vScale * 0.8 + 6);
      ctx.lineTo(4, -vScale * 0.8 + 6);
      ctx.closePath();
      ctx.fill();

      ctx.fillText("Vf", 5, -vScale * 0.8 + 5);
      ctx.restore();
    }

    ctx.restore();

    // Draw labels and dimensions
    ctx.strokeStyle = "rgba(255,255,255,0.3)";
    ctx.setLineDash([3, 3]);
    ctx.lineWidth = 1;

    // lf dimension
    ctx.beginPath();
    ctx.moveTo(centerX + 30, centerY);
    ctx.lineTo(centerX + 30, centerY - lf * scale * Math.cos(betaRad));
    ctx.stroke();

    ctx.font = "12px monospace";
    ctx.fillStyle = "#ff5cf4";
    ctx.fillText(`lf=${lf.toFixed(1)}m`, centerX + 35, centerY - lf * scale * 0.5 * Math.cos(betaRad));

    // lr dimension
    ctx.beginPath();
    ctx.moveTo(centerX + 30, centerY);
    ctx.lineTo(centerX + 30, centerY + lr * scale * Math.cos(betaRad));
    ctx.stroke();

    ctx.fillStyle = "#7df9ff";
    ctx.fillText(`lr=${lr.toFixed(1)}m`, centerX + 35, centerY + lr * scale * 0.5 * Math.cos(betaRad));

    ctx.setLineDash([]);

    // Display parameters
    ctx.font = "13px monospace";
    ctx.fillStyle = "#a8b3d1";
    const params = [
      `δ (steering): ${delta.toFixed(1)}°`,
      `β (sideslip): ${beta.toFixed(1)}°`,
      `V (velocity): ${v.toFixed(1)} m/s`,
      `L (wheelbase): ${L.toFixed(2)} m`
    ];
    params.forEach((param, i) => {
      ctx.fillText(param, 15, h - 80 + i * 20);
    });

  }, [lf, lr, delta, beta, v, showVelocities]);

  return (
    <canvas
      ref={canvasRef}
      width={700}
      height={500}
      style={{
        width: "100%",
        height: "auto",
        border: "1px solid rgba(255,255,255,0.2)",
        borderRadius: "10px",
        background: "#0a0e1a"
      }}
    />
  );
}

export default function BicycleModel() {
  const [lf, setLf] = useState(1.2);
  const [lr, setLr] = useState(1.5);
  const [delta, setDelta] = useState(5);
  const [beta, setBeta] = useState(0);
  const [velocity, setVelocity] = useState(10);
  const [showVelocities, setShowVelocities] = useState(true);

  const L = lf + lr;

  return (
    <div className="slide">
      <h2>Bicycle Model (2-Wheel Simplification)</h2>

      <div className="slide-card">
        <div className="slide-card__title">Model Simplification</div>
        <p>
          The <b>bicycle model</b> simplifies a 4-wheel vehicle into a 2-wheel representation by
          combining the left and right wheels into single "virtual" wheels at the front and rear axles.
          This model captures the essential lateral dynamics while remaining mathematically tractable.
        </p>
      </div>

      <div className="slide-columns">
        <div>
          <div className="slide-card">
            <div className="slide-card__title">Kinematic Equations</div>
            <div className="slide-code">
              {`dx/dt = V·cos(ψ + β)
dy/dt = V·sin(ψ + β)
dψ/dt = (V/L)·tan(δ)

Where:
• x, y = Position
• ψ = Yaw angle (heading)
• β = Sideslip angle at CG
• δ = Steering angle
• V = Velocity
• L = Wheelbase (lf + lr)`}
            </div>
          </div>

          <div className="slide-card" style={{ marginTop: ".75rem" }}>
            <div className="slide-card__title">Key Parameters</div>
            <div style={{ display: "grid", gap: ".5rem", fontSize: "0.9rem" }}>
              <div>
                <b>lf:</b> Distance from CG to front axle
              </div>
              <div>
                <b>lr:</b> Distance from CG to rear axle
              </div>
              <div>
                <b>δ:</b> Front wheel steering angle
              </div>
              <div>
                <b>β:</b> Sideslip angle (angle between velocity and vehicle heading)
              </div>
            </div>
          </div>

          <div className="slide-card" style={{ marginTop: ".75rem" }}>
            <div className="slide-card__title">Interactive Controls</div>
            <div className="slide-controls">
              <div className="slide-slider">
                <span className="slide-slider__label">
                  lf (front): <span className="slide-slider__value">{lf.toFixed(1)}m</span>
                </span>
                <input
                  type="range"
                  min="0.8"
                  max="2.0"
                  step="0.1"
                  value={lf}
                  onChange={(e) => setLf(Number(e.target.value))}
                  className="slide-slider__input"
                />
              </div>

              <div className="slide-slider">
                <span className="slide-slider__label">
                  lr (rear): <span className="slide-slider__value">{lr.toFixed(1)}m</span>
                </span>
                <input
                  type="range"
                  min="0.8"
                  max="2.0"
                  step="0.1"
                  value={lr}
                  onChange={(e) => setLr(Number(e.target.value))}
                  className="slide-slider__input"
                />
              </div>

              <div className="slide-slider">
                <span className="slide-slider__label">
                  Steering δ: <span className="slide-slider__value">{delta.toFixed(1)}°</span>
                </span>
                <input
                  type="range"
                  min="-30"
                  max="30"
                  step="0.5"
                  value={delta}
                  onChange={(e) => setDelta(Number(e.target.value))}
                  className="slide-slider__input"
                />
              </div>

              <div className="slide-slider">
                <span className="slide-slider__label">
                  Sideslip β: <span className="slide-slider__value">{beta.toFixed(1)}°</span>
                </span>
                <input
                  type="range"
                  min="-15"
                  max="15"
                  step="0.5"
                  value={beta}
                  onChange={(e) => setBeta(Number(e.target.value))}
                  className="slide-slider__input"
                />
              </div>

              <div className="slide-slider">
                <span className="slide-slider__label">
                  Velocity: <span className="slide-slider__value">{velocity.toFixed(1)} m/s</span>
                </span>
                <input
                  type="range"
                  min="0"
                  max="30"
                  step="1"
                  value={velocity}
                  onChange={(e) => setVelocity(Number(e.target.value))}
                  className="slide-slider__input"
                />
              </div>

              <label style={{ display: "flex", gap: ".5rem", alignItems: "center", fontSize: "11px", padding: ".25rem 0" }}>
                <input
                  type="checkbox"
                  checked={showVelocities}
                  onChange={(e) => setShowVelocities(e.target.checked)}
                />
                <span>Show velocity vectors</span>
              </label>

              <div className="slide-code" style={{ fontSize: "11px", padding: ".4rem" }}>
                Wheelbase L = <b style={{ color: "#7df9ff" }}>{L.toFixed(2)} m</b>
                <br />
                Yaw rate: <b style={{ color: "#ff5cf4" }}>{((velocity / L) * Math.tan(delta * Math.PI / 180)).toFixed(3)} rad/s</b>
              </div>
            </div>
          </div>
        </div>

        <div className="slide-figure">
          <BicycleModelVisualization
            lf={lf}
            lr={lr}
            delta={delta}
            beta={beta}
            v={velocity}
            showVelocities={showVelocities}
          />
          <figcaption>
            Top view of bicycle model. The CG (gold) is located between front (pink) and rear (cyan) wheels.
            Adjust parameters to see how the model responds.
          </figcaption>
        </div>
      </div>

      <div className="slide-callout slide-callout--info">
        <b>Assumptions:</b> The kinematic bicycle model assumes no tire slip (infinite tire stiffness).
        This works well for low-speed maneuvers. For higher speeds and more accurate predictions,
        we need the <b>dynamic bicycle model</b> which includes tire forces and slip angles.
      </div>
    </div>
  );
}
