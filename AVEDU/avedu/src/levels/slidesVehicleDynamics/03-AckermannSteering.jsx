// src/levels/slidesVehicleDynamics/03-AckermannSteering.jsx
import React, { useState, useRef, useEffect } from "react";
import "../../components/slides/SlideLayout.scss";

export const meta = {
  id: "vd-ackermann",
  title: "Ackermann Steering Geometry",
  order: 3,
  objectiveCode: "vd-slide-ackermann",
};

function AckermannVisualization({ wheelbase, trackWidth, innerAngle, carX = 0, carY = 0, carHeading = 0, pathHistory = [] }) {
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

    const scale = 40;
    const centerX = w / 2;
    const centerY = h / 2 + 50;

    // Draw path history (trail)
    if (pathHistory.length > 1) {
      ctx.strokeStyle = "rgba(125, 249, 255, 0.4)";
      ctx.lineWidth = 2;
      ctx.beginPath();
      for (let i = 0; i < pathHistory.length; i++) {
        const px = centerX + (pathHistory[i].x - carX) * scale;
        const py = centerY + (pathHistory[i].y - carY) * scale;
        if (i === 0) {
          ctx.moveTo(px, py);
        } else {
          ctx.lineTo(px, py);
        }
      }
      ctx.stroke();
    }

    // Calculate outer wheel angle using Ackermann formula
    const innerRad = (innerAngle * Math.PI) / 180;
    const L = wheelbase;
    const t = trackWidth;

    // Ackermann formula: cot(δo) = cot(δi) + t/L
    const cotInner = 1 / Math.tan(innerRad);
    const cotOuter = cotInner + t / L;
    const outerRad = Math.atan(1 / cotOuter);
    const outerAngle = (outerRad * 180) / Math.PI;

    // Calculate ICR position relative to car
    let icrOffsetX = 0;
    let icrOffsetY = 0;
    let R = Infinity;

    if (Math.abs(innerRad) > 0.001) {
      R = L / Math.tan(Math.abs(innerRad));

      // Calculate rear axle position
      const rearAxleOffsetX = -(wheelbase / 2) * Math.cos(carHeading);
      const rearAxleOffsetY = -(wheelbase / 2) * Math.sin(carHeading);
      const rearAxleX = carX + rearAxleOffsetX;
      const rearAxleY = carY + rearAxleOffsetY;

      // ICR perpendicular to car's heading
      const carForwardAngle = carHeading - Math.PI / 2;
      const perpAngle = carForwardAngle + (innerAngle > 0 ? Math.PI / 2 : -Math.PI / 2);
      icrOffsetX = R * Math.cos(perpAngle);
      icrOffsetY = R * Math.sin(perpAngle);

      var icrWorldX = rearAxleX + icrOffsetX;
      var icrWorldY = rearAxleY + icrOffsetY;
    } else {
      var icrWorldX = carX;
      var icrWorldY = carY;
    }

    // Transform world coordinates to screen (camera follows car)
    const screenCarX = centerX;
    const screenCarY = centerY;
    const screenIcrX = centerX + (icrWorldX - carX) * scale;
    const screenIcrY = centerY + (icrWorldY - carY) * scale;

    // Draw vehicle
    ctx.save();
    ctx.translate(screenCarX, screenCarY);
    ctx.rotate(carHeading);

    // Vehicle body
    ctx.strokeStyle = "#7df9ff";
    ctx.lineWidth = 3;
    ctx.strokeRect(
      -trackWidth * scale / 2,
      -wheelbase * scale / 2,
      trackWidth * scale,
      wheelbase * scale
    );

    const frontY = -wheelbase * scale / 2;
    const rearY = wheelbase * scale / 2;
    const leftX = -trackWidth * scale / 2;
    const rightX = trackWidth * scale / 2;

    // Determine which wheel is inner/outer based on turn direction
    const isLeftTurn = innerAngle > 0;
    const innerWheel = isLeftTurn ? { x: leftX, angle: innerRad } : { x: rightX, angle: innerRad };
    const outerWheel = isLeftTurn ? { x: rightX, angle: outerRad } : { x: leftX, angle: outerRad };

    // Draw inner wheel (pink)
    ctx.strokeStyle = "#ff5cf4";
    ctx.lineWidth = 4;
    ctx.save();
    ctx.translate(innerWheel.x, frontY);
    ctx.rotate(innerWheel.angle);
    ctx.beginPath();
    ctx.moveTo(0, -20);
    ctx.lineTo(0, 20);
    ctx.stroke();
    ctx.restore();

    // Draw outer wheel (cyan)
    ctx.strokeStyle = "#7df9ff";
    ctx.lineWidth = 4;
    ctx.save();
    ctx.translate(outerWheel.x, frontY);
    ctx.rotate(outerWheel.angle);
    ctx.beginPath();
    ctx.moveTo(0, -20);
    ctx.lineTo(0, 20);
    ctx.stroke();
    ctx.restore();

    // Rear wheels
    ctx.strokeStyle = "#a8b3d1";
    ctx.lineWidth = 4;
    ctx.beginPath();
    ctx.moveTo(leftX, rearY - 20);
    ctx.lineTo(leftX, rearY + 20);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(rightX, rearY - 20);
    ctx.lineTo(rightX, rearY + 20);
    ctx.stroke();

    // Direction indicator (front of car)
    ctx.fillStyle = "#7df9ff";
    ctx.beginPath();
    ctx.moveTo(0, -wheelbase * scale / 2 - 10);
    ctx.lineTo(-8, -wheelbase * scale / 2 - 20);
    ctx.lineTo(8, -wheelbase * scale / 2 - 20);
    ctx.closePath();
    ctx.fill();

    ctx.restore();

    // Draw lines to ICR from both front wheels
    if (Math.abs(innerRad) > 0.001 && Math.abs(screenIcrX - centerX) < w * 2) {
      ctx.setLineDash([5, 5]);

      // Line from inner wheel
      ctx.strokeStyle = "rgba(255, 92, 244, 0.5)";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(screenCarX + innerWheel.x, screenCarY + frontY);
      ctx.lineTo(screenIcrX, screenIcrY);
      ctx.stroke();

      // Line from outer wheel
      ctx.strokeStyle = "rgba(125, 249, 255, 0.5)";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(screenCarX + outerWheel.x, screenCarY + frontY);
      ctx.lineTo(screenIcrX, screenIcrY);
      ctx.stroke();

      ctx.setLineDash([]);

      // Turning circle arc
      ctx.strokeStyle = "rgba(125, 249, 255, 0.3)";
      ctx.lineWidth = 1;
      ctx.beginPath();
      const radius = Math.sqrt(
        Math.pow(icrOffsetX, 2) + Math.pow(icrOffsetY, 2)
      ) * scale;
      ctx.arc(screenIcrX, screenIcrY, radius, 0, Math.PI * 2);
      ctx.stroke();

      // ICR point
      ctx.fillStyle = "#ffd700";
      ctx.beginPath();
      ctx.arc(screenIcrX, screenIcrY, 10, 0, Math.PI * 2);
      ctx.fill();

      ctx.font = "bold 14px monospace";
      ctx.fillStyle = "#ffd700";
      ctx.fillText("ICR", screenIcrX + 15, screenIcrY);

      // Display turning radius
      ctx.font = "12px monospace";
      ctx.fillStyle = "#7df9ff";
      ctx.fillText(`R = ${R.toFixed(2)} m`, 20, h - 20);
    } else {
      ctx.font = "14px monospace";
      ctx.fillStyle = "#a8b3d1";
      ctx.fillText("Straight ahead (ICR at infinity)", centerX - 120, h - 20);
    }

    // Display info
    ctx.font = "12px monospace";
    ctx.fillStyle = "#a8b3d1";
    ctx.fillText(`Position: (${carX.toFixed(1)}, ${carY.toFixed(1)})`, 20, 30);
    ctx.fillText(`Heading: ${((carHeading * 180 / Math.PI) % 360).toFixed(1)}°`, 20, 50);

    ctx.fillStyle = "#ff5cf4";
    ctx.fillText(`Inner: ${innerAngle.toFixed(1)}°`, 20, 75);
    ctx.fillStyle = "#7df9ff";
    ctx.fillText(`Outer: ${outerAngle.toFixed(1)}°`, 20, 95);

    if (Math.abs(innerRad) > 0.001) {
      ctx.fillStyle = "#ffd700";
      ctx.fillText(`Δ = ${Math.abs(innerAngle - outerAngle).toFixed(2)}°`, 20, 115);
    }

  }, [wheelbase, trackWidth, innerAngle, carX, carY, carHeading, pathHistory]);

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

export default function AckermannSteering() {
  const [innerAngle, setInnerAngle] = useState(0);
  const [wheelbase, setWheelbase] = useState(2.7);
  const [trackWidth, setTrackWidth] = useState(1.5);
  const [isPlaying, setIsPlaying] = useState(false);
  const [carX, setCarX] = useState(0);
  const [carY, setCarY] = useState(0);
  const [carHeading, setCarHeading] = useState(0);
  const [pathHistory, setPathHistory] = useState([]);
  const animationRef = useRef(null);

  // Calculate outer angle using Ackermann formula
  const innerRad = (innerAngle * Math.PI) / 180;
  const cotInner = innerRad !== 0 ? 1 / Math.tan(innerRad) : Infinity;
  const cotOuter = cotInner + trackWidth / wheelbase;
  const outerAngle = cotOuter !== Infinity ? (Math.atan(1 / cotOuter) * 180) / Math.PI : 0;

  // Animation loop
  useEffect(() => {
    if (!isPlaying) {
      if (animationRef.current) {
        cancelAnimationFrame(animationRef.current);
        animationRef.current = null;
      }
      return;
    }

    let lastTime = performance.now();
    const speed = 2.5; // meters per second

    const animate = (currentTime) => {
      const deltaTime = (currentTime - lastTime) / 1000; // Convert to seconds
      lastTime = currentTime;

      // Calculate movement using Ackermann steering geometry
      const innerRad = (innerAngle * Math.PI) / 180;

      setCarHeading(prevHeading => {
        let newHeading = prevHeading;

        if (Math.abs(innerRad) > 0.001) {
          // Ackermann steering: Use the average of inner and outer wheel angles
          // or the turning radius from the rear axle center
          // For bicycle model, we use the inner wheel angle as reference
          // R = L / tan(δ_inner) where L is wheelbase
          const R = wheelbase / Math.tan(Math.abs(innerRad));

          // Angular velocity around ICR
          const angularVelocity = speed / R; // radians per second
          const angleChange = angularVelocity * deltaTime * (innerRad > 0 ? 1 : -1);
          newHeading = prevHeading + angleChange;
        }

        // Update position based on current heading
        // In Ackermann, the rear axle center follows a circular path
        setCarX(prevX => {
          const newX = prevX + speed * deltaTime * Math.cos(newHeading);
          return newX;
        });
        setCarY(prevY => {
          const newY = prevY + speed * deltaTime * Math.sin(newHeading);
          return newY;
        });

        // Update path history
        setPathHistory(prev => {
          const newPath = [...prev, { x: carX, y: carY }];
          // Keep only last 100 points
          return newPath.slice(-100);
        });

        return newHeading;
      });

      animationRef.current = requestAnimationFrame(animate);
    };

    animationRef.current = requestAnimationFrame(animate);

    return () => {
      if (animationRef.current) {
        cancelAnimationFrame(animationRef.current);
      }
    };
  }, [isPlaying, innerAngle, wheelbase, carX, carY]);

  const handlePlayPause = () => {
    setIsPlaying(!isPlaying);
  };

  const handleReset = () => {
    setIsPlaying(false);
    setCarX(0);
    setCarY(0);
    setCarHeading(0);
    setPathHistory([]);
  };

  return (
    <div className="slide">
      <h2>Ackermann Steering Geometry</h2>

      <div className="slide-card">
        <div className="slide-card__title">Why Ackermann?</div>
        <p>
          In a turn, the <b>inner wheel</b> travels a tighter radius than the <b>outer wheel</b>.
          If both wheels had the same steering angle, they would fight each other (tire scrub).
          <b> Ackermann steering geometry</b> ensures both front wheels point toward the same
          instantaneous center of rotation (ICR), minimizing tire wear and improving handling.
        </p>
      </div>

      <div className="slide-columns">
        <div>
          <div className="slide-card">
            <div className="slide-card__title">Ackermann Formula</div>
            <div className="slide-code">
              cot(δₒ) = cot(δᵢ) + t/L
              <br /><br />
              Or equivalently:
              <br />
              tan(δₒ) = L / (R + t/2)
              <br />
              tan(δᵢ) = L / (R - t/2)
              <br /><br />
              Where:
              <br />• δₒ = Outer wheel angle
              <br />• δᵢ = Inner wheel angle
              <br />• L = Wheelbase
              <br />• t = Track width
              <br />• R = Turning radius
            </div>
          </div>

          <div className="slide-card" style={{ marginTop: ".75rem" }}>
            <div className="slide-card__title">Interactive Controls</div>
            <div className="slide-controls">
              <div className="slide-slider">
                <span className="slide-slider__label">
                  Inner Angle: <span className="slide-slider__value">{innerAngle}°</span>
                </span>
                <input
                  type="range"
                  min="-30"
                  max="30"
                  step="0.5"
                  value={innerAngle}
                  onChange={(e) => setInnerAngle(Number(e.target.value))}
                  className="slide-slider__input"
                />
              </div>

              <div className="slide-code" style={{ fontSize: "11px", padding: ".4rem" }}>
                Calculated Outer Angle: <b style={{ color: "#7df9ff" }}>{outerAngle.toFixed(2)}°</b>
                <br />
                Angle Difference: <b style={{ color: "#ff5cf4" }}>{Math.abs(innerAngle - outerAngle).toFixed(2)}°</b>
              </div>

              <div className="slide-slider">
                <span className="slide-slider__label">
                  Wheelbase: <span className="slide-slider__value">{wheelbase}m</span>
                </span>
                <input
                  type="range"
                  min="2.0"
                  max="4.0"
                  step="0.1"
                  value={wheelbase}
                  onChange={(e) => setWheelbase(Number(e.target.value))}
                  className="slide-slider__input"
                />
              </div>

              <div className="slide-slider">
                <span className="slide-slider__label">
                  Car Width: <span className="slide-slider__value">{trackWidth}m</span>
                </span>
                <input
                  type="range"
                  min="1.2"
                  max="2.0"
                  step="0.1"
                  value={trackWidth}
                  onChange={(e) => setTrackWidth(Number(e.target.value))}
                  className="slide-slider__input"
                />
              </div>

              <div style={{ display: "flex", gap: ".4rem", marginTop: ".25rem" }}>
                <button
                  onClick={handlePlayPause}
                  style={{
                    flex: 1,
                    padding: ".5rem .75rem",
                    background: isPlaying ? "#ff5cf4" : "#7df9ff",
                    color: "#0a0e1a",
                    border: "none",
                    borderRadius: "5px",
                    fontWeight: "600",
                    cursor: "pointer",
                    fontSize: "11px",
                    transition: "all 0.2s"
                  }}
                >
                  {isPlaying ? "⏸ Pause" : "▶ Play"}
                </button>
                <button
                  onClick={handleReset}
                  style={{
                    padding: ".5rem .75rem",
                    background: "#a8b3d1",
                    color: "#0a0e1a",
                    border: "none",
                    borderRadius: "5px",
                    fontWeight: "600",
                    cursor: "pointer",
                    fontSize: "11px",
                    transition: "all 0.2s"
                  }}
                >
                  ↺ Reset
                </button>
              </div>

              {isPlaying && (
                <div style={{
                  padding: ".5rem",
                  background: "rgba(125, 249, 255, 0.08)",
                  borderRadius: "4px",
                  fontSize: "10px",
                  color: "#7df9ff",
                  borderLeft: "2px solid #7df9ff",
                  marginTop: ".25rem"
                }}>
                  <strong>Tip:</strong> Watch how the inner and outer wheels maintain different angles while both pointing toward the ICR!
                </div>
              )}
            </div>
          </div>
        </div>

        <div className="slide-figure">
          <AckermannVisualization
            wheelbase={wheelbase}
            trackWidth={trackWidth}
            innerAngle={innerAngle}
            carX={carX}
            carY={carY}
            carHeading={carHeading}
            pathHistory={pathHistory}
          />
          <figcaption>
            Top view: The <span style={{ color: "#ff5cf4" }}>inner wheel (pink)</span> has a
            larger steering angle than the <span style={{ color: "#7df9ff" }}>outer wheel (cyan)</span>.
            Both wheel axes intersect at the <span style={{ color: "#ffd700" }}>ICR (gold point)</span>.
          </figcaption>
        </div>
      </div>

      <div className="slide-callout slide-callout--info">
        <b>Real-World Application:</b> Most vehicles use an approximation of Ackermann geometry.
        Perfect Ackermann works best at low speeds. At higher speeds, tire slip angles become significant,
        and vehicles may use less than 100% Ackermann (or even anti-Ackermann for race cars).
      </div>
    </div>
  );
}
