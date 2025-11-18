// src/levels/slidesVehicleDynamics/02-InstantaneousCenterRotation.jsx
import React, { useState, useRef, useEffect } from "react";
import "../../components/slides/SlideLayout.scss";

export const meta = {
  id: "vd-icr",
  title: "Instantaneous Center of Rotation (ICR)",
  order: 2,
  objectiveCode: "vd-slide-icr",
};

function ICRVisualization({ wheelbase, trackWidth, steeringAngle, carX = 0, carY = 0, carHeading = 0, pathHistory = [] }) {
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

    // Scale and center
    const scale = 30;
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

    // Convert steering angle to radians
    const deltaRad = (steeringAngle * Math.PI) / 180;

    // Calculate turning radius and ICR position relative to car
    let R = Infinity; // Turning radius
    let icrOffsetX = 0;
    let icrOffsetY = 0;

    if (Math.abs(deltaRad) > 0.001) {
      R = wheelbase / Math.tan(Math.abs(deltaRad));

      // First, calculate the rear axle position (center of rear axle is the reference point)
      const rearAxleOffsetX = -(wheelbase / 2) * Math.cos(carHeading);
      const rearAxleOffsetY = -(wheelbase / 2) * Math.sin(carHeading);
      const rearAxleX = carX + rearAxleOffsetX;
      const rearAxleY = carY + rearAxleOffsetY;

      // ICR is perpendicular to the car's heading at distance R from rear axle
      // The car is drawn pointing UP in its local frame (-Y direction)
      // carHeading rotates the car from this base orientation
      // So actual forward direction is carHeading - π/2 (rotated 90° from standard)
      // For ICR perpendicular to car: need to be perpendicular to (carHeading - π/2)
      // Left perpendicular: add another +π/2, Right perpendicular: add -π/2
      const carForwardAngle = carHeading - Math.PI / 2;
      const perpAngle = carForwardAngle + (deltaRad > 0 ? Math.PI / 2 : -Math.PI / 2);
      icrOffsetX = R * Math.cos(perpAngle);
      icrOffsetY = R * Math.sin(perpAngle);

      // ICR world position is rear axle position + perpendicular offset
      var icrWorldX = rearAxleX + icrOffsetX;
      var icrWorldY = rearAxleY + icrOffsetY;
    } else {
      var icrWorldX = carX;
      var icrWorldY = carY;
    }

    // Transform world coordinates to screen coordinates (camera follows car)
    const screenCarX = centerX;
    const screenCarY = centerY;
    const screenIcrX = centerX + (icrWorldX - carX) * scale;
    const screenIcrY = centerY + (icrWorldY - carY) * scale;

    // Draw ICR if turning
    if (Math.abs(deltaRad) > 0.001 && Math.abs(screenIcrX - centerX) < w * 2) {
      // ICR point
      ctx.fillStyle = "#ff5cf4";
      ctx.beginPath();
      ctx.arc(screenIcrX, screenIcrY, 8, 0, Math.PI * 2);
      ctx.fill();

      // Label
      ctx.font = "14px monospace";
      ctx.fillStyle = "#ff5cf4";
      ctx.fillText("ICR", screenIcrX + 15, screenIcrY - 10);

      // Line from rear axle center to ICR
      ctx.strokeStyle = "rgba(255, 92, 244, 0.5)";
      ctx.lineWidth = 2;
      ctx.setLineDash([5, 5]);
      ctx.beginPath();
      const rearAxleOffsetX = -(wheelbase / 2) * Math.cos(carHeading);
      const rearAxleOffsetY = -(wheelbase / 2) * Math.sin(carHeading);
      ctx.moveTo(screenCarX + rearAxleOffsetX * scale, screenCarY + rearAxleOffsetY * scale);
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

      // Display turning radius
      ctx.font = "12px monospace";
      ctx.fillStyle = "#7df9ff";
      ctx.fillText(`R = ${R.toFixed(2)} m`, 20, h - 20);
    } else {
      ctx.font = "14px monospace";
      ctx.fillStyle = "#a8b3d1";
      ctx.fillText("Straight ahead (ICR at infinity)", centerX - 120, h - 20);
    }

    // Draw vehicle (simplified top view)
    ctx.save();
    ctx.translate(screenCarX, screenCarY);
    ctx.rotate(carHeading);

    // Vehicle body (rectangle)
    ctx.strokeStyle = "#7df9ff";
    ctx.lineWidth = 3;
    ctx.strokeRect(
      -trackWidth * scale / 2,
      -wheelbase * scale / 2,
      trackWidth * scale,
      wheelbase * scale
    );

    // Front wheels
    const frontY = -wheelbase * scale / 2;
    const rearY = wheelbase * scale / 2;
    const leftX = -trackWidth * scale / 2;
    const rightX = trackWidth * scale / 2;

    // Draw front wheels with steering angle
    ctx.strokeStyle = "#ff5cf4";
    ctx.lineWidth = 4;

    // Left front wheel
    ctx.save();
    ctx.translate(leftX, frontY);
    ctx.rotate(deltaRad);
    ctx.beginPath();
    ctx.moveTo(-10, -15);
    ctx.lineTo(-10, 15);
    ctx.stroke();
    ctx.restore();

    // Right front wheel
    ctx.save();
    ctx.translate(rightX, frontY);
    ctx.rotate(deltaRad);
    ctx.beginPath();
    ctx.moveTo(10, -15);
    ctx.lineTo(10, 15);
    ctx.stroke();
    ctx.restore();

    // Rear wheels (no steering)
    ctx.strokeStyle = "#a8b3d1";
    ctx.lineWidth = 4;
    ctx.beginPath();
    ctx.moveTo(leftX - 10, rearY - 15);
    ctx.lineTo(leftX - 10, rearY + 15);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(rightX + 10, rearY - 15);
    ctx.lineTo(rightX + 10, rearY + 15);
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

    // Display info
    ctx.font = "12px monospace";
    ctx.fillStyle = "#a8b3d1";
    ctx.fillText(`Position: (${carX.toFixed(1)}, ${carY.toFixed(1)})`, 20, 30);
    ctx.fillText(`Heading: ${((carHeading * 180 / Math.PI) % 360).toFixed(1)}°`, 20, 50);

  }, [wheelbase, trackWidth, steeringAngle, carX, carY, carHeading, pathHistory]);

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

export default function InstantaneousCenterRotation() {
  const [steeringAngle, setSteeringAngle] = useState(0);
  const [wheelbase, setWheelbase] = useState(2.7);
  const [trackWidth, setTrackWidth] = useState(1.5);
  const [isPlaying, setIsPlaying] = useState(false);
  const [carX, setCarX] = useState(0);
  const [carY, setCarY] = useState(0);
  const [carHeading, setCarHeading] = useState(0);
  const [pathHistory, setPathHistory] = useState([]);
  const animationRef = useRef(null);

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

      // Calculate movement based on current steering angle and heading
      const deltaRad = (steeringAngle * Math.PI) / 180;

      setCarHeading(prevHeading => {
        let newHeading = prevHeading;

        if (Math.abs(deltaRad) > 0.001) {
          // Turning - use bicycle model
          const R = wheelbase / Math.tan(Math.abs(deltaRad));
          const angularVelocity = speed / R; // radians per second
          const angleChange = angularVelocity * deltaTime * (deltaRad > 0 ? 1 : -1);
          newHeading = prevHeading + angleChange;
        }

        // Update position based on current heading
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
  }, [isPlaying, steeringAngle, wheelbase]);

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
      <h2>Instantaneous Center of Rotation (ICR)</h2>

      <div className="slide-card">
        <div className="slide-card__title">Concept</div>
        <p>
          The <b>Instantaneous Center of Rotation (ICR)</b> is the point about which a vehicle
          appears to rotate at any given instant. For a vehicle with front-wheel steering,
          the ICR lies on the extension of the rear axle line.
        </p>
      </div>

      <div className="slide-card" style={{ marginTop: ".75rem" }}>
        <div className="slide-card__title">Key Formula</div>
        <div className="slide-code">
          R = L / tan(δ)
          <br /><br />
          Where:
          <br />• R = Turning radius
          <br />• L = Wheelbase
          <br />• δ = Steering angle
        </div>
      </div>

      <div className="slide-card" style={{ marginTop: ".75rem" }}>
        <div className="slide-card__title">Interactive Simulation</div>

        <div style={{ position: "relative" }}>
          <ICRVisualization
            wheelbase={wheelbase}
            trackWidth={trackWidth}
            steeringAngle={steeringAngle}
            carX={carX}
            carY={carY}
            carHeading={carHeading}
            pathHistory={pathHistory}
          />

          <div className="slide-controls" style={{ marginTop: ".75rem" }}>
            <div className="slide-slider">
              <span className="slide-slider__label">
                Steering: <span className="slide-slider__value">{steeringAngle}°</span>
              </span>
              <input
                type="range"
                min="-30"
                max="30"
                step="1"
                value={steeringAngle}
                onChange={(e) => setSteeringAngle(Number(e.target.value))}
                className="slide-slider__input"
              />
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
                <strong>Tip:</strong> Change the steering angle while driving to see the ICR shift in real-time!
              </div>
            )}
          </div>
        </div>

        <p style={{ marginTop: "1rem", fontSize: "13px", color: "#a8b3d1", fontStyle: "italic" }}>
          The pink dot shows the ICR location, and the cyan circle shows the turning radius.
          The blue trail shows the car's path.
        </p>
      </div>

      <div className="slide-callout slide-callout--info" style={{ marginTop: ".75rem" }}>
        <b>Key Insight:</b> When the steering angle is zero (straight ahead), the ICR is at infinity.
        As you increase the steering angle, the turning radius decreases and the ICR moves closer to the vehicle.
      </div>
    </div>
  );
}
