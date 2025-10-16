// components/blocks/UrdfLinkNodeV2.jsx
import React from "react";
import { Position } from "@xyflow/react";
import HandleWithLabel from "./HandleWithLabel";

/**
 * URDF Link Node V2 - Improved modular version
 * Accepts connections from Inertial, Visual, and Collision nodes
 */
export default function UrdfLinkNodeV2({ id, data }) {
  const d = data || {};
  const edit = (patch) => d.onChange?.(id, patch);

  const visuals = Array.isArray(d.visuals) ? d.visuals : [];
  const collisions = Array.isArray(d.collisions) ? d.collisions : [];
  const hasInertial = !!d.inertial;

  return (
    <div className="rf-card rf-card--link" style={{ minWidth: 380 }}>
      <div className="rf-card__title">URDF Link</div>

      <div className="rf-card__body" style={{ display: "grid", gap: ".5rem" }}>
        <div className="rf-field">
          <label>Link Name</label>
          <input
            className="rf-input"
            placeholder="base_link"
            value={d.name || ""}
            onChange={(e) => edit({ name: e.target.value })}
          />
        </div>

        {/* Status indicators */}
        <div style={{
          display: "grid",
          gridTemplateColumns: "1fr 1fr 1fr",
          gap: ".4rem",
          padding: ".5rem",
          background: "rgba(0, 0, 0, 0.2)",
          borderRadius: "8px",
          marginTop: ".25rem"
        }}>
          <div style={{ textAlign: "center" }}>
            <div style={{
              fontSize: "1.2em",
              fontWeight: "bold",
              color: hasInertial ? "#ff9800" : "#666"
            }}>
              {hasInertial ? "✓" : "○"}
            </div>
            <div style={{ fontSize: "0.75em", opacity: 0.8 }}>Inertial</div>
          </div>
          <div style={{ textAlign: "center" }}>
            <div style={{
              fontSize: "1.2em",
              fontWeight: "bold",
              color: visuals.length > 0 ? "#2196f3" : "#666"
            }}>
              {visuals.length || "○"}
            </div>
            <div style={{ fontSize: "0.75em", opacity: 0.8 }}>Visual</div>
          </div>
          <div style={{ textAlign: "center" }}>
            <div style={{
              fontSize: "1.2em",
              fontWeight: "bold",
              color: collisions.length > 0 ? "#f44336" : "#666"
            }}>
              {collisions.length || "○"}
            </div>
            <div style={{ fontSize: "0.75em", opacity: 0.8 }}>Collision</div>
          </div>
        </div>
      </div>

      {/* Input handles - positioned below link name */}
      <HandleWithLabel
        type="target"
        position={Position.Left}
        id="inertial"
        label="inertial"
        color="orange"
        top="55%"
      />
      <HandleWithLabel
        type="target"
        position={Position.Left}
        id="visual"
        label="visual"
        color="blue"
        top="70%"
      />
      <HandleWithLabel
        type="target"
        position={Position.Left}
        id="collision"
        label="collision"
        color="red"
        top="85%"
      />

      <HandleWithLabel
        type="source"
        position={Position.Right}
        id="link"
        label="link"
        color="green"
        top="50%"
      />
    </div>
  );
}
