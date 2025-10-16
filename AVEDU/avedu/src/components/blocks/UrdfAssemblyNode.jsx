// components/blocks/UrdfAssemblyNode.jsx
import React from "react";
import { Position } from "@xyflow/react";
import HandleWithLabel from "./HandleWithLabel";

/**
 * URDF Assembly Node - Groups links and joints before sending to Robot
 * Provides better organization for complex robots
 */
export default function UrdfAssemblyNode({ id, data }) {
  const d = data || {};
  const edit = (patch) => d.onChange?.(id, patch);

  const links = Array.isArray(d.links) ? d.links : [];
  const joints = Array.isArray(d.joints) ? d.joints : [];

  return (
    <div className="rf-card rf-card--assembly" style={{ minWidth: 380 }}>
      <div className="rf-card__title">Assembly</div>

      <div className="rf-card__body" style={{ display: "grid", gap: ".5rem" }}>
        {/* Assembly Name */}
        <div className="rf-field">
          <label>Assembly Name</label>
          <input
            className="rf-input"
            placeholder="arm_assembly"
            value={d.name || ""}
            onChange={(e) => edit({ name: e.target.value })}
          />
        </div>

        {/* Description */}
        <div className="rf-field">
          <label>Description</label>
          <input
            className="rf-input"
            placeholder="Left arm assembly with 3 joints"
            value={d.description || ""}
            onChange={(e) => edit({ description: e.target.value })}
          />
        </div>

        {/* Statistics */}
        <div style={{
          display: "grid",
          gridTemplateColumns: "1fr 1fr",
          gap: ".4rem",
          padding: ".5rem",
          background: "rgba(0, 0, 0, 0.2)",
          borderRadius: "8px",
          marginTop: ".25rem"
        }}>
          <div style={{ textAlign: "center" }}>
            <div style={{ fontSize: "1.5em", fontWeight: "bold", color: "#4caf50" }}>
              {links.length}
            </div>
            <div style={{ fontSize: "0.75em", opacity: 0.8 }}>Links</div>
          </div>
          <div style={{ textAlign: "center" }}>
            <div style={{ fontSize: "1.5em", fontWeight: "bold", color: "#2196f3" }}>
              {joints.length}
            </div>
            <div style={{ fontSize: "0.75em", opacity: 0.8 }}>Joints</div>
          </div>
        </div>
      </div>

      {/* Input handles - positioned below description */}
      <HandleWithLabel
        type="target"
        position={Position.Left}
        id="links"
        label="links"
        color="green"
        top="60%"
      />
      <HandleWithLabel
        type="target"
        position={Position.Left}
        id="joints"
        label="joints"
        color="blue"
        top="75%"
      />

      {/* Output handle */}
      <HandleWithLabel
        type="source"
        position={Position.Right}
        id="assembly"
        label="assembly"
        color="purple"
      />
    </div>
  );
}
