// components/blocks/UrdfInertialNode.jsx
import React, { useEffect } from "react";
import { Position, useStore } from "@xyflow/react";
import HandleWithLabel from "./HandleWithLabel";

/**
 * URDF Inertial Node - Modular component for link inertia
 * Can accept Coordinates node for origin
 */
export default function UrdfInertialNode({ id, data }) {
  const d = data || {};
  const edit = (patch) => d.onChange?.(id, patch);

  const edges = useStore((state) => state.edges);
  const nodes = useStore((state) => state.nodes);

  const connectedHandles = edges
    .filter((e) => e.target === id)
    .map((e) => e.targetHandle);

  const isOriginConnected = connectedHandles.includes("origin");

  const inertia = d.inertia || {};
  const origin = d.origin || { xyz: [0, 0, 0], rpy: [0, 0, 0] };

  // Update from external connections
  useEffect(() => {
    const srcFor = (handleId) => {
      const edge = edges.find((e) => e.target === id && e.targetHandle === handleId);
      if (!edge) return null;
      return nodes.find((n) => n.id === edge.source);
    };

    if (isOriginConnected) {
      const originSrc = srcFor("origin");
      if (originSrc?.data?.xyz || originSrc?.data?.rpy) {
        const newOrigin = {
          xyz: originSrc.data.xyz || origin.xyz,
          rpy: originSrc.data.rpy || origin.rpy
        };
        if (JSON.stringify(newOrigin) !== JSON.stringify(origin)) {
          edit({ origin: newOrigin });
        }
      }
    }
  }, [edges, nodes, isOriginConnected]);

  const setInertia = (key, value) => {
    edit({ inertia: { ...inertia, [key]: value } });
  };

  return (
    <div className="rf-card rf-card--inertial" style={{ minWidth: 320 }}>
      <div className="rf-card__title">Inertial</div>

      <div className="rf-card__body" style={{ display: "grid", gap: ".5rem" }}>
        <div className="rf-field">
          <label>Mass (kg)</label>
          <input
            className="rf-input"
            type="number"
            step="0.01"
            placeholder="1.0"
            value={d.mass ?? ""}
            onChange={(e) => edit({ mass: parseFloat(e.target.value) || 0 })}
          />
        </div>

        <div className="rf-field">
          <label>Inertia Tensor</label>
          <div className="rf-grid" style={{ gridTemplateColumns: "1fr 1fr 1fr", gap: ".3rem" }}>
            <input
              className="rf-input"
              type="number"
              step="0.001"
              placeholder="ixx"
              title="ixx"
              value={inertia.ixx ?? ""}
              onChange={(e) => setInertia("ixx", parseFloat(e.target.value) || 0)}
            />
            <input
              className="rf-input"
              type="number"
              step="0.001"
              placeholder="iyy"
              title="iyy"
              value={inertia.iyy ?? ""}
              onChange={(e) => setInertia("iyy", parseFloat(e.target.value) || 0)}
            />
            <input
              className="rf-input"
              type="number"
              step="0.001"
              placeholder="izz"
              title="izz"
              value={inertia.izz ?? ""}
              onChange={(e) => setInertia("izz", parseFloat(e.target.value) || 0)}
            />
          </div>
          <div className="rf-grid" style={{ gridTemplateColumns: "1fr 1fr 1fr", gap: ".3rem", marginTop: ".3rem" }}>
            <input
              className="rf-input"
              type="number"
              step="0.001"
              placeholder="ixy"
              title="ixy"
              value={inertia.ixy ?? 0}
              onChange={(e) => setInertia("ixy", parseFloat(e.target.value) || 0)}
            />
            <input
              className="rf-input"
              type="number"
              step="0.001"
              placeholder="ixz"
              title="ixz"
              value={inertia.ixz ?? 0}
              onChange={(e) => setInertia("ixz", parseFloat(e.target.value) || 0)}
            />
            <input
              className="rf-input"
              type="number"
              step="0.001"
              placeholder="iyz"
              title="iyz"
              value={inertia.iyz ?? 0}
              onChange={(e) => setInertia("iyz", parseFloat(e.target.value) || 0)}
            />
          </div>
        </div>

        {/* Origin - collapsible */}
        <div className={`rf-field--collapsible ${isOriginConnected ? 'rf-field--collapsed' : ''}`}>
          <span className="rf-field__label">Origin Transform</span>
          <div className="rf-field__input-wrapper">
            <div style={{ fontSize: "0.85em", opacity: 0.7 }}>
              xyz: [{origin.xyz.join(', ')}] | rpy: [{origin.rpy.join(', ')}]
            </div>
          </div>
        </div>
      </div>

      {/* Input handle for origin */}
      <HandleWithLabel
        type="target"
        position={Position.Left}
        id="origin"
        label="origin"
        color="purple"
        top="65%"
      />

      {/* Output handle */}
      <HandleWithLabel
        type="source"
        position={Position.Right}
        id="inertial"
        label="inertial"
        color="orange"
      />
    </div>
  );
}
