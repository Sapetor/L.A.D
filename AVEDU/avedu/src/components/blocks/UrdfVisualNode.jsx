// components/blocks/UrdfVisualNode.jsx
import React, { useEffect } from "react";
import { Position, useStore } from "@xyflow/react";
import HandleWithLabel from "./HandleWithLabel";

/**
 * URDF Visual Node - Modular component for link visual geometry
 * Can accept Geometry and Coordinates nodes as inputs
 */
export default function UrdfVisualNode({ id, data }) {
  const d = data || {};
  const edit = (patch) => d.onChange?.(id, patch);

  const edges = useStore((state) => state.edges);
  const nodes = useStore((state) => state.nodes);

  const connectedHandles = edges
    .filter((e) => e.target === id)
    .map((e) => e.targetHandle);

  const isGeometryConnected = connectedHandles.includes("geometry");
  const isOriginConnected = connectedHandles.includes("origin");

  const geometry = d.geometry || { type: "box", size: [1, 1, 1] };
  const origin = d.origin || { xyz: [0, 0, 0], rpy: [0, 0, 0] };
  const material = d.material || {};

  // Update from external connections
  useEffect(() => {
    const srcFor = (handleId) => {
      const edge = edges.find((e) => e.target === id && e.targetHandle === handleId);
      if (!edge) return null;
      return nodes.find((n) => n.id === edge.source);
    };

    let updated = false;
    let newData = {};

    if (isGeometryConnected) {
      const geomSrc = srcFor("geometry");
      if (geomSrc?.data?.geometry && JSON.stringify(geomSrc.data.geometry) !== JSON.stringify(geometry)) {
        newData.geometry = geomSrc.data.geometry;
        updated = true;
      }
    }

    if (isOriginConnected) {
      const originSrc = srcFor("origin");
      if (originSrc?.data?.xyz || originSrc?.data?.rpy) {
        const newOrigin = {
          xyz: originSrc.data.xyz || origin.xyz,
          rpy: originSrc.data.rpy || origin.rpy
        };
        if (JSON.stringify(newOrigin) !== JSON.stringify(origin)) {
          newData.origin = newOrigin;
          updated = true;
        }
      }
    }

    if (updated) {
      edit(newData);
    }
  }, [edges, nodes, isGeometryConnected, isOriginConnected]);

  const setGeometry = (patch) => {
    edit({ geometry: { ...geometry, ...patch } });
  };

  return (
    <div className="rf-card rf-card--visual" style={{ minWidth: 320 }}>
      <div className="rf-card__title">Visual Geometry</div>

      <div className="rf-card__body" style={{ display: "grid", gap: ".5rem" }}>
        {/* Geometry - collapsible */}
        <div className={`rf-field--collapsible ${isGeometryConnected ? 'rf-field--collapsed' : ''}`}>
          <span className="rf-field__label">Geometry</span>
          <div className="rf-field__input-wrapper">
            <div style={{ fontSize: "0.85em", opacity: 0.7, marginBottom: ".25rem" }}>
              {geometry.type} {geometry.type === 'mesh' && geometry.filename && `(${geometry.filename.split('/').pop()})`}
            </div>
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

        {/* Material */}
        <details>
          <summary style={{ cursor: "pointer", fontSize: "0.9em", opacity: 0.8 }}>
            Material
          </summary>
          <div className="rf-field" style={{ marginTop: ".5rem" }}>
            <label>Material Name</label>
            <input
              className="rf-input"
              placeholder="blue_material"
              value={material.name || ""}
              onChange={(e) => edit({ material: { ...material, name: e.target.value } })}
            />
          </div>
          <div className="rf-field">
            <label>Color (RGBA)</label>
            <div className="rf-grid" style={{ gridTemplateColumns: "1fr 1fr 1fr 1fr", gap: ".3rem" }}>
              {["r", "g", "b", "a"].map((c, i) => (
                <input
                  key={c}
                  className="rf-input"
                  type="number"
                  step="0.1"
                  min="0"
                  max="1"
                  placeholder={c}
                  value={material.color?.[i] ?? (c === "a" ? 1 : 0.5)}
                  onChange={(e) => {
                    const color = [...(material.color || [0.5, 0.5, 0.5, 1])];
                    color[i] = parseFloat(e.target.value) || 0;
                    edit({ material: { ...material, color } });
                  }}
                />
              ))}
            </div>
          </div>
        </details>
      </div>

      {/* Input handles */}
      <HandleWithLabel
        type="target"
        position={Position.Left}
        id="geometry"
        label="geometry"
        color="blue"
        top="30%"
      />
      <HandleWithLabel
        type="target"
        position={Position.Left}
        id="origin"
        label="origin"
        color="purple"
        top="50%"
      />

      {/* Output handle */}
      <HandleWithLabel
        type="source"
        position={Position.Right}
        id="visual"
        label="visual"
        color="blue"
      />
    </div>
  );
}
