// components/blocks/UrdfCollisionNode.jsx
import React, { useEffect } from "react";
import { Position, useStore } from "@xyflow/react";
import HandleWithLabel from "./HandleWithLabel";

/**
 * URDF Collision Node - Modular component for link collision geometry
 * Can accept Geometry and Coordinates nodes as inputs
 */
export default function UrdfCollisionNode({ id, data }) {
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

  return (
    <div className="rf-card rf-card--collision" style={{ minWidth: 320 }}>
      <div className="rf-card__title">Collision Geometry</div>

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
        id="collision"
        label="collision"
        color="red"
      />
    </div>
  );
}
