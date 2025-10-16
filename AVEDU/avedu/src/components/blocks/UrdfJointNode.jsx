// components/blocks/UrdfJointNode.jsx
import React, { useEffect, useState } from "react";
import { Position, useStore } from "@xyflow/react";
import HandleWithLabel from "./HandleWithLabel";

/**
 * URDF Joint - Can accept external inputs for parent, child, origin, and axis
 */
export default function UrdfJointNode({ id, data }) {
  const d = data || {};
  const edit = (patch) => d.onChange?.(id, patch);

  const edges = useStore((state) => state.edges);
  const nodes = useStore((state) => state.nodes);

  const connectedHandles = edges
    .filter((e) => e.target === id)
    .map((e) => e.targetHandle);

  const isParentConnected = connectedHandles.includes("parent");
  const isChildConnected = connectedHandles.includes("child");
  const isOriginConnected = connectedHandles.includes("origin");
  const isAxisConnected = connectedHandles.includes("axis");

  const [parent, setParent] = useState(d.parent || "");
  const [child, setChild] = useState(d.child || "");

  const origin = d.origin || { xyz: [0, 0, 0], rpy: [0, 0, 0] };
  const axis = d.axis || { xyz: [1, 0, 0] };

  // Update from external connections
  useEffect(() => {
    const srcFor = (handleId) => {
      const edge = edges.find((e) => e.target === id && e.targetHandle === handleId);
      if (!edge) return null;
      return nodes.find((n) => n.id === edge.source);
    };

    let updated = false;
    let newData = {};

    if (isParentConnected) {
      const parentSrc = srcFor("parent");
      if (parentSrc?.data?.value && parentSrc.data.value !== parent) {
        setParent(parentSrc.data.value);
        newData.parent = parentSrc.data.value;
        updated = true;
      }
    }

    if (isChildConnected) {
      const childSrc = srcFor("child");
      if (childSrc?.data?.value && childSrc.data.value !== child) {
        setChild(childSrc.data.value);
        newData.child = childSrc.data.value;
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

    if (isAxisConnected) {
      const axisSrc = srcFor("axis");
      if (axisSrc?.data?.axis) {
        const newAxis = { xyz: axisSrc.data.axis };
        if (JSON.stringify(newAxis) !== JSON.stringify(axis)) {
          newData.axis = newAxis;
          updated = true;
        }
      }
    }

    if (updated) {
      edit(newData);
    }
  }, [edges, nodes, isParentConnected, isChildConnected, isOriginConnected, isAxisConnected]);

  const onParentChange = (val) => {
    setParent(val);
    edit({ parent: val });
  };

  const onChildChange = (val) => {
    setChild(val);
    edit({ child: val });
  };

  return (
    <div className="rf-card" style={{ minWidth: 340 }}>
      <div className="rf-card__title">URDF Joint</div>

      <div className="rf-card__body" style={{ display: "grid", gap: ".5rem" }}>
        <div className="rf-field">
          <label>Name</label>
          <input
            className="rf-input"
            value={d.name || ""}
            placeholder="joint1"
            onChange={(e) => edit({ name: e.target.value })}
          />
        </div>

        <div className="rf-field">
          <label>Type</label>
          <select
            className="rf-input"
            value={d.type || "fixed"}
            onChange={(e) => edit({ type: e.target.value })}
          >
            <option value="fixed">fixed</option>
            <option value="revolute">revolute</option>
            <option value="continuous">continuous</option>
            <option value="prismatic">prismatic</option>
            <option value="floating">floating</option>
            <option value="planar">planar</option>
          </select>
        </div>

        {/* Parent - collapsible */}
        <div className={`rf-field--collapsible ${isParentConnected ? 'rf-field--collapsed' : ''}`}>
          <span className="rf-field__label">Parent Link</span>
          <div className="rf-field__input-wrapper">
            <input
              className="rf-input"
              value={parent}
              placeholder="base_link"
              onChange={(e) => onParentChange(e.target.value)}
            />
          </div>
        </div>

        {/* Child - collapsible */}
        <div className={`rf-field--collapsible ${isChildConnected ? 'rf-field--collapsed' : ''}`}>
          <span className="rf-field__label">Child Link</span>
          <div className="rf-field__input-wrapper">
            <input
              className="rf-input"
              value={child}
              placeholder="link1"
              onChange={(e) => onChildChange(e.target.value)}
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

        {/* Axis - collapsible */}
        <div className={`rf-field--collapsible ${isAxisConnected ? 'rf-field--collapsed' : ''}`}>
          <span className="rf-field__label">Axis</span>
          <div className="rf-field__input-wrapper">
            <div style={{ fontSize: "0.85em", opacity: 0.7 }}>
              [{axis.xyz.join(', ')}]
            </div>
          </div>
        </div>
      </div>

      {/* Input handles */}
      <HandleWithLabel
        type="target"
        position={Position.Left}
        id="parent"
        label="parent"
        color="blue"
        top="35%"
      />
      <HandleWithLabel
        type="target"
        position={Position.Left}
        id="child"
        label="child"
        color="blue"
        top="50%"
      />
      <HandleWithLabel
        type="target"
        position={Position.Left}
        id="origin"
        label="origin"
        color="purple"
        top="65%"
      />
      <HandleWithLabel
        type="target"
        position={Position.Left}
        id="axis"
        label="axis"
        color="green"
        top="80%"
      />

      {/* Output handle */}
      <HandleWithLabel
        type="source"
        position={Position.Right}
        id="out"
        label="joint"
        color="blue"
      />
    </div>
  );
}
