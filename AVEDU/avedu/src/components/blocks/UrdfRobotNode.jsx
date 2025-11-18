// components/blocks/UrdfRobotNode.jsx
import React from "react";
import { Position } from "@xyflow/react";
import HandleWithLabel from "./HandleWithLabel";

/**
 * URDF Robot Node
 * Root node that aggregates incoming links, joints, and assemblies
 * Generates complete robot URDF XML
 */
export default function UrdfRobotNode({ id, data }) {
  const handleChange = (key, value) => {
    data?.onChange?.(id, { [key]: value });
  };

  const links = Array.isArray(data?.links) ? data.links : [];
  const joints = Array.isArray(data?.joints) ? data.joints : [];

  const linksCount = links.length;
  const jointsCount = joints.length;

  // Validate joints - check for missing required fields
  const invalidJoints = joints.filter(j => !j?.name || !j.parent || !j.child || !j.type);
  const validJointsCount = jointsCount - invalidJoints.length;

  // Validate links
  const invalidLinks = links.filter(l => !l?.name);

  return (
    <div className="rf-card" style={{ minWidth: 400 }}>
      <div className="rf-card__title">URDF Robot</div>

      <div className="rf-card__body" style={{ display: "grid", gap: ".5rem" }}>
        <div className="rf-field">
          <label>Robot Name</label>
          <input
            type="text"
            className="rf-input"
            placeholder="my_robot"
            value={data?.name || ""}
            onChange={(e) => handleChange("name", e.target.value)}
          />
        </div>

        <div style={{
          display: "grid",
          gridTemplateColumns: "1fr 1fr 1fr",
          gap: ".5rem",
          padding: ".5rem",
          background: "rgba(0, 0, 0, 0.2)",
          borderRadius: "8px"
        }}>
          <div style={{ textAlign: "center" }}>
            <div style={{ fontSize: "1.5em", fontWeight: "bold", color: linksCount > 0 ? "#4caf50" : "#666" }}>
              {linksCount}
            </div>
            <div style={{ fontSize: "0.85em", opacity: 0.8 }}>Links</div>
            {invalidLinks.length > 0 && (
              <div style={{ fontSize: "0.7em", color: "#ff9800", marginTop: "0.25rem" }}>
                {invalidLinks.length} missing name
              </div>
            )}
          </div>
          <div style={{ textAlign: "center" }}>
            <div style={{ fontSize: "1.5em", fontWeight: "bold", color: validJointsCount > 0 ? "#2196f3" : "#666" }}>
              {jointsCount}
            </div>
            <div style={{ fontSize: "0.85em", opacity: 0.8 }}>Joints</div>
            {invalidJoints.length > 0 && (
              <div style={{ fontSize: "0.7em", color: "#ff9800", marginTop: "0.25rem" }}>
                {invalidJoints.length} incomplete
              </div>
            )}
          </div>
          <div style={{ textAlign: "center" }}>
            <div style={{ fontSize: "1.5em", fontWeight: "bold", color: data?.xml ? "#4caf50" : "#666" }}>
              {data?.xml ? "✓" : "○"}
            </div>
            <div style={{ fontSize: "0.85em", opacity: 0.8 }}>XML</div>
          </div>
        </div>

        {/* Validation warnings */}
        {(invalidJoints.length > 0 || invalidLinks.length > 0) && (
          <div style={{
            padding: ".5rem",
            background: "rgba(255, 152, 0, 0.1)",
            border: "1px solid rgba(255, 152, 0, 0.3)",
            borderRadius: "6px",
            fontSize: "0.85em"
          }}>
            <div style={{ fontWeight: "bold", marginBottom: ".25rem", color: "#ff9800" }}>
              ⚠ Validation Issues:
            </div>
            {invalidLinks.length > 0 && (
              <div>• {invalidLinks.length} link(s) missing name</div>
            )}
            {invalidJoints.length > 0 && (
              <div>• {invalidJoints.length} joint(s) missing required fields (name, parent, child, or type)</div>
            )}
          </div>
        )}

        {data?.xml && (
          <details>
            <summary style={{ cursor: "pointer", fontSize: "0.9em", opacity: 0.8 }}>
              View Generated XML
            </summary>
            <pre
              className="rf-terminal__code"
              style={{
                maxHeight: 200,
                overflow: "auto",
                margin: ".5rem 0 0 0",
                fontSize: "0.8em"
              }}
            >
{data.xml}
            </pre>
          </details>
        )}
      </div>

      {/* Input handles - positioned below robot name */}
      <HandleWithLabel
        type="target"
        position={Position.Left}
        id="links"
        label="links"
        color="green"
        top="50%"
      />
      <HandleWithLabel
        type="target"
        position={Position.Left}
        id="joints"
        label="joints"
        color="blue"
        top="65%"
      />
      <HandleWithLabel
        type="target"
        position={Position.Left}
        id="assemblies"
        label="assemblies"
        color="purple"
        top="80%"
      />

      <HandleWithLabel
        type="source"
        position={Position.Right}
        id="xml"
        label="xml"
        color="red"
        top="50%"
      />
    </div>
  );
}
