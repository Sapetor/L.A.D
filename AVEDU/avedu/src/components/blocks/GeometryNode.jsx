import React from "react";
import { Position } from "@xyflow/react";
import HandleWithLabel from "./HandleWithLabel";

/**
 * GeometryNode - Reusable block for geometry configuration
 * Supports mesh, box, cylinder, and sphere geometries
 */
export default function GeometryNode({ id, data }) {
  const d = data || {};
  const edit = (patch) => d.onChange?.(id, patch);

  const geometry = d.geometry || { type: "box", size: [1, 1, 1] };

  const setGeometry = (patch) => {
    edit({ geometry: { ...geometry, ...patch } });
  };

  const setSize = (index, value) => {
    const size = [...(geometry.size || [1, 1, 1])];
    size[index] = parseFloat(value) || 1;
    setGeometry({ size });
  };

  const setScale = (index, value) => {
    const scale = [...(geometry.scale || [1, 1, 1])];
    scale[index] = parseFloat(value) || 1;
    setGeometry({ scale });
  };

  return (
    <div className="rf-card" style={{ minWidth: 300 }}>
      <div className="rf-card__title">Geometry</div>

      <div className="rf-card__body" style={{ display: "grid", gap: ".5rem" }}>
        <div className="rf-field">
          <label>Type</label>
          <select
            className="rf-input"
            value={geometry.type || "box"}
            onChange={(e) => {
              const type = e.target.value;
              const newGeom = { type };

              if (type === "mesh") {
                newGeom.filename = geometry.filename || "";
                newGeom.scale = geometry.scale || [1, 1, 1];
              } else if (type === "box") {
                newGeom.size = geometry.size || [1, 1, 1];
              } else if (type === "cylinder") {
                newGeom.radius = geometry.radius || 0.5;
                newGeom.length = geometry.length || 1;
              } else if (type === "sphere") {
                newGeom.radius = geometry.radius || 0.5;
              }

              edit({ geometry: newGeom });
            }}
          >
            <option value="mesh">Mesh</option>
            <option value="box">Box</option>
            <option value="cylinder">Cylinder</option>
            <option value="sphere">Sphere</option>
          </select>
        </div>

        {geometry.type === "mesh" && (
          <>
            <div className="rf-field">
              <label>Mesh File</label>
              <input
                className="rf-input"
                placeholder="package://path/to/mesh.dae"
                value={geometry.filename || ""}
                onChange={(e) => setGeometry({ filename: e.target.value })}
              />
            </div>
            <div className="rf-field">
              <label>Scale (x y z)</label>
              <div className="rf-grid" style={{ gridTemplateColumns: "1fr 1fr 1fr", gap: ".3rem" }}>
                {[0, 1, 2].map((i) => (
                  <input
                    key={`scale-${i}`}
                    className="rf-input"
                    type="number"
                    step="0.1"
                    placeholder={["x", "y", "z"][i]}
                    value={geometry.scale?.[i] ?? 1}
                    onChange={(e) => setScale(i, e.target.value)}
                  />
                ))}
              </div>
            </div>
          </>
        )}

        {geometry.type === "box" && (
          <div className="rf-field">
            <label>Size (x y z)</label>
            <div className="rf-grid" style={{ gridTemplateColumns: "1fr 1fr 1fr", gap: ".3rem" }}>
              {[0, 1, 2].map((i) => (
                <input
                  key={`size-${i}`}
                  className="rf-input"
                  type="number"
                  step="0.1"
                  placeholder={["width", "depth", "height"][i]}
                  value={geometry.size?.[i] ?? 1}
                  onChange={(e) => setSize(i, e.target.value)}
                />
              ))}
            </div>
          </div>
        )}

        {(geometry.type === "cylinder" || geometry.type === "sphere") && (
          <div className="rf-field">
            <label>Radius</label>
            <input
              className="rf-input"
              type="number"
              step="0.1"
              placeholder="0.5"
              value={geometry.radius ?? 0.5}
              onChange={(e) => setGeometry({ radius: parseFloat(e.target.value) || 0.5 })}
            />
          </div>
        )}

        {geometry.type === "cylinder" && (
          <div className="rf-field">
            <label>Length</label>
            <input
              className="rf-input"
              type="number"
              step="0.1"
              placeholder="1.0"
              value={geometry.length ?? 1}
              onChange={(e) => setGeometry({ length: parseFloat(e.target.value) || 1 })}
            />
          </div>
        )}
      </div>

      <HandleWithLabel
        type="source"
        position={Position.Right}
        id="geometry"
        label="geometry"
        color="blue"
      />
    </div>
  );
}
