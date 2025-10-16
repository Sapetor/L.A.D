import React from "react";
import { Position } from "@xyflow/react";
import HandleWithLabel from "./HandleWithLabel";

/**
 * CoordinatesNode - Reusable block for xyz/rpy coordinates
 * Can be connected to origin transforms, positions, etc.
 */
export default function CoordinatesNode({ id, data }) {
  const d = data || {};
  const edit = (patch) => d.onChange?.(id, patch);

  const xyz = d.xyz || [0, 0, 0];
  const rpy = d.rpy || [0, 0, 0];

  const setXyz = (index, value) => {
    const newXyz = [...xyz];
    newXyz[index] = parseFloat(value) || 0;
    edit({ xyz: newXyz });
  };

  const setRpy = (index, value) => {
    const newRpy = [...rpy];
    newRpy[index] = parseFloat(value) || 0;
    edit({ rpy: newRpy });
  };

  return (
    <div className="rf-card" style={{ minWidth: 280 }}>
      <div className="rf-card__title">Coordinates</div>

      <div className="rf-card__body" style={{ display: "grid", gap: ".5rem" }}>
        <div className="rf-field">
          <label>Position (xyz)</label>
          <div className="rf-grid" style={{ gridTemplateColumns: "1fr 1fr 1fr", gap: ".3rem" }}>
            {[0, 1, 2].map((i) => (
              <input
                key={`xyz-${i}`}
                className="rf-input"
                type="number"
                step="0.01"
                placeholder={["x", "y", "z"][i]}
                value={xyz[i]}
                onChange={(e) => setXyz(i, e.target.value)}
              />
            ))}
          </div>
        </div>

        <div className="rf-field">
          <label>Rotation (rpy)</label>
          <div className="rf-grid" style={{ gridTemplateColumns: "1fr 1fr 1fr", gap: ".3rem" }}>
            {[0, 1, 2].map((i) => (
              <input
                key={`rpy-${i}`}
                className="rf-input"
                type="number"
                step="0.01"
                placeholder={["r", "p", "y"][i]}
                value={rpy[i]}
                onChange={(e) => setRpy(i, e.target.value)}
              />
            ))}
          </div>
        </div>
      </div>

      <HandleWithLabel
        type="source"
        position={Position.Right}
        id="coordinates"
        label="coordinates"
        color="purple"
      />
    </div>
  );
}
