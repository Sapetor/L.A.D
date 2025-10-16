import React, { useState } from "react";
import { Position } from "@xyflow/react";
import HandleWithLabel from "./HandleWithLabel";

/**
 * TextNode - Generic text input node
 * Can be used for package names, executable names, namespaces, etc.
 */
export default function TextNode({ id, data }) {
  const [value, setValue] = useState(String(data.value ?? ""));

  const change = (v) => {
    setValue(v);
    data.onChange?.(id, { value: v });
  };

  return (
    <div className="rf-card" style={{ minWidth: 220 }}>
      <div className="rf-card__title">{data.label || "Text"}</div>
      <div className="rf-card__body">
        <input
          className="rf-input"
          value={value}
          onChange={(e) => change(e.target.value)}
          placeholder={data.placeholder || "enter text..."}
        />
      </div>
      <HandleWithLabel
        type="source"
        position={Position.Right}
        id="out"
        label="text"
      />
    </div>
  );
}

// Keep StringNode as an alias for backward compatibility
export { TextNode as StringNode };
