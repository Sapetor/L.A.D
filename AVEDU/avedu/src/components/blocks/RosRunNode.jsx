import React, { useState, useEffect } from "react";
import { Position, useStore } from "@xyflow/react";
import HandleWithLabel from "./HandleWithLabel";

/**
 * RosRunNode - Editable node that can accept connections or manual input
 * Inputs collapse when external nodes are connected
 */
export default function RosRunNode({ id, data }) {
  const edges = useStore((state) => state.edges);
  const nodes = useStore((state) => state.nodes);

  const connectedHandles = edges
    .filter((e) => e.target === id)
    .map((e) => e.targetHandle);

  const isPkgConnected = connectedHandles.includes("pkg");
  const isExeConnected = connectedHandles.includes("exe");
  const isNsConnected = connectedHandles.includes("ns");
  const isArgsConnected = connectedHandles.includes("args");

  const [pkg, setPkg] = useState(data.pkg || "");
  const [exe, setExe] = useState(data.exe || "");
  const [ns, setNs] = useState(data.ns || "");
  const [args, setArgs] = useState(Array.isArray(data.args) ? data.args : []);
  const [argInput, setArgInput] = useState("");

  // Update state when external connections provide data
  useEffect(() => {
    const srcFor = (handleId) => {
      const edge = edges.find((e) => e.target === id && e.targetHandle === handleId);
      if (!edge) return null;
      return nodes.find((n) => n.id === edge.source);
    };

    let updated = false;
    let newData = {};

    if (isPkgConnected) {
      const pkgSrc = srcFor("pkg");
      if (pkgSrc?.data?.value && pkgSrc.data.value !== pkg) {
        setPkg(pkgSrc.data.value);
        newData.pkg = pkgSrc.data.value;
        updated = true;
      }
    }

    if (isExeConnected) {
      const exeSrc = srcFor("exe");
      if (exeSrc?.data?.value && exeSrc.data.value !== exe) {
        setExe(exeSrc.data.value);
        newData.exe = exeSrc.data.value;
        updated = true;
      }
    }

    if (isNsConnected) {
      const nsSrc = srcFor("ns");
      if (nsSrc?.data?.value && nsSrc.data.value !== ns) {
        setNs(nsSrc.data.value);
        newData.ns = nsSrc.data.value;
        updated = true;
      }
    }

    if (isArgsConnected) {
      const argsSrc = srcFor("args");
      const srcArgs = Array.isArray(argsSrc?.data?.args) ? argsSrc.data.args :
                     Array.isArray(argsSrc?.data?.items) ? argsSrc.data.items : null;
      if (srcArgs && JSON.stringify(srcArgs) !== JSON.stringify(args)) {
        setArgs(srcArgs);
        newData.args = srcArgs;
        updated = true;
      }
    }

    if (updated) {
      notify({ pkg, exe, ns, args, ...newData });
    }
  }, [edges, nodes, isPkgConnected, isExeConnected, isNsConnected, isArgsConnected]);

  const notify = (next) => data.onChange?.(id, { ...next });

  const onPkgChange = (v) => {
    setPkg(v);
    notify({ pkg: v, exe, ns, args });
  };

  const onExeChange = (v) => {
    setExe(v);
    notify({ pkg, exe: v, ns, args });
  };

  const onNsChange = (v) => {
    setNs(v);
    notify({ pkg, exe, ns: v, args });
  };

  const addArg = () => {
    const parts = argInput
      .split(/[,\s]+/)
      .map((s) => s.trim())
      .filter(Boolean);
    if (!parts.length) return;
    const next = [...args, ...parts];
    setArgs(next);
    setArgInput("");
    notify({ pkg, exe, ns, args: next });
  };

  const removeArg = (idx) => {
    const next = args.filter((_, i) => i !== idx);
    setArgs(next);
    notify({ pkg, exe, ns, args: next });
  };

  return (
    <div className="rf-card" style={{ minWidth: 320 }}>
      <div className="rf-card__title">ROS Run</div>
      <div className="rf-card__body" style={{ display: "grid", gap: ".4rem" }}>
        <div className={`rf-field--collapsible ${isPkgConnected ? 'rf-field--collapsed' : ''}`}>
          <span className="rf-field__label">Package</span>
          <div className="rf-field__input-wrapper">
            <input
              value={pkg}
              onChange={(e) => onPkgChange(e.target.value)}
              placeholder="turtlesim"
              className="rf-input"
            />
          </div>
        </div>

        <div className={`rf-field--collapsible ${isExeConnected ? 'rf-field--collapsed' : ''}`}>
          <span className="rf-field__label">Executable</span>
          <div className="rf-field__input-wrapper">
            <input
              value={exe}
              onChange={(e) => onExeChange(e.target.value)}
              placeholder="turtlesim_node"
              className="rf-input"
            />
          </div>
        </div>

        <div className={`rf-field--collapsible ${isNsConnected ? 'rf-field--collapsed' : ''}`}>
          <span className="rf-field__label">Namespace</span>
          <div className="rf-field__input-wrapper">
            <input
              value={ns}
              onChange={(e) => onNsChange(e.target.value)}
              placeholder="/demo (optional)"
              className="rf-input"
            />
          </div>
        </div>

        <div className={`rf-field--collapsible ${isArgsConnected ? 'rf-field--collapsed' : ''}`}>
          <span className="rf-field__label">Args</span>
          <div className="rf-field__input-wrapper">
            <div className="rf-row">
              <input
                value={argInput}
                onChange={(e) => setArgInput(e.target.value)}
                placeholder="--ros-args ..."
                className="rf-input"
              />
              <button className="btn" onClick={addArg}>add</button>
            </div>
            <div className="rf-chips">
              {args.map((a, i) => (
                <span key={`${a}-${i}`} className="rf-chip" onClick={() => removeArg(i)} title="Remove">
                  {a} ✕
                </span>
              ))}
              {args.length === 0 && <span className="rf-chip rf-chip--ghost">No args</span>}
            </div>
          </div>
        </div>
      </div>

      <HandleWithLabel
        type="target"
        position={Position.Left}
        id="pkg"
        label="pkg"
        top="17%"
      />
      <HandleWithLabel
        type="target"
        position={Position.Left}
        id="exe"
        label="exe"
        top="33%"
      />
      <HandleWithLabel
        type="target"
        position={Position.Left}
        id="ns"
        label="ns"
        top="50%"
      />
      <HandleWithLabel
        type="target"
        position={Position.Left}
        id="args"
        label="args"
        top="67%"
      />

      <HandleWithLabel
        type="source"
        position={Position.Right}
        id="out"
        label="output"
        top="50%"
      />
    </div>
  );
}
