// components/ide/BlockCanvas.jsx
import React, { useCallback, useEffect, useMemo } from "react";
import PropTypes from "prop-types";
import {
  ReactFlow,
  Background,
  Controls,
  addEdge,
  useEdgesState,
  useNodesState,
  useReactFlow,
} from "@xyflow/react";
import "@xyflow/react/dist/style.css";

import { nodeTypes, defaultDataFor } from "../blocks";
import { syncUrdfDerived, syncJointStates } from "../blocks/urdf-helpers";
import "./BlockCanvas.scss";

/**
 * Reusable block-based programming canvas using ReactFlow
 *
 * @param {Object} props
 * @param {Array} props.initialNodes - Initial nodes to render
 * @param {Array} props.initialEdges - Initial edges to connect nodes
 * @param {Object} props.palette - Block palette (nodeTypes)
 * @param {Function} props.onGraphChange - Callback when graph changes
 * @param {Function} props.onCodeGenerated - Callback with generated code
 * @param {Function} props.codeGenerator - Function to generate code from graph
 * @param {boolean} props.readOnly - Disable editing
 */
export function BlockCanvas({
  initialNodes = [],
  initialEdges = [],
  palette = {},
  onGraphChange,
  onCodeGenerated,
  codeGenerator,
  readOnly = false,
  className = "",
}) {
  const [nodes, setNodes, onNodesChange] = useNodesState(initialNodes);
  const [edges, setEdges, onEdgesChange] = useEdgesState(initialEdges);
  const { screenToFlowPosition } = useReactFlow();

  // Notify parent of graph changes
  useEffect(() => {
    if (onGraphChange) {
      onGraphChange({ nodes, edges });
    }
  }, [nodes, edges, onGraphChange]);

  // Generate code when graph changes
  useEffect(() => {
    if (codeGenerator && onCodeGenerated) {
      const code = codeGenerator(nodes, edges);
      onCodeGenerated(code);
    }
  }, [nodes, edges, codeGenerator, onCodeGenerated]);

  // Sync URDF XML to viewer nodes when graph changes
  useEffect(() => {
    syncUrdfDerived(nodes, edges, setNodes);
  }, [nodes, edges, setNodes]);

  // Sync joint states from control nodes to viewer nodes
  useEffect(() => {
    syncJointStates(nodes, edges, setNodes);
  }, [nodes, edges, setNodes]);

  // Connection handler
  const onConnect = useCallback(
    (params) => {
      if (readOnly) return;
      setEdges((eds) => addEdge({ ...params, type: "smoothstep" }, eds));
    },
    [readOnly, setEdges]
  );

  // Node data change handler
  const onNodeDataChange = useCallback(
    (id, patch) => {
      if (readOnly) return;
      setNodes((nds) =>
        nds.map((n) => {
          if (n.id !== id) return n;
          const curr = n.data || {};

          // Check if data actually changed
          let changed = false;
          for (const k of Object.keys(patch)) {
            const a = curr[k];
            const b = patch[k];
            const same =
              Array.isArray(a) || Array.isArray(b)
                ? JSON.stringify(a) === JSON.stringify(b)
                : a === b;
            if (!same) {
              changed = true;
              break;
            }
          }

          if (!changed) return n;
          return { ...n, data: { ...curr, ...patch } };
        })
      );
    },
    [readOnly, setNodes]
  );

  // Drag and drop handlers
  const onDrop = useCallback(
    (evt) => {
      if (readOnly) return;
      evt.preventDefault();

      const type = evt.dataTransfer.getData("application/rf-node");
      if (!type) return;

      const position = screenToFlowPosition({
        x: evt.clientX,
        y: evt.clientY,
      });

      // Create node with proper styling for nodes that need fixed dimensions
      let nodeStyle;
      if (type === "urdfViewer") {
        nodeStyle = { width: 600, height: 650 };
      } else if (type === "urdfXmlPreview") {
        nodeStyle = { width: 600, height: 550 };
      } else if (type === "urdfControl") {
        nodeStyle = { width: 450, height: 600 };
      }

      setNodes((nds) => [
        ...nds,
        {
          id: `${type}-${Date.now()}-${Math.random().toString(36).slice(2, 8)}`,
          type,
          position,
          data: { ...defaultDataFor(type), onChange: onNodeDataChange },
          style: nodeStyle,
        },
      ]);
    },
    [readOnly, screenToFlowPosition, setNodes, onNodeDataChange]
  );

  const onDragOver = useCallback(
    (evt) => {
      if (readOnly) return;
      evt.preventDefault();
      evt.dataTransfer.dropEffect = "move";
    },
    [readOnly]
  );

  // Update nodes when initialNodes change (for loading saved graphs)
  useEffect(() => {
    if (initialNodes.length > 0) {
      setNodes(
        initialNodes.map((n) => {
          // Ensure proper styling for nodes that need fixed dimensions
          let nodeStyle = n.style;
          if (!n.style) {
            if (n.type === "urdfViewer") {
              nodeStyle = { width: 600, height: 650 };
            } else if (n.type === "urdfXmlPreview") {
              nodeStyle = { width: 600, height: 550 };
            } else if (n.type === "urdfControl") {
              nodeStyle = { width: 450, height: 600 };
            }
          }

          return {
            ...n,
            data: { ...n.data, onChange: onNodeDataChange },
            style: nodeStyle,
          };
        })
      );
    }
  }, []); // Only on mount

  useEffect(() => {
    if (initialEdges.length > 0) {
      setEdges(initialEdges);
    }
  }, []); // Only on mount

  return (
    <div className={`block-canvas ${className}`}>
      <ReactFlow
        nodes={nodes}
        edges={edges}
        onNodesChange={readOnly ? undefined : onNodesChange}
        onEdgesChange={readOnly ? undefined : onEdgesChange}
        onConnect={onConnect}
        nodeTypes={nodeTypes}
        onDrop={onDrop}
        onDragOver={onDragOver}
        fitView
        proOptions={{ hideAttribution: true }}
        deleteKeyCode={readOnly ? [] : ["Backspace", "Delete"]}
        nodesDraggable={!readOnly}
        nodesConnectable={!readOnly}
        elementsSelectable={!readOnly}
        className="block-canvas__inner"
      >
        <Background />
        <Controls showInteractive={!readOnly} />
      </ReactFlow>
    </div>
  );
}

BlockCanvas.propTypes = {
  initialNodes: PropTypes.array,
  initialEdges: PropTypes.array,
  palette: PropTypes.object,
  onGraphChange: PropTypes.func,
  onCodeGenerated: PropTypes.func,
  codeGenerator: PropTypes.func,
  readOnly: PropTypes.bool,
  className: PropTypes.string,
};

export default BlockCanvas;
