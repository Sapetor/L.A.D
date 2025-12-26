// src/levels/slidesROS2Concepts/09-ConnectTheFlowGame.jsx
// Adapted from slidesROSBasic/03-comunication.jsx
import React, { useCallback, useEffect, useState, useMemo } from "react";
import {
  ReactFlow,
  Background,
  Controls,
  Handle,
  addEdge,
  useEdgesState,
  useNodesState,
  MarkerType,
  BaseEdge,
  getSmoothStepPath,
  Position,
} from "@xyflow/react";
import "@xyflow/react/dist/style.css";

export const meta = {
  id: "connect-flow-game",
  title: "Mini Game: Connect the Communication Flow",
  order: 9,
  objectiveCode: "ros2-minigame-connect-flow",
};

/** Simple edge without animation */
function SimpleEdge(props) {
  const { id, sourceX, sourceY, targetX, targetY, sourcePosition, targetPosition } = props;
  const [edgePath] = getSmoothStepPath({
    sourceX,
    sourceY,
    targetX,
    targetY,
    sourcePosition,
    targetPosition,
  });
  return <BaseEdge id={id} path={edgePath} />;
}

/** Animated edge showing message flow */
function PacketEdge(props) {
  const { id, data } = props;
  const { p, n, s, hasPubToNode, hasNodeToSub } = data || {};

  if (!hasPubToNode) return null;

  const [pathPN] = getSmoothStepPath({
    sourceX: p.x,
    sourceY: p.y,
    targetX: n.x,
    targetY: n.y,
    sourcePosition: Position.Right,
    targetPosition: Position.Left,
  });

  let combinedPath = pathPN;
  if (hasNodeToSub) {
    const [pathNS] = getSmoothStepPath({
      sourceX: n.x,
      sourceY: n.y,
      targetX: s.x,
      targetY: s.y,
      sourcePosition: Position.Right,
      targetPosition: Position.Left,
    });
    combinedPath = `${pathPN} ${pathNS}`;
  }

  return (
    <>
      <BaseEdge id={`${id}-invisible`} path={combinedPath} style={{ stroke: "transparent" }} />
      <circle r="8" fill="url(#pktGrad)">
        <animateMotion dur="2s" repeatCount="indefinite" path={combinedPath} />
      </circle>
      <defs>
        <linearGradient id="pktGrad" x1="0" y1="0" x2="0" y2="1">
          <stop offset="0%" stopColor="#7df9ff" />
          <stop offset="100%" stopColor="#ff5cf4" />
        </linearGradient>
      </defs>
    </>
  );
}

function IoNode({ data, isConnectable }) {
  return (
    <div
      style={{
        padding: "8px 12px",
        border: "1px solid rgba(255,255,255,.2)",
        borderRadius: 10,
        background: "rgba(255,255,255,.06)",
        color: "var(--text,#e6f1ff)",
        minWidth: 120,
        textAlign: "center",
      }}
    >
      <div style={{ fontWeight: 700 }}>{data.label}</div>
      {data.showLeft && <Handle type="target" position="left" id="left" isConnectable={isConnectable} />}
      {data.showRight && <Handle type="source" position="right" id="right" isConnectable={isConnectable} />}
    </div>
  );
}

const nodeTypes = { ioNode: IoNode };
const edgeTypes = { simple: SimpleEdge, packet: PacketEdge };

export default function ConnectTheFlowGame({ onObjectiveHit }) {
  const P = { x: 50,  y: 100 };
  const N = { x: 300, y: 100 };
  const S = { x: 560, y: 100 };

  const initialNodes = useMemo(() => ([
    { id: "pub",  type: "ioNode", position: P, data: { label: "Publisher",  showRight: true } },
    { id: "node", type: "ioNode", position: N, data: { label: "Topic\n/chatter", showLeft: true, showRight: true } },
    { id: "sub",  type: "ioNode", position: S, data: { label: "Subscriber", showLeft: true } },
  ]), []);

  const [nodes, , onNodesChange] = useNodesState(initialNodes);
  const [connEdges, setConnEdges, onEdgesChange] = useEdgesState([]);
  const [canSend, setCanSend] = useState(false);
  const [validated, setValidated] = useState(false);

  const onConnect = useCallback(
    (params) => {
      setConnEdges((eds) =>
        addEdge(
          {
            ...params,
            type: "simple",
            markerEnd: { type: MarkerType.ArrowClosed, width: 18, height: 18 },
            style: { stroke: "rgba(255,255,255,.65)", strokeWidth: 2 },
          },
          eds
        )
      );
    },
    [setConnEdges]
  );

  const { hasPubToNode, hasNodeToSub, pPos, nPos, sPos } = useMemo(() => {
    const hasP2N = connEdges.some((e) => e.source === "pub" && e.target === "node");
    const hasN2S = connEdges.some((e) => e.source === "node" && e.target === "sub");
    const pub = nodes.find((n) => n.id === "pub")?.position ?? P;
    const nod = nodes.find((n) => n.id === "node")?.position ?? N;
    const sub = nodes.find((n) => n.id === "sub")?.position ?? S;
    return { hasPubToNode: hasP2N, hasNodeToSub: hasN2S, pPos: pub, nPos: nod, sPos: sub };
  }, [connEdges, nodes]);

  useEffect(() => {
    setCanSend(hasPubToNode && hasNodeToSub);
  }, [hasPubToNode, hasNodeToSub]);

  const packetEdge = useMemo(
    () => ({
      id: "packet",
      source: "pub",
      target: "sub",
      type: "packet",
      style: { stroke: "transparent" },
      data: {
        p: pPos,
        n: nPos,
        s: sPos,
        hasPubToNode,
        hasNodeToSub,
      },
    }),
    [pPos, nPos, sPos, hasPubToNode, hasNodeToSub]
  );

  const handleValidate = () => {
    if (!canSend) return;
    if (!validated) {
      setValidated(true);
      onObjectiveHit?.(meta.objectiveCode);
    }
  };

  return (
    <div className="slide-wrap" style={{ display: "grid", gap: "0.75rem" }}>
      <h2>{meta.title}</h2>

      <div className="slide-card">
        <div className="slide-card__title">Learn by Doing</div>
        <p>
          Connect the <b>Publisher</b> to the <b>Topic</b>, then connect the <b>Topic</b> to the <b>Subscriber</b>.
          This demonstrates how messages flow in ROS 2: Publisher → Topic → Subscriber(s).
        </p>
      </div>

      <div style={{ height: 300, borderRadius: 12, overflow: "hidden", border: "1px solid rgba(255,255,255,0.1)" }}>
        <ReactFlow
          nodes={nodes}
          edges={[...connEdges, packetEdge]}
          onNodesChange={onNodesChange}
          onEdgesChange={onEdgesChange}
          onConnect={onConnect}
          nodeTypes={nodeTypes}
          edgeTypes={edgeTypes}
          fitView
          fitViewOptions={{ padding: 0.18 }}
        >
          <Background />
          <Controls />
        </ReactFlow>
      </div>

      <div style={{ display: "flex", gap: ".5rem", alignItems: "center", flexWrap: "wrap" }}>
        <button className="btn" onClick={handleValidate} disabled={!canSend}>
          {validated ? "✓ Connection Validated!" : "Validate Connection"}
        </button>
        {!canSend && (
          <span style={{ opacity: .85 }}>
            Connect <b>Publisher → Topic</b> and <b>Topic → Subscriber</b>
          </span>
        )}
        {canSend && !validated && (
          <span style={{ color: "var(--neon, #7df9ff)" }}>
            Great! Now click "Validate Connection" to complete the challenge.
          </span>
        )}
      </div>

      {validated && (
        <div className="slide-card" style={{
          background: "rgba(0, 255, 0, 0.1)",
          border: "2px solid rgba(0, 255, 0, 0.5)"
        }}>
          <div className="slide-card__title">🎉 Well Done!</div>
          <p>
            You've successfully demonstrated the ROS 2 publish-subscribe pattern!
            The animated message shows how data flows from publisher through the topic to subscribers.
          </p>
        </div>
      )}
    </div>
  );
}
