// src/levels/slidesROS2Concepts/03-CreatingPackageInteractive.jsx
import React, { useState, useCallback, useEffect, useMemo } from "react";
import { ReactFlowProvider } from "@xyflow/react";
import "@xyflow/react/dist/style.css";
import Terminal from "../../components/ide/Terminal";
import BlockCanvas from "../../components/ide/BlockCanvas";
import { CategorizedPalette, defaultDataFor } from "../../components/blocks";
import { executeCommand } from "../../services/fileApi";
import useROS2Workspace from "../../hooks/useROS2Workspace";
import "../../styles/_rosflow.scss";

export const meta = {
  id: "creating-package-interactive",
  title: "Create Your First Package (Interactive)",
  order: 3,
  objectiveCode: "ros2-package-creating",
};

// Use existing ROS palette with unique keys
const ros2PackagePalette = {
  "ROS Inputs": [
    { type: "text", label: "Package Name", id: "pkgName" },
    { type: "text", label: "Node Name", id: "nodeName" },
    { type: "listDeps", label: "Dependencies", id: "deps" },
  ],
  "ROS Nodes": [
    { type: "createPackage", label: "Create Package", id: "createPkg" },
  ],
  "Output": [
    { type: "toCode", label: "Generate Command", id: "toCode" },
  ],
};

// Helper: read package data from connected nodes
function computePackageData(id, nodes, edges) {
  const pkgNode = nodes.find((n) => n.id === id);
  if (!pkgNode || pkgNode.type !== "createPackage") return null;

  const incoming = edges.filter((e) => e.target === id);
  const srcFor = (handleId) => {
    const ed = incoming.find((e) => e.targetHandle === handleId);
    if (!ed) return undefined;
    return nodes.find((n) => n.id === ed.source);
  };

  const base = pkgNode.data || {};
  let pkgName = base.pkgName || "my_package";
  let nodeName = base.nodeName || "";
  let deps = Array.isArray(base.deps) ? base.deps : ["rclpy", "std_msgs"];
  const lang = base.lang || "python";
  const buildType = base.buildType || (lang === "cpp" ? "ament_cmake" : "ament_python");

  const pkgSrc = srcFor("pkgName");
  if (pkgSrc?.type === "string" && pkgSrc.data?.value) pkgName = String(pkgSrc.data.value);

  const nodeSrc = srcFor("nodeName");
  if (nodeSrc?.type === "string" && nodeSrc.data?.value) nodeName = String(nodeSrc.data.value);

  const depsSrc = srcFor("deps");
  if (depsSrc?.type === "listDeps" && Array.isArray(depsSrc.data?.items)) {
    deps = depsSrc.data.items;
  }

  return { pkgName, nodeName, deps, lang, buildType };
}

// Helper: generate ros2 pkg create command in src/ folder
function buildCreatePkgCmd(pkgData) {
  if (!pkgData) return "";

  const { pkgName, nodeName, lang, buildType, deps } = pkgData;
  const bt = buildType || (lang === "cpp" ? "ament_cmake" : "ament_python");
  const depsList = (deps || []).filter(Boolean).join(" ");
  const depsPart = depsList ? ` --dependencies ${depsList}` : "";
  const nodePart = nodeName ? ` --node-name ${nodeName}` : "";
  const pkgPart = pkgName || "my_ros2_package";

  // Create src directory first if it doesn't exist, then create package inside it
  return `mkdir -p src && cd src && ros2 pkg create --build-type ${bt}${nodePart} ${pkgPart}${depsPart}`;
}

function CreatingPackageInteractiveInner({ onObjectiveHit }) {
  const [mode, setMode] = useState("canvas");
  const [status, setStatus] = useState("");
  const [packageCreated, setPackageCreated] = useState(false);
  const [packageBuilt, setPackageBuilt] = useState(false);
  const [currentStep, setCurrentStep] = useState(1); // 1: Create, 2: Build, 3: Done

  // Store graph state from BlockCanvas
  const [nodes, setNodes] = useState([]);
  const [edges, setEdges] = useState([]);

  // Use shared ROS2 workspace hook
  const { workspace, loading: workspaceLoading, error: workspaceError, canvasId, retry } = useROS2Workspace();

  // Update status based on workspace state
  useEffect(() => {
    if (workspaceLoading) {
      setStatus("Loading ROS2 workspace...");
    } else if (workspaceError) {
      setStatus(`⚠ Workspace error: ${workspaceError}`);
    } else if (workspace) {
      setStatus(`Connected to workspace "${workspace.name}"! Ready to create packages.`);
    }
  }, [workspace, workspaceLoading, workspaceError]);

  // Compute generated command from canvas
  const commandResult = useMemo(() => {
    const toCodeNode = nodes.find(n => n.type === "toCode");
    if (!toCodeNode) return { command: "", pkgData: null };

    const incomingEdges = edges.filter(e => e.target === toCodeNode.id);
    const createPkgNode = incomingEdges
      .map(e => nodes.find(n => n.id === e.source))
      .find(n => n && n.type === "createPackage");

    if (!createPkgNode) return { command: "", pkgData: null };

    const pkgData = computePackageData(createPkgNode.id, nodes, edges);
    if (!pkgData) return { command: "", pkgData: null };

    const command = buildCreatePkgCmd(pkgData);

    return { command, pkgData };
  }, [nodes, edges]);

  // Manual execution handler for when user clicks "Run in Terminal" button
  const handleExecuteCommand = useCallback(async (command) => {
    if (!command || !canvasId || packageCreated) return;

    try {
      // Parse package name from command
      const pkgNameMatch = command.match(/--node-name\s+\S+\s+(\S+)/);
      const pkgName = pkgNameMatch ? pkgNameMatch[1] : "package";

      setStatus(`Creating package: ${pkgName}...`);

      console.log("[Package Creator] Executing command:", command);
      console.log("[Package Creator] Canvas ID:", canvasId);

      // Execute the ros2 pkg create command in Docker
      const result = await executeCommand(canvasId, command);

      console.log("[Package Creator] Command result:", result);

      if (result.exit_code === 0 || result.stdout?.includes("creating") || result.output?.includes("creating")) {
        setPackageCreated(true);
        setCurrentStep(2);
        setStatus(`✅ Package "${pkgName}" created! Now build it with colcon build.`);
      } else {
        console.warn("[Package Creator] Command executed but uncertain success:", result);
        setStatus(`⚠ Package creation command executed. Exit code: ${result.exit_code}. Check terminal for details.`);
      }
    } catch (error) {
      console.error("[Package Creator] Failed to create package:", error);
      console.error("[Package Creator] Error details:", {
        message: error.message,
        response: error.response,
        stack: error.stack
      });
      setStatus(`⚠ Error: ${error.message}`);
    }
  }, [canvasId, packageCreated, onObjectiveHit]);

  // Handle graph changes from BlockCanvas
  const handleGraphChange = useCallback(({ nodes: newNodes, edges: newEdges }) => {
    setNodes(newNodes);
    setEdges(newEdges);
  }, []);

  // Initial nodes - Use existing blocks!
  const initialNodes = useMemo(() => {
    const mk = (type, x, y, extra = {}) => ({
      id: `${type}-${Math.random().toString(36).slice(2, 8)}`,
      type,
      position: { x, y },
      data: { ...defaultDataFor(type), ...extra },
    });

    return [
      mk("text", 50, 80, { label: "Package Name", value: "my_robot_pkg", placeholder: "my_package" }),
      mk("text", 50, 180, { label: "Node Name", value: "sensor_node", placeholder: "my_node" }),
      mk("listDeps", 50, 280, { title: "Dependencies", keyName: "items", items: ["rclpy", "std_msgs"], placeholder: "rclpy" }),
      mk("createPackage", 400, 180, { pkgName: "", nodeName: "", lang: "python", deps: [] }),
      mk("toCode", 720, 180, { onExecute: handleExecuteCommand }),
    ];
  }, [handleExecuteCommand]);

  const handleCommandExecute = useCallback(
    async (command, callback) => {
      if (!canvasId) {
        callback("⚠ Workspace not ready");
        return;
      }

      try {
        const result = await executeCommand(canvasId, command);
        const output = result.output || result.stdout || "Command executed";
        callback(output);

        // Detect colcon build success
        if (command.includes("colcon build") && (output.includes("Finished") || output.includes("Summary"))) {
          setPackageBuilt(true);
          setCurrentStep(3);
          setStatus(`🎉 Package built successfully! Ready to create publishers.`);
          onObjectiveHit?.(meta.objectiveCode);
        }
      } catch (error) {
        callback(`Error: ${error.message}`);
      }
    },
    [canvasId, onObjectiveHit]
  );

  // Show loading state
  if (workspaceLoading) {
    return (
      <div className="slide-wrap" style={{ display: "grid", gap: "0.75rem", placeItems: "center", minHeight: "400px" }}>
        <div style={{ textAlign: "center" }}>
          <div style={{ fontSize: "2em", marginBottom: "1rem" }}>⏳</div>
          <h3>Loading ROS2 Workspace...</h3>
          <p style={{ opacity: 0.7 }}>Please wait while we connect to your workspace</p>
        </div>
      </div>
    );
  }

  // Show error state with retry option
  if (workspaceError) {
    return (
      <div className="slide-wrap" style={{ display: "grid", gap: "0.75rem", placeItems: "center", minHeight: "400px" }}>
        <div style={{ textAlign: "center", maxWidth: "500px" }}>
          <div style={{ fontSize: "2em", marginBottom: "1rem" }}>⚠️</div>
          <h3>Workspace Error</h3>
          <p style={{ opacity: 0.7, margin: "1rem 0" }}>{workspaceError}</p>
          <button className="btn btn--primary" onClick={retry}>
            🔄 Retry Connection
          </button>
        </div>
      </div>
    );
  }

  return (
    <div className="slide-wrap" style={{ display: "grid", gap: "0.75rem" }}>
      <h2>{meta.title}</h2>

      <div className="slide-card">
        <div className="slide-card__title">Visual Package Builder</div>
        <p>
          Connect the <b>Package Name</b>, <b>Node Name</b>, and <b>Dependencies</b> to the <b>Create Package</b> block,
          then connect it to <b>Generate Command</b> to automatically create a ROS 2 package in Docker!
        </p>
        <p style={{ fontSize: "0.9em", opacity: 0.8, marginTop: "0.5rem" }}>
          This will execute <code>ros2 pkg create</code> with your configuration.
        </p>
        <p style={{ fontSize: "0.9em", opacity: 0.8, marginTop: "0.5rem" }}>
          💡 Workspace <b>"{workspace?.name}"</b> is shared across all ROS2 lessons. Files you create here persist!
        </p>
      </div>

      {/* Mode Selector & Status */}
      <div style={{ display: "flex", gap: "0.5rem", alignItems: "center", flexWrap: "wrap" }}>
        <button
          className="btn"
          onClick={() => setMode("canvas")}
          style={{
            opacity: mode === "canvas" ? 1 : 0.6,
            background: mode === "canvas" ? "var(--neon, #7df9ff)" : "var(--glass, rgba(255,255,255,.06))",
            color: mode === "canvas" ? "#000" : "inherit",
          }}
        >
          Canvas Mode
        </button>
        <button
          className="btn"
          onClick={() => setMode("terminal")}
          style={{
            opacity: mode === "terminal" ? 1 : 0.6,
            background: mode === "terminal" ? "var(--neon, #7df9ff)" : "var(--glass, rgba(255,255,255,.06))",
            color: mode === "terminal" ? "#000" : "inherit",
          }}
        >
          Terminal Mode
        </button>

        {status && (
          <div style={{
            flex: 1,
            padding: "0.5rem 1rem",
            background: packageCreated ? "rgba(0, 255, 0, 0.1)" : "rgba(255, 255, 255, 0.05)",
            border: `1px solid ${packageCreated ? "rgba(0, 255, 0, 0.3)" : "rgba(255, 255, 255, 0.1)"}`,
            borderRadius: "4px",
            fontSize: "0.9em",
          }}>
            {status}
          </div>
        )}
      </div>

      {/* Main Content */}
      <div style={{ display: "grid", gridTemplateColumns: "1.2fr 0.8fr", gap: "1rem", minHeight: 500, maxWidth: "100%" }}>
        {/* Left: Canvas or Terminal */}
        <div style={{ display: "flex", flexDirection: "column", minHeight: 0, maxWidth: "100%" }}>
          {mode === "terminal" ? (
            <div style={{
              border: "1px solid rgba(255,255,255,0.1)",
              borderRadius: "8px",
              overflow: "hidden",
              height: 500,
            }}>
              <Terminal
                onCommandExecute={handleCommandExecute}
                workingDirectory="/workspace"
                username="learner"
                canvasId={canvasId || "loading"}
              />
            </div>
          ) : (
            <div className="rfp-wrap" style={{ display: "flex", flexDirection: "column", height: 500 }}>
              <CategorizedPalette
                categories={ros2PackagePalette}
                defaultCategory="ROS Inputs"
              />

              <div style={{ flex: 1, minHeight: 0 }}>
                <BlockCanvas
                  initialNodes={initialNodes}
                  initialEdges={[]}
                  onGraphChange={handleGraphChange}
                  canvasId={canvasId}
                />
              </div>

              {packageCreated && !packageBuilt && (
                <div style={{
                  padding: "0.75rem",
                  background: "rgba(255, 200, 0, 0.1)",
                  borderTop: "1px solid rgba(255, 200, 0, 0.3)",
                  textAlign: "center",
                  fontSize: "0.9em",
                }}>
                  ⚠️ Next step: Switch to Terminal and run <code style={{padding: "2px 6px", background: "rgba(0,0,0,0.3)", borderRadius: "3px"}}>colcon build</code>
                </div>
              )}
              {packageBuilt && (
                <div style={{
                  padding: "0.75rem",
                  background: "rgba(0, 255, 0, 0.1)",
                  borderTop: "1px solid rgba(0, 255, 0, 0.3)",
                  textAlign: "center",
                }}>
                  ✅ Package built! You can now proceed to the next lesson to create publishers.
                </div>
              )}
            </div>
          )}
        </div>

        {/* Right: Command Preview & Instructions */}
        <div style={{ display: "flex", flexDirection: "column", gap: "0.75rem", minHeight: 0, maxWidth: "100%", overflow: "hidden" }}>
          <div className="slide-card" style={{ padding: "0.75rem" }}>
            <div className="slide-card__title" style={{ fontSize: "0.95em" }}>Step-by-Step Workflow</div>
            <ol style={{ fontSize: "0.85em", lineHeight: "1.6", margin: 0, paddingLeft: "1.25rem" }}>
              <li style={{opacity: currentStep >= 1 ? 1 : 0.5}}>
                {currentStep > 1 ? "✅" : "📝"} Edit <b>Package Name</b> and <b>Node Name</b>
              </li>
              <li style={{opacity: currentStep >= 1 ? 1 : 0.5}}>
                {currentStep > 1 ? "✅" : "📝"} Connect blocks and click "Run in Terminal"
              </li>
              <li style={{opacity: currentStep >= 2 ? 1 : 0.5, fontWeight: currentStep === 2 ? "bold" : "normal"}}>
                {currentStep > 2 ? "✅" : currentStep === 2 ? "⏳" : "⏹"} Switch to Terminal and run <code>colcon build</code>
              </li>
              <li style={{opacity: currentStep >= 3 ? 1 : 0.5}}>
                {currentStep >= 3 ? "✅" : "⏹"} Package is ready! Continue to next lesson
              </li>
            </ol>
          </div>

          <div className="slide-card" style={{ flex: 1, display: "flex", flexDirection: "column", minHeight: 0, padding: "0.75rem" }}>
            <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center", marginBottom: "0.5rem" }}>
              <div className="slide-card__title" style={{ fontSize: "0.95em" }}>Generated Command</div>
              <button
                className="btn"
                onClick={() => navigator.clipboard?.writeText(commandResult.command)}
                style={{ fontSize: "0.75em", padding: "0.25rem 0.5rem" }}
                disabled={!commandResult.command}
              >
                Copy
              </button>
            </div>

            <pre style={{
              flex: 1,
              minHeight: 0,
              maxHeight: "200px",
              overflow: "auto",
              background: "rgba(0,0,0,0.4)",
              border: "1px solid rgba(255,255,255,0.1)",
              borderRadius: "6px",
              padding: "0.75rem",
              color: "var(--text, #e6f1ff)",
              fontFamily: '"Cascadia Code", "Fira Code", Consolas, monospace',
              fontSize: "0.75em",
              lineHeight: "1.4",
              margin: 0,
              wordBreak: "break-all",
              whiteSpace: "pre-wrap",
            }}>
              {commandResult.command || "# Connect blocks to generate command"}
            </pre>

            {commandResult.pkgData && (
              <div style={{
                marginTop: "0.75rem",
                padding: "0.5rem",
                background: "rgba(125, 249, 255, 0.05)",
                border: "1px solid rgba(125, 249, 255, 0.2)",
                borderRadius: "6px",
                fontSize: "0.75em",
              }}>
                <b style={{ fontSize: "0.9em" }}>Package Info:</b>
                <div style={{ marginTop: "0.35rem", display: "grid", gap: "0.2rem" }}>
                  <div>• <code>{commandResult.pkgData.pkgName}</code></div>
                  {commandResult.pkgData.nodeName && <div>• Node: <code>{commandResult.pkgData.nodeName}</code></div>}
                  <div>• {commandResult.pkgData.buildType}</div>
                  <div>• {commandResult.pkgData.deps.join(", ")}</div>
                </div>
              </div>
            )}
          </div>
        </div>
      </div>

      {packageCreated && !packageBuilt && (
        <div className="slide-card" style={{
          background: "rgba(255, 200, 0, 0.1)",
          border: "2px solid rgba(255, 200, 0, 0.5)"
        }}>
          <div className="slide-card__title">📦 Package Created - Now Build It!</div>
          <p>
            Your ROS 2 package has been created in <code>src/{commandResult.pkgData?.pkgName}</code>! The package contains:
          </p>
          <ul style={{ marginTop: "0.5rem", fontSize: "0.9em" }}>
            <li>Package structure with <code>package.xml</code></li>
            <li>Build configuration (<code>setup.py</code> or <code>CMakeLists.txt</code>)</li>
            {commandResult.pkgData?.nodeName && <li>Starter node: <code>{commandResult.pkgData.nodeName}</code></li>}
            <li>Dependencies: {commandResult.pkgData?.deps.join(", ")}</li>
          </ul>
          <div style={{ marginTop: "1rem", padding: "1rem", background: "rgba(0,0,0,0.3)", borderRadius: "6px", border: "1px solid rgba(255,200,0,0.3)" }}>
            <div style={{ fontWeight: "bold", marginBottom: "0.5rem" }}>⚠️ Next Step: Build the Package</div>
            <p style={{ fontSize: "0.9em", margin: "0.5rem 0" }}>
              Switch to <b>Terminal Mode</b> and run:
            </p>
            <code style={{ display: "block", padding: "0.5rem", background: "rgba(0,0,0,0.5)", borderRadius: "4px", fontFamily: "monospace" }}>
              colcon build
            </code>
            <p style={{ fontSize: "0.85em", margin: "0.5rem 0", opacity: 0.9 }}>
              This compiles your package and makes it ready to use. Wait for "Finished" message.
            </p>
          </div>
        </div>
      )}

      {packageBuilt && (
        <div className="slide-card" style={{
          background: "rgba(0, 255, 0, 0.1)",
          border: "2px solid rgba(0, 255, 0, 0.5)"
        }}>
          <div className="slide-card__title">🎉 Package Built Successfully!</div>
          <p>
            Your ROS 2 package <code>{commandResult.pkgData?.pkgName}</code> is now compiled and ready to use!
          </p>
          <div style={{ marginTop: "1rem", padding: "1rem", background: "rgba(0,255,0,0.05)", borderRadius: "6px", border: "1px solid rgba(0,255,0,0.3)" }}>
            <div style={{ fontWeight: "bold", marginBottom: "0.5rem" }}>✅ You're ready for the next lesson!</div>
            <p style={{ fontSize: "0.9em", margin: 0 }}>
              Proceed to "Creating Publishers" to add publisher nodes to your package.
            </p>
          </div>
        </div>
      )}
    </div>
  );
}

export default function CreatingPackageInteractive(props) {
  return (
    <ReactFlowProvider>
      <CreatingPackageInteractiveInner {...props} />
    </ReactFlowProvider>
  );
}
