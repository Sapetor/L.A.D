// src/levels/slidesROS2Concepts/05-CreatingPublishers.jsx
import { useState, useCallback, useEffect, useMemo } from "react";
import { ReactFlowProvider } from "@xyflow/react";
import "@xyflow/react/dist/style.css";
import BlockCanvas from "../../components/ide/BlockCanvas";
import FileExplorer from "../../components/ide/FileExplorer";
import Terminal from "../../components/ide/Terminal";
import CategorizedPalette from "../../components/blocks/CategorizedPalette";
import StringNode from "../../components/blocks/StringNode";
import RosPublisherNode from "../../components/blocks/RosPublisherNode";
import { createFile, executeCommand } from "../../services/fileApi";
import useROS2Workspace from "../../hooks/useROS2Workspace";
import "../../components/ide/BlockCanvas.scss";
import "../../styles/_rosflow.scss";

export const meta = {
  id: "creating-publishers",
  title: "Creating Publishers (Interactive)",
  order: 5,
  objectiveCode: "ros2-topics-publishing",
};

// Node type registry
const nodeTypes = {
  text: StringNode,
  rosPublisher: RosPublisherNode,
};

// Palette configuration
const ros2PublisherPalette = {
  "Input": [
    { type: "text", label: "Text/Data", id: "textData" },
  ],
  "ROS2": [
    { type: "rosPublisher", label: "Publisher", id: "publisher" },
  ],
};

// Publisher code templates
const generatePythonPublisher = ({ topicName, msgType, msgPackage, timerInterval, className, fileName, dataInput }) => {
  const interval = parseFloat(timerInterval) || 0.5;

  // Generate message assignment based on type
  let msgAssignment = "";
  if (msgType === "String") {
    const dataValue = dataInput || "Hello ROS 2";
    msgAssignment = `msg.data = f'${dataValue}: {self.count}'`;
  } else if (["Int32", "Int64", "UInt8", "UInt16"].includes(msgType)) {
    const dataValue = dataInput || "0";
    msgAssignment = `msg.data = ${dataValue} + self.count`;
  } else if (["Float32", "Float64"].includes(msgType)) {
    const dataValue = dataInput || "0.0";
    msgAssignment = `msg.data = ${dataValue} + float(self.count)`;
  } else if (msgType === "Bool") {
    msgAssignment = `msg.data = self.count % 2 == 0`;
  } else {
    // Complex types - use custom data if provided
    if (dataInput) {
      msgAssignment = `# Set custom data\n        # ${dataInput}`;
    } else {
      msgAssignment = `# Configure your ${msgType} message here\n        pass`;
    }
  }

  return `import rclpy
from rclpy.node import Node
from ${msgPackage}.msg import ${msgType}

class ${className}(Node):
    def __init__(self):
        super().__init__('${fileName}')
        # Create publisher: topic name, message type, queue size
        self.publisher = self.create_publisher(${msgType}, '${topicName}', 10)

        # Create timer to publish periodically (every ${interval} seconds)
        self.timer = self.create_timer(${interval}, self.timer_callback)
        self.count = 0
        self.get_logger().info(f'Publisher initialized on topic: ${topicName}')

    def timer_callback(self):
        msg = ${msgType}()
        ${msgAssignment}
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data if hasattr(msg, "data") else msg}"')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = ${className}()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
`;
};

function CreatingPublishersInner({ onObjectiveHit }) {
  const [nodes, setNodes] = useState([]);
  const [edges, setEdges] = useState([]);
  const [status, setStatus] = useState("");
  const [selectedFile, setSelectedFile] = useState(null);
  const [loading, setLoading] = useState(false);
  const [mode, setMode] = useState("canvas"); // "canvas" or "terminal"
  const [packageName, setPackageName] = useState("");
  const [publisherCreated, setPublisherCreated] = useState(false);
  const [setupUpdated, setSetupUpdated] = useState(false);
  const [packageBuilt, setPackageBuilt] = useState(false);
  const [publisherRunning, setPublisherRunning] = useState(false);
  const [currentStep, setCurrentStep] = useState(1); // 1: Create file, 2: Update setup, 3: Build, 4: Run, 5: Test

  // Use shared ROS2 workspace hook (now includes file tree caching)
  const {
    workspace,
    fileTree,
    loading: workspaceLoading,
    error: workspaceError,
    canvasId,
    loadedFromCache,
    refreshWorkspace,
    retry
  } = useROS2Workspace();

  // Update status when workspace is ready
  useEffect(() => {
    if (workspaceLoading) {
      setStatus("Loading ROS2 workspace...");
    } else if (workspaceError) {
      setStatus(`⚠ Workspace error: ${workspaceError}`);
    } else if (workspace) {
      const cacheIndicator = loadedFromCache ? " (loaded from cache)" : "";
      setStatus(`Connected to workspace "${workspace.name}"${cacheIndicator}! Ready to create publisher nodes.`);
    }
  }, [workspace, workspaceLoading, workspaceError, loadedFromCache]);

  const handleGraphChange = useCallback(({ nodes: newNodes, edges: newEdges }) => {
    setNodes(newNodes);
    setEdges(newEdges);
  }, []);

  // Terminal command execution handler
  const handleCommandExecute = useCallback(
    async (command, callback) => {
      if (!canvasId) {
        callback?.("⚠ Workspace not ready");
        return;
      }

      try {
        console.log("[Publisher Creator] Executing command:", command);
        const result = await executeCommand(canvasId, command);
        console.log("[Publisher Creator] Command result:", result);

        // Send output to terminal
        const output = result.output || result.stdout || "Command executed";
        callback?.(output);

        // Detect successful colcon build
        if (command.includes("colcon build") && (output.includes("Finished") || output.includes("Summary"))) {
          setPackageBuilt(true);
          setCurrentStep(4);
          setStatus(`🎉 Package built! Now run your publisher with ros2 run.`);
        }

        // Detect publisher running
        if (command.includes("ros2 run") && packageName) {
          setPublisherRunning(true);
          setCurrentStep(5);
          setStatus(`✅ Publisher is running! Open another terminal and use ros2 topic echo to see messages.`);
        }

        // Detect ros2 topic echo success
        if (command.includes("ros2 topic echo") && output.includes("data:")) {
          onObjectiveHit?.(meta.objectiveCode);
          setStatus(`🎉 Success! You're receiving messages from your publisher!`);
        }

        if (result.error || result.stderr) {
          callback?.(`\x1b[31m${result.error || result.stderr}\x1b[0m`);
        }
      } catch (error) {
        console.error("[Publisher Creator] Command execution error:", error);
        callback?.(`\x1b[31mError: ${error.message}\x1b[0m`);
      }
    },
    [canvasId, packageName, onObjectiveHit]
  );

  const createPublisherFile = async () => {
    try {
      // Find the publisher node
      const publisherNode = nodes.find(n => n.type === "rosPublisher");
      if (!publisherNode) {
        setStatus("⚠ Error: No publisher block found! Add a ROS2 Publisher block first.");
        return;
      }

      const publisherData = publisherNode.data;
      const {
        topicName = "/chatter",
        msgType = "String",
        msgPackage = "std_msgs",
        frequency = "1.0",
        dataInput = "",
        queueSize = "10"
      } = publisherData;

      // Calculate timer interval from frequency
      const timerInterval = frequency ? (1.0 / parseFloat(frequency)).toFixed(3) : "1.0";

      // Default class and file names
      const className = "MinimalPublisher";
      const fileName = "publisher_node";
      const lang = "python"; // Currently only Python

      // Find package name from file tree (look for src/package_name structure)
      let detectedPackage = "";

      // Helper function to recursively search for src folder
      const findSrcFolder = (items) => {
        for (const item of items) {
          if (item.name === "src" && (item.type === "dir" || item.type === "directory")) {
            return item;
          }
          if (item.children && item.children.length > 0) {
            const found = findSrcFolder(item.children);
            if (found) return found;
          }
        }
        return null;
      };

      if (fileTree && fileTree.length > 0) {
        const srcFolder = findSrcFolder(fileTree);
        if (srcFolder && srcFolder.children && srcFolder.children.length > 0) {
          // Get first package in src folder (should be a directory)
          const packageFolder = srcFolder.children.find(
            child => (child.type === "dir" || child.type === "directory")
          );
          if (packageFolder) {
            detectedPackage = packageFolder.name;
          }
        }
      }

      if (!detectedPackage) {
        setStatus("⚠ Error: No package found in src/ folder. Create a package first in lesson 3!");
        console.error("[Publisher Creator] File tree structure:", JSON.stringify(fileTree, null, 2));
        return;
      }

      setPackageName(detectedPackage);
      setStatus(`📝 Creating ${lang} publisher in package "${detectedPackage}"...`);

      // Generate code
      const code = generatePythonPublisher({
        topicName,
        msgType,
        msgPackage,
        timerInterval,
        className,
        fileName,
        dataInput
      });

      const fileExt = "py";
      // Place file in src/package_name/package_name/
      const filePath = `src/${detectedPackage}/${detectedPackage}/${fileName}.${fileExt}`;

      console.log("[Publisher Creator] Creating file:", filePath);

      // Create the file in the workspace
      await createFile(canvasId, {
        path: filePath,
        content: code,
        file_type: lang,
      });

      setPublisherCreated(true);
      setCurrentStep(2);
      setStatus(`✅ Created ${fileName}.${fileExt}! Now update setup.py to add entry point.`);

      // Refresh file tree to show new file
      await refreshWorkspace(false);

      console.log("[Publisher Creator] File created successfully:", filePath);
    } catch (err) {
      console.error("[Publisher Creator] Failed to create file:", err);
      setStatus(`❌ Error: ${err.message}`);
    }
  };

  const handleFileSelect = (path) => {
    setSelectedFile(path);
  };

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
    <div className="slide-wrap" style={{ display: "flex", flexDirection: "column", gap: "0.75rem", height: "100%" }}>
      {/* Header */}
      <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center", marginBottom: "0.5rem" }}>
        <div>
          <h2 style={{ margin: 0, marginBottom: "0.25rem" }}>{meta.title}</h2>
          <p style={{ margin: 0, fontSize: "0.9em", opacity: 0.8 }}>
            Build and test ROS2 publishers visually
            {loadedFromCache && (
              <span
                style={{
                  marginLeft: "0.5rem",
                  padding: "0.15rem 0.4rem",
                  background: "rgba(125, 249, 255, 0.15)",
                  border: "1px solid var(--neon)",
                  borderRadius: "4px",
                  color: "var(--neon)",
                  fontSize: "0.85em",
                  fontWeight: "600",
                }}
                title="Files loaded from cache for instant access"
              >
                ⚡ CACHED
              </span>
            )}
          </p>
        </div>

        {/* Mode Tabs */}
        <div style={{ display: "flex", gap: "0.5rem" }}>
          <button
            className="btn btn--small"
            onClick={() => setMode("canvas")}
            style={{
              background: mode === "canvas" ? "var(--neon, #7df9ff)" : "rgba(255,255,255,.06)",
              color: mode === "canvas" ? "#000" : "inherit",
              fontWeight: mode === "canvas" ? "600" : "normal",
            }}
          >
            🎨 Visual Editor
          </button>
          <button
            className="btn btn--small"
            onClick={() => setMode("terminal")}
            style={{
              background: mode === "terminal" ? "var(--neon, #7df9ff)" : "rgba(255,255,255,.06)",
              color: mode === "terminal" ? "#000" : "inherit",
              fontWeight: mode === "terminal" ? "600" : "normal",
            }}
          >
            💻 Terminal
          </button>
        </div>
      </div>

      {/* Status Bar */}
      {status && (
        <div style={{
          padding: "0.75rem 1rem",
          background: status.includes("✅")
            ? "rgba(0, 255, 0, 0.1)"
            : status.includes("❌") || status.includes("⚠")
            ? "rgba(255, 100, 100, 0.1)"
            : "rgba(125, 249, 255, 0.1)",
          border: `1px solid ${
            status.includes("✅")
              ? "rgba(0, 255, 0, 0.3)"
              : status.includes("❌") || status.includes("⚠")
              ? "rgba(255, 100, 100, 0.3)"
              : "rgba(125, 249, 255, 0.3)"
          }`,
          borderRadius: "6px",
          fontSize: "0.9em",
          fontWeight: "500",
        }}>
          {status}
        </div>
      )}

      {/* Main Content Area */}
      <div style={{ flex: 1, display: "grid", gridTemplateColumns: "2fr 1fr", gap: "1rem", minHeight: 0 }}>
        {/* Left Panel: Canvas/Terminal */}
        <div style={{ display: "flex", flexDirection: "column", gap: "0.75rem", minHeight: 0 }}>
          {mode === "terminal" ? (
            <>
              {/* Terminal View */}
              <div style={{
                flex: 1,
                border: "1px solid rgba(255, 255, 255, 0.2)",
                borderRadius: "8px",
                overflow: "hidden",
                background: "rgba(0, 0, 0, 0.3)",
              }}>
                <Terminal
                  onCommandExecute={handleCommandExecute}
                  workingDirectory="/workspace"
                  username="ros-learner"
                  canvasId={canvasId || "loading"}
                />
              </div>
            </>
          ) : (
            <>
              {/* Palette */}
              <div style={{ borderRadius: "6px", overflow: "hidden" }}>
                <CategorizedPalette
                  categories={ros2PublisherPalette}
                  defaultCategory="ROS2"
                />
              </div>

              {/* Canvas */}
              <div style={{
                flex: 1,
                border: "1px solid rgba(255, 255, 255, 0.2)",
                borderRadius: "8px",
                overflow: "hidden",
                background: "rgba(0, 0, 0, 0.3)",
                minHeight: 400,
              }}>
                <BlockCanvas
                  initialNodes={[]}
                  initialEdges={[]}
                  onGraphChange={handleGraphChange}
                  nodeTypes={nodeTypes}
                  canvasId={canvasId}
                />
              </div>

              {/* Action Button */}
              <button
                className="btn btn--primary"
                onClick={createPublisherFile}
                disabled={!canvasId || nodes.length === 0 || loading}
                style={{
                  padding: "0.75rem 1.5rem",
                  fontSize: "1rem",
                  fontWeight: "600",
                }}
              >
                {loading ? "⏳ Creating..." : "📄 Generate Publisher Code"}
              </button>
            </>
          )}
        </div>

        {/* Right Panel: Info & Files */}
        <div style={{ display: "flex", flexDirection: "column", gap: "0.75rem", minHeight: 0 }}>
          {mode === "canvas" ? (
            <>
              {/* Instructions */}
              <div className="slide-card" style={{ padding: "1rem" }}>
                <div className="slide-card__title" style={{ fontSize: "1em", marginBottom: "0.75rem" }}>
                  📚 Quick Start
                </div>
                <ol style={{ fontSize: "0.85em", lineHeight: "1.8", margin: 0, paddingLeft: "1.5rem" }}>
                  <li>Drag a <b>ROS2 Publisher</b> block to the canvas</li>
                  <li>Configure topic name, message type, and frequency</li>
                  <li>Optionally connect a <b>Text/Data</b> block for custom data</li>
                  <li>Click <b>Generate Publisher Code</b></li>
                  <li>Switch to Terminal mode to test your publisher!</li>
                </ol>
              </div>

              {/* File Explorer */}
              <div style={{
                flex: 1,
                border: "1px solid rgba(255, 255, 255, 0.2)",
                borderRadius: "8px",
                overflow: "hidden",
                background: "rgba(0, 0, 0, 0.2)",
                minHeight: 200,
              }}>
                <div style={{
                  padding: "0.75rem 1rem",
                  background: "rgba(255, 255, 255, 0.05)",
                  borderBottom: "1px solid rgba(255, 255, 255, 0.1)",
                  fontSize: "0.9em",
                  fontWeight: "600",
                }}>
                  📁 Workspace Files
                </div>
                <FileExplorer
                  files={fileTree}
                  currentFile={selectedFile}
                  onFileSelect={handleFileSelect}
                  loading={loading}
                />
              </div>

              {/* Publisher Info */}
              <div className="slide-card slide-card--aside" style={{ padding: "0.75rem 1rem" }}>
                <div className="slide-card__title" style={{ fontSize: "0.9em", marginBottom: "0.5rem" }}>
                  💡 Message Types
                </div>
                <div style={{ fontSize: "0.75em", lineHeight: "1.6", opacity: 0.9 }}>
                  <p style={{ margin: "0.25rem 0" }}><b>std_msgs:</b> String, Int32, Float64, Bool...</p>
                  <p style={{ margin: "0.25rem 0" }}><b>geometry_msgs:</b> Twist, Pose, Point...</p>
                  <p style={{ margin: "0.25rem 0" }}><b>sensor_msgs:</b> Image, LaserScan, Imu...</p>
                </div>
              </div>
            </>
          ) : (
            <>
              {/* Terminal Instructions */}
              <div className="slide-card" style={{ padding: "1rem" }}>
                <div className="slide-card__title" style={{ fontSize: "1em", marginBottom: "0.75rem" }}>
                  🧪 Testing Your Publisher
                </div>
                <p style={{ fontSize: "0.85em", lineHeight: "1.6", margin: "0.5rem 0" }}>
                  Run these commands in the terminal to test your publisher:
                </p>
              </div>

              {/* Commands Reference */}
              <div className="slide-card" style={{ padding: "1rem", flex: 1, background: "rgba(0,0,0,0.3)" }}>
                <div className="slide-card__title" style={{ fontSize: "0.9em", marginBottom: "0.75rem" }}>
                  ⌨️ Common Commands
                </div>
                <div style={{ fontSize: "0.8em", lineHeight: "2", fontFamily: "monospace" }}>
                  <div style={{ marginBottom: "0.5rem" }}>
                    <div style={{ opacity: 0.7, fontSize: "0.9em" }}>Run your publisher:</div>
                    <code style={{ color: "var(--neon)" }}>python3 publisher_node.py</code>
                  </div>
                  <div style={{ marginBottom: "0.5rem" }}>
                    <div style={{ opacity: 0.7, fontSize: "0.9em" }}>List active topics:</div>
                    <code style={{ color: "var(--neon)" }}>ros2 topic list</code>
                  </div>
                  <div style={{ marginBottom: "0.5rem" }}>
                    <div style={{ opacity: 0.7, fontSize: "0.9em" }}>Listen to messages:</div>
                    <code style={{ color: "var(--neon)" }}>ros2 topic echo /chatter</code>
                  </div>
                  <div style={{ marginBottom: "0.5rem" }}>
                    <div style={{ opacity: 0.7, fontSize: "0.9em" }}>Check publish rate:</div>
                    <code style={{ color: "var(--neon)" }}>ros2 topic hz /chatter</code>
                  </div>
                  <div>
                    <div style={{ opacity: 0.7, fontSize: "0.9em" }}>Topic information:</div>
                    <code style={{ color: "var(--neon)" }}>ros2 topic info /chatter</code>
                  </div>
                </div>
              </div>

              {/* Testing Tips */}
              <div className="slide-card slide-card--aside" style={{ padding: "0.75rem 1rem" }}>
                <div className="slide-card__title" style={{ fontSize: "0.9em", marginBottom: "0.5rem" }}>
                  ✅ Testing Tips
                </div>
                <ul style={{ fontSize: "0.75em", margin: 0, paddingLeft: "1.2rem", lineHeight: "1.6" }}>
                  <li>Press <kbd>Ctrl+C</kbd> to stop the publisher</li>
                  <li>Use multiple terminals to run publisher and subscriber</li>
                  <li>Check <code>ls</code> to verify your file was created</li>
                </ul>
              </div>
            </>
          )}
        </div>
      </div>
    </div>
  );
}

export default function CreatingPublishers(props) {
  return (
    <ReactFlowProvider>
      <CreatingPublishersInner {...props} />
    </ReactFlowProvider>
  );
}
