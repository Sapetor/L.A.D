// =============================================================
// FILE: src/pages/IDETestPage.jsx
// Test page for the ROS Visual IDE components (BlockCanvas + FileExplorer)
// =============================================================
import React, { useState, useCallback, useMemo, useEffect, useRef } from "react";
import { ReactFlowProvider } from "@xyflow/react";
import { useNavigate } from "react-router-dom";
import { BlockCanvas } from "../components/ide/BlockCanvas";
import { FileExplorer } from "../components/ide/FileExplorer";
import { TabBar } from "../components/ide/TabBar";
import { Terminal } from "../components/ide/Terminal";
import { CategorizedPalette } from "../components/blocks";
import { paletteCategorized } from "../components/blocks";
import { computeUrdfXml } from "../components/blocks/urdf-helpers";
import CanvasSelector from "../components/ide/CanvasSelector";
import fileApi from "../services/fileApi";
import "../styles/_rosflow.scss";
import "../styles/pages/_ide-test.scss";
import "../styles/components/_canvas-selector.scss";

function IDETestPageInner() {
  const navigate = useNavigate();

  // Canvas state
  const [canvas, setCanvas] = useState(null);
  const [canvasLoading, setCanvasLoading] = useState(false);
  const [showCanvasSelector, setShowCanvasSelector] = useState(true);

  // File tree state
  const [fileTree, setFileTree] = useState([
    {
      name: "src",
      path: "/src",
      type: "directory",
      children: [
        {
          name: "my_robot",
          path: "/src/my_robot",
          type: "directory",
          children: [
            {
              name: "urdf",
              path: "/src/my_robot/urdf",
              type: "directory",
              children: [
                {
                  name: "robot.urdf",
                  path: "/src/my_robot/urdf/robot.urdf",
                  type: "file",
                  unsaved: false,
                },
              ],
            },
            {
              name: "package.xml",
              path: "/src/my_robot/package.xml",
              type: "file",
            },
            {
              name: "setup.py",
              path: "/src/my_robot/setup.py",
              type: "file",
            },
          ],
        },
      ],
    },
    {
      name: "build",
      path: "/build",
      type: "directory",
      children: [],
    },
    {
      name: "install",
      path: "/install",
      type: "directory",
      children: [],
    },
  ]);

  // Current file and graph state
  const [currentFile, setCurrentFile] = useState("/src/my_robot/urdf/robot.urdf");
  const [graphs, setGraphs] = useState({
    "/src/my_robot/urdf/robot.urdf": {
      nodes: [],
      edges: [],
    },
  });
  const [generatedCode, setGeneratedCode] = useState("");

  // Tabs state
  const [openTabs, setOpenTabs] = useState([
    {
      path: "/src/my_robot/urdf/robot.urdf",
      name: "robot.urdf",
      type: "file",
      unsaved: false,
    },
  ]);
  const [activeTab, setActiveTab] = useState("/src/my_robot/urdf/robot.urdf");
  const [showTerminal, setShowTerminal] = useState(false);

  const [logs, setLogs] = useState([
    { type: "info", message: "🚀 IDE Test Environment Initialized" },
    { type: "success", message: "✓ BlockCanvas loaded" },
    { type: "success", message: "✓ FileExplorer loaded" },
    { type: "info", message: "Drag blocks from the palette to the canvas" },
  ]);

  // Handle canvas selection from CanvasSelector
  const handleCanvasSelect = useCallback(async (selectedCanvas) => {
    try {
      setCanvasLoading(true);
      setCanvas(selectedCanvas);
      addLog("success", `✓ Loaded canvas: ${selectedCanvas.name}`);

      // Load file tree
      const tree = await fileApi.getFileTree(selectedCanvas.id);
      if (tree.length > 0) {
        setFileTree(tree);
        addLog("success", `✓ Loaded ${tree.length} root items`);
      }

      // Hide canvas selector and show IDE
      setShowCanvasSelector(false);
    } catch (error) {
      addLog("error", `Failed to load canvas: ${error.message}`);
      console.error("Canvas load error:", error);
    } finally {
      setCanvasLoading(false);
    }
  }, []);

  // Get current graph
  const currentGraph = useMemo(() => {
    return graphs[currentFile] || { nodes: [], edges: [] };
  }, [graphs, currentFile]);

  // Handle file selection
  const handleFileSelect = useCallback((path) => {
    setCurrentFile(path);
    setActiveTab(path);

    // Add to tabs if not already open
    setOpenTabs((prev) => {
      if (prev.some((tab) => tab.path === path)) return prev;

      const fileName = path.split("/").pop();
      return [
        ...prev,
        {
          path,
          name: fileName,
          type: "file",
          unsaved: false,
        },
      ];
    });

    addLog("info", `📂 Opened file: ${path}`);
  }, []);

  // Helper to add logs (defined early so other functions can use it)
  const addLog = useCallback((type, message) => {
    const timestamp = new Date().toLocaleTimeString();
    setLogs((prev) => [...prev, { type, message, timestamp }]);
  }, []);

  // Handle graph changes
  const handleGraphChange = useCallback(
    ({ nodes, edges }) => {
      setGraphs((prev) => ({
        ...prev,
        [currentFile]: { nodes, edges },
      }));
    },
    [currentFile]
  );

  // Handle code generation (debounced logging to avoid spam during node dragging)
  const lastLoggedCodeRef = useRef("");
  const handleCodeGenerated = useCallback(
    (result) => {
      // Handle both object {xml, robotId} and string returns
      const code = typeof result === 'string' ? result : (result?.xml || "");
      setGeneratedCode(code);

      // Only log if the code content actually changed (not just node positions)
      if (code && code !== lastLoggedCodeRef.current) {
        lastLoggedCodeRef.current = code;
        addLog("success", `✓ Code updated (${code.length} chars)`);
      }
    },
    [addLog]
  );

  // File operations
  const handleFileCreate = useCallback(
    async (path, type) => {
      if (!canvas) return;

      try {
        addLog("info", `➕ Creating ${type}: ${path}`);
        await fileApi.createFile(canvas.id, {
          path,
          file_type: type,
          content: type === "file" ? "" : undefined,
        });

        // Reload file tree
        const tree = await fileApi.getFileTree(canvas.id);
        setFileTree(tree);
        addLog("success", `✓ Created ${type}: ${path}`);
      } catch (error) {
        addLog("error", `Failed to create ${type}: ${error.message}`);
      }
    },
    [canvas]
  );

  const handleFileDelete = useCallback(
    async (path) => {
      if (!canvas) return;

      if (!window.confirm(`Delete ${path}?`)) return;

      try {
        addLog("warning", `🗑️ Deleting: ${path}`);

        // Find file by path
        const files = await fileApi.listFiles(canvas.id);
        const file = files.find((f) => f.path === path);

        if (file) {
          await fileApi.deleteFile(canvas.id, file.id);

          // Reload file tree
          const tree = await fileApi.getFileTree(canvas.id);
          setFileTree(tree);
          addLog("success", `✓ Deleted: ${path}`);
        } else {
          addLog("error", `File not found: ${path}`);
        }
      } catch (error) {
        addLog("error", `Failed to delete: ${error.message}`);
      }
    },
    [canvas]
  );

  const handleFileRename = useCallback(
    async (oldPath, newPath) => {
      if (!canvas) return;

      try {
        addLog("info", `✏️ Renaming: ${oldPath} → ${newPath}`);

        // Find file by old path
        const files = await fileApi.listFiles(canvas.id);
        const file = files.find((f) => f.path === oldPath);

        if (file) {
          await fileApi.updateFile(canvas.id, file.id, { path: newPath });

          // Reload file tree
          const tree = await fileApi.getFileTree(canvas.id);
          setFileTree(tree);
          addLog("success", `✓ Renamed: ${oldPath} → ${newPath}`);
        } else {
          addLog("error", `File not found: ${oldPath}`);
        }
      } catch (error) {
        addLog("error", `Failed to rename: ${error.message}`);
      }
    },
    [canvas]
  );

  // Save current file
  const handleSave = useCallback(async () => {
    if (!canvas) {
      addLog("error", "⚠️ Canvas not loaded");
      return;
    }

    if (!generatedCode) {
      addLog("warning", "⚠️ No code to save");
      return;
    }

    if (!currentFile) {
      addLog("warning", "⚠️ No file selected");
      return;
    }

    try {
      addLog("info", `💾 Saving ${currentFile}...`);
      await fileApi.saveGeneratedCode(canvas.id, currentFile, generatedCode);
      addLog("success", `✓ Saved ${currentFile} (${generatedCode.length} chars)`);

      // Reload file tree to reflect changes
      const tree = await fileApi.getFileTree(canvas.id);
      setFileTree(tree);
    } catch (error) {
      addLog("error", `Failed to save: ${error.message}`);
    }
  }, [canvas, currentFile, generatedCode]);

  // Clear canvas
  const handleClear = useCallback(() => {
    if (window.confirm("Clear the canvas?")) {
      setGraphs((prev) => ({
        ...prev,
        [currentFile]: { nodes: [], edges: [] },
      }));
      setGeneratedCode("");
      addLog("info", "🧹 Canvas cleared");
    }
  }, [currentFile]);

  // Tab management
  const handleTabSelect = useCallback((path) => {
    if (path === "terminal") {
      setShowTerminal(true);
      setActiveTab("terminal");
      addLog("info", "📟 Opened terminal");
    } else {
      setShowTerminal(false);
      setActiveTab(path);
      setCurrentFile(path);
      addLog("info", `📂 Switched to: ${path}`);
    }
  }, []);

  const handleTabClose = useCallback((path) => {
    setOpenTabs((prev) => {
      const filtered = prev.filter((tab) => tab.path !== path);

      // If closing active tab, switch to another tab
      if (activeTab === path && filtered.length > 0) {
        const nextTab = filtered[filtered.length - 1];
        setActiveTab(nextTab.path);
        setCurrentFile(nextTab.path);
        setShowTerminal(false);
      } else if (filtered.length === 0) {
        setActiveTab(null);
        setCurrentFile(null);
        setShowTerminal(false);
      }

      addLog("info", `✕ Closed: ${path.split("/").pop()}`);
      return filtered;
    });
  }, [activeTab]);

  const handleToggleTerminal = useCallback(() => {
    if (showTerminal) {
      setShowTerminal(false);
      if (openTabs.length > 0) {
        const lastFileTab = openTabs[openTabs.length - 1];
        setActiveTab(lastFileTab.path);
        setCurrentFile(lastFileTab.path);
      }
      addLog("info", "✕ Closed terminal");
    } else {
      setShowTerminal(true);
      setActiveTab("terminal");

      // Add terminal tab if not already present
      setOpenTabs((prev) => {
        if (prev.some((tab) => tab.path === "terminal")) return prev;
        return [...prev, { path: "terminal", name: "Terminal", type: "terminal", unsaved: false }];
      });

      addLog("info", "📟 Opened terminal");
    }
  }, [showTerminal, openTabs]);

  const handleCommandExecute = useCallback(async (command, callback) => {
    if (!canvas) {
      callback?.("Error: Canvas not loaded");
      addLog("error", "Canvas not loaded");
      return;
    }

    addLog("info", `$ ${command}`);

    try {
      const result = await fileApi.executeCommand(canvas.id, command);

      // Send output to terminal
      if (result.output) {
        callback?.(result.output);
      }
      if (result.error) {
        callback?.(`\x1b[31m${result.error}\x1b[0m`); // Red color for errors
      }

      if (result.exit_code === 0) {
        addLog("success", `✓ Command executed successfully`);
      } else {
        addLog("warning", `Command exited with code ${result.exit_code}`);
      }
    } catch (error) {
      callback?.(`Error: ${error.message}`);
      addLog("error", `Failed to execute command: ${error.message}`);
    }
  }, [canvas]);

  // Show canvas selector if no canvas is selected
  if (showCanvasSelector) {
    return <CanvasSelector onCanvasSelect={handleCanvasSelect} />;
  }

  // Show loading state
  if (canvasLoading) {
    return (
      <div className="ide-test">
        <div className="ide-test__loading">Loading workspace...</div>
      </div>
    );
  }

  return (
    <div className="ide-test">
      {/* Header */}
      <header className="ide-test__header">
        <button className="ide-test__back" onClick={() => setShowCanvasSelector(true)}>
          ← Change Workspace
        </button>
        <h1 className="ide-test__title">
          <span className="ide-test__title-main">ROS Visual IDE</span>
          <span className="ide-test__title-sub">{canvas?.name || "Test Environment"}</span>
        </h1>
        <div className="ide-test__actions">
          <button
            className="btn btn--small"
            onClick={handleToggleTerminal}
            title="Toggle Terminal"
          >
            📟 Terminal
          </button>
          <button className="btn btn--small" onClick={handleClear}>
            Clear
          </button>
          <button className="btn btn--small btn--primary" onClick={handleSave}>
            💾 Save
          </button>
        </div>
      </header>

      {/* Main Layout */}
      <div className="ide-test__layout">
        {/* Left Sidebar - File Explorer */}
        <aside className="ide-test__explorer">
          <FileExplorer
            files={fileTree}
            currentFile={currentFile}
            onFileSelect={handleFileSelect}
            onFileCreate={handleFileCreate}
            onFileDelete={handleFileDelete}
            onFileRename={handleFileRename}
            loading={canvasLoading}
          />
        </aside>

        {/* Center - Canvas with Palette on Top */}
        <main className="ide-test__canvas">
          {/* Block Palette - Top Tabs */}
          <div className="ide-test__palette">
            <CategorizedPalette
              categories={paletteCategorized}
              defaultCategory="URDF"
            />
          </div>

          {/* Tab Bar */}
          <TabBar
            tabs={openTabs}
            activeTab={activeTab}
            onTabSelect={handleTabSelect}
            onTabClose={handleTabClose}
          />

          {/* Content Area */}
          <div className="ide-test__canvas-content">
            {showTerminal ? (
              <Terminal
                onCommandExecute={handleCommandExecute}
                workingDirectory="/workspaces"
                username="developer"
                canvasId="test-workspace"
              />
            ) : currentFile ? (
              <BlockCanvas
                key={currentFile}
                initialNodes={currentGraph.nodes}
                initialEdges={currentGraph.edges}
                onGraphChange={handleGraphChange}
                codeGenerator={computeUrdfXml}
                onCodeGenerated={handleCodeGenerated}
                readOnly={false}
              />
            ) : (
              <div className="ide-test__empty">
                <h3>No file selected</h3>
                <p>Select a file from the explorer or create a new one</p>
              </div>
            )}
          </div>
        </main>

        {/* Right Sidebar - Code Preview & Logs */}
        <aside className="ide-test__sidebar">
          {/* Generated Code */}
          <div className="ide-test__code">
            <div className="ide-test__code-header">
              <span className="ide-test__code-title">Generated Code</span>
              <button
                className="ide-test__code-copy"
                onClick={() => {
                  navigator.clipboard.writeText(generatedCode);
                  addLog("success", "📋 Code copied to clipboard");
                }}
                disabled={!generatedCode}
              >
                Copy
              </button>
            </div>
            <pre className="ide-test__code-content">
              {generatedCode || "// No code generated yet\n// Drag blocks to canvas to generate code"}
            </pre>
          </div>

          {/* Logs */}
          <div className="ide-test__logs">
            <div className="ide-test__logs-header">
              <span className="ide-test__logs-title">Console</span>
              <button
                className="ide-test__logs-clear"
                onClick={() => {
                  setLogs([]);
                  addLog("info", "🧹 Console cleared");
                }}
              >
                Clear
              </button>
            </div>
            <div className="ide-test__logs-content">
              {logs.map((log, i) => (
                <div key={i} className={`ide-test__log ide-test__log--${log.type}`}>
                  {log.timestamp && (
                    <span className="ide-test__log-time">[{log.timestamp}]</span>
                  )}
                  <span className="ide-test__log-message">{log.message}</span>
                </div>
              ))}
            </div>
          </div>
        </aside>
      </div>

      {/* Info Panel */}
      <div className="ide-test__info">
        <div className="ide-test__info-item">
          <strong>📦 Components:</strong> BlockCanvas, FileExplorer, CategorizedPalette
        </div>
        <div className="ide-test__info-item">
          <strong>🎯 Features:</strong> Drag blocks, Connect nodes, Generate code, Manage files
        </div>
        <div className="ide-test__info-item">
          <strong>⚠️ Status:</strong> Test mode - Backend integration pending
        </div>
      </div>
    </div>
  );
}

export default function IDETestPage() {
  return (
    <ReactFlowProvider>
      <IDETestPageInner />
    </ReactFlowProvider>
  );
}
