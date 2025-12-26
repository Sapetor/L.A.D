// src/components/ide/EmbeddedIDE.jsx
// Embedded IDE component for lesson slides with tutorial overlay
import React, { useState, useCallback, useMemo, useEffect } from "react";
import { ReactFlowProvider } from "@xyflow/react";
import { BlockCanvas } from "./BlockCanvas";
import { FileExplorer } from "./FileExplorer";
import { TabBar } from "./TabBar";
import { Terminal } from "./Terminal";
import { CodeEditor } from "./CodeEditor";
import IDETutorial from "./IDETutorial";
import { CategorizedPalette } from "../blocks";
import { paletteCategorized } from "../blocks";
import { computeUrdfXml } from "../blocks/urdf-helpers";
import { useROS2Workspace } from "../../hooks/useROS2Workspace";
import fileApi from "../../services/fileApi";
import "../../styles/_rosflow.scss";
import "../../styles/components/_embedded-ide.scss";

function EmbeddedIDEInner({ tutorial, onTutorialComplete, onTutorialSkip }) {
  // Use existing ROS2 workspace hook
  const { workspace, fileTree, loading: workspaceLoading, error: workspaceError, canvasId, refreshWorkspace } = useROS2Workspace();

  // Current file state
  const [currentFile, setCurrentFile] = useState(null);
  const [fileContents, setFileContents] = useState({}); // Map of path -> content
  const [fileMetadata, setFileMetadata] = useState({}); // Map of path -> { id, type, isVisual }
  const [editorMode, setEditorMode] = useState("visual"); // "visual" or "text"
  const [generatedCode, setGeneratedCode] = useState("");
  const [textContent, setTextContent] = useState(""); // Current text editor content

  // Tabs state
  const [openTabs, setOpenTabs] = useState([]);
  const [activeTab, setActiveTab] = useState(null);
  const [showTerminal, setShowTerminal] = useState(false);
  const [fileExplorerCollapsed, setFileExplorerCollapsed] = useState(false);

  // Ensure src directory exists
  useEffect(() => {
    const ensureSrcDirectory = async () => {
      if (!canvasId) return;

      try {
        const files = await fileApi.listFiles(canvasId);
        const srcExists = files.some(f => f.path === "/src" && f.file_type === "directory");

        if (!srcExists) {
          console.log("[Embedded IDE] Creating src directory");
          await fileApi.createFile(canvasId, {
            path: "/src",
            file_type: "directory",
          });
          await refreshWorkspace(false);
        }
      } catch (error) {
        console.warn("[Embedded IDE] Failed to ensure src directory:", error);
      }
    };

    ensureSrcDirectory();
  }, [canvasId, refreshWorkspace]);

  // File selection - receives path string from FileExplorer
  const handleFileSelect = useCallback(async (path) => {
    if (!canvasId) return;

    try {
      // Find file in file tree
      const findFileInTree = (items, targetPath) => {
        for (const item of items) {
          if (item.path === targetPath) return item;
          if (item.children) {
            const found = findFileInTree(item.children, targetPath);
            if (found) return found;
          }
        }
        return null;
      };

      const file = findFileInTree(fileTree, path);
      if (!file) {
        console.error("[Embedded IDE] File not found in tree:", path);
        return;
      }

      if (file.type === "directory") return;

      setCurrentFile(path);

      // Determine if file should be opened in visual mode
      const isVisualFile = file.isVisual || path.endsWith('.canvas') || path.endsWith('.blocks');

      // Check if already loaded
      if (fileContents[path] !== undefined) {
        const content = fileContents[path];
        console.log("[Embedded IDE] Loading cached file:", path, "Length:", content.length);

        // IMPORTANT: Always update textContent when switching files
        setTextContent(content);
        setEditorMode(isVisualFile ? "visual" : "text");

        // Add to tabs if not already there
        setOpenTabs((prev) => {
          if (prev.some((tab) => tab.path === path)) return prev;
          const fileName = path.split("/").pop();
          return [...prev, { path, name: fileName, type: "file", unsaved: false }];
        });
        setActiveTab(path);
        setShowTerminal(false);
        return;
      }

      // Fetch file content
      let content = "";
      let fileId = file.id;

      // Try to get file from database
      const files = await fileApi.listFiles(canvasId);
      const fileData = files.find((f) => f.path === path);

      if (fileData && fileData.content) {
        content = fileData.content;
        fileId = fileData.id;
        console.log("[Embedded IDE] Loaded file from database:", path);
      } else {
        // Read from Docker filesystem
        console.log("[Embedded IDE] File not in database, reading from Docker:", path);
        try {
          const dockerFile = await fileApi.readFromDocker(canvasId, path);
          content = dockerFile.content || "";
        } catch (error) {
          console.warn("[Embedded IDE] Failed to read from Docker:", error);
          content = "";
        }
      }

      // Check if content is actually a visual block graph
      let actuallyVisual = isVisualFile;
      if (!actuallyVisual && content) {
        try {
          const parsed = JSON.parse(content);
          if (parsed && parsed.nodes && Array.isArray(parsed.nodes)) {
            actuallyVisual = true;
            console.log("[Embedded IDE] Detected visual file by content structure");
          }
        } catch (e) {
          // Not JSON, definitely not visual
        }
      }

      setFileContents((prev) => ({
        ...prev,
        [path]: content,
      }));

      setFileMetadata((prev) => ({
        ...prev,
        [path]: {
          id: fileId,
          type: "file",
          isVisual: actuallyVisual,
        },
      }));

      console.log("[Embedded IDE] Loaded new file:", path, "Visual:", actuallyVisual, "Length:", content.length);
      setTextContent(content);
      setEditorMode(actuallyVisual ? "visual" : "text");

      // Add to tabs
      setOpenTabs((prev) => {
        if (prev.some((tab) => tab.path === path)) return prev;
        const fileName = path.split("/").pop();
        return [...prev, { path, name: fileName, type: "file", unsaved: false }];
      });
      setActiveTab(path);
      setShowTerminal(false);
    } catch (error) {
      console.error("[Embedded IDE] Failed to load file:", error);
      alert(`Failed to load file: ${error.message}`);
    }
  }, [canvasId, fileContents, fileTree]);

  // Auto-create and open default canvas file for blocks
  useEffect(() => {
    if (!canvasId || currentFile) return;

    const createDefaultCanvas = async () => {
      try {
        const defaultPath = "/ros2_commands.canvas";

        // Check if default canvas file exists
        const files = await fileApi.listFiles(canvasId);
        let canvasFile = files.find((f) => f.path === defaultPath);

        if (!canvasFile) {
          // Create default canvas file
          console.log("[Embedded IDE] Creating default canvas file");
          canvasFile = await fileApi.createFile(canvasId, {
            path: defaultPath,
            file_type: "file",
            content: JSON.stringify({ nodes: [], edges: [] }, null, 2),
          });
        }

        // Open the default canvas file
        handleFileSelect(defaultPath);
      } catch (error) {
        console.error("[Embedded IDE] Failed to create default canvas:", error);
      }
    };

    createDefaultCanvas();
  }, [canvasId, currentFile, handleFileSelect]);

  // Parse current graph
  const currentGraph = useMemo(() => {
    if (!currentFile || editorMode !== "visual") {
      return { nodes: [], edges: [] };
    }

    const content = fileContents[currentFile];
    if (!content) return { nodes: [], edges: [] };

    try {
      return JSON.parse(content);
    } catch {
      return { nodes: [], edges: [] };
    }
  }, [currentFile, fileContents, editorMode]);

  // Handle graph changes
  const handleGraphChange = useCallback(({ nodes, edges }) => {
    if (!currentFile) return;

    const newGraph = JSON.stringify({ nodes, edges }, null, 2);
    setFileContents((prev) => ({
      ...prev,
      [currentFile]: newGraph,
    }));

    // Mark as unsaved
    setOpenTabs((prev) =>
      prev.map((tab) =>
        tab.path === currentFile ? { ...tab, unsaved: true } : tab
      )
    );
  }, [currentFile]);

  // Handle code generation
  const handleCodeGenerated = useCallback((code) => {
    // Extract string from code generator result
    // computeUrdfXml returns { xml, robotId } but we only need the xml string
    if (typeof code === 'object' && code !== null) {
      setGeneratedCode(code.xml || code.code || JSON.stringify(code, null, 2));
    } else {
      setGeneratedCode(code || "");
    }
  }, []);

  // Extract generated code from toCode nodes (for ROS2 blocks)
  useEffect(() => {
    if (!currentFile) return;

    // Check if this is a visual file with blocks
    const isVisual = fileMetadata[currentFile]?.isVisual ||
                     currentFile.endsWith('.canvas') ||
                     currentFile.endsWith('.blocks');

    if (!isVisual) return;

    try {
      const graph = JSON.parse(fileContents[currentFile] || "{}");
      const toCodeNode = graph.nodes?.find(n => n.type === "toCode");

      if (toCodeNode?.data?.preview) {
        // ROS2 code is stored in the toCode node's preview field
        setGeneratedCode(toCodeNode.data.preview);
        console.log("[EmbeddedIDE] Extracted code from toCode node:", toCodeNode.data.preview.substring(0, 100));
      } else {
        // No toCode node or no preview - clear generated code
        setGeneratedCode("");
      }
    } catch (e) {
      console.warn("[EmbeddedIDE] Failed to parse graph for code extraction:", e);
    }
  }, [currentFile, fileContents, fileMetadata]);

  // Handle text changes
  const handleTextChange = useCallback((newText) => {
    setTextContent(newText);

    setFileContents((prev) => ({
      ...prev,
      [currentFile]: newText,
    }));

    // Mark as unsaved
    setOpenTabs((prev) =>
      prev.map((tab) =>
        tab.path === currentFile ? { ...tab, unsaved: true } : tab
      )
    );
  }, [currentFile]);

  // Save file
  const handleSave = useCallback(async () => {
    if (!canvasId || !currentFile) return;

    try {
      let content;
      if (editorMode === "text") {
        content = textContent;
      } else {
        content = fileContents[currentFile];
      }

      const metadata = fileMetadata[currentFile];

      if (content === undefined || content === null) {
        alert("No content to save");
        return;
      }

      if (metadata && metadata.id) {
        await fileApi.updateFile(canvasId, metadata.id, { content });
      } else {
        const newFile = await fileApi.createFile(canvasId, {
          path: currentFile,
          file_type: "file",
          content,
        });

        setFileMetadata((prev) => ({
          ...prev,
          [currentFile]: {
            id: newFile.id,
            type: "file",
            fromDocker: false,
          },
        }));
      }

      setFileContents((prev) => ({
        ...prev,
        [currentFile]: content,
      }));

      setOpenTabs((prev) =>
        prev.map((tab) =>
          tab.path === currentFile ? { ...tab, unsaved: false } : tab
        )
      );

      setTimeout(() => refreshWorkspace(false), 500);
    } catch (error) {
      console.error("[Embedded IDE] Failed to save:", error);
      alert(`Failed to save file: ${error.message}`);
    }
  }, [canvasId, currentFile, fileContents, fileMetadata, editorMode, textContent, refreshWorkspace]);

  // Tab management
  const handleTabSelect = useCallback((path) => {
    if (path === "terminal") {
      setShowTerminal(true);
      setActiveTab("terminal");
    } else {
      setShowTerminal(false);
      setActiveTab(path);
      setCurrentFile(path);
    }
  }, []);

  const handleTabClose = useCallback((path) => {
    setOpenTabs((prev) => {
      const filtered = prev.filter((tab) => tab.path !== path);

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
    } else {
      setShowTerminal(true);
      setActiveTab("terminal");

      setOpenTabs((prev) => {
        if (prev.some((tab) => tab.path === "terminal")) return prev;
        return [...prev, { path: "terminal", name: "Terminal", type: "terminal", unsaved: false }];
      });
    }
  }, [showTerminal, openTabs]);

  const handleCommandExecute = useCallback(async (command, callback) => {
    if (!canvasId) {
      callback?.("Error: Canvas not loaded");
      return;
    }

    try {
      // Execute commands in workspace root (will default to canvas.docker_path on backend)
      // This ensures ros2 pkg create works in the proper src/ directory
      const result = await fileApi.executeCommand(canvasId, command);

      if (result.output) {
        callback?.(result.output);
      }
      if (result.error) {
        callback?.(`\x1b[31m${result.error}\x1b[0m`);
      }

      // Auto-refresh file tree after filesystem-modifying commands
      const shouldRefresh = command.match(/^(ros2 pkg create|mkdir|touch|rm|mv|cp|colcon build)/);
      if (shouldRefresh) {
        console.log("[EmbeddedIDE] Auto-refreshing file tree after command:", command);
        setTimeout(() => refreshWorkspace(), 1000); // Force refresh after 1 second
      }
    } catch (error) {
      callback?.(`Error: ${error.message}`);
    }
  }, [canvasId, refreshWorkspace]);

  const handleExecuteFromNode = useCallback((command) => {
    if (!showTerminal) {
      setShowTerminal(true);
      setActiveTab("terminal");

      setOpenTabs((prev) => {
        if (prev.some((tab) => tab.path === "terminal")) return prev;
        return [...prev, { path: "terminal", name: "Terminal", type: "terminal", unsaved: false }];
      });
    }

    setTimeout(() => {
      window.dispatchEvent(new CustomEvent('executeCommand', { detail: { command } }));
    }, 100);
  }, [showTerminal]);

  const detectLanguage = useCallback((path) => {
    if (!path) return "text";
    const ext = path.split(".").pop();
    const languageMap = {
      py: "python",
      js: "javascript",
      jsx: "javascript",
      ts: "typescript",
      tsx: "typescript",
      json: "json",
      xml: "xml",
      urdf: "xml",
      yaml: "yaml",
      yml: "yaml",
      md: "markdown",
      txt: "text",
    };
    return languageMap[ext] || "text";
  }, []);

  const handleFileSaved = useCallback(async () => {
    await handleSave();
    await refreshWorkspace(false);
  }, [handleSave, refreshWorkspace]);

  // File operations
  const handleFileCreate = useCallback(
    async (path, type) => {
      if (!canvasId) return;

      try {
        await fileApi.createFile(canvasId, {
          path,
          file_type: type,
          content: type === "file" ? "" : undefined,
        });

        // Auto-refresh file tree
        await refreshWorkspace(false);
      } catch (error) {
        console.error(`Failed to create ${type}:`, error);
        alert(`Failed to create ${type}: ${error.message}`);
      }
    },
    [canvasId, refreshWorkspace]
  );

  const handleFileDelete = useCallback(
    async (path) => {
      if (!canvasId) return;

      try {
        // Find file by path
        const files = await fileApi.listFiles(canvasId);
        const file = files.find((f) => f.path === path);

        if (file) {
          await fileApi.deleteFile(canvasId, file.id);

          // Auto-refresh file tree
          await refreshWorkspace(false);
        }
      } catch (error) {
        console.error("Failed to delete:", error);
        alert(`Failed to delete: ${error.message}`);
      }
    },
    [canvasId, refreshWorkspace]
  );

  const handleFileRename = useCallback(
    async (oldPath, newPath) => {
      if (!canvasId) return;

      try {
        // Find file by old path
        const files = await fileApi.listFiles(canvasId);
        const file = files.find((f) => f.path === oldPath);

        if (file) {
          await fileApi.updateFile(canvasId, file.id, { path: newPath });

          // Auto-refresh file tree
          await refreshWorkspace(false);
        }
      } catch (error) {
        console.error("Failed to rename:", error);
        alert(`Failed to rename: ${error.message}`);
      }
    },
    [canvasId, refreshWorkspace]
  );

  if (workspaceLoading) {
    return (
      <div className="embedded-ide">
        <div className="embedded-ide__loading">Loading ROS2 workspace...</div>
      </div>
    );
  }

  if (workspaceError) {
    return (
      <div className="embedded-ide">
        <div className="embedded-ide__loading" style={{ color: "red" }}>
          Error loading workspace: {workspaceError}
        </div>
      </div>
    );
  }

  return (
    <div className="embedded-ide">
      {/* Header */}
      <header className="embedded-ide__header">
        <h2 className="embedded-ide__title">{workspace?.name || "ROS2 Workspace"}</h2>
        <div className="embedded-ide__actions">
          <button
            className="btn btn--small"
            onClick={async () => {
              const name = prompt("Enter canvas file name:", "new_canvas.canvas");
              if (!name) return;

              try {
                const newFile = await fileApi.createFile(canvasId, {
                  path: `/${name}`,
                  file_type: "file",
                  content: JSON.stringify({ nodes: [], edges: [] }, null, 2),
                });

                await refreshWorkspace(false);

                handleFileSelect(newFile.path);
              } catch (error) {
                alert(`Failed to create canvas: ${error.message}`);
              }
            }}
          >
            🎨 New Canvas
          </button>
          <button className="btn btn--small" onClick={handleToggleTerminal}>
            📟 Terminal
          </button>
          <button
            className="btn btn--small"
            onClick={async () => {
              try {
                console.log("[Embedded IDE] Refreshing workspace...");
                await refreshWorkspace(true);
                console.log("[Embedded IDE] Workspace refreshed successfully");
              } catch (error) {
                console.error("[Embedded IDE] Failed to refresh workspace:", error);
                alert(`Failed to refresh: ${error.message}`);
              }
            }}
          >
            🔄 Refresh
          </button>
          <button className="btn btn--small btn--primary" onClick={handleSave}>
            💾 Save
          </button>
        </div>
      </header>

      {/* Main Layout */}
      <div className={`embedded-ide__layout ${fileExplorerCollapsed ? 'embedded-ide__layout--collapsed' : ''}`}>
        {/* Left Sidebar - File Explorer */}
        <aside className={`embedded-ide__explorer ${fileExplorerCollapsed ? 'embedded-ide__explorer--collapsed' : ''}`}>
          <div className="embedded-ide__explorer-header">
            <span>FILES</span>
            <button onClick={() => setFileExplorerCollapsed(!fileExplorerCollapsed)}>
              {fileExplorerCollapsed ? '▶' : '◀'}
            </button>
          </div>
          {!fileExplorerCollapsed && (
            <FileExplorer
              files={fileTree}
              currentFile={currentFile}
              onFileSelect={handleFileSelect}
              onFileCreate={handleFileCreate}
              onFileDelete={handleFileDelete}
              onFileRename={handleFileRename}
              loading={workspaceLoading}
            />
          )}
        </aside>

        {/* Center - Canvas and Code */}
        <main className="embedded-ide__main">
          {/* Block Palette */}
          <div className="embedded-ide__palette">
            <CategorizedPalette
              categories={paletteCategorized}
              defaultCategory="ROS"
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
          <div className="embedded-ide__content">
            {showTerminal ? (
              <Terminal
                onCommandExecute={handleCommandExecute}
                workingDirectory="~"
                username="developer"
                canvasId={canvasId}
              />
            ) : currentFile ? (
              <>
                {/* Editor Mode Selector */}
                <div className="embedded-ide__editor-mode">
                  <button
                    className={`embedded-ide__editor-mode-btn ${editorMode === "visual" ? "active" : ""}`}
                    onClick={() => setEditorMode("visual")}
                  >
                    🧩 Visual
                  </button>
                  <button
                    className={`embedded-ide__editor-mode-btn ${editorMode === "text" ? "active" : ""}`}
                    onClick={() => setEditorMode("text")}
                  >
                    📝 Text
                  </button>
                  <span className="embedded-ide__file-name">
                    {currentFile.split("/").pop()}
                  </span>
                </div>

                {/* Editor */}
                <div className="embedded-ide__editor">
                  {editorMode === "visual" ? (
                    <BlockCanvas
                      key={currentFile}
                      initialNodes={currentGraph.nodes}
                      initialEdges={currentGraph.edges}
                      onGraphChange={handleGraphChange}
                      codeGenerator={computeUrdfXml}
                      onCodeGenerated={handleCodeGenerated}
                      readOnly={false}
                      canvasId={canvasId}
                      onExecute={handleExecuteFromNode}
                      currentFile={currentFile}
                      onFileSaved={handleFileSaved}
                    />
                  ) : (
                    <CodeEditor
                      key={currentFile} // Force re-render when switching files
                      content={(() => {
                        // For visual files, show generated code in text mode (read-only)
                        // For regular text files, show editable content
                        const isVisual = fileMetadata[currentFile]?.isVisual;
                        const content = isVisual
                          ? (generatedCode || "// No code generated yet\n// Switch to Visual mode and add blocks to generate code")
                          : textContent;

                        console.log("[EmbeddedIDE] Text mode content:", {
                          currentFile,
                          isVisual,
                          generatedCodeLength: generatedCode?.length || 0,
                          textContentLength: textContent?.length || 0,
                          contentPreview: content?.substring(0, 100)
                        });

                        return content;
                      })()}
                      language={detectLanguage(currentFile)}
                      onChange={handleTextChange}
                      readOnly={fileMetadata[currentFile]?.isVisual} // Visual files are read-only in text mode
                      theme="dark"
                    />
                  )}
                </div>

                {/* Generated Code Below Canvas */}
                {editorMode === "visual" && generatedCode && typeof generatedCode === "string" && (
                  <div className="embedded-ide__code">
                    <div className="embedded-ide__code-header">
                      <span>Generated Code</span>
                      <button onClick={() => navigator.clipboard.writeText(generatedCode)}>
                        Copy
                      </button>
                    </div>
                    <pre className="embedded-ide__code-content">{generatedCode}</pre>
                  </div>
                )}
              </>
            ) : (
              <div className="embedded-ide__empty">
                <h3>No file selected</h3>
                <p>Select a file from the explorer or create a new one</p>
              </div>
            )}
          </div>
        </main>
      </div>

      {/* Tutorial Overlay */}
      {tutorial && tutorial.length > 0 && (
        <IDETutorial
          steps={tutorial}
          onComplete={onTutorialComplete}
          onSkip={onTutorialSkip}
          autoStart={true}
        />
      )}
    </div>
  );
}

export default function EmbeddedIDE(props) {
  return (
    <ReactFlowProvider>
      <EmbeddedIDEInner {...props} />
    </ReactFlowProvider>
  );
}
