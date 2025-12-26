// =============================================================
// FILE: src/pages/IDETestPage.jsx
// Test page for the ROS Visual IDE components (BlockCanvas + FileExplorer)
// =============================================================
import React, { useState, useCallback, useMemo, useEffect } from "react";
import { ReactFlowProvider } from "@xyflow/react";
import { useNavigate } from "react-router-dom";
import { BlockCanvas } from "../components/ide/BlockCanvas";
import { FileExplorer } from "../components/ide/FileExplorer";
import { TabBar } from "../components/ide/TabBar";
import { Terminal } from "../components/ide/Terminal";
import { URDFViewer } from "../components/ide/URDFViewer";
import { CodeEditor } from "../components/ide/CodeEditor";
import { CategorizedPalette } from "../components/blocks";
import { paletteCategorized } from "../components/blocks";
import { computeUrdfXml } from "../components/blocks/urdf-helpers";
import CanvasSelector from "../components/ide/CanvasSelector";
import ThemeToggle from "../components/ThemeToggle";
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
  const [fileTree, setFileTree] = useState([]);

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

  // Workspace cache key
  const WORKSPACE_CACHE_KEY = "ide_workspace_cache";
  const [loadedFromCache, setLoadedFromCache] = useState(false);

  // Load cached workspace on mount
  useEffect(() => {
    const cachedWorkspace = localStorage.getItem(WORKSPACE_CACHE_KEY);
    if (cachedWorkspace) {
      try {
        const { canvas: cachedCanvas, fileTree: cachedTree, timestamp } = JSON.parse(cachedWorkspace);
        const age = Date.now() - timestamp;

        // Only use cache if less than 5 minutes old
        if (age < 5 * 60 * 1000) {
          console.log("[IDE] ⚡ Loading from cache (age:", Math.round(age / 1000), "s)");
          setCanvas(cachedCanvas);
          setFileTree(cachedTree);
          setShowCanvasSelector(false);
          setLoadedFromCache(true);

          // Refresh in background
          setTimeout(() => {
            refreshWorkspaceInBackground(cachedCanvas.id);
          }, 500);
        } else {
          console.log("[IDE] Cache expired, clearing");
          localStorage.removeItem(WORKSPACE_CACHE_KEY);
        }
      } catch (error) {
        console.error("[IDE] Failed to load cached workspace:", error);
        localStorage.removeItem(WORKSPACE_CACHE_KEY);
      }
    }
  }, []);

  // Refresh workspace data in background
  const refreshWorkspaceInBackground = useCallback(async (canvasId) => {
    try {
      console.log("[IDE] Refreshing workspace in background");
      const tree = await fileApi.getFileTree(canvasId, false);
      setFileTree(tree);
      setLoadedFromCache(false); // Clear cache indicator
      console.log("[IDE] ✓ Background refresh complete");
    } catch (error) {
      console.error("[IDE] Background refresh failed:", error);
    }
  }, []);

  // Handle canvas selection from CanvasSelector
  const handleCanvasSelect = useCallback(async (selectedCanvas) => {
    try {
      setCanvasLoading(true);
      setCanvas(selectedCanvas);

      // Check if we have cached data for this specific canvas
      const cachedWorkspace = localStorage.getItem(WORKSPACE_CACHE_KEY);
      let usedCache = false;

      if (cachedWorkspace) {
        try {
          const { canvas: cachedCanvas, fileTree: cachedTree, timestamp } = JSON.parse(cachedWorkspace);
          const age = Date.now() - timestamp;

          // Use cache if it's for the same canvas and less than 5 minutes old
          if (cachedCanvas.id === selectedCanvas.id && age < 5 * 60 * 1000) {
            console.log("[IDE] ⚡ Using cached file tree for this workspace");
            setFileTree(cachedTree);
            setLoadedFromCache(true);
            usedCache = true;
            setCanvasLoading(false);

            // Hide canvas selector and show IDE immediately
            setShowCanvasSelector(false);

            // Refresh in background
            setTimeout(() => {
              refreshWorkspaceInBackground(selectedCanvas.id);
            }, 500);

            return;
          }
        } catch (error) {
          console.error("[IDE] Failed to use cached data:", error);
        }
      }

      // No cache or different canvas - load from backend (but don't force Docker scan)
      setLoadedFromCache(false);
      const tree = await fileApi.getFileTree(selectedCanvas.id, false); // false = use backend cache
      if (tree.length > 0) {
        setFileTree(tree);
      }

      // Cache workspace data for fast loading next time
      const cacheData = {
        canvas: selectedCanvas,
        fileTree: tree,
        timestamp: Date.now(),
      };
      localStorage.setItem(WORKSPACE_CACHE_KEY, JSON.stringify(cacheData));
      console.log("[IDE] 💾 Workspace cached for fast loading");

      // Hide canvas selector and show IDE
      setShowCanvasSelector(false);
    } catch (error) {
      console.error("Canvas load error:", error);
    } finally {
      setCanvasLoading(false);
    }
  }, [WORKSPACE_CACHE_KEY]);

  // Refresh file tree (manual or after operations)
  const refreshFileTree = useCallback(async (forceRefresh = false) => {
    if (!canvas) return;

    try {
      console.log("[IDE] Refreshing file tree (force:", forceRefresh, ")");
      const tree = await fileApi.getFileTree(canvas.id, forceRefresh);
      setFileTree(tree);
      console.log("[IDE] File tree refreshed:", tree.length, "items");

      // Update cache with new file tree
      const cacheData = {
        canvas,
        fileTree: tree,
        timestamp: Date.now(),
      };
      localStorage.setItem(WORKSPACE_CACHE_KEY, JSON.stringify(cacheData));
    } catch (error) {
      console.error("[IDE] Failed to refresh file tree:", error);
    }
  }, [canvas, WORKSPACE_CACHE_KEY]);

  // Load file content from backend
  const loadFileContent = useCallback(async (path) => {
    if (!canvas) return;

    try {
      // Try to find file in database first
      const files = await fileApi.listFiles(canvas.id);
      const fileData = files.find((f) => f.path === path);

      let content = "";
      let fileId = null;

      if (fileData && fileData.content) {
        // File exists in database with content
        content = fileData.content;
        fileId = fileData.id;
        console.log("[IDE] Loaded file from database:", path);
      } else {
        // File doesn't exist in database or has no content
        // Try to read from Docker filesystem
        console.log("[IDE] File not in database, reading from Docker:", path);
        try {
          const dockerFile = await fileApi.readFromDocker(canvas.id, path);
          content = dockerFile.content || "";
          console.log("[IDE] Successfully read from Docker:", path);

          // If file exists in DB but has no content, update it
          if (fileData) {
            fileId = fileData.id;
            // Optionally auto-save to database
            await fileApi.updateFile(canvas.id, fileData.id, { content });
            console.log("[IDE] Updated database with Docker content");
          }
        } catch (dockerError) {
          console.error("[IDE] Failed to read from Docker:", dockerError);
          content = "// Error: Could not load file content";
        }
      }

      // Check if file has block metadata (was generated from blocks)
      let blockMetadata = null;
      if (fileData && fileData.metadata) {
        try {
          blockMetadata = JSON.parse(fileData.metadata);
          console.log("[IDE] File has block metadata:", blockMetadata);
        } catch (e) {
          console.warn("[IDE] Could not parse file metadata:", e);
        }
      }

      // Store metadata
      setFileMetadata((prev) => ({
        ...prev,
        [path]: {
          id: fileId,
          type: "file",
          fromDocker: !fileData || !fileData.content,
          blockMetadata: blockMetadata,
        },
      }));

      // Detect if file is visual blocks or text
      let isVisual = false;
      let parsedContent = null;

      // First check if file has block metadata (was generated from blocks)
      if (blockMetadata && blockMetadata.isBlockGenerated && blockMetadata.blockGraph) {
        console.log("[IDE] File was generated from blocks, loading in visual mode");
        isVisual = true;
        parsedContent = blockMetadata.blockGraph;
      } else {
        // Otherwise try to parse as JSON block format
        try {
          parsedContent = JSON.parse(content);
          // Check if it has nodes and edges (visual block format)
          if (parsedContent && parsedContent.nodes && parsedContent.edges) {
            isVisual = true;
          }
        } catch (e) {
          // Not JSON, treat as text
          isVisual = false;
        }
      }

      // Store the block graph if we have it
      if (isVisual && parsedContent) {
        const blockContent = JSON.stringify(parsedContent, null, 2);
        setFileContents((prev) => ({
          ...prev,
          [path]: blockContent,
        }));
      } else {
        // Store content as-is
        setFileContents((prev) => ({
          ...prev,
          [path]: content,
        }));
      }

      // Set editor mode
      setEditorMode(isVisual ? "visual" : "text");

      if (isVisual) {
        setGeneratedCode("");
      } else {
        setTextContent(content);
      }

      console.log("[IDE] Loaded file:", path, "Mode:", isVisual ? "visual" : "text");
    } catch (error) {
      console.error("[IDE] Failed to load file content:", error);
    }
  }, [canvas]);

  // Get current graph for visual editor
  const currentGraph = useMemo(() => {
    if (!currentFile || editorMode !== "visual") {
      return { nodes: [], edges: [] };
    }

    const content = fileContents[currentFile];
    if (!content) return { nodes: [], edges: [] };

    try {
      const parsed = JSON.parse(content);
      return {
        nodes: parsed.nodes || [],
        edges: parsed.edges || [],
      };
    } catch (e) {
      return { nodes: [], edges: [] };
    }
  }, [currentFile, fileContents, editorMode]);

  // Detect language for text editor
  const detectLanguage = useCallback((filename) => {
    if (!filename) return "python";

    const ext = filename.split(".").pop().toLowerCase();
    const languageMap = {
      py: "python",
      cpp: "cpp",
      c: "c",
      h: "cpp",
      hpp: "cpp",
      xml: "xml",
      urdf: "xml",
      xacro: "xml",
      yaml: "yaml",
      yml: "yaml",
      json: "json",
      md: "markdown",
      txt: "plaintext",
      sh: "shell",
    };

    return languageMap[ext] || "plaintext";
  }, []);

  // Handle file selection
  const handleFileSelect = useCallback((path) => {
    setCurrentFile(path);
    setActiveTab(path);

    // Load file content
    loadFileContent(path);

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
  }, [loadFileContent]);

  // Handle graph changes (visual editor)
  const handleGraphChange = useCallback(
    ({ nodes, edges }) => {
      if (!currentFile) return;

      const content = JSON.stringify({ nodes, edges }, null, 2);
      setFileContents((prev) => ({
        ...prev,
        [currentFile]: content,
      }));

      // Mark tab as unsaved
      setOpenTabs((prev) =>
        prev.map((tab) =>
          tab.path === currentFile ? { ...tab, unsaved: true } : tab
        )
      );
    },
    [currentFile]
  );

  // Handle text editor changes
  const handleTextChange = useCallback(
    (newContent) => {
      if (!currentFile) return;

      setTextContent(newContent);
      setFileContents((prev) => ({
        ...prev,
        [currentFile]: newContent,
      }));

      // Mark tab as unsaved
      setOpenTabs((prev) =>
        prev.map((tab) =>
          tab.path === currentFile ? { ...tab, unsaved: true } : tab
        )
      );
    },
    [currentFile]
  );

  // Handle code generation
  const handleCodeGenerated = useCallback((result) => {
    // Handle both object {xml, robotId} and string returns
    const code = typeof result === 'string' ? result : (result?.xml || "");
    setGeneratedCode(code);
  }, []);

  // Handle file saved from Convert2Code node
  const handleFileSaved = useCallback(
    async (filePath, content) => {
      console.log("[IDE] File saved from Convert2Code:", filePath);

      // Update file contents in memory (only if different)
      setFileContents((prev) => {
        if (prev[filePath] === content) return prev; // No change, prevent re-render
        return {
          ...prev,
          [filePath]: content,
        };
      });

      // Update text content if this is the current file
      if (filePath === currentFile) {
        setTextContent((prev) => {
          if (prev === content) return prev; // No change
          console.log("[IDE] Updated text content for current file");
          return content;
        });
      }

      // Mark file as saved (no unsaved changes)
      setOpenTabs((prev) =>
        prev.map((tab) =>
          tab.path === filePath ? { ...tab, unsaved: false } : tab
        )
      );

      // Refresh file metadata to get the block metadata (debounced)
      setTimeout(async () => {
        try {
          const files = await fileApi.listFiles(canvas.id);
          const fileData = files.find((f) => f.path === filePath);
          if (fileData && fileData.metadata) {
            const blockMetadata = JSON.parse(fileData.metadata);
            setFileMetadata((prev) => {
              // Only update if different
              if (prev[filePath]?.blockMetadata === blockMetadata) return prev;
              return {
                ...prev,
                [filePath]: {
                  ...prev[filePath],
                  blockMetadata: blockMetadata,
                },
              };
            });
            console.log("[IDE] Updated file metadata with block info");
          }
        } catch (error) {
          console.warn("[IDE] Could not refresh file metadata:", error);
        }

        // Refresh file tree to show the file
        refreshFileTree(false);
      }, 300); // Debounce to prevent rapid updates
    },
    [currentFile, refreshFileTree, canvas]
  );

  // File operations
  const handleFileCreate = useCallback(
    async (path, type) => {
      if (!canvas) return;

      try {
        await fileApi.createFile(canvas.id, {
          path,
          file_type: type,
          content: type === "file" ? "" : undefined,
        });

        // Auto-refresh file tree (cache will be invalidated by backend)
        await refreshFileTree(false);
      } catch (error) {
        console.error(`Failed to create ${type}:`, error);
      }
    },
    [canvas, refreshFileTree]
  );

  const handleFileDelete = useCallback(
    async (path) => {
      if (!canvas) return;

      if (!window.confirm(`Delete ${path}?`)) return;

      try {
        // Find file by path
        const files = await fileApi.listFiles(canvas.id);
        const file = files.find((f) => f.path === path);

        if (file) {
          await fileApi.deleteFile(canvas.id, file.id);

          // Auto-refresh file tree (cache will be invalidated by backend)
          await refreshFileTree(false);
        }
      } catch (error) {
        console.error("Failed to delete:", error);
      }
    },
    [canvas, refreshFileTree]
  );

  const handleFileRename = useCallback(
    async (oldPath, newPath) => {
      if (!canvas) return;

      try {
        // Find file by old path
        const files = await fileApi.listFiles(canvas.id);
        const file = files.find((f) => f.path === oldPath);

        if (file) {
          await fileApi.updateFile(canvas.id, file.id, { path: newPath });

          // Auto-refresh file tree (cache will be invalidated by backend)
          await refreshFileTree(false);
        }
      } catch (error) {
        console.error("Failed to rename:", error);
      }
    },
    [canvas, refreshFileTree]
  );

  // Save current file
  const handleSave = useCallback(async () => {
    if (!canvas || !currentFile) {
      alert("No file selected to save");
      return;
    }

    try {
      // Get content from appropriate source based on editor mode
      let content;
      if (editorMode === "text") {
        content = textContent;
      } else {
        content = fileContents[currentFile];
      }

      const metadata = fileMetadata[currentFile];

      if (content === undefined || content === null) {
        console.error("[IDE] No content to save for:", currentFile);
        alert("No content to save");
        return;
      }

      console.log("[IDE] Saving file:", currentFile, "Mode:", editorMode, "Content length:", content.length);

      // If file has ID in metadata, update it
      if (metadata && metadata.id) {
        console.log("[IDE] Updating existing file with ID:", metadata.id);
        await fileApi.updateFile(canvas.id, metadata.id, {
          content: content,
        });
        console.log("[IDE] ✓ File updated in database");
      } else {
        // File doesn't exist in database yet, create it
        console.log("[IDE] Creating new file:", currentFile);
        const newFile = await fileApi.createFile(canvas.id, {
          path: currentFile,
          file_type: "file",
          content: content,
        });

        // Update metadata with new file ID
        setFileMetadata((prev) => ({
          ...prev,
          [currentFile]: {
            id: newFile.id,
            type: "file",
            fromDocker: false,
          },
        }));

        console.log("[IDE] ✓ File created in database with ID:", newFile.id);
      }

      // Update fileContents to match saved state
      setFileContents((prev) => ({
        ...prev,
        [currentFile]: content,
      }));

      // Mark tab as saved
      setOpenTabs((prev) =>
        prev.map((tab) =>
          tab.path === currentFile ? { ...tab, unsaved: false } : tab
        )
      );

      console.log("[IDE] ✓ File saved successfully:", currentFile);

      // Show success feedback
      const fileName = currentFile.split('/').pop();
      // You could add a toast notification here
      console.log(`✅ Saved: ${fileName}`);

      // Optional: Trigger a refresh of the file tree to show the file
      setTimeout(() => refreshFileTree(false), 500);
    } catch (error) {
      console.error("[IDE] ❌ Failed to save file:", error);
      console.error("[IDE] Error details:", {
        message: error.message,
        stack: error.stack,
        currentFile,
        editorMode,
        hasContent: !!content,
      });
      alert(`Failed to save file: ${error.message}\n\nCheck console for details.`);
    }
  }, [canvas, currentFile, fileContents, fileMetadata, editorMode, textContent, refreshFileTree]);

  // Keyboard shortcut for save (Ctrl+S / Cmd+S)
  useEffect(() => {
    const handleKeyDown = (e) => {
      // Check for Ctrl+S (Windows/Linux) or Cmd+S (Mac)
      if ((e.ctrlKey || e.metaKey) && e.key === 's') {
        e.preventDefault(); // Prevent browser's default save dialog
        handleSave();
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
    };
  }, [handleSave]);

  // Clear canvas
  const handleClear = useCallback(() => {
    if (!currentFile) return;

    if (window.confirm("Clear the current file?")) {
      // Clear content based on mode
      if (editorMode === "visual") {
        // Clear visual blocks (empty graph)
        const emptyGraph = JSON.stringify({ nodes: [], edges: [] }, null, 2);
        setFileContents((prev) => ({
          ...prev,
          [currentFile]: emptyGraph,
        }));
      } else {
        // Clear text content
        setTextContent("");
        setFileContents((prev) => ({
          ...prev,
          [currentFile]: "",
        }));
      }

      setGeneratedCode("");

      // Mark as unsaved
      setOpenTabs((prev) =>
        prev.map((tab) =>
          tab.path === currentFile ? { ...tab, unsaved: true } : tab
        )
      );
    }
  }, [currentFile, editorMode]);

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

      // Add terminal tab if not already present
      setOpenTabs((prev) => {
        if (prev.some((tab) => tab.path === "terminal")) return prev;
        return [...prev, { path: "terminal", name: "Terminal", type: "terminal", unsaved: false }];
      });
    }
  }, [showTerminal, openTabs]);

  const handleCommandExecute = useCallback(async (command, callback) => {
    if (!canvas) {
      callback?.("Error: Canvas not loaded");
      return;
    }

    try {
      const result = await fileApi.executeCommand(canvas.id, command);

      // Send output to terminal
      if (result.output) {
        callback?.(result.output);
      }
      if (result.error) {
        callback?.(`\x1b[31m${result.error}\x1b[0m`); // Red color for errors
      }
    } catch (error) {
      callback?.(`Error: ${error.message}`);
    }
  }, [canvas]);

  // Handle execute from ConvertToCodeNode - opens terminal and runs command
  const handleExecuteFromNode = useCallback((command) => {
    // Open terminal if not already open
    if (!showTerminal) {
      setShowTerminal(true);
      setActiveTab("terminal");

      // Add terminal tab if not already present
      setOpenTabs((prev) => {
        if (prev.some((tab) => tab.path === "terminal")) return prev;
        return [...prev, { path: "terminal", name: "Terminal", type: "terminal", unsaved: false }];
      });
    }

    // Execute the command after a short delay to ensure terminal is ready
    setTimeout(() => {
      // Trigger command execution in terminal by dispatching a custom event
      window.dispatchEvent(new CustomEvent('executeCommand', { detail: { command } }));
    }, 100);
  }, [showTerminal]);

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
        <button className="ide-test__back" onClick={() => {
          // Don't clear cache - we check canvas ID when loading from cache
          setShowCanvasSelector(true);
        }}>
          ← Change Workspace
        </button>
        <h1 className="ide-test__title">
          <span className="ide-test__title-main">ROS Visual IDE</span>
          <span className="ide-test__title-sub">
            {canvas?.name || "Test Environment"}
            {loadedFromCache && (
              <span
                style={{
                  marginLeft: "0.5rem",
                  fontSize: "0.7em",
                  padding: "0.15rem 0.4rem",
                  background: "rgba(125, 249, 255, 0.15)",
                  border: "1px solid var(--neon)",
                  borderRadius: "4px",
                  color: "var(--neon)",
                  fontWeight: "600",
                }}
                title="Loaded from cache, refreshing in background..."
              >
                ⚡ CACHED
              </span>
            )}
          </span>
        </h1>
        <div className="ide-test__actions">
          <button
            className="btn btn--small"
            onClick={handleToggleTerminal}
            title="Toggle Terminal"
          >
            📟 Terminal
          </button>
          <button
            className="btn btn--small"
            onClick={() => refreshFileTree(true)}
            title="Force refresh from Docker"
          >
            🔄 Refresh
          </button>
          <button className="btn btn--small" onClick={handleClear}>
            Clear
          </button>
          <button className="btn btn--small btn--primary" onClick={handleSave}>
            💾 Save
          </button>
          <ThemeToggle />
        </div>
      </header>

      {/* Main Layout */}
      <div className={`ide-test__layout ${fileExplorerCollapsed ? 'ide-test__layout--collapsed' : ''} ${!currentFile || editorMode !== 'visual' ? 'ide-test__layout--no-sidebar' : ''}`}>
        {/* Left Sidebar - File Explorer */}
        <aside className={`ide-test__explorer ${fileExplorerCollapsed ? 'ide-test__explorer--collapsed' : ''}`}>
          <div className="ide-test__explorer-header">
            <span className="ide-test__explorer-title">FILES</span>
            <div style={{ display: "flex", gap: "0.25rem" }}>
              <button
                className="ide-test__explorer-refresh"
                onClick={() => refreshFileTree(true)}
                title="Refresh from Docker"
                style={{
                  background: "transparent",
                  border: "none",
                  color: "var(--neon)",
                  cursor: "pointer",
                  padding: "0.25rem",
                  fontSize: "1rem",
                  opacity: 0.7,
                  transition: "opacity 0.2s",
                }}
                onMouseEnter={(e) => e.target.style.opacity = "1"}
                onMouseLeave={(e) => e.target.style.opacity = "0.7"}
              >
                🔄
              </button>
              <button
                className="ide-test__explorer-toggle"
                onClick={() => setFileExplorerCollapsed(!fileExplorerCollapsed)}
                title={fileExplorerCollapsed ? "Expand file explorer" : "Collapse file explorer"}
              >
                {fileExplorerCollapsed ? '▶' : '◀'}
              </button>
            </div>
          </div>
          {!fileExplorerCollapsed && (
            <FileExplorer
              files={fileTree}
              currentFile={currentFile}
              onFileSelect={handleFileSelect}
              onFileCreate={handleFileCreate}
              onFileDelete={handleFileDelete}
              onFileRename={handleFileRename}
              loading={canvasLoading}
            />
          )}
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
                workingDirectory="/workspace"
                username="developer"
                canvasId={canvas?.id || "workspace"}
              />
            ) : currentFile ? (
              <>
                {/* Editor Mode Selector */}
                <div className="ide-test__editor-mode">
                  <button
                    className={`ide-test__editor-mode-btn ${editorMode === "visual" ? "active" : ""}`}
                    onClick={() => setEditorMode("visual")}
                    title="Visual Block Editor"
                  >
                    🧩 Visual
                  </button>
                  <button
                    className={`ide-test__editor-mode-btn ${editorMode === "text" ? "active" : ""}`}
                    onClick={() => setEditorMode("text")}
                    title="Text Editor"
                  >
                    📝 Text
                  </button>
                  <span className="ide-test__file-name">
                    {fileMetadata[currentFile]?.blockMetadata?.isBlockGenerated && (
                      <span style={{
                        marginRight: "0.5rem",
                        padding: "2px 6px",
                        background: "rgba(125, 249, 255, 0.2)",
                        border: "1px solid var(--neon)",
                        borderRadius: "3px",
                        fontSize: "0.75em",
                        fontWeight: "600",
                        color: "var(--neon)",
                      }} title="This file was generated from visual blocks">
                        🧩
                      </span>
                    )}
                    {currentFile.split("/").pop()}
                  </span>
                </div>

                {/* Editor Content */}
                <div className="ide-test__editor-container">
                  {editorMode === "visual" ? (
                    <BlockCanvas
                      key={currentFile}
                      initialNodes={currentGraph.nodes}
                      initialEdges={currentGraph.edges}
                      onGraphChange={handleGraphChange}
                      codeGenerator={computeUrdfXml}
                      onCodeGenerated={handleCodeGenerated}
                      readOnly={false}
                      canvasId={canvas?.id}
                      onExecute={handleExecuteFromNode}
                      currentFile={currentFile}
                      onFileSaved={handleFileSaved}
                    />
                  ) : (
                    <CodeEditor
                      content={textContent}
                      language={detectLanguage(currentFile)}
                      onChange={handleTextChange}
                      readOnly={false}
                      theme="dark"
                    />
                  )}
                </div>
              </>
            ) : (
              <div className="ide-test__empty">
                <h3>No file selected</h3>
                <p>Select a file from the explorer or create a new one</p>
              </div>
            )}
          </div>
        </main>

        {/* Right Sidebar - URDF Viewer & Code Preview (only in visual mode) */}
        {currentFile && editorMode === "visual" && (
          <aside className="ide-test__sidebar">
            {/* URDF Viewer */}
            <div className="ide-test__urdf-viewer">
              <URDFViewer xmlCode={generatedCode} />
            </div>

            {/* Generated Code */}
            <div className="ide-test__code">
              <div className="ide-test__code-header">
                <span className="ide-test__code-title">Generated Code</span>
                <button
                  className="ide-test__code-copy"
                  onClick={() => {
                    navigator.clipboard.writeText(generatedCode);
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
          </aside>
        )}
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
