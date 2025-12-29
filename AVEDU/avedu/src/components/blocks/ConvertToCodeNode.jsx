import React, { useState } from "react";
import { Handle, Position } from "@xyflow/react";
import { createFile, updateFile, listFiles } from "../../services/fileApi";

export default function ConvertToCodeNode({ data, id }) {
  const count = Number(data?.inCount || 0);
  const preview = data?.preview || "";
  const onExecute = data?.onExecute;
  const canvasId = data?.canvasId;
  const currentFile = data?.currentFile; // Current file path being edited
  const onFileSaved = data?.onFileSaved; // Callback to notify parent that file was saved
  const getGraphSnapshot = data?.getGraphSnapshot; // Function to get current block graph
  const hasCommand = preview && preview !== "# (aún no hay nada conectado…)";

  const [fileName, setFileName] = useState(currentFile || "generated_code.py");
  const [saving, setSaving] = useState(false);
  const [saveStatus, setSaveStatus] = useState("");

  const handleRunClick = () => {
    if (hasCommand && onExecute) {
      onExecute(preview);
    }
  };

  const handleSaveFile = async () => {
    if (!hasCommand || !canvasId) {
      setSaveStatus("⚠ No code to save or workspace not ready");
      setTimeout(() => setSaveStatus(""), 3000);
      return;
    }

    setSaving(true);
    setSaveStatus("");

    try {
      // Ensure filename has .py extension
      let finalFileName = fileName.endsWith(".py") ? fileName : `${fileName}.py`;

      // Normalize path separators (handle both / and \)
      finalFileName = finalFileName.replace(/\\/g, '/');

      // Remove leading slash if present
      if (finalFileName.startsWith('/')) {
        finalFileName = finalFileName.substring(1);
      }

      const filePath = finalFileName;

      console.log("[Convert2Code] Saving file:", { canvasId, filePath, contentLength: preview.length });

      // Create parent directories if the path has subdirectories
      const pathParts = filePath.split('/');
      if (pathParts.length > 1) {
        // Need to create parent directories
        const dirs = pathParts.slice(0, -1); // All parts except the filename
        let currentPath = '';

        for (const dir of dirs) {
          currentPath = currentPath ? `${currentPath}/${dir}` : dir;

          // Check if directory exists
          try {
            const files = await listFiles(canvasId);
            const dirExists = files.some((f) => f.path === currentPath && f.file_type === "directory");

            if (!dirExists) {
              console.log("[Convert2Code] Creating directory:", currentPath);
              await createFile(canvasId, {
                path: currentPath,
                file_type: "directory",
              });
            }
          } catch (dirError) {
            console.warn("[Convert2Code] Error checking/creating directory:", currentPath, dirError);
            // Continue anyway - backend might handle it
          }
        }
      }

      console.log("[Convert2Code] Parent directories ready, saving file:", filePath);

      // Try to check if file exists, but don't fail if listFiles doesn't work
      let existingFile = null;
      try {
        const files = await listFiles(canvasId);
        console.log("[Convert2Code] Found", files.length, "files");
        existingFile = files.find((f) => f.path === filePath);
        console.log("[Convert2Code] Existing file:", existingFile ? existingFile.id : "none");
      } catch (listError) {
        console.warn("[Convert2Code] Could not list files, will try to create:", listError);
      }

      // TODO: Metadata feature disabled until backend supports it
      // Get current block graph to save as metadata
      // let blockGraph = null;
      // if (getGraphSnapshot) {
      //   blockGraph = getGraphSnapshot();
      //   console.log("[Convert2Code] Captured block graph:", blockGraph);
      // }

      if (existingFile) {
        // Update existing file
        console.log("[Convert2Code] Updating existing file:", existingFile.id);
        await updateFile(canvasId, existingFile.id, {
          content: preview,
          // TODO: Add metadata field to backend WorkspaceFile model
          // metadata: blockGraph ? JSON.stringify({
          //   isBlockGenerated: true,
          //   blockGraph: blockGraph,
          //   generatedAt: new Date().toISOString(),
          // }) : undefined,
        });
        setSaveStatus(`✅ Updated ${finalFileName}`);
      } else {
        // Create new file
        console.log("[Convert2Code] Creating new file:", filePath);
        const createData = {
          path: filePath,
          content: preview,
          file_type: "file",
          // TODO: Add metadata field to backend WorkspaceFile model
          // metadata: blockGraph ? JSON.stringify({
          //   isBlockGenerated: true,
          //   blockGraph: blockGraph,
          //   generatedAt: new Date().toISOString(),
          // }) : undefined,
        };
        console.log("[Convert2Code] Create data:", createData);

        const result = await createFile(canvasId, createData);
        console.log("[Convert2Code] Create result:", result);
        setSaveStatus(`✅ Created ${finalFileName}`);
      }

      console.log("[Convert2Code] File saved successfully:", filePath);

      // Notify parent component that file was saved (so it can refresh)
      if (onFileSaved) {
        console.log("[Convert2Code] Notifying parent of file save");
        onFileSaved(filePath, preview);
      }

      // Clear success message after 5 seconds
      setTimeout(() => setSaveStatus(""), 5000);
    } catch (error) {
      console.error("[Convert2Code] Failed to save file:", error);
      console.error("[Convert2Code] Error details:", {
        message: error.message,
        stack: error.stack,
        canvasId,
        fileName,
        response: error.response?.data,
        status: error.response?.status
      });

      let errorMsg = error.message;
      if (error.response?.status === 500) {
        errorMsg = "Server error. Check that the package exists and path is valid.";
      } else if (error.response?.data?.error) {
        errorMsg = error.response.data.error;
      }

      setSaveStatus(`❌ Error: ${errorMsg}`);
      setTimeout(() => setSaveStatus(""), 8000); // Longer timeout for error messages
    } finally {
      setSaving(false);
    }
  };

  return (
    <div className="rf-card" style={{ minWidth: 320 }}>
      <div className="rf-card__title">Convert2Code</div>
      <div className="rf-card__body" style={{ display: "grid", gap: 8 }}>
        <div className="rf-chip">{count} bloque(s) conectado(s)</div>
        <div style={{ opacity:.8, fontSize:12 }}>
          Conecta aquí <b>RosPublisher</b>, <b>CreatePackage</b>, <b>RosRun</b>, etc.
          El código se generará automáticamente.
        </div>
        <pre className="rfp-terminal__code" style={{ marginTop: 6, maxHeight: 140, overflow: "auto" }}>
{preview || "# (aún no hay nada conectado…)"}
        </pre>

        {/* Filename Input */}
        {hasCommand && (
          <div style={{ display: "flex", flexDirection: "column", gap: 4 }}>
            <label style={{ fontSize: "12px", opacity: 0.8, fontWeight: 500 }}>
              {currentFile ? "Save to file:" : "Create file:"}
            </label>
            <input
              type="text"
              value={fileName}
              onChange={(e) => setFileName(e.target.value)}
              placeholder="my_publisher.py"
              className="rf-input"
              style={{
                padding: "6px 8px",
                background: "rgba(255,255,255,0.05)",
                border: "1px solid rgba(125, 249, 255, 0.3)",
                borderRadius: "4px",
                color: "#fff",
                fontSize: "13px",
              }}
              disabled={!!currentFile}
            />
            {currentFile && (
              <div style={{ fontSize: "11px", opacity: 0.6, fontStyle: "italic" }}>
                Will update current file
              </div>
            )}
          </div>
        )}

        {/* Status Message */}
        {saveStatus && (
          <div style={{
            padding: "6px 10px",
            background: saveStatus.includes("✅")
              ? "rgba(0, 255, 0, 0.1)"
              : "rgba(255, 100, 100, 0.1)",
            border: `1px solid ${saveStatus.includes("✅")
              ? "rgba(0, 255, 0, 0.3)"
              : "rgba(255, 100, 100, 0.3)"}`,
            borderRadius: "4px",
            fontSize: "12px",
            fontWeight: 500,
          }}>
            {saveStatus}
          </div>
        )}

        {/* Action Buttons */}
        <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: 8, marginTop: 4 }}>
          {/* Save File Button */}
          <button
            className="btn"
            onClick={handleSaveFile}
            disabled={!hasCommand || !canvasId || saving}
            style={{
              padding: "8px 12px",
              background: hasCommand && canvasId ? "rgba(125, 249, 255, 0.2)" : "rgba(255,255,255,0.1)",
              color: hasCommand && canvasId ? "var(--neon, #7df9ff)" : "rgba(255,255,255,0.4)",
              border: "1px solid rgba(125, 249, 255, 0.3)",
              borderRadius: "4px",
              cursor: hasCommand && canvasId ? "pointer" : "not-allowed",
              fontSize: "13px",
              fontWeight: 600,
              transition: "all 0.2s ease",
              opacity: hasCommand && canvasId ? 1 : 0.5,
            }}
            title={hasCommand && canvasId ? (currentFile ? "Update current file with generated code" : "Save code as Python file") : "Connect blocks and ensure workspace is ready"}
          >
            {saving ? "💾 Saving..." : currentFile ? "💾 Update File" : "💾 Save File"}
          </button>

          {/* Run in Terminal Button */}
          <button
            className="btn"
            onClick={handleRunClick}
            disabled={!hasCommand || !onExecute}
            style={{
              padding: "8px 12px",
              background: hasCommand ? "var(--neon, #7df9ff)" : "rgba(255,255,255,0.1)",
              color: hasCommand ? "#000" : "rgba(255,255,255,0.4)",
              border: "1px solid rgba(255,255,255,0.2)",
              borderRadius: "4px",
              cursor: hasCommand ? "pointer" : "not-allowed",
              fontSize: "13px",
              fontWeight: 600,
              transition: "all 0.2s ease",
              opacity: hasCommand ? 1 : 0.5,
            }}
            title={hasCommand ? "Execute command in terminal" : "Connect blocks to generate a command"}
          >
            ▶ Run
          </button>
        </div>
      </div>

      <Handle type="target" position={Position.Left} id="in" />
      <Handle type="source" position={Position.Right} id="out" />
    </div>
  );
}
