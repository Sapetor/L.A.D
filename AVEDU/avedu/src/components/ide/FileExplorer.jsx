// components/ide/FileExplorer.jsx
import React, { useState, useCallback } from "react";
import PropTypes from "prop-types";
import "./FileExplorer.scss";

/**
 * File tree explorer with create/delete/rename operations
 *
 * @param {Object} props
 * @param {Array} props.files - File tree structure
 * @param {string} props.currentFile - Currently selected file path
 * @param {Function} props.onFileSelect - Callback when file is selected
 * @param {Function} props.onFileCreate - Callback to create new file
 * @param {Function} props.onFileDelete - Callback to delete file
 * @param {Function} props.onFileRename - Callback to rename file
 * @param {boolean} props.loading - Show loading state
 */
export function FileExplorer({
  files = [],
  currentFile = null,
  onFileSelect,
  onFileCreate,
  onFileDelete,
  onFileRename,
  loading = false,
}) {
  const [expanded, setExpanded] = useState(new Set(["/"]));
  const [contextMenu, setContextMenu] = useState(null);
  const [renaming, setRenaming] = useState(null);
  const [newName, setNewName] = useState("");

  // Toggle folder expand/collapse
  const toggleExpand = useCallback((path) => {
    setExpanded((prev) => {
      const next = new Set(prev);
      if (next.has(path)) {
        next.delete(path);
      } else {
        next.add(path);
      }
      return next;
    });
  }, []);

  // Handle file/folder click
  const handleClick = useCallback(
    (item, evt) => {
      evt.stopPropagation();

      if (item.type === "directory") {
        toggleExpand(item.path);
      } else if (item.type === "file") {
        onFileSelect?.(item.path);
      }
    },
    [toggleExpand, onFileSelect]
  );

  // Handle right-click context menu
  const handleContextMenu = useCallback((item, evt) => {
    evt.preventDefault();
    evt.stopPropagation();

    setContextMenu({
      item,
      x: evt.clientX,
      y: evt.clientY,
    });
  }, []);

  // Close context menu
  const closeContextMenu = useCallback(() => {
    setContextMenu(null);
  }, []);

  // Handle context menu actions
  const handleNewFile = useCallback(() => {
    const parentPath = contextMenu.item.type === "directory"
      ? contextMenu.item.path
      : contextMenu.item.path.split("/").slice(0, -1).join("/");

    const fileName = window.prompt("Enter file name:");
    if (fileName) {
      onFileCreate?.(`${parentPath}/${fileName}`, "file");
    }
    closeContextMenu();
  }, [contextMenu, onFileCreate, closeContextMenu]);

  const handleNewFolder = useCallback(() => {
    const parentPath = contextMenu.item.type === "directory"
      ? contextMenu.item.path
      : contextMenu.item.path.split("/").slice(0, -1).join("/");

    const folderName = window.prompt("Enter folder name:");
    if (folderName) {
      onFileCreate?.(`${parentPath}/${folderName}`, "directory");
    }
    closeContextMenu();
  }, [contextMenu, onFileCreate, closeContextMenu]);

  const handleDelete = useCallback(() => {
    if (window.confirm(`Delete ${contextMenu.item.path}?`)) {
      onFileDelete?.(contextMenu.item.path);
    }
    closeContextMenu();
  }, [contextMenu, onFileDelete, closeContextMenu]);

  const handleRename = useCallback(() => {
    setRenaming(contextMenu.item.path);
    setNewName(contextMenu.item.name);
    closeContextMenu();
  }, [contextMenu, closeContextMenu]);

  const confirmRename = useCallback(
    (oldPath) => {
      if (newName && newName !== oldPath.split("/").pop()) {
        const newPath = oldPath.split("/").slice(0, -1).concat(newName).join("/");
        onFileRename?.(oldPath, newPath);
      }
      setRenaming(null);
      setNewName("");
    },
    [newName, onFileRename]
  );

  const cancelRename = useCallback(() => {
    setRenaming(null);
    setNewName("");
  }, []);

  // Get file icon based on extension
  const getFileIcon = useCallback((item) => {
    if (item.type === "directory") {
      return expanded.has(item.path) ? "📂" : "📁";
    }

    const ext = item.name.split(".").pop().toLowerCase();
    const iconMap = {
      py: "🐍",
      cpp: "⚙️",
      h: "⚙️",
      xml: "📄",
      yaml: "📋",
      yml: "📋",
      urdf: "🤖",
      xacro: "🤖",
      launch: "🚀",
      md: "📝",
      txt: "📝",
    };

    return iconMap[ext] || "📄";
  }, [expanded]);

  // Render tree recursively
  const renderTree = useCallback(
    (items, level = 0) => {
      return items.map((item) => {
        const isExpanded = expanded.has(item.path);
        const isSelected = currentFile === item.path;
        const isRenaming = renaming === item.path;

        return (
          <div key={item.path} className="file-tree-item-container">
            <div
              className={`file-tree-item ${
                isSelected ? "file-tree-item--selected" : ""
              }`}
              style={{ paddingLeft: `${level * 16 + 8}px` }}
              onClick={(e) => handleClick(item, e)}
              onContextMenu={(e) => handleContextMenu(item, e)}
            >
              <span className="file-tree-item__icon">
                {getFileIcon(item)}
              </span>

              {isRenaming ? (
                <input
                  className="file-tree-item__input"
                  type="text"
                  value={newName}
                  onChange={(e) => setNewName(e.target.value)}
                  onBlur={() => confirmRename(item.path)}
                  onKeyDown={(e) => {
                    if (e.key === "Enter") confirmRename(item.path);
                    if (e.key === "Escape") cancelRename();
                  }}
                  autoFocus
                  onClick={(e) => e.stopPropagation()}
                />
              ) : (
                <span className="file-tree-item__name">{item.name}</span>
              )}

              {item.unsaved && (
                <span className="file-tree-item__badge">●</span>
              )}
            </div>

            {item.type === "directory" &&
              isExpanded &&
              item.children &&
              item.children.length > 0 && (
                <div className="file-tree-children">
                  {renderTree(item.children, level + 1)}
                </div>
              )}
          </div>
        );
      });
    },
    [
      expanded,
      currentFile,
      renaming,
      newName,
      handleClick,
      handleContextMenu,
      getFileIcon,
      confirmRename,
      cancelRename,
    ]
  );

  // Click outside to close context menu
  React.useEffect(() => {
    if (contextMenu) {
      document.addEventListener("click", closeContextMenu);
      return () => document.removeEventListener("click", closeContextMenu);
    }
  }, [contextMenu, closeContextMenu]);

  return (
    <div className="file-explorer">
      <div className="file-explorer__header">
        <span className="file-explorer__title">Files</span>
        <button
          className="file-explorer__action"
          onClick={() => onFileCreate?.("newfile.py", "file")}
          title="New File"
        >
          +
        </button>
      </div>

      <div className="file-explorer__tree">
        {loading ? (
          <div className="file-explorer__loading">Loading...</div>
        ) : files.length === 0 ? (
          <div className="file-explorer__empty">
            No files. Right-click to create.
          </div>
        ) : (
          renderTree(files)
        )}
      </div>

      {/* Context Menu */}
      {contextMenu && (
        <div
          className="file-explorer__context-menu"
          style={{
            position: "fixed",
            left: `${contextMenu.x}px`,
            top: `${contextMenu.y}px`,
          }}
        >
          <button onClick={handleNewFile}>New File</button>
          <button onClick={handleNewFolder}>New Folder</button>
          <button onClick={handleRename}>Rename</button>
          <button onClick={handleDelete} className="danger">
            Delete
          </button>
        </div>
      )}
    </div>
  );
}

FileExplorer.propTypes = {
  files: PropTypes.array,
  currentFile: PropTypes.string,
  onFileSelect: PropTypes.func,
  onFileCreate: PropTypes.func,
  onFileDelete: PropTypes.func,
  onFileRename: PropTypes.func,
  loading: PropTypes.bool,
};

export default FileExplorer;
