/**
 * File API Service
 * Handles all workspace file operations and command execution
 */

import { API_BASE } from "../config";

/**
 * Get authentication headers with JWT token
 */
function getAuthHeaders() {
  const token = localStorage.getItem("token");
  return {
    "Content-Type": "application/json",
    ...(token && { Authorization: `Bearer ${token}` }),
  };
}

/**
 * Handle API response errors
 */
async function handleResponse(response) {
  if (!response.ok) {
    const error = await response.json().catch(() => ({ detail: response.statusText }));
    throw new Error(error.detail || error.message || "API request failed");
  }
  return response.json();
}

// =============================================================================
// Canvas/Workspace Operations
// =============================================================================

/**
 * List all canvases for the current user
 */
export async function listCanvases() {
  const response = await fetch(`${API_BASE}/workspace/canvases/`, {
    headers: getAuthHeaders(),
  });
  return handleResponse(response);
}

/**
 * Get a specific canvas by ID
 */
export async function getCanvas(canvasId) {
  const response = await fetch(`${API_BASE}/workspace/canvases/${canvasId}/`, {
    headers: getAuthHeaders(),
  });
  return handleResponse(response);
}

/**
 * Create a new canvas
 */
export async function createCanvas(data) {
  const response = await fetch(`${API_BASE}/workspace/canvases/`, {
    method: "POST",
    headers: getAuthHeaders(),
    body: JSON.stringify(data),
  });
  return handleResponse(response);
}

/**
 * Update a canvas
 */
export async function updateCanvas(canvasId, data) {
  const response = await fetch(`${API_BASE}/workspace/canvases/${canvasId}/`, {
    method: "PATCH",
    headers: getAuthHeaders(),
    body: JSON.stringify(data),
  });
  return handleResponse(response);
}

/**
 * Delete a canvas
 */
export async function deleteCanvas(canvasId) {
  const response = await fetch(`${API_BASE}/workspace/canvases/${canvasId}/`, {
    method: "DELETE",
    headers: getAuthHeaders(),
  });
  if (!response.ok) {
    throw new Error("Failed to delete canvas");
  }
}

/**
 * Get file tree for a canvas
 */
export async function getFileTree(canvasId) {
  const response = await fetch(`${API_BASE}/workspace/canvases/${canvasId}/file_tree/`, {
    headers: getAuthHeaders(),
  });
  return handleResponse(response);
}

// =============================================================================
// File Operations
// =============================================================================

/**
 * List all files in a canvas
 */
export async function listFiles(canvasId) {
  const response = await fetch(`${API_BASE}/workspace/canvases/${canvasId}/files/`, {
    headers: getAuthHeaders(),
  });
  return handleResponse(response);
}

/**
 * Get a specific file
 */
export async function getFile(canvasId, fileId) {
  const response = await fetch(`${API_BASE}/workspace/canvases/${canvasId}/files/${fileId}/`, {
    headers: getAuthHeaders(),
  });
  return handleResponse(response);
}

/**
 * Create a new file or directory
 */
export async function createFile(canvasId, data) {
  const response = await fetch(`${API_BASE}/workspace/canvases/${canvasId}/files/`, {
    method: "POST",
    headers: getAuthHeaders(),
    body: JSON.stringify(data),
  });
  return handleResponse(response);
}

/**
 * Update a file's content
 */
export async function updateFile(canvasId, fileId, data) {
  const response = await fetch(`${API_BASE}/workspace/canvases/${canvasId}/files/${fileId}/`, {
    method: "PATCH",
    headers: getAuthHeaders(),
    body: JSON.stringify(data),
  });
  return handleResponse(response);
}

/**
 * Delete a file or directory
 */
export async function deleteFile(canvasId, fileId) {
  const response = await fetch(`${API_BASE}/workspace/canvases/${canvasId}/files/${fileId}/`, {
    method: "DELETE",
    headers: getAuthHeaders(),
  });
  if (!response.ok) {
    throw new Error("Failed to delete file");
  }
}

// =============================================================================
// Command Execution
// =============================================================================

/**
 * Execute a command in the Docker container
 */
export async function executeCommand(canvasId, command, workingDirectory = null) {
  const response = await fetch(`${API_BASE}/workspace/canvases/${canvasId}/execute/`, {
    method: "POST",
    headers: getAuthHeaders(),
    body: JSON.stringify({
      command,
      working_directory: workingDirectory,
    }),
  });
  return handleResponse(response);
}

// =============================================================================
// Helper Functions
// =============================================================================

/**
 * Save generated code to a file
 */
export async function saveGeneratedCode(canvasId, filePath, code) {
  // Check if file already exists
  const files = await listFiles(canvasId);
  const existingFile = files.find((f) => f.path === filePath);

  if (existingFile) {
    // Update existing file
    return updateFile(canvasId, existingFile.id, { content: code });
  } else {
    // Create new file
    return createFile(canvasId, {
      path: filePath,
      file_type: "file",
      content: code,
    });
  }
}

export default {
  // Canvas operations
  listCanvases,
  getCanvas,
  createCanvas,
  updateCanvas,
  deleteCanvas,
  getFileTree,
  
  // File operations
  listFiles,
  getFile,
  createFile,
  updateFile,
  deleteFile,
  
  // Command execution
  executeCommand,
  
  // Helpers
  saveGeneratedCode,
};
