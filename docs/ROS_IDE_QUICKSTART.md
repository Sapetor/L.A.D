# ROS Visual IDE - Quick Start Guide

## What We've Built So Far ✅

### 1. **BlockCanvas Component** ✅
A reusable ReactFlow-based canvas for block programming.

**Location**: `AVEDU/avedu/src/components/ide/BlockCanvas.jsx`

**Features**:
- Drag & drop blocks from palette
- Connect blocks with edges
- Auto-generate code from block graph
- Read-only mode support
- Neon aesthetic styling

**Usage Example**:
```jsx
import { BlockCanvas } from "./components/ide/BlockCanvas";
import { computeUrdfXml } from "./components/blocks/urdf-helpers";

<BlockCanvas
  initialNodes={[]}
  initialEdges={[]}
  onGraphChange={({ nodes, edges }) => {
    console.log("Graph changed", nodes, edges);
  }}
  onCodeGenerated={(code) => {
    console.log("Generated code:", code);
    saveFile(currentFile, code);
  }}
  codeGenerator={computeUrdfXml}
  readOnly={false}
/>
```

### 2. **FileExplorer Component** ✅
A tree view file explorer similar to VSCode.

**Location**: `AVEDU/avedu/src/components/ide/FileExplorer.jsx`

**Features**:
- Hierarchical tree view with expand/collapse
- File icons based on extension
- Right-click context menu
- Create/delete/rename operations
- Inline rename with input field
- Unsaved changes indicator (●)

**File Tree Structure**:
```javascript
const fileTree = [
  {
    name: "src",
    path: "/src",
    type: "directory",
    children: [
      {
        name: "my_package",
        path: "/src/my_package",
        type: "directory",
        children: [
          {
            name: "package.xml",
            path: "/src/my_package/package.xml",
            type: "file",
            unsaved: false,
          },
          {
            name: "my_node.py",
            path: "/src/my_package/my_package/my_node.py",
            type: "file",
            unsaved: true, // Has unsaved changes
          },
        ],
      },
    ],
  },
];
```

**Usage Example**:
```jsx
import { FileExplorer } from "./components/ide/FileExplorer";

<FileExplorer
  files={fileTree}
  currentFile={"/src/my_package/my_node.py"}
  onFileSelect={(path) => {
    console.log("File selected:", path);
    loadFile(path);
  }}
  onFileCreate={(path, type) => {
    console.log("Create:", path, type);
    createFile(path, type);
  }}
  onFileDelete={(path) => {
    console.log("Delete:", path);
    deleteFile(path);
  }}
  onFileRename={(oldPath, newPath) => {
    console.log("Rename:", oldPath, "to", newPath);
    renameFile(oldPath, newPath);
  }}
  loading={false}
/>
```

## What's Next 🚧

### Phase 1: Complete Frontend Components (Continue)

#### 3. Terminal Component
**Goal**: Interactive terminal with command execution

**NPM Dependencies Needed**:
```bash
cd AVEDU/avedu
npm install xterm xterm-addon-fit xterm-addon-web-links
```

**Implementation**:
```jsx
// components/ide/Terminal.jsx
import { Terminal as XTerm } from "xterm";
import { FitAddon } from "xterm-addon-fit";
import "xterm/css/xterm.css";

export function Terminal({ username, canvasId, onCommandExecute }) {
  const terminalRef = useRef(null);
  const xtermRef = useRef(null);

  useEffect(() => {
    const term = new XTerm({
      cursorBlink: true,
      theme: {
        background: "#0b1020",
        foreground: "#e6f1ff",
        cursor: "#7df9ff",
      },
    });

    const fitAddon = new FitAddon();
    term.loadAddon(fitAddon);
    term.open(terminalRef.current);
    fitAddon.fit();

    xtermRef.current = term;

    // Handle input
    let currentLine = "";
    term.onData((data) => {
      if (data === "\r") {
        // Enter key
        term.write("\r\n");
        onCommandExecute?.(currentLine);
        currentLine = "";
        term.write("$ ");
      } else if (data === "\u007F") {
        // Backspace
        if (currentLine.length > 0) {
          currentLine = currentLine.slice(0, -1);
          term.write("\b \b");
        }
      } else {
        currentLine += data;
        term.write(data);
      }
    });

    term.write("$ ");

    return () => term.dispose();
  }, []);

  return <div ref={terminalRef} style={{ height: "100%", width: "100%" }} />;
}
```

#### 4. TabBar Component
**Goal**: Manage open files with tabs

**Implementation**:
```jsx
// components/ide/TabBar.jsx
export function TabBar({ tabs, activeTab, onTabSelect, onTabClose, onTabCreate }) {
  return (
    <div className="tab-bar">
      {tabs.map((tab) => (
        <div
          key={tab.path}
          className={`tab ${activeTab === tab.path ? "tab--active" : ""}`}
          onClick={() => onTabSelect(tab.path)}
        >
          <span className="tab__icon">{getFileIcon(tab.name)}</span>
          <span className="tab__name">{tab.name}</span>
          {tab.unsaved && <span className="tab__badge">●</span>}
          <button
            className="tab__close"
            onClick={(e) => {
              e.stopPropagation();
              onTabClose(tab.path);
            }}
          >
            ✕
          </button>
        </div>
      ))}
      <button className="tab-bar__new" onClick={onTabCreate}>
        +
      </button>
    </div>
  );
}
```

#### 5. ROSCodeEditor (Main Container)
**Goal**: Orchestrate all components

**Layout**:
```
┌─────────────────────────────────────────────────────┐
│                   Header / Title Bar                 │
├───────────────┬─────────────────────────────────────┤
│               │         Tab Bar                      │
│               │  [file1] [file2] [terminal] [+]     │
│               ├─────────────────────────────────────┤
│               │                                      │
│  File         │      BlockCanvas                     │
│  Explorer     │      (ReactFlow)                     │
│               │                                      │
│               │                                      │
│               ├─────────────────────────────────────┤
│               │      Code Preview / Terminal         │
└───────────────┴─────────────────────────────────────┘
```

**State Management**:
```jsx
const [fileTree, setFileTree] = useState([]);
const [openTabs, setOpenTabs] = useState([]);
const [currentTab, setCurrentTab] = useState(null);
const [graphs, setGraphs] = useState({}); // path -> { nodes, edges }
const [unsavedFiles, setUnsavedFiles] = useState(new Set());
```

### Phase 2: Backend Integration

#### 6. Django Workspace API
**Goal**: File CRUD operations

**Django App Structure**:
```bash
cd LAD/lad
python manage.py startapp workspace
```

**Models** (`workspace/models.py`):
```python
from django.db import models
from django.contrib.auth.models import User

class Canvas(models.Model):
    """User workspace for a specific level/unit"""
    user = models.ForeignKey(User, on_delete=models.CASCADE)
    canvas_id = models.CharField(max_length=100)  # e.g., "unit1_level2"
    created_at = models.DateTimeField(auto_now_add=True)
    updated_at = models.DateTimeField(auto_now=True)

    class Meta:
        unique_together = ('user', 'canvas_id')

class CanvasFile(models.Model):
    """Individual file in a canvas"""
    canvas = models.ForeignKey(Canvas, on_delete=models.CASCADE, related_name='files')
    path = models.CharField(max_length=500)  # relative path like "src/pkg/node.py"
    content = models.TextField()
    file_type = models.CharField(max_length=10, choices=[('file', 'File'), ('directory', 'Directory')])
    created_at = models.DateTimeField(auto_now_add=True)
    updated_at = models.DateTimeField(auto_now=True)

    class Meta:
        unique_together = ('canvas', 'path')

class BlockGraph(models.Model):
    """Saved block graph for a file"""
    canvas_file = models.OneToOneField(CanvasFile, on_delete=models.CASCADE)
    nodes = models.JSONField()
    edges = models.JSONField()
    updated_at = models.DateTimeField(auto_now=True)
```

**API Views** (`workspace/views.py`):
```python
from rest_framework import viewsets, status
from rest_framework.decorators import action
from rest_framework.response import Response
from rest_framework.permissions import IsAuthenticated
import subprocess
import json

class WorkspaceViewSet(viewsets.ViewSet):
    permission_classes = [IsAuthenticated]

    def list_files(self, request, username, canvas_id):
        """GET /api/workspace/<username>/<canvas_id>/"""
        canvas = Canvas.objects.get(user__username=username, canvas_id=canvas_id)
        files = canvas.files.all()
        # Build tree structure
        tree = build_file_tree(files)
        return Response({"files": tree})

    def read_file(self, request, username, canvas_id):
        """GET /api/workspace/<username>/<canvas_id>/files/?path=..."""
        path = request.query_params.get('path')
        canvas = Canvas.objects.get(user__username=username, canvas_id=canvas_id)
        file = canvas.files.get(path=path)
        return Response({"path": path, "content": file.content})

    def create_file(self, request, username, canvas_id):
        """POST /api/workspace/<username>/<canvas_id>/files/"""
        path = request.data.get('path')
        content = request.data.get('content', '')

        canvas, _ = Canvas.objects.get_or_create(
            user=request.user,
            canvas_id=canvas_id
        )

        file, created = CanvasFile.objects.update_or_create(
            canvas=canvas,
            path=path,
            defaults={'content': content, 'file_type': 'file'}
        )

        # Write to Docker
        docker_path = f"/workspaces/{username}/{canvas_id}/{path}"
        write_file_to_docker(docker_path, content)

        return Response({"success": True, "path": path},
                       status=status.HTTP_201_CREATED if created else status.HTTP_200_OK)

    @action(detail=False, methods=['post'])
    def execute(self, request, username, canvas_id):
        """POST /api/workspace/<username>/<canvas_id>/execute/"""
        command = request.data.get('command')
        working_dir = f"/workspaces/{username}/{canvas_id}"

        # Execute in Docker
        result = subprocess.run(
            ['docker', 'exec', 'ros', 'bash', '-c',
             f'cd {working_dir} && {command}'],
            capture_output=True,
            text=True,
            timeout=30
        )

        return Response({
            "stdout": result.stdout,
            "stderr": result.stderr,
            "exit_code": result.returncode
        })

def write_file_to_docker(path, content):
    """Write file content to Docker container"""
    # Ensure parent directory exists
    parent_dir = os.path.dirname(path)
    subprocess.run(['docker', 'exec', 'ros', 'mkdir', '-p', parent_dir])

    # Write file
    subprocess.run(
        ['docker', 'exec', '-i', 'ros', 'bash', '-c', f'cat > {path}'],
        input=content.encode(),
        check=True
    )
```

**URLs** (`workspace/urls.py`):
```python
from django.urls import path
from . import views

urlpatterns = [
    path('<str:username>/<str:canvas_id>/', views.WorkspaceViewSet.as_view({
        'get': 'list_files',
    })),
    path('<str:username>/<str:canvas_id>/files/', views.WorkspaceViewSet.as_view({
        'get': 'read_file',
        'post': 'create_file',
        'delete': 'delete_file',
    })),
    path('<str:username>/<str:canvas_id>/execute/', views.WorkspaceViewSet.as_view({
        'post': 'execute',
    })),
]
```

#### 7. Docker Configuration

**Update `docker-compose.yml`**:
```yaml
services:
  ros:
    volumes:
      - user_workspaces:/workspaces
      - ../config/ip_config.json:/config/ip_config.json:ro

volumes:
  user_workspaces:
    driver: local
```

**Restart Docker**:
```bash
cd qcar_docker
docker compose down
docker compose up -d
```

### Phase 3: Frontend-Backend Integration

#### 8. FileAPI Service
**Goal**: Abstract API calls

**Implementation** (`src/services/fileAPI.js`):
```javascript
import { apiFetch } from "../context/AuthContext";

export class FileAPI {
  constructor(username, canvasId) {
    this.username = username;
    this.canvasId = canvasId;
    this.baseUrl = `/api/workspace/${username}/${canvasId}`;
  }

  async listFiles() {
    const response = await apiFetch(this.baseUrl);
    return response.files;
  }

  async readFile(path) {
    const response = await apiFetch(`${this.baseUrl}/files/?path=${encodeURIComponent(path)}`);
    return response.content;
  }

  async createFile(path, content) {
    const response = await apiFetch(`${this.baseUrl}/files/`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ path, content }),
    });
    return response;
  }

  async deleteFile(path) {
    const response = await apiFetch(`${this.baseUrl}/files/?path=${encodeURIComponent(path)}`, {
      method: "DELETE",
    });
    return response;
  }

  async executeCommand(command, workingDir) {
    const response = await apiFetch(`${this.baseUrl}/execute/`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ command, working_dir: workingDir }),
    });
    return response;
  }
}
```

## Testing the IDE

### 1. Test BlockCanvas
Create a test page to verify the canvas works:

```jsx
// src/pages/TestBlockCanvas.jsx
import React, { useState } from "react";
import { ReactFlowProvider } from "@xyflow/react";
import { BlockCanvas } from "../components/ide/BlockCanvas";
import { CategorizedPalette } from "../components/blocks";
import { paletteCategorized } from "../components/blocks";
import { computeUrdfXml } from "../components/blocks/urdf-helpers";

export default function TestBlockCanvas() {
  const [code, setCode] = useState("");

  return (
    <ReactFlowProvider>
      <div style={{ display: "flex", height: "100vh" }}>
        <div style={{ width: "250px" }}>
          <CategorizedPalette
            categories={paletteCategorized}
            defaultCategory="URDF"
          />
        </div>
        <div style={{ flex: 1 }}>
          <BlockCanvas
            initialNodes={[]}
            initialEdges={[]}
            codeGenerator={computeUrdfXml}
            onCodeGenerated={(generatedCode) => setCode(generatedCode.xml)}
          />
        </div>
        <div style={{ width: "300px", padding: "1rem", background: "#0b1020" }}>
          <h3>Generated Code:</h3>
          <pre style={{ fontSize: "10px", overflow: "auto" }}>{code}</pre>
        </div>
      </div>
    </ReactFlowProvider>
  );
}
```

### 2. Test FileExplorer
```jsx
// src/pages/TestFileExplorer.jsx
import React, { useState } from "react";
import { FileExplorer } from "../components/ide/FileExplorer";

export default function TestFileExplorer() {
  const [files] = useState([
    {
      name: "src",
      path: "/src",
      type: "directory",
      children: [
        {
          name: "my_package",
          path: "/src/my_package",
          type: "directory",
          children: [
            { name: "package.xml", path: "/src/my_package/package.xml", type: "file" },
            { name: "setup.py", path: "/src/my_package/setup.py", type: "file" },
          ],
        },
      ],
    },
  ]);

  return (
    <div style={{ height: "100vh", padding: "1rem", background: "#0b1020" }}>
      <div style={{ height: "100%", maxWidth: "400px" }}>
        <FileExplorer
          files={files}
          onFileSelect={(path) => console.log("Selected:", path)}
          onFileCreate={(path, type) => console.log("Create:", path, type)}
          onFileDelete={(path) => console.log("Delete:", path)}
          onFileRename={(old, newPath) => console.log("Rename:", old, "to", newPath)}
        />
      </div>
    </div>
  );
}
```

## Next Steps Summary

✅ **Completed**:
1. BlockCanvas component (reusable ReactFlow canvas)
2. FileExplorer component (tree view with CRUD operations)

🚧 **In Progress**:
3. Terminal component (needs xterm.js)
4. TabBar component
5. ROSCodeEditor main container

📋 **To Do**:
6. Django workspace API
7. Docker volume configuration
8. FileAPI service
9. Integration testing
10. Documentation

## Questions?

**Q: How do I test what we've built so far?**
A: Create test pages (see examples above) in `src/pages/` and add routes in `App.js`.

**Q: Can I use BlockCanvas with different block types?**
A: Yes! Just pass a different `codeGenerator` function. For example:
- `computeUrdfXml` for URDF blocks
- `generateLaunchFile` for launch file blocks
- Custom generator for ROS 2 packages

**Q: How do I customize the FileExplorer styling?**
A: Edit `components/ide/FileExplorer.scss`. All CSS variables are defined at the top.

**Q: What's the workflow for saving a file?**
A:
1. User creates blocks in BlockCanvas
2. `onCodeGenerated` callback fires with generated code
3. Call `FileAPI.createFile(path, code)`
4. Backend writes to Docker
5. Refresh FileExplorer to show new file

**Q: Do I need to implement everything?**
A: No! You can implement just what you need. The architecture is modular:
- Use BlockCanvas alone for block programming
- Use FileExplorer alone for file management
- Combine them in ROSCodeEditor for full IDE

Ready to continue? Let me know which component to build next! 🚀
