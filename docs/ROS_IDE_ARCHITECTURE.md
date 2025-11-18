# ROS Visual IDE Architecture

## Overview

A browser-based integrated development environment for ROS 2 with:
- **Block-based programming** (visual node graph using ReactFlow)
- **File system management** (create, edit, delete ROS packages/files)
- **Integrated terminal** (execute ROS commands directly)
- **Docker backend** (persistent workspace storage)
- **User isolation** (each user/canvas has separate workspace)

## System Architecture

```
┌─────────────────────────────────────────────────────┐
│                  React Frontend                      │
│  ┌─────────────────────────────────────────────┐   │
│  │         ROSCodeEditor (Main Container)       │   │
│  │  ┌──────────────┐  ┌────────────────────┐  │   │
│  │  │ FileExplorer │  │   Tab Bar           │  │   │
│  │  │  - Tree View │  │  [file1] [file2]..  │  │   │
│  │  │  - Create    │  │                      │  │   │
│  │  │  - Delete    │  │  ┌────────────────┐ │  │   │
│  │  │  - Rename    │  │  │  BlockCanvas   │ │  │   │
│  │  └──────────────┘  │  │  (ReactFlow)   │ │  │   │
│  │                     │  │                │ │  │   │
│  │  ┌──────────────┐  │  └────────────────┘ │  │   │
│  │  │  Terminal    │  │                      │  │   │
│  │  │  (xterm.js)  │  │  ┌────────────────┐ │  │   │
│  │  └──────────────┘  │  │  Code Preview  │ │  │   │
│  │                     │  └────────────────┘ │  │   │
│  └─────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────┘
                        ↕ HTTP/WebSocket
┌─────────────────────────────────────────────────────┐
│              Django Backend (REST API)               │
│  ┌─────────────────────────────────────────────┐   │
│  │  Workspace API         Terminal API          │   │
│  │  - List files          - Execute command     │   │
│  │  - Read file           - Stream output       │   │
│  │  - Write file                                 │   │
│  │  - Delete file                                │   │
│  └─────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────┘
                        ↕ Docker exec
┌─────────────────────────────────────────────────────┐
│                 Docker Container                     │
│  ┌─────────────────────────────────────────────┐   │
│  │  /workspaces/                                 │   │
│  │    └─ <username>/                             │   │
│  │         └─ <canvas_id>/                       │   │
│  │              ├─ src/                          │   │
│  │              │   └─ my_package/               │   │
│  │              │       ├─ package.xml           │   │
│  │              │       ├─ setup.py              │   │
│  │              │       └─ my_package/           │   │
│  │              │           └─ my_node.py        │   │
│  │              ├─ build/                        │   │
│  │              ├─ install/                      │   │
│  │              └─ log/                          │   │
│  └─────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────┘
```

## Component Details

### Frontend Components

#### 1. **BlockCanvas** (Reusable)
```jsx
// components/ide/BlockCanvas.jsx
<BlockCanvas
  nodes={nodes}
  edges={edges}
  onNodesChange={handleNodesChange}
  onEdgesChange={handleEdgesChange}
  onConnect={handleConnect}
  palette={paletteUrdf}
  onCodeGenerated={(code) => saveFile(currentFile, code)}
/>
```

**Purpose**: Reusable ReactFlow canvas for block-based programming

**Features**:
- Drag & drop blocks from palette
- Connect blocks with edges
- Auto-generate code from block graph
- Support multiple block types (URDF, ROS packages, launch files)

#### 2. **FileExplorer**
```jsx
// components/ide/FileExplorer.jsx
<FileExplorer
  files={fileTree}
  currentFile={currentFile}
  onFileSelect={handleFileSelect}
  onFileCreate={handleFileCreate}
  onFileDelete={handleFileDelete}
  onFileRename={handleFileRename}
/>
```

**Purpose**: Tree view of workspace files

**Features**:
- Expandable/collapsible folders
- File icons based on type
- Context menu (create, delete, rename)
- Search/filter files

**UI Style**: Similar to WorkspaceAndBuild slide tree

#### 3. **Terminal**
```jsx
// components/ide/Terminal.jsx
<Terminal
  username={username}
  canvasId={canvasId}
  workingDirectory={`/workspaces/${username}/${canvasId}`}
  onCommandExecute={handleCommandExecute}
/>
```

**Purpose**: Interactive terminal for ROS commands

**Features**:
- Full terminal emulation (xterm.js)
- Command history
- Working directory aware
- Execute commands in Docker
- Stream output in real-time

**Common Commands**:
```bash
colcon build
source install/setup.bash
ros2 run my_package my_node
ros2 topic list
```

#### 4. **TabBar**
```jsx
// components/ide/TabBar.jsx
<TabBar
  tabs={openFiles}
  activeTab={currentFile}
  onTabSelect={handleTabSelect}
  onTabClose={handleTabClose}
  onTabCreate={handleNewFile}
/>
```

**Purpose**: Manage multiple open files

**Features**:
- Tab per open file
- Close tabs individually
- Active tab highlighting
- Unsaved changes indicator (*)

#### 5. **ROSCodeEditor** (Main Container)
```jsx
// components/ide/ROSCodeEditor.jsx
<ROSCodeEditor
  username={currentUser}
  canvasId={levelId}
  onObjectiveHit={onObjectiveHit}
/>
```

**Purpose**: Main IDE container orchestrating all components

**State Management**:
- File tree
- Open tabs
- Current file
- Block graph per file
- Terminal state

### Backend API

#### Django Apps Structure
```
LAD/lad/apps/
├── workspace/          # New app for IDE functionality
│   ├── models.py       # Canvas, File models
│   ├── views.py        # API endpoints
│   ├── serializers.py  # DRF serializers
│   └── docker_exec.py  # Docker command execution
└── learning/           # Existing app
```

#### API Endpoints

##### **Workspace Management**

**1. List Files**
```http
GET /api/workspace/<username>/<canvas_id>/
Response: {
  "files": [
    {
      "path": "src/my_package/package.xml",
      "type": "file",
      "size": 1024,
      "modified": "2025-01-17T12:00:00Z"
    },
    {
      "path": "src/my_package",
      "type": "directory",
      "children": [...]
    }
  ]
}
```

**2. Read File**
```http
GET /api/workspace/<username>/<canvas_id>/files/?path=src/my_package/setup.py
Response: {
  "path": "src/my_package/setup.py",
  "content": "from setuptools import setup...",
  "encoding": "utf-8"
}
```

**3. Create/Update File**
```http
POST /api/workspace/<username>/<canvas_id>/files/
Body: {
  "path": "src/my_package/my_package/my_node.py",
  "content": "import rclpy...",
  "create_dirs": true
}
Response: {
  "success": true,
  "path": "src/my_package/my_package/my_node.py"
}
```

**4. Delete File**
```http
DELETE /api/workspace/<username>/<canvas_id>/files/?path=src/old_package
Response: {
  "success": true,
  "deleted": "src/old_package"
}
```

**5. Execute Command**
```http
POST /api/workspace/<username>/<canvas_id>/execute/
Body: {
  "command": "colcon build --packages-select my_package",
  "working_dir": "/workspaces/nicomedes/01"
}
Response: {
  "stdout": "Starting >>> my_package\nFinished <<< my_package [2.5s]",
  "stderr": "",
  "exit_code": 0
}
```

##### **Block Graph Storage** (Optional)
```http
GET /api/canvas/<username>/<canvas_id>/graph/
POST /api/canvas/<username>/<canvas_id>/graph/
```
Store the block graph JSON so it persists between sessions.

### Docker Configuration

#### Volume Mounts

**docker-compose.yml**
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

#### Workspace Structure
```
/workspaces/
├── nicomedes/          # Username
│   ├── 01/             # Canvas ID (level/unit)
│   │   ├── src/
│   │   │   └── my_robot_pkg/
│   │   │       ├── package.xml
│   │   │       ├── setup.py
│   │   │       ├── resource/
│   │   │       └── my_robot_pkg/
│   │   │           └── my_node.py
│   │   ├── build/
│   │   ├── install/
│   │   └── log/
│   └── 02/             # Another canvas
│       └── ...
└── other_user/
    └── ...
```

## Data Flow Examples

### Example 1: Create URDF with Blocks

1. **User**: Drags "Link" and "Joint" blocks onto canvas
2. **Frontend**: Updates ReactFlow nodes/edges state
3. **Frontend**: Generates URDF XML from block graph
4. **User**: Clicks "Save" button
5. **Frontend**: `POST /api/workspace/nicomedes/01/files/`
   ```json
   {
     "path": "src/my_robot/urdf/robot.urdf",
     "content": "<robot name=\"my_robot\">...</robot>"
   }
   ```
6. **Backend**: Creates file in Docker at `/workspaces/nicomedes/01/src/my_robot/urdf/robot.urdf`
7. **Backend**: Returns success
8. **Frontend**: Updates file tree to show new file

### Example 2: Create ROS Package

1. **User**: Clicks "New Package" in file explorer
2. **Frontend**: Shows dialog for package name, dependencies
3. **User**: Enters "my_package", deps ["rclpy", "std_msgs"]
4. **User**: Clicks "Create"
5. **Frontend**: `POST /api/workspace/nicomedes/01/execute/`
   ```json
   {
     "command": "ros2 pkg create my_package --build-type ament_python --dependencies rclpy std_msgs",
     "working_dir": "/workspaces/nicomedes/01/src"
   }
   ```
6. **Backend**: Executes command in Docker
7. **Docker**: Creates package structure
8. **Backend**: Returns stdout/stderr
9. **Frontend**: Refreshes file tree
10. **Frontend**: Opens new package in file explorer

### Example 3: Run Node from Terminal

1. **User**: Types in terminal: `colcon build`
2. **Frontend**: `POST /api/workspace/nicomedes/01/execute/`
3. **Backend**: Executes `docker exec ros bash -c "cd /workspaces/nicomedes/01 && colcon build"`
4. **Docker**: Builds packages
5. **Backend**: Streams output back
6. **Frontend**: Displays output in terminal
7. **User**: Types: `source install/setup.bash && ros2 run my_package my_node`
8. **Process repeats**: Execute → Stream → Display

## Code Generation

### Block Types and Their Code Output

#### 1. **URDF Blocks → .urdf File**
```jsx
// blocks/urdf-helpers.js
export function computeUrdfXml(nodes, edges) {
  const robot = findNodeByType(nodes, 'urdfRobot');
  const links = getConnectedNodes(robot, 'links', nodes, edges);
  const joints = getConnectedNodes(robot, 'joints', nodes, edges);

  return `<?xml version="1.0"?>
<robot name="${robot.data.name}">
  ${links.map(link => generateLinkXml(link)).join('\n')}
  ${joints.map(joint => generateJointXml(joint)).join('\n')}
</robot>`;
}
```

#### 2. **Package Blocks → Python Files**
```jsx
// blocks/package-helpers.js
export function generateNodeCode(nodes, edges) {
  const node = findNodeByType(nodes, 'rosNode');
  const publishers = getConnectedNodes(node, 'publishers', nodes, edges);
  const subscribers = getConnectedNodes(node, 'subscribers', nodes, edges);

  return `import rclpy
from rclpy.node import Node
${generateImports(publishers, subscribers)}

class ${node.data.className}(Node):
    def __init__(self):
        super().__init__('${node.data.nodeName}')
        ${generatePublishers(publishers)}
        ${generateSubscribers(subscribers)}

def main(args=None):
    rclpy.init(args=args)
    node = ${node.data.className}()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
`;
}
```

#### 3. **Launch Blocks → .launch.py File**
```jsx
// blocks/launch-helpers.js
export function generateLaunchFile(nodes, edges) {
  const launch = findNodeByType(nodes, 'launchFile');
  const nodeBlocks = getConnectedNodes(launch, 'nodes', nodes, edges);

  return `from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        ${nodeBlocks.map(n => `
        Node(
            package='${n.data.package}',
            executable='${n.data.executable}',
            name='${n.data.name}'
        ),`).join('')}
    ])
`;
}
```

## Security Considerations

1. **User Isolation**: Each user's workspace is isolated
2. **Path Traversal Protection**: Validate all file paths
3. **Command Injection**: Sanitize terminal commands
4. **Resource Limits**: Limit workspace size per user
5. **Authentication**: Require JWT token for all API calls

## Implementation Phases

### Phase 1: Basic File Management (Week 1)
- [ ] Create Django workspace app
- [ ] Implement file CRUD endpoints
- [ ] Configure Docker volume
- [ ] Build FileExplorer component
- [ ] Build TabBar component

### Phase 2: Block Canvas Integration (Week 2)
- [ ] Extract BlockCanvas component
- [ ] Implement code generation helpers
- [ ] Connect canvas to file saving
- [ ] Test URDF generation workflow

### Phase 3: Terminal Integration (Week 3)
- [ ] Build Terminal component with xterm.js
- [ ] Implement command execution endpoint
- [ ] Stream output from Docker
- [ ] Test package creation workflow

### Phase 4: Integration & Polish (Week 4)
- [ ] Build ROSCodeEditor main container
- [ ] Connect all components
- [ ] Add error handling
- [ ] Add loading states
- [ ] Write tests
- [ ] Documentation

## Testing Strategy

### Unit Tests
- Code generation functions
- File path validation
- Command sanitization

### Integration Tests
- File CRUD operations
- Command execution
- Block graph → code → file workflow

### E2E Tests
- Create package via UI → verify in Docker
- Create URDF with blocks → save → load in RViz
- Run node from terminal → verify ROS topic exists

## Future Enhancements

1. **Collaborative Editing**: Multiple users on same canvas
2. **Git Integration**: Commit/push from IDE
3. **Debugging**: Attach debugger to running nodes
4. **Code Templates**: Pre-built block configurations
5. **Export/Import**: Share block graphs between users
6. **Real-time Compilation**: Live feedback as blocks change
7. **ROS Graph Visualization**: See nodes/topics in real-time
