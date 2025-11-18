// src/levels/rosbasic/slides/WorkspaceAndBuildSlide.jsx
import React from "react";
import "../../styles/pages/_folder.scss";

export const meta = {
  id: "ros2-structure-and-colcon",
  title: "ROS 2 Workspace Structure + colcon build",
  order: 5,
  objectiveCode: "ros-slide-workspace-and-colcon",
};

const ITEMS = [
  { id: "ws", label: "ros2_ws/  (workspace)", kind: "dir", desc: `Your working directory. ROS 2 packages will live here inside "src/".
You can have multiple workspaces and "chain" them with overlays.` },
  { id: "src", label: "└─ src/", kind: "dir", desc: `Source code. Inside "src/" you place one or more ROS 2 "packages".
Each package is a unit that is compiled/installed separately.` },
  { id: "pkg", label: "   └─ my_package/", kind: "dir", desc: `A ROS 2 package. Must have at least "package.xml" and build files:
- Python: setup.py, setup.cfg (or pyproject.toml)
- C++: CMakeLists.txt
Plus your source code: scripts/ or src/, launch/, msg/srv/action if you define interfaces.` },
  { id: "pkgxml", label: "      ├─ package.xml", kind: "file", desc: `Package manifest. Declares name, version, maintainers and dependencies.
colcon and ament use it to resolve the dependency graph.` },
  { id: "setup", label: "      ├─ setup.py / setup.cfg (Python)", kind: "file", desc: `Metadata and installation rules for Python packages.
Allows installing scripts as entry points and copying resources during build/installation.` },
  { id: "cmake", label: "      ├─ CMakeLists.txt (C++)", kind: "file", desc: `CMake script for C++ packages. Declares executables, include dirs, dependencies
(and installation targets) using ament_cmake.` },
  { id: "code", label: "      └─ src/  /  scripts/  /  launch/", kind: "dir", desc: `Your code:
- src/: C++ sources or Python modules
- scripts/: executable Python scripts (with shebang and permissions)
- launch/: *.launch.py files to run nodes/compositions.` },
  { id: "build", label: "build/  (auto-generated)", kind: "dir", desc: `Intermediate build output (objects, CMake cache, etc.).
colcon recreates it; you can delete it for a "clean" build.` },
  { id: "install", label: "install/  (auto-generated)", kind: "dir", desc: `Installable result of the workspace. When you "source install/setup.bash"
your environment will see the executables and resources from the workspace.` },
  { id: "log", label: "log/  (auto-generated)", kind: "dir", desc: `Build/run logs. Useful for debugging colcon or test errors.` },
];

const DEFAULT_CMD =
  "colcon build --symlink-install\n\n# after building:\nsource install/setup.bash";

function SectionCard({ title, children, right }) {
  return (
    <div className={`section-card ${right ? "section-card--right" : ""}`}>
      <div className="section-card__title">{title}</div>
      {children}
    </div>
  );
}

export default function WorkspaceAndBuildSlide({ onObjectiveHit }) {
  const [selected, setSelected] = React.useState("ws");

  const current = React.useMemo(
    () => ITEMS.find((i) => i.id === selected) || ITEMS[0],
    [selected]
  );

  const markDone = () => onObjectiveHit?.(meta.objectiveCode);

  return (
    <div className="slide-wrap" key={meta.id}>
      <h2>{meta.title}</h2>

      {/* Top section: tree + detail */}
      <SectionCard title="Basic workspace structure">
        <div className="ws-grid">
          {/* Simple, clickable tree */}
          <div className="tree">
            <pre className="tree__pre">
              {ITEMS.slice(0, 7).map((it) => (
                <div
                  key={it.id}
                  role="button"
                  onClick={() => setSelected(it.id)}
                  title="Click to see details"
                  className={`tree__item ${selected === it.id ? "is-selected" : ""}`}
                >
                  <code>{it.label}</code>
                </div>
              ))}
              <div className="tree__spacer" />
              {ITEMS.slice(7).map((it) => (
                <div
                  key={it.id}
                  role="button"
                  onClick={() => setSelected(it.id)}
                  title="Click to see details"
                  className={`tree__item ${selected === it.id ? "is-selected" : ""}`}
                >
                  <code>{it.label}</code>
                </div>
              ))}
            </pre>
          </div>

          {/* Item detail */}
          <div className="detail">
            <div className="detail__label">
              {current?.label?.replace(/^\s+/, "")}
            </div>
            <div className="detail__desc">
              {current?.desc}
            </div>
          </div>
        </div>
      </SectionCard>

      {/* Bottom section: colcon build */}
      <SectionCard title="What does colcon build do?">
        <div className="colcon-grid">
          <ul className="colcon-list">
            <li>
              <b>Discovers packages</b> in <code>src/</code> (looks for <code>package.xml</code>) and <b>resolves dependencies</b>.
            </li>
            <li>
              Executes the corresponding <b>build system</b>:
              <ul className="colcon-sublist">
                <li><code>ament_python</code> → installs Python modules/scripts.</li>
                <li><code>ament_cmake</code> → invokes CMake/Make for C++, generates and installs executables and resources.</li>
              </ul>
            </li>
            <li>
              Generates <code>build/</code> (intermediate artifacts), <code>install/</code> (installable result) and <code>log/</code> (logs).
            </li>
            <li>
              After building, you must <b>activate the overlay</b>:
              <pre className="code-inline">source install/setup.bash</pre>
              so your shell sees the executables/packages from the workspace.
            </li>
          </ul>

          <div className="cmd-card">
            <div className="cmd-card__title">Typical command</div>
            <pre className="cmd-card__code">{DEFAULT_CMD}</pre>
            <div className="cmd-card__actions">
              <button
                className="btn"
                onClick={() => {
                  navigator.clipboard?.writeText(DEFAULT_CMD).catch(() => {});
                }}
              >
                Copy
              </button>
              <button className="btn" onClick={markDone}>
                Mark complete
              </button>
            </div>
            <div className="cmd-card__hint">
              <b>--symlink-install</b>: for Python development, installs symbolic links
              so code changes are reflected without reinstalling.
            </div>
          </div>
        </div>
      </SectionCard>
    </div>
  );
}
