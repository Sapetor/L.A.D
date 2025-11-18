// components/blocks/index.js
// Node registration + palettes + default data

import CreatePackageNode from "./CreatePackageNode";
import RosRunNode from "./RosRunNode";
import StringNode from "./StringNode";
import ListItemsNode from "./ListItemsNode";
import ConvertToCodeNode from "./ConvertToCodeNode";

// URDF blocks (original)
import UrdfLinkNode from "./UrdfLinkNode";
import UrdfJointNode from "./UrdfJointNode";
import UrdfRobotNode from "./UrdfRobotNode";
import UrdfPreviewNode from "./UrdfPreviewNode";
import UrdfViewerNode from "./UrdfViewerNode";
import UrdfControlNode from "./UrdfControlNode";

// URDF blocks V2 (modular/improved)
import UrdfLinkNodeV2 from "./UrdfLinkNodeV2";
import UrdfInertialNode from "./UrdfInertialNode";
import UrdfVisualNode from "./UrdfVisualNode";
import UrdfCollisionNode from "./UrdfCollisionNode";
import UrdfAssemblyNode from "./UrdfAssemblyNode";
import UrdfXmlPreviewNode from "./UrdfXmlPreviewNode";

// New utility blocks
import CoordinatesNode from "./CoordinatesNode";
import GeometryNode from "./GeometryNode";

// Palette component
export { default as CategorizedPalette } from "./CategorizedPalette";

export const nodeTypes = {
  // ROS blocks
  createPackage: CreatePackageNode,
  rosRun: RosRunNode,
  string: StringNode,
  text: StringNode, // Alias for generic text node
  listArgs: ListItemsNode,
  listDeps: ListItemsNode,
  toCode: ConvertToCodeNode,

  // URDF (original)
  urdfLink: UrdfLinkNode,
  urdfJoint: UrdfJointNode,
  urdfRobot: UrdfRobotNode,
  urdfPreview: UrdfPreviewNode,
  urdfViewer: UrdfViewerNode,
  urdfControl: UrdfControlNode,

  // URDF V2 (modular/improved)
  urdfLinkV2: UrdfLinkNodeV2,
  urdfInertial: UrdfInertialNode,
  urdfVisual: UrdfVisualNode,
  urdfCollision: UrdfCollisionNode,
  urdfAssembly: UrdfAssemblyNode,
  urdfXmlPreview: UrdfXmlPreviewNode,

  // Utility blocks
  coordinates: CoordinatesNode,
  geometry: GeometryNode,
};

// Legacy palettes (for backward compatibility)
export const paletteRun = [
  { type: "text", label: "Text", category: "Input" },
  { type: "listArgs", label: "Args", category: "Input" },
  { type: "rosRun", label: "ROS Run", category: "ROS" },
  { type: "toCode", label: "Convert2Code", category: "Output" },
];

export const paletteCreate = [
  { type: "text", label: "Text", category: "Input" },
  { type: "listDeps", label: "Dependencies", category: "Input" },
  { type: "createPackage", label: "Create Package", category: "ROS" },
  { type: "toCode", label: "Convert2Code", category: "Output" },
];

export const paletteUrdf = [
  { type: "urdfLink", label: "URDF Link", category: "URDF" },
  { type: "urdfJoint", label: "URDF Joint", category: "URDF" },
  { type: "urdfRobot", label: "URDF Robot", category: "URDF" },
  { type: "urdfPreview", label: "URDF XML", category: "Output" },
  { type: "urdfViewer", label: "URDF Viewer", category: "Visualization" },
];

export const paletteUrdfV2 = [
  { type: "urdfInertial", label: "Inertial", category: "URDF" },
  { type: "urdfVisual", label: "Visual", category: "URDF" },
  { type: "urdfCollision", label: "Collision", category: "URDF" },
  { type: "urdfLinkV2", label: "Link", category: "URDF" },
  { type: "urdfJoint", label: "Joint", category: "URDF" },
  { type: "urdfAssembly", label: "Assembly", category: "URDF" },
  { type: "urdfRobot", label: "Robot", category: "URDF" },
  { type: "urdfXmlPreview", label: "XML Preview", category: "Output" },
  { type: "urdfViewer", label: "3D Viewer", category: "Visualization" },
];

// New categorized palettes
export const paletteCategorized = {
  Input: [
    { type: "text", label: "Text" },
    { type: "listArgs", label: "Args List" },
    { type: "listDeps", label: "Dependencies List" },
    { type: "coordinates", label: "Coordinates" },
    { type: "geometry", label: "Geometry" },
  ],
  ROS: [
    { type: "createPackage", label: "Create Package" },
    { type: "rosRun", label: "ROS Run" },
  ],
  URDF: [
    { type: "urdfInertial", label: "Inertial" },
    { type: "urdfVisual", label: "Visual" },
    { type: "urdfCollision", label: "Collision" },
    { type: "urdfLinkV2", label: "Link" },
    { type: "urdfJoint", label: "Joint" },
    { type: "urdfAssembly", label: "Assembly" },
    { type: "urdfRobot", label: "Robot" },
  ],
  "URDF (Legacy)": [
    { type: "urdfLink", label: "Link (All-in-one)" },
    { type: "urdfJoint", label: "Joint" },
    { type: "urdfRobot", label: "Robot" },
  ],
  Output: [
    { type: "toCode", label: "Convert to Code" },
    { type: "urdfPreview", label: "URDF XML" },
    { type: "urdfXmlPreview", label: "XML Preview" },
  ],
  Visualization: [
    { type: "urdfViewer", label: "URDF 3D Viewer" },
    { type: "urdfControl", label: "Joint Controller" },
  ],
};

export function defaultDataFor(typeOrPreset) {
  // -------- existentes --------
  if (typeOrPreset === "rosRun")
    return { pkg: "", exe: "", ns: "", args: [] };

  if (typeOrPreset === "string:pkg")
    return { label: "Package", value: "turtlesim", placeholder: "turtlesim" };
  if (typeOrPreset === "string:exe")
    return { label: "Executable", value: "turtlesim_node", placeholder: "turtlesim_node" };
  if (typeOrPreset === "string:ns")
    return { label: "Namespace", value: "", placeholder: "/demo (opcional)" };

  if (typeOrPreset === "string:pkgName")
    return { label: "Package Name", value: "my_ros2_package" };
  if (typeOrPreset === "string:nodeName")
    return { label: "Node Name", value: "my_node" };

  if (typeOrPreset === "listArgs")
    return { title: "Args", keyName: "args", items: [], placeholder: "--ros-args ..." };
  if (typeOrPreset === "listDeps")
    return { title: "Dependencies", keyName: "deps", items: [], placeholder: "rclpy" };

  if (typeOrPreset === "createPackage") {
    return {
      pkgName: "my_ros2_package",
      nodeName: "my_node",
      lang: "python",
      buildType: "ament_python",
      deps: ["rclpy", "std_msgs"],
    };
  }

  if (typeOrPreset === "toCode")
    return { inCount: 0, preview: "" };

  // -------- nuevos URDF --------
  if (typeOrPreset === "urdfLink")
    return {
      id: "",
      name: "",
      visuals: [],
      collisions: [],
      inertial: {
        mass: 1,
        inertia: { ixx: 0.01, iyy: 0.01, izz: 0.01 },
        origin: { xyz: [0, 0, 0], rpy: [0, 0, 0] },
      },
    };

  if (typeOrPreset === "urdfJoint")
    return {
      id: "",
      name: "",
      type: "fixed",
      parent: "",
      child: "",
      origin: { xyz: [0, 0, 0], rpy: [0, 0, 0] },
      axis: { xyz: [1, 0, 0] },
    };

  if (typeOrPreset === "urdfRobot")
    return { id: "", name: "my_robot", links: [], joints: [], xml: "" };

  if (typeOrPreset === "urdfPreview")
    return { id: "", xml: "" };

  if (typeOrPreset === "urdfViewer")
    return { id: "", xml: "", jointStates: {} };

  if (typeOrPreset === "urdfControl")
    return { id: "", xml: "", jointStates: {} };

  // -------- URDF V2 (modular) --------
  if (typeOrPreset === "urdfInertial")
    return {
      mass: 1.0,
      inertia: { ixx: 0.01, iyy: 0.01, izz: 0.01, ixy: 0, ixz: 0, iyz: 0 },
      origin: { xyz: [0, 0, 0], rpy: [0, 0, 0] }
    };

  if (typeOrPreset === "urdfVisual")
    return {
      geometry: { type: "box", size: [1, 1, 1] },
      origin: { xyz: [0, 0, 0], rpy: [0, 0, 0] },
      material: { name: "", color: [0.5, 0.5, 0.5, 1] }
    };

  if (typeOrPreset === "urdfCollision")
    return {
      geometry: { type: "box", size: [1, 1, 1] },
      origin: { xyz: [0, 0, 0], rpy: [0, 0, 0] }
    };

  if (typeOrPreset === "urdfLinkV2")
    return {
      name: "",
      inertial: null,
      visuals: [],
      collisions: []
    };

  if (typeOrPreset === "urdfAssembly")
    return {
      name: "",
      description: "",
      links: [],
      joints: []
    };

  if (typeOrPreset === "urdfXmlPreview")
    return { xml: "" };

  // -------- New utility blocks --------
  if (typeOrPreset === "coordinates")
    return {
      xyz: [0, 0, 0],
      rpy: [0, 0, 0]
    };

  if (typeOrPreset === "geometry")
    return {
      geometry: { type: "box", size: [1, 1, 1] }
    };

  if (typeOrPreset === "text")
    return { label: "Text", value: "", placeholder: "enter text..." };

  return {};
}
