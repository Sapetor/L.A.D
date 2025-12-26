// ROS2 command generation helpers for BlockCanvas

/**
 * Helper: read package data from connected nodes
 */
function computePackageData(id, nodes, edges) {
  const pkgNode = nodes.find((n) => n.id === id);
  if (!pkgNode || pkgNode.type !== "createPackage") return null;

  const incoming = edges.filter((e) => e.target === id);
  const srcFor = (handleId) => {
    const ed = incoming.find((e) => e.targetHandle === handleId);
    if (!ed) return undefined;
    return nodes.find((n) => n.id === ed.source);
  };

  const base = pkgNode.data || {};
  let pkgName = base.pkgName || "my_package";
  let nodeName = base.nodeName || "";
  let deps = Array.isArray(base.deps) ? base.deps : ["rclpy", "std_msgs"];
  const lang = base.lang || "python";
  const buildType = base.buildType || (lang === "cpp" ? "ament_cmake" : "ament_python");

  const pkgSrc = srcFor("pkgName");
  if ((pkgSrc?.type === "string" || pkgSrc?.type === "text") && pkgSrc.data?.value) {
    pkgName = String(pkgSrc.data.value);
  }

  const nodeSrc = srcFor("nodeName");
  if ((nodeSrc?.type === "string" || nodeSrc?.type === "text") && nodeSrc.data?.value) {
    nodeName = String(nodeSrc.data.value);
  }

  const depsSrc = srcFor("deps");
  if (depsSrc?.type === "listDeps" && Array.isArray(depsSrc.data?.items)) {
    deps = depsSrc.data.items;
  }

  return { pkgName, nodeName, deps, lang, buildType };
}

/**
 * Helper: generate ros2 pkg create command
 */
function buildCreatePkgCmd(pkgData) {
  if (!pkgData) return "";

  const { pkgName, nodeName, lang, buildType, deps } = pkgData;
  const bt = buildType || (lang === "cpp" ? "ament_cmake" : "ament_python");
  const depsList = (deps || []).filter(Boolean).join(" ");
  const depsPart = depsList ? ` --dependencies ${depsList}` : "";
  const nodePart = nodeName ? ` --node-name ${nodeName}` : "";
  const pkgPart = pkgName || "my_ros2_package";

  // Create src directory if it doesn't exist, then create package inside it
  return `mkdir -p src && cd src && ros2 pkg create --build-type ${bt}${nodePart} ${pkgPart}${depsPart}`;
}

/**
 * Generate Python ROS2 publisher code from RosPublisherNode data
 */
function generatePublisherCode(publisherData) {
  if (!publisherData) return "";

  const {
    publisherName = "publisher_node",
    topicName = "/chatter",
    msgPackage = "std_msgs",
    msgType = "String",
    frequency = "1.0",
    dataInput = "",
    queueSize = "10",
  } = publisherData;

  // Import statement
  const importLine = `from ${msgPackage}.msg import ${msgType}`;

  // Publisher creation
  const publisherLine = `self.publisher_ = self.create_publisher(${msgType}, '${topicName}', ${queueSize})`;

  // Timer setup
  const timerPeriod = (1.0 / parseFloat(frequency || 1.0)).toFixed(3);
  const timerLine = `self.timer = self.create_timer(${timerPeriod}, self.timer_callback)`;

  // Message preparation based on type
  let msgPreparation = "";
  if (msgType === "String") {
    msgPreparation = `msg = ${msgType}()\n        msg.data = ${dataInput ? `'${dataInput}'` : "'Hello World'"}`;
  } else if (msgType.match(/^(Int32|Int64|UInt8|UInt16|Float32|Float64)$/)) {
    msgPreparation = `msg = ${msgType}()\n        msg.data = ${dataInput || "0"}`;
  } else if (msgType === "Bool") {
    msgPreparation = `msg = ${msgType}()\n        msg.data = ${dataInput || "True"}`;
  } else if (msgType === "Point" || msgType === "Vector3") {
    msgPreparation = `msg = ${msgType}()\n        # TODO: Set x, y, z values\n        msg.x = 0.0\n        msg.y = 0.0\n        msg.z = 0.0`;
  } else if (msgType === "Twist") {
    msgPreparation = `msg = ${msgType}()\n        # TODO: Set linear and angular velocities\n        msg.linear.x = 0.0\n        msg.angular.z = 0.0`;
  } else {
    msgPreparation = `msg = ${msgType}()\n        # TODO: Configure message fields`;
  }

  // Convert publisher name to valid Python class name (PascalCase)
  const className = publisherName
    .split('_')
    .map(word => word.charAt(0).toUpperCase() + word.slice(1))
    .join('');

  // Full code template
  const code = `#!/usr/bin/env python3
# =============================================================================
# 🧩 This file was generated from visual blocks!
# You can edit it as text, but re-opening it will show the block editor.
# =============================================================================

import rclpy
from rclpy.node import Node
${importLine}

class ${className}(Node):
    def __init__(self):
        super().__init__('${publisherName}')
        ${publisherLine}
        ${timerLine}
        self.get_logger().info('Publisher started on ${topicName}')

    def timer_callback(self):
        ${msgPreparation}
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = ${className}()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()`;

  return code;
}

/**
 * Generate Python ROS2 subscriber code from RosSubscriberNode data
 */
function generateSubscriberCode(subscriberData) {
  if (!subscriberData) return "";

  const {
    subscriberName = "subscriber_node",
    topicName = "/chatter",
    msgPackage = "std_msgs",
    msgType = "String",
    queueSize = "10",
  } = subscriberData;

  // Convert subscriber name to valid Python class name (PascalCase)
  const className = subscriberName
    .split('_')
    .map(word => word.charAt(0).toUpperCase() + word.slice(1))
    .join('');

  // Import statement
  const importLine = `from ${msgPackage}.msg import ${msgType}`;

  // Callback processing based on type
  let msgProcessing = "";
  if (msgType === "String") {
    msgProcessing = `self.get_logger().info(f'Received: "{msg.data}"')`;
  } else if (msgType.match(/^(Int32|Int64|UInt8|UInt16|Float32|Float64)$/)) {
    msgProcessing = `self.get_logger().info(f'Received: {msg.data}')`;
  } else if (msgType === "Bool") {
    msgProcessing = `self.get_logger().info(f'Received: {msg.data}')`;
  } else if (msgType === "Point" || msgType === "Vector3") {
    msgProcessing = `self.get_logger().info(f'Received Point - x: {msg.x}, y: {msg.y}, z: {msg.z}')`;
  } else if (msgType === "Twist") {
    msgProcessing = `self.get_logger().info(f'Linear: ({msg.linear.x}, {msg.linear.y}, {msg.linear.z}), Angular: ({msg.angular.x}, {msg.angular.y}, {msg.angular.z})')`;
  } else {
    msgProcessing = `self.get_logger().info(f'Received message on ${topicName}')`;
  }

  // Full code template
  const code = `#!/usr/bin/env python3
# =============================================================================
# 🧩 This file was generated from visual blocks!
# You can edit it as text, but re-opening it will show the block editor.
# =============================================================================

import rclpy
from rclpy.node import Node
${importLine}


class ${className}(Node):
    def __init__(self):
        super().__init__('${subscriberName}')

        # Create subscriber
        self.subscription = self.create_subscription(
            ${msgType},
            '${topicName}',
            self.listener_callback,
            ${queueSize}
        )
        self.subscription  # prevent unused variable warning

        self.get_logger().info('Subscriber started on ${topicName}')

    def listener_callback(self, msg):
        ${msgProcessing}


def main(args=None):
    rclpy.init(args=args)
    node = ${className}()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()`;

  return code;
}

/**
 * Sync toCode node previews with generated commands/code from connected nodes
 * Similar to syncUrdfDerived and syncJointStates in urdf-helpers.js
 */
export function syncRos2Commands(nodes, edges, setNodes) {
  const toCodeNodes = nodes.filter((n) => n.type === "toCode");
  if (toCodeNodes.length === 0) return;

  let changed = false;
  const updates = [];

  for (const toCodeNode of toCodeNodes) {
    // Find incoming edges to this toCode node
    const incomingEdges = edges.filter((e) => e.target === toCodeNode.id);
    const inCount = incomingEdges.length;

    let newPreview = "";

    // Check for connected createPackage node
    const createPkgNode = incomingEdges
      .map((e) => nodes.find((n) => n.id === e.source))
      .find((n) => n && n.type === "createPackage");

    if (createPkgNode) {
      const pkgData = computePackageData(createPkgNode.id, nodes, edges);
      if (pkgData) {
        newPreview = buildCreatePkgCmd(pkgData);
      }
    }

    // Check for connected rosPublisher node
    const publisherNode = incomingEdges
      .map((e) => nodes.find((n) => n.id === e.source))
      .find((n) => n && n.type === "rosPublisher");

    if (publisherNode) {
      newPreview = generatePublisherCode(publisherNode.data);
    }

    // Check for connected rosSubscriber node
    const subscriberNode = incomingEdges
      .map((e) => nodes.find((n) => n.id === e.source))
      .find((n) => n && n.type === "rosSubscriber");

    if (subscriberNode) {
      newPreview = generateSubscriberCode(subscriberNode.data);
    }

    const currentPreview = toCodeNode.data?.preview || "";
    const currentInCount = Number(toCodeNode.data?.inCount || 0);

    // Check if anything changed
    if (currentPreview !== newPreview || currentInCount !== inCount) {
      changed = true;
      updates.push({
        id: toCodeNode.id,
        preview: newPreview,
        inCount,
      });
    }
  }

  // Apply updates if anything changed
  if (changed && updates.length > 0) {
    setNodes((nds) =>
      nds.map((n) => {
        const update = updates.find((u) => u.id === n.id);
        if (!update) return n;

        return {
          ...n,
          data: {
            ...n.data,
            preview: update.preview,
            inCount: update.inCount,
          },
        };
      })
    );
  }
}
