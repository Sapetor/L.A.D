import React, { useState, useEffect, useRef, useCallback } from "react";
import { Position, useStore } from "@xyflow/react";
import HandleWithLabel from "./HandleWithLabel";
import "../../styles/components/_ros-subscriber-node.scss";

/**
 * Message type configurations for different ROS2 message types
 */
const MESSAGE_TYPES = {
  "std_msgs": [
    { type: "String", format: "text", example: "Hello World" },
    { type: "Int32", format: "integer", example: "42" },
    { type: "Int64", format: "integer", example: "1000" },
    { type: "Float32", format: "float", example: "3.14" },
    { type: "Float64", format: "float", example: "3.14159" },
    { type: "Bool", format: "boolean", example: "true" },
    { type: "UInt8", format: "integer", example: "255" },
    { type: "UInt16", format: "integer", example: "65535" },
  ],
  "geometry_msgs": [
    { type: "Point", format: "vector3", example: "{x: 0.0, y: 0.0, z: 0.0}" },
    { type: "Pose", format: "pose", example: "{position: {...}, orientation: {...}}" },
    { type: "Twist", format: "twist", example: "{linear: {x,y,z}, angular: {x,y,z}}" },
    { type: "Vector3", format: "vector3", example: "{x: 0.0, y: 0.0, z: 0.0}" },
    { type: "Quaternion", format: "quaternion", example: "{x: 0, y: 0, z: 0, w: 1}" },
  ],
  "sensor_msgs": [
    { type: "Image", format: "image", example: "{width, height, encoding, data}" },
    { type: "LaserScan", format: "lidar", example: "{ranges[], intensities[], ...}" },
    { type: "PointCloud2", format: "pointcloud", example: "{points[], ...}" },
    { type: "Imu", format: "imu", example: "{orientation, angular_velocity, ...}" },
    { type: "CameraInfo", format: "camera", example: "{width, height, K[], ...}" },
    { type: "Temperature", format: "float", example: "{temperature: 25.5}" },
  ],
  "nav_msgs": [
    { type: "Odometry", format: "odometry", example: "{pose, twist, ...}" },
    { type: "Path", format: "path", example: "{poses[], ...}" },
  ],
};

/**
 * RosSubscriberNode - Comprehensive ROS2 Subscriber Block
 * Can subscribe to various topics and message types
 */
export default function RosSubscriberNode({ id, data }) {
  // Get edges and nodes to determine connections
  const edges = useStore((state) => state.edges);
  const nodes = useStore((state) => state.nodes);

  const connectedHandles = edges
    .filter((e) => e.target === id)
    .map((e) => e.targetHandle);

  const isTopicConnected = connectedHandles.includes("topic");

  // State management
  const [subscriberName, setSubscriberName] = useState(data.subscriberName ?? "subscriber_node");
  const [topicName, setTopicName] = useState(data.topicName ?? "/chatter");
  const [msgPackage, setMsgPackage] = useState(data.msgPackage ?? "std_msgs");
  const [msgType, setMsgType] = useState(data.msgType ?? "String");
  const [queueSize, setQueueSize] = useState(data.queueSize ?? "10");
  const [expanded, setExpanded] = useState(data.expanded ?? true);
  const [lastMessage, setLastMessage] = useState(data.lastMessage ?? "");

  // Get message format info
  const messageInfo = MESSAGE_TYPES[msgPackage]?.find(m => m.type === msgType) ||
    { format: "text", example: "data" };

  // Track previous values from connections to prevent infinite loops
  const prevConnectedValuesRef = useRef({});

  // Store the onChange callback in a ref to keep it stable
  const onChangeRef = useRef(data.onChange);
  useEffect(() => {
    onChangeRef.current = data.onChange;
  }, [data.onChange]);

  // Update state when external connections provide data
  useEffect(() => {
    const srcFor = (handleId) => {
      const edge = edges.find((e) => e.target === id && e.targetHandle === handleId);
      if (!edge) return null;
      return nodes.find((n) => n.id === edge.source);
    };

    if (isTopicConnected) {
      const topicSrc = srcFor("topic");
      if (topicSrc?.data?.value) {
        const value = topicSrc.data.value;
        // Only update if value actually changed
        if (value !== prevConnectedValuesRef.current.topicName) {
          prevConnectedValuesRef.current.topicName = value;
          setTopicName(value);
        }
      }
    }
  }, [edges, nodes, id, isTopicConnected]);

  // Stable notify function using ref - only called on user interactions
  const notifyChange = useCallback((updates) => {
    if (onChangeRef.current) {
      onChangeRef.current(id, updates);
    }
  }, [id]);

  const onSubscriberNameChange = (value) => {
    setSubscriberName(value);
    notifyChange({ subscriberName: value, topicName, msgPackage, msgType, queueSize, lastMessage });
  };

  const onTopicChange = (value) => {
    setTopicName(value);
    notifyChange({ subscriberName, topicName: value, msgPackage, msgType, queueSize, lastMessage });
  };

  const onPackageChange = (value) => {
    setMsgPackage(value);
    // Reset message type to first available in new package
    const newType = MESSAGE_TYPES[value]?.[0]?.type || "String";
    setMsgType(newType);
    notifyChange({ subscriberName, topicName, msgPackage: value, msgType: newType, queueSize, lastMessage });
  };

  const onTypeChange = (value) => {
    setMsgType(value);
    notifyChange({ subscriberName, topicName, msgPackage, msgType: value, queueSize, lastMessage });
  };

  const onQueueChange = (value) => {
    setQueueSize(value);
    notifyChange({ subscriberName, topicName, msgPackage, msgType, queueSize: value, lastMessage });
  };

  return (
    <div className="rf-card ros-subscriber-node">
      <div
        className="ros-subscriber-node__header"
        onClick={() => setExpanded(!expanded)}
      >
        <span className="ros-subscriber-node__title">📥 ROS2 Subscriber</span>
        <span className={`ros-subscriber-node__toggle ${expanded ? 'ros-subscriber-node__toggle--expanded' : 'ros-subscriber-node__toggle--collapsed'}`}>
          {expanded ? "▼" : "▶"}
        </span>
      </div>

      <div className="ros-subscriber-node__body">
        {/* Topic Name */}
        <div className={`ros-subscriber-node__field-collapsible ${isTopicConnected ? 'ros-subscriber-node__field-collapsible--collapsed' : ''}`}>
          <span className="ros-subscriber-node__field-label">Topic Name</span>
          <div className="ros-subscriber-node__field-input-wrapper">
            <input
              value={topicName}
              onChange={(e) => onTopicChange(e.target.value)}
              placeholder="/chatter"
              className="rf-input"
            />
          </div>
        </div>

        {expanded && (
          <>
            {/* Subscriber Name */}
            <label className="rf-field">
              <span>Subscriber Name</span>
              <input
                type="text"
                value={subscriberName}
                onChange={(e) => onSubscriberNameChange(e.target.value)}
                placeholder="subscriber_node"
                className="rf-input"
              />
            </label>

            {/* Message Package and Type */}
            <div className="ros-subscriber-node__grid-2">
              <label className="rf-field">
                <span>Package</span>
                <select
                  value={msgPackage}
                  onChange={(e) => onPackageChange(e.target.value)}
                  className="rf-input"
                >
                  <option value="std_msgs">std_msgs</option>
                  <option value="geometry_msgs">geometry_msgs</option>
                  <option value="sensor_msgs">sensor_msgs</option>
                  <option value="nav_msgs">nav_msgs</option>
                </select>
              </label>

              <label className="rf-field">
                <span>Message Type</span>
                <select
                  value={msgType}
                  onChange={(e) => onTypeChange(e.target.value)}
                  className="rf-input"
                >
                  {MESSAGE_TYPES[msgPackage]?.map((msg) => (
                    <option key={msg.type} value={msg.type}>
                      {msg.type}
                    </option>
                  ))}
                </select>
              </label>
            </div>

            {/* Queue Size */}
            <label className="rf-field">
              <span>Queue Size</span>
              <input
                type="number"
                value={queueSize}
                onChange={(e) => onQueueChange(e.target.value)}
                placeholder="10"
                min="1"
                className="rf-input"
              />
            </label>

            {/* Last Message Preview */}
            {lastMessage && (
              <div className="ros-subscriber-node__message-preview">
                <span className="ros-subscriber-node__message-label">Last Message:</span>
                <div className="ros-subscriber-node__message-content">
                  {lastMessage.length > 100 ? lastMessage.substring(0, 100) + "..." : lastMessage}
                </div>
              </div>
            )}

            {/* Message Info */}
            <div className="ros-subscriber-node__info-panel">
              <div className="ros-subscriber-node__info-title">
                Subscribing: {msgPackage}/{msgType}
              </div>
              <div className="ros-subscriber-node__info-line">
                Node: {subscriberName}
              </div>
              <div className="ros-subscriber-node__info-line">
                Topic: {topicName}
              </div>
              <div className="ros-subscriber-node__info-line">
                Queue: {queueSize}
              </div>
            </div>
          </>
        )}
      </div>

      {/* Input Handles */}
      <HandleWithLabel
        type="target"
        position={Position.Left}
        id="topic"
        label="topic"
        top="50%"
      />

      {/* Output Handle - for passing received data */}
      <HandleWithLabel
        type="source"
        position={Position.Right}
        id="out"
        label="data"
        top="50%"
      />
    </div>
  );
}
