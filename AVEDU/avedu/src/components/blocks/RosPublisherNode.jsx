import React, { useState, useEffect, useRef, useCallback } from "react";
import { Position, useStore } from "@xyflow/react";
import HandleWithLabel from "./HandleWithLabel";
import "../../styles/components/_ros-publisher-node.scss";

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
 * RosPublisherNode - Comprehensive ROS2 Publisher Block
 * Can publish various message types with configurable frequency and topic
 */
export default function RosPublisherNode({ id, data }) {
  // Get edges and nodes to determine connections
  const edges = useStore((state) => state.edges);
  const nodes = useStore((state) => state.nodes);

  const connectedHandles = edges
    .filter((e) => e.target === id)
    .map((e) => e.targetHandle);

  const isDataConnected = connectedHandles.includes("data");
  const isTopicConnected = connectedHandles.includes("topic");
  const isFrequencyConnected = connectedHandles.includes("frequency");

  // State management
  const [publisherName, setPublisherName] = useState(data.publisherName ?? "publisher_node");
  const [topicName, setTopicName] = useState(data.topicName ?? "/chatter");
  const [msgPackage, setMsgPackage] = useState(data.msgPackage ?? "std_msgs");
  const [msgType, setMsgType] = useState(data.msgType ?? "String");
  const [frequency, setFrequency] = useState(data.frequency ?? "1.0");
  const [dataInput, setDataInput] = useState(data.dataInput ?? "");
  const [queueSize, setQueueSize] = useState(data.queueSize ?? "10");
  const [expanded, setExpanded] = useState(data.expanded ?? true);

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
  // Note: We don't notify parent here - parent reads from node state directly
  useEffect(() => {
    const srcFor = (handleId) => {
      const edge = edges.find((e) => e.target === id && e.targetHandle === handleId);
      if (!edge) return null;
      return nodes.find((n) => n.id === edge.source);
    };

    if (isDataConnected) {
      const dataSrc = srcFor("data");
      if (dataSrc?.data?.value !== undefined) {
        const value = String(dataSrc.data.value);
        // Only update if value actually changed
        if (value !== prevConnectedValuesRef.current.dataInput) {
          prevConnectedValuesRef.current.dataInput = value;
          setDataInput(value);
        }
      }
    }

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

    if (isFrequencyConnected) {
      const freqSrc = srcFor("frequency");
      if (freqSrc?.data?.value) {
        const value = freqSrc.data.value;
        // Only update if value actually changed
        if (value !== prevConnectedValuesRef.current.frequency) {
          prevConnectedValuesRef.current.frequency = value;
          setFrequency(value);
        }
      }
    }
  }, [edges, nodes, id, isDataConnected, isTopicConnected, isFrequencyConnected]);

  // Stable notify function using ref - only called on user interactions
  const notifyChange = useCallback((updates) => {
    if (onChangeRef.current) {
      onChangeRef.current(id, updates);
    }
  }, [id]);

  const onPublisherNameChange = (value) => {
    setPublisherName(value);
    notifyChange({ publisherName: value, topicName, msgPackage, msgType, frequency, dataInput, queueSize });
  };

  const onTopicChange = (value) => {
    setTopicName(value);
    notifyChange({ publisherName, topicName: value, msgPackage, msgType, frequency, dataInput, queueSize });
  };

  const onPackageChange = (value) => {
    setMsgPackage(value);
    // Reset message type to first available in new package
    const newType = MESSAGE_TYPES[value]?.[0]?.type || "String";
    setMsgType(newType);
    notifyChange({ publisherName, topicName, msgPackage: value, msgType: newType, frequency, dataInput, queueSize });
  };

  const onTypeChange = (value) => {
    setMsgType(value);
    notifyChange({ publisherName, topicName, msgPackage, msgType: value, frequency, dataInput, queueSize });
  };

  const onFrequencyChange = (value) => {
    setFrequency(value);
    notifyChange({ publisherName, topicName, msgPackage, msgType, frequency: value, dataInput, queueSize });
  };

  const onDataChange = (value) => {
    setDataInput(value);
    notifyChange({ publisherName, topicName, msgPackage, msgType, frequency, dataInput: value, queueSize });
  };

  const onQueueChange = (value) => {
    setQueueSize(value);
    notifyChange({ publisherName, topicName, msgPackage, msgType, frequency, dataInput, queueSize: value });
  };

  const renderDataInput = () => {
    if (isDataConnected) {
      return (
        <div className="ros-publisher-node__data-field ros-publisher-node__data-field--connected">
          <span>Data: Connected from block</span>
          <div className="ros-publisher-node__data-preview">
            {dataInput || "(no data yet)"}
          </div>
        </div>
      );
    }

    // Different input types based on message format
    switch (messageInfo.format) {
      case "boolean":
        return (
          <label className="rf-field">
            <span>Data Value</span>
            <select
              value={dataInput || "true"}
              onChange={(e) => onDataChange(e.target.value)}
              className="rf-input"
            >
              <option value="true">True</option>
              <option value="false">False</option>
            </select>
          </label>
        );

      case "integer":
      case "float":
        return (
          <label className="rf-field">
            <span>Data Value</span>
            <input
              type="number"
              value={dataInput}
              onChange={(e) => onDataChange(e.target.value)}
              placeholder={messageInfo.example}
              className="rf-input"
              step={messageInfo.format === "float" ? "0.01" : "1"}
            />
          </label>
        );

      case "vector3":
      case "quaternion":
      case "pose":
      case "twist":
        return (
          <label className="rf-field">
            <span>Data (JSON)</span>
            <textarea
              value={dataInput}
              onChange={(e) => onDataChange(e.target.value)}
              placeholder={messageInfo.example}
              className="rf-input"
              rows={3}
              style={{ fontFamily: "monospace", fontSize: "0.85em" }}
            />
          </label>
        );

      default:
        return (
          <label className="rf-field">
            <span>Data Value</span>
            <input
              type="text"
              value={dataInput}
              onChange={(e) => onDataChange(e.target.value)}
              placeholder={messageInfo.example}
              className="rf-input"
            />
          </label>
        );
    }
  };

  return (
    <div className="rf-card ros-publisher-node">
      <div
        className="ros-publisher-node__header"
        onClick={() => setExpanded(!expanded)}
      >
        <span className="ros-publisher-node__title">📡 ROS2 Publisher</span>
        <span className={`ros-publisher-node__toggle ${expanded ? 'ros-publisher-node__toggle--expanded' : 'ros-publisher-node__toggle--collapsed'}`}>
          {expanded ? "▼" : "▶"}
        </span>
      </div>

      <div className="ros-publisher-node__body">
        {/* Topic Name */}
        <div className={`ros-publisher-node__field-collapsible ${isTopicConnected ? 'ros-publisher-node__field-collapsible--collapsed' : ''}`}>
          <span className="ros-publisher-node__field-label">Topic Name</span>
          <div className="ros-publisher-node__field-input-wrapper">
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
            {/* Publisher Name */}
            <label className="rf-field">
              <span>Publisher Name</span>
              <input
                type="text"
                value={publisherName}
                onChange={(e) => onPublisherNameChange(e.target.value)}
                placeholder="publisher_node"
                className="rf-input"
              />
            </label>

            {/* Message Package and Type */}
            <div className="ros-publisher-node__grid-2">
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

            {/* Data Input */}
            {renderDataInput()}

            {/* Frequency and Queue Size */}
            <div className="ros-publisher-node__grid-2">
              <div className={`ros-publisher-node__field-collapsible ${isFrequencyConnected ? 'ros-publisher-node__field-collapsible--collapsed' : ''}`}>
                <span className="ros-publisher-node__field-label">Frequency (Hz)</span>
                <div className="ros-publisher-node__field-input-wrapper">
                  <input
                    type="number"
                    value={frequency}
                    onChange={(e) => onFrequencyChange(e.target.value)}
                    placeholder="1.0"
                    step="0.1"
                    min="0.01"
                    className="rf-input"
                  />
                </div>
              </div>

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
            </div>

            {/* Message Info */}
            <div className="ros-publisher-node__info-panel">
              <div className="ros-publisher-node__info-title">
                Publishing: {msgPackage}/{msgType}
              </div>
              <div className="ros-publisher-node__info-line">
                Node: {publisherName}
              </div>
              <div className="ros-publisher-node__info-line">
                Topic: {topicName}
              </div>
              <div className="ros-publisher-node__info-line">
                Rate: {frequency} Hz
              </div>
              {dataInput && (
                <div className="ros-publisher-node__info-data">
                  Data: {dataInput.length > 50 ? dataInput.substring(0, 50) + "..." : dataInput}
                </div>
              )}
            </div>
          </>
        )}
      </div>

      {/* Input Handles */}
      <HandleWithLabel
        type="target"
        position={Position.Left}
        id="data"
        label="data"
        top="25%"
      />
      <HandleWithLabel
        type="target"
        position={Position.Left}
        id="topic"
        label="topic"
        top="45%"
      />
      <HandleWithLabel
        type="target"
        position={Position.Left}
        id="frequency"
        label="frequency"
        top="65%"
      />

      {/* Output Handle */}
      <HandleWithLabel
        type="source"
        position={Position.Right}
        id="out"
        label="publisher"
        top="50%"
      />
    </div>
  );
}
