// components/ide/LidarVisualizer.jsx
import React, { useState, useEffect, useRef } from "react";
import PropTypes from "prop-types";
import "./LidarVisualizer.scss";

/**
 * LIDAR Visualizer Side Panel
 * Displays real-time LIDAR scan data from ROS2 /scan topic
 */
export function LidarVisualizer({ canvasId, isVisible }) {
  const canvasRef = useRef(null);
  const wsRef = useRef(null);
  const [isConnected, setIsConnected] = useState(false);
  const [scanData, setScanData] = useState(null);
  const [topicName, setTopicName] = useState("/qcar/lidar/scan");
  const [showGrid, setShowGrid] = useState(true);
  const [maxRange, setMaxRange] = useState(10.0);
  const [connectionError, setConnectionError] = useState(null);
  const [stats, setStats] = useState({ messagesReceived: 0, lastUpdate: null });

  // Connect to rosbridge WebSocket
  useEffect(() => {
    if (!isConnected) return;

    // ROSBridge WebSocket URL (adjust based on your setup)
    const rosbridgeUrl = "ws://localhost:9090";

    try {
      const ws = new WebSocket(rosbridgeUrl);
      wsRef.current = ws;

      ws.onopen = () => {
        console.log("[LIDAR Visualizer] WebSocket connected");
        setConnectionError(null);

        // Subscribe to the LIDAR topic
        const subscribeMsg = {
          op: "subscribe",
          topic: topicName,
          type: "sensor_msgs/LaserScan",
          throttle_rate: 100, // Limit to 10Hz
          queue_length: 1
        };

        ws.send(JSON.stringify(subscribeMsg));
        console.log(`[LIDAR Visualizer] Subscribed to ${topicName}`);
      };

      ws.onmessage = (event) => {
        try {
          const message = JSON.parse(event.data);

          if (message.op === "publish" && message.topic === topicName) {
            const msg = message.msg;

            setScanData({
              ranges: msg.ranges,
              angle_min: msg.angle_min,
              angle_max: msg.angle_max,
              angle_increment: msg.angle_increment,
              range_min: msg.range_min,
              range_max: msg.range_max,
              timestamp: Date.now()
            });

            setStats(prev => ({
              messagesReceived: prev.messagesReceived + 1,
              lastUpdate: new Date().toLocaleTimeString()
            }));
          }
        } catch (error) {
          console.error("[LIDAR Visualizer] Error parsing message:", error);
        }
      };

      ws.onerror = (error) => {
        console.error("[LIDAR Visualizer] WebSocket error:", error);
        setConnectionError("WebSocket connection error. Is rosbridge running?");
      };

      ws.onclose = () => {
        console.log("[LIDAR Visualizer] WebSocket disconnected");
        setIsConnected(false);
        setConnectionError("Connection closed");
      };

      return () => {
        if (ws.readyState === WebSocket.OPEN) {
          // Unsubscribe before closing
          const unsubscribeMsg = {
            op: "unsubscribe",
            topic: topicName
          };
          ws.send(JSON.stringify(unsubscribeMsg));
        }
        ws.close();
      };
    } catch (error) {
      console.error("[LIDAR Visualizer] Failed to create WebSocket:", error);
      setConnectionError("Failed to connect. Make sure rosbridge is running on ws://localhost:9090");
      setIsConnected(false);
    }
  }, [isConnected, topicName]);

  // Draw LIDAR visualization
  useEffect(() => {
    if (!canvasRef.current || !scanData || !isVisible) return;

    const canvas = canvasRef.current;
    const ctx = canvas.getContext("2d");
    const width = canvas.width;
    const height = canvas.height;
    const centerX = width / 2;
    const centerY = height / 2;
    const scale = Math.min(width, height) / 2 / maxRange * 0.85;

    // Clear canvas
    ctx.fillStyle = "rgba(11, 16, 32, 0.95)";
    ctx.fillRect(0, 0, width, height);

    // Draw grid if enabled
    if (showGrid) {
      ctx.strokeStyle = "rgba(125, 249, 255, 0.15)";
      ctx.lineWidth = 1;

      // Draw concentric circles
      for (let r = 1; r <= Math.floor(maxRange); r++) {
        ctx.beginPath();
        ctx.arc(centerX, centerY, r * scale, 0, 2 * Math.PI);
        ctx.stroke();

        // Draw range labels
        ctx.fillStyle = "rgba(125, 249, 255, 0.5)";
        ctx.font = "11px 'Fira Code', monospace";
        ctx.fillText(`${r}m`, centerX + 4, centerY - r * scale + 4);
      }

      // Draw angle lines (every 30 degrees)
      ctx.strokeStyle = "rgba(125, 249, 255, 0.1)";
      for (let angle = 0; angle < 360; angle += 30) {
        const rad = (angle * Math.PI) / 180;
        ctx.beginPath();
        ctx.moveTo(centerX, centerY);
        ctx.lineTo(
          centerX + Math.cos(rad - Math.PI / 2) * maxRange * scale,
          centerY + Math.sin(rad - Math.PI / 2) * maxRange * scale
        );
        ctx.stroke();

        // Draw angle labels
        ctx.fillStyle = "rgba(125, 249, 255, 0.6)";
        ctx.font = "10px 'Fira Code', monospace";
        const labelRadius = maxRange * scale * 0.92;
        const x = centerX + Math.cos(rad - Math.PI / 2) * labelRadius;
        const y = centerY + Math.sin(rad - Math.PI / 2) * labelRadius;
        ctx.fillText(`${angle}°`, x - 12, y + 4);
      }
    }

    // Draw LIDAR points
    const { ranges, angle_min, angle_increment } = scanData;

    // Use gradient for points based on distance
    for (let i = 0; i < ranges.length; i++) {
      const range = ranges[i];

      // Skip invalid readings
      if (isNaN(range) || range < scanData.range_min || range > scanData.range_max) {
        continue;
      }

      const angle = angle_min + i * angle_increment;
      const x = centerX + Math.cos(angle) * range * scale;
      const y = centerY + Math.sin(angle) * range * scale;

      // Color based on distance (closer = red, farther = cyan)
      const normalizedDist = range / maxRange;
      const red = Math.floor((1 - normalizedDist) * 255);
      const cyan = Math.floor(normalizedDist * 255);

      ctx.fillStyle = `rgb(${red}, ${cyan}, 255)`;
      ctx.beginPath();
      ctx.arc(x, y, 2.5, 0, 2 * Math.PI);
      ctx.fill();
    }

    // Draw robot center
    ctx.fillStyle = "rgba(255, 95, 244, 0.9)";
    ctx.beginPath();
    ctx.arc(centerX, centerY, 5, 0, 2 * Math.PI);
    ctx.fill();

    // Draw robot direction indicator (forward = up)
    ctx.strokeStyle = "rgba(255, 95, 244, 0.9)";
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(centerX, centerY);
    ctx.lineTo(centerX, centerY - 20);
    ctx.stroke();

    // Draw arrow head
    ctx.beginPath();
    ctx.moveTo(centerX, centerY - 20);
    ctx.lineTo(centerX - 5, centerY - 14);
    ctx.moveTo(centerX, centerY - 20);
    ctx.lineTo(centerX + 5, centerY - 14);
    ctx.stroke();

  }, [scanData, showGrid, maxRange, isVisible]);

  const handleToggleConnection = () => {
    if (isConnected) {
      setIsConnected(false);
      setScanData(null);
      setStats({ messagesReceived: 0, lastUpdate: null });
    } else {
      setConnectionError(null);
      setIsConnected(true);
    }
  };

  if (!isVisible) return null;

  return (
    <div className="lidar-visualizer">
      <div className="lidar-visualizer__header">
        <h3 className="lidar-visualizer__title">LIDAR Visualizer</h3>
        <div className="lidar-visualizer__controls">
          <button
            className={`btn btn--small ${isConnected ? "btn--success" : "btn--default"}`}
            onClick={handleToggleConnection}
            title={isConnected ? "Disconnect from topic" : "Connect to topic"}
          >
            {isConnected ? "● Connected" : "○ Disconnected"}
          </button>
        </div>
      </div>

      <div className="lidar-visualizer__settings">
        <div className="lidar-visualizer__field">
          <label>Topic:</label>
          <input
            type="text"
            value={topicName}
            onChange={(e) => setTopicName(e.target.value)}
            disabled={isConnected}
            placeholder="/qcar/lidar/scan"
          />
        </div>

        <div className="lidar-visualizer__field lidar-visualizer__field--inline">
          <label>Max Range:</label>
          <input
            type="number"
            value={maxRange}
            onChange={(e) => setMaxRange(parseFloat(e.target.value) || 10.0)}
            min="1"
            max="50"
            step="0.5"
          />
          <span className="lidar-visualizer__unit">m</span>
        </div>

        <div className="lidar-visualizer__field lidar-visualizer__field--checkbox">
          <label>
            <input
              type="checkbox"
              checked={showGrid}
              onChange={(e) => setShowGrid(e.target.checked)}
            />
            Show Grid
          </label>
        </div>
      </div>

      {connectionError && (
        <div className="lidar-visualizer__error">
          ⚠️ {connectionError}
          <div className="lidar-visualizer__help">
            Run: <code>ros2 launch rosbridge_server rosbridge_websocket_launch.xml</code>
          </div>
        </div>
      )}

      <div className="lidar-visualizer__canvas-container">
        <canvas
          ref={canvasRef}
          width={400}
          height={400}
          className="lidar-visualizer__canvas"
        />
      </div>

      {scanData && (
        <div className="lidar-visualizer__stats">
          <div className="lidar-visualizer__stat">
            <span className="lidar-visualizer__stat-label">Points:</span>
            <span className="lidar-visualizer__stat-value">{scanData.ranges.length}</span>
          </div>
          <div className="lidar-visualizer__stat">
            <span className="lidar-visualizer__stat-label">Range:</span>
            <span className="lidar-visualizer__stat-value">
              {scanData.range_min.toFixed(2)}m - {scanData.range_max.toFixed(2)}m
            </span>
          </div>
          <div className="lidar-visualizer__stat">
            <span className="lidar-visualizer__stat-label">Messages:</span>
            <span className="lidar-visualizer__stat-value">{stats.messagesReceived}</span>
          </div>
          {stats.lastUpdate && (
            <div className="lidar-visualizer__stat">
              <span className="lidar-visualizer__stat-label">Last Update:</span>
              <span className="lidar-visualizer__stat-value">{stats.lastUpdate}</span>
            </div>
          )}
        </div>
      )}

      {!isConnected && !connectionError && (
        <div className="lidar-visualizer__placeholder">
          <p>Click "Connect" to start receiving LIDAR data</p>
          <p className="lidar-visualizer__note">
            Make sure rosbridge_server is running and your LIDAR node is publishing to <code>{topicName}</code>
          </p>
        </div>
      )}
    </div>
  );
}

LidarVisualizer.propTypes = {
  canvasId: PropTypes.string,
  isVisible: PropTypes.bool,
};

LidarVisualizer.defaultProps = {
  isVisible: true,
};
