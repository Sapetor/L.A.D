// src/levels/slidesROS2Concepts/04-WhatAreTopics.jsx
import React from "react";

export const meta = {
  id: "what-are-topics",
  title: "What are Topics?",
  order: 4,
  objectiveCode: "ros2-topics-what",
};

export default function WhatAreTopics() {
  return (
    <div className="slide">
      <h2>What are ROS 2 Topics?</h2>

      <div className="slide-card">
        <div className="slide-card__title">Definition</div>
        <p>
          A <b>Topic</b> is a named bus over which nodes exchange messages.
          Topics implement a <b>publish-subscribe pattern</b>: nodes can publish messages
          to a topic, and other nodes can subscribe to receive those messages.
        </p>
      </div>

      <div className="slide-card">
        <div className="slide-card__title">The Publish-Subscribe Pattern</div>
        <div style={{
          display: "grid",
          gridTemplateColumns: "1fr auto 1fr auto 1fr",
          alignItems: "center",
          gap: "1rem",
          padding: "1.5rem",
          background: "rgba(0,0,0,0.2)",
          borderRadius: "8px"
        }}>
          <div style={{ textAlign: "center" }}>
            <div style={{
              padding: "1rem",
              background: "rgba(125, 249, 255, 0.2)",
              borderRadius: "8px",
              border: "2px solid rgba(125, 249, 255, 0.5)"
            }}>
              <b>Publisher Node</b>
              <p style={{ fontSize: "0.85em", marginTop: "0.5rem" }}>
                Sends messages
              </p>
            </div>
          </div>

          <div style={{ fontSize: "2em", color: "var(--neon, #7df9ff)" }}>→</div>

          <div style={{ textAlign: "center" }}>
            <div style={{
              padding: "1rem",
              background: "rgba(255, 95, 244, 0.2)",
              borderRadius: "8px",
              border: "2px solid rgba(255, 95, 244, 0.5)"
            }}>
              <b>Topic</b>
              <p style={{ fontSize: "0.85em", marginTop: "0.5rem" }}>
                /camera/image
              </p>
            </div>
          </div>

          <div style={{ fontSize: "2em", color: "var(--neon, #7df9ff)" }}>→</div>

          <div style={{ textAlign: "center" }}>
            <div style={{
              padding: "1rem",
              background: "rgba(125, 249, 255, 0.2)",
              borderRadius: "8px",
              border: "2px solid rgba(125, 249, 255, 0.5)"
            }}>
              <b>Subscriber Node(s)</b>
              <p style={{ fontSize: "0.85em", marginTop: "0.5rem" }}>
                Receive messages
              </p>
            </div>
          </div>
        </div>
      </div>

      <div className="slide-card slide-card--aside">
        <div>
          <div className="slide-card__title">Key Features</div>
          <ul>
            <li><b>Decoupled:</b> Publishers don't know who's listening</li>
            <li><b>Many-to-Many:</b> Multiple publishers and subscribers on one topic</li>
            <li><b>Asynchronous:</b> Publishers send without waiting for subscribers</li>
            <li><b>Typed:</b> Each topic has a specific message type</li>
          </ul>
        </div>
        <div className="slide-callout slide-callout--info">
          <b>Analogy:</b> Topics are like radio stations. Publishers broadcast on a frequency,
          and anyone can tune in to listen. Subscribers don't affect the broadcast.
        </div>
      </div>

      <div className="slide-card">
        <div className="slide-card__title">Common Topic Examples</div>
        <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: "1rem" }}>
          <div>
            <code>/camera/image_raw</code>
            <p>Raw camera images</p>
          </div>
          <div>
            <code>/cmd_vel</code>
            <p>Velocity commands for robot movement</p>
          </div>
          <div>
            <code>/odom</code>
            <p>Odometry (position & speed)</p>
          </div>
          <div>
            <code>/scan</code>
            <p>Laser scanner data</p>
          </div>
        </div>
      </div>
    </div>
  );
}
