// src/levels/slidesROS2Concepts/08-TopicCommandsGame.jsx
import React from "react";

export const meta = {
  id: "topic-commands",
  title: "ROS 2 Topic Commands",
  order: 8,
  objectiveCode: "ros2-minigame-commands",
};

export default function TopicCommandsGame({ onObjectiveHit }) {
  const commands = [
    {
      command: "ros2 topic list",
      description: "List all active topics in the ROS 2 network",
      example: "$ ros2 topic list\n/chatter\n/parameter_events\n/rosout",
      useWhen: "You want to see what topics are available"
    },
    {
      command: "ros2 topic echo /topic_name",
      description: "Display messages being published on a topic in real-time",
      example: "$ ros2 topic echo /chatter\ndata: 'Hello ROS 2: 0'\n---\ndata: 'Hello ROS 2: 1'",
      useWhen: "You want to see what data is being published"
    },
    {
      command: "ros2 topic info /topic_name",
      description: "Show information about a topic (type, publishers, subscribers)",
      example: "$ ros2 topic info /chatter\nType: std_msgs/msg/String\nPublisher count: 1\nSubscription count: 1",
      useWhen: "You want to see who's using a topic"
    },
    {
      command: "ros2 topic hz /topic_name",
      description: "Measure the publishing rate (frequency) of a topic",
      example: "$ ros2 topic hz /chatter\naverage rate: 2.000\n\tmin: 0.500s max: 0.500s",
      useWhen: "You want to check how fast messages are being published"
    },
    {
      command: "ros2 topic pub /topic_name msg_type data",
      description: "Manually publish a message to a topic from the command line",
      example: "$ ros2 topic pub /chatter std_msgs/msg/String \"data: 'Hello from CLI'\"",
      useWhen: "You want to test a subscriber or send test data"
    },
  ];

  const [selected, setSelected] = React.useState(0);
  const [completed, setCompleted] = React.useState(new Set());

  const handleMarkComplete = (idx) => {
    const newCompleted = new Set(completed);
    newCompleted.add(idx);
    setCompleted(newCompleted);

    if (newCompleted.size === commands.length) {
      onObjectiveHit?.(meta.objectiveCode);
    }
  };

  return (
    <div className="slide">
      <h2>ROS 2 Topic Commands</h2>

      <div className="slide-card">
        <div className="slide-card__title">Essential CLI Tools</div>
        <p>
          ROS 2 provides powerful command-line tools to inspect and debug topics.
          Click through each command to learn what it does.
        </p>
        <p style={{ marginTop: "0.5rem", fontSize: "0.9em", opacity: 0.8 }}>
          Progress: {completed.size} / {commands.length} commands learned
        </p>
      </div>

      <div style={{ display: "grid", gridTemplateColumns: "250px 1fr", gap: "1rem" }}>
        {/* Command list */}
        <div style={{ display: "flex", flexDirection: "column", gap: "0.5rem" }}>
          {commands.map((cmd, idx) => (
            <button
              key={idx}
              className="btn"
              onClick={() => setSelected(idx)}
              style={{
                textAlign: "left",
                opacity: selected === idx ? 1 : 0.6,
                background: completed.has(idx)
                  ? "rgba(0, 255, 0, 0.2)"
                  : selected === idx
                    ? "var(--neon, #7df9ff)"
                    : "var(--glass, rgba(255,255,255,.06))",
                color: selected === idx ? "#000" : "inherit",
                fontSize: "0.75em",
                position: "relative",
              }}
            >
              {completed.has(idx) && <span style={{ marginRight: "0.5rem" }}>✓</span>}
              {cmd.command}
            </button>
          ))}
        </div>

        {/* Command details */}
        <div className="slide-card">
          <div className="slide-card__title">
            <code>{commands[selected].command}</code>
          </div>

          <div style={{ marginTop: "1rem" }}>
            <b>What it does:</b>
            <p>{commands[selected].description}</p>
          </div>

          <div style={{ marginTop: "1rem" }}>
            <b>When to use it:</b>
            <p>{commands[selected].useWhen}</p>
          </div>

          <div style={{ marginTop: "1rem" }}>
            <b>Example output:</b>
            <pre style={{
              background: "rgba(0,0,0,0.4)",
              padding: "0.75rem",
              borderRadius: "6px",
              fontSize: "0.85em",
              overflow: "auto",
              marginTop: "0.5rem"
            }}>
              <code>{commands[selected].example}</code>
            </pre>
          </div>

          <div style={{ display: "flex", gap: "0.5rem", marginTop: "1rem" }}>
            <button
              className="btn"
              onClick={() => navigator.clipboard?.writeText(commands[selected].command)}
            >
              Copy Command
            </button>
            {!completed.has(selected) && (
              <button
                className="btn"
                onClick={() => handleMarkComplete(selected)}
                style={{ background: "rgba(0, 255, 0, 0.3)" }}
              >
                Mark as Learned
              </button>
            )}
          </div>
        </div>
      </div>

      {completed.size === commands.length && (
        <div className="slide-card" style={{
          marginTop: "1rem",
          background: "rgba(0, 255, 0, 0.1)",
          border: "2px solid rgba(0, 255, 0, 0.5)"
        }}>
          <div className="slide-card__title">🎉 Congratulations!</div>
          <p>
            You've learned all the essential ROS 2 topic commands! These tools will be
            invaluable for debugging and understanding your ROS 2 systems.
          </p>
        </div>
      )}
    </div>
  );
}
