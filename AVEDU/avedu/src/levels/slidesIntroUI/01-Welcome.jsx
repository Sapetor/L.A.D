// src/levels/slidesIntroUI/01-Welcome.jsx
export const meta = {
  id: "welcome",
  title: "Welcome to L.A.D",
  order: 1,
  objectiveCode: "INTRO_UI_WELCOME",
};

export default function Welcome({ meta, goNext }) {
  return (
    <div className="slide-wrap" style={{ padding: "2rem", maxWidth: "800px", margin: "0 auto" }}>
      <h1 style={{ fontSize: "2.5rem", marginBottom: "1rem", textAlign: "center" }}>
        Welcome to L.A.D Platform
      </h1>
      <h2 style={{ fontSize: "1.5rem", marginBottom: "2rem", textAlign: "center", opacity: 0.8 }}>
        Learning Autonomous Driving
      </h2>

      <div style={{ marginBottom: "2rem" }}>
        <p style={{ fontSize: "1.1rem", lineHeight: "1.6" }}>
          This platform will guide you through the fundamentals of autonomous vehicles,
          from basic robotics concepts to advanced perception and control systems.
        </p>
      </div>

      <div className="slide-card" style={{ marginBottom: "1.5rem" }}>
        <div className="slide-card__title">What You'll Learn</div>
        <ul style={{ listStyle: "none", padding: 0 }}>
          <li style={{ marginBottom: ".5rem" }}>✓ <b>ROS 2</b> - Robot Operating System fundamentals and advanced concepts</li>
          <li style={{ marginBottom: ".5rem" }}>✓ <b>Vehicle Dynamics</b> - Physics and control of autonomous vehicles</li>
          <li style={{ marginBottom: ".5rem" }}>✓ <b>Perception</b> - Sensor integration and computer vision</li>
          <li style={{ marginBottom: ".5rem" }}>✓ <b>Planning & Control</b> - Path planning and motion control algorithms</li>
          <li style={{ marginBottom: ".5rem" }}>✓ <b>AI Integration</b> - Machine learning for autonomous systems</li>
          <li>✓ <b>Safety & Verification</b> - Testing and validation methods</li>
        </ul>
      </div>

      <div className="slide-callout slide-callout--info" style={{ marginBottom: "2rem" }}>
        <b>Before we start:</b> This tutorial will show you how to navigate through
        the platform and use all the available features.
      </div>

      <div className="slide-actions" style={{ textAlign: "center" }}>
        <button className="btn" onClick={goNext} style={{ fontSize: "1.1rem", padding: ".75rem 2rem" }}>
          Let's Get Started →
        </button>
      </div>
    </div>
  );
}
