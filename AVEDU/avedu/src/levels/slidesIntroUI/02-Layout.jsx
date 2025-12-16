// src/levels/slidesIntroUI/02-Layout.jsx
export const meta = {
  id: "layout",
  title: "Platform Layout",
  order: 2,
  objectiveCode: "INTRO_UI_LAYOUT",
};

export default function Layout({ meta }) {
  return (
    <div className="slide" style={{ padding: "2rem" }}>
      <h2>{meta.title}</h2>

      <p style={{ fontSize: "1.1rem", marginBottom: "2rem" }}>
        The L.A.D platform has a simple, intuitive layout designed to help you focus on learning.
      </p>

      <div style={{ display: "grid", gap: "1.5rem", gridTemplateColumns: "1fr 1fr" }}>
        <div className="slide-card">
          <div className="slide-card__title">📋 Sidebar (Left)</div>
          <p>
            The sidebar shows your current context:
          </p>
          <ul>
            <li><b>Units View:</b> All available learning units</li>
            <li><b>Levels View:</b> Lessons within the selected unit</li>
            <li><b>Progress:</b> Completion badges (✔) for finished content</li>
          </ul>
          <div className="slide-callout slide-callout--tip" style={{ marginTop: "1rem" }}>
            <b>Tip:</b> Click the ✕ button to collapse the sidebar and maximize your workspace!
          </div>
        </div>

        <div className="slide-card">
          <div className="slide-card__title">📚 Main Stage (Center)</div>
          <p>
            The main area displays your current learning content:
          </p>
          <ul>
            <li><b>Interactive Lessons:</b> Slides, simulations, and exercises</li>
            <li><b>Level Header:</b> Title and learning objectives</li>
            <li><b>Navigation Controls:</b> Progress through content</li>
          </ul>
        </div>
      </div>

      <div className="slide-card slide-card--highlight" style={{ marginTop: "1.5rem" }}>
        <div className="slide-card__title">🎨 Theme Toggle (Top-Right)</div>
        <p>
          Switch between dark and light modes using the theme toggle button in the top-right corner.
          Choose the mode that's most comfortable for your eyes!
        </p>
      </div>

      <div className="slide-callout slide-callout--info" style={{ marginTop: "1.5rem" }}>
        <b>Navigation Tip:</b> When the sidebar is collapsed, look for the hamburger menu (☰)
        in the top-left to open it again.
      </div>
    </div>
  );
}
