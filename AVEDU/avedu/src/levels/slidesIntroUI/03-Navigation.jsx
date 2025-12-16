// src/levels/slidesIntroUI/03-Navigation.jsx
export const meta = {
  id: "navigation",
  title: "Navigating the Platform",
  order: 3,
  objectiveCode: "INTRO_UI_NAV",
};

export default function Navigation({ meta }) {
  return (
    <div className="slide" style={{ padding: "2rem" }}>
      <h2>{meta.title}</h2>

      <p style={{ fontSize: "1.1rem", marginBottom: "2rem" }}>
        Moving through the curriculum is simple and intuitive. Here's how:
      </p>

      <div style={{ display: "grid", gap: "1.5rem" }}>
        <div className="slide-card">
          <div className="slide-card__title">1️⃣ Selecting a Unit</div>
          <div style={{ display: "grid", gap: "1rem" }}>
            <p>
              Start by clicking any unit in the sidebar (like "Introduction", "Vehicle Dynamics", "ROS 2 Concepts", etc.)
            </p>
            <div className="slide-callout slide-callout--example">
              <b>Example:</b> Click "Vehicle Dynamics" to learn about vehicle physics and control.
            </div>
          </div>
        </div>

        <div className="slide-card">
          <div className="slide-card__title">2️⃣ Choosing a Level</div>
          <div style={{ display: "grid", gap: "1rem" }}>
            <p>
              After selecting a unit, the sidebar will show all available levels (lessons) for that unit.
              Click any level to start learning!
            </p>
            <div className="slide-callout slide-callout--tip">
              <b>Pro Tip:</b> Completed levels show a ✔ badge. Try to complete them in order for the best learning experience.
            </div>
          </div>
        </div>

        <div className="slide-card">
          <div className="slide-card__title">3️⃣ Going Back</div>
          <div style={{ display: "grid", gap: "1rem" }}>
            <p>
              When viewing levels, you'll see a "← Back to Units" button at the top of the sidebar
              to return to the units list.
            </p>
          </div>
        </div>
      </div>

      <div className="slide-card slide-card--highlight" style={{ marginTop: "1.5rem" }}>
        <div className="slide-card__title">⌨️ Keyboard Shortcuts</div>
        <div style={{ display: "grid", gridTemplateColumns: "repeat(auto-fit, minmax(200px, 1fr))", gap: "1rem" }}>
          <div>
            <span className="slide-kbd">←</span> <span className="slide-kbd">→</span>
            <p style={{ marginTop: ".5rem", opacity: 0.8 }}>Navigate between slides</p>
          </div>
        </div>
      </div>
    </div>
  );
}
