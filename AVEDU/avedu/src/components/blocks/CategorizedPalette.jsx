import React, { useState } from "react";

/**
 * CategorizedPalette - Tabbed palette component with category-based organization
 *
 * @param {object} props
 * @param {object} props.categories - Object with category names as keys and arrays of node types as values
 * @param {string} [props.defaultCategory] - Default active category
 */
export default function CategorizedPalette({ categories, defaultCategory }) {
  const categoryNames = Object.keys(categories);
  const [activeCategory, setActiveCategory] = useState(
    defaultCategory || categoryNames[0]
  );

  const activeNodes = categories[activeCategory] || [];

  return (
    <div className="rfp-palette rfp-palette--tabbed">
      <div className="rfp-palette__tabs">
        {categoryNames.map((cat) => (
          <div
            key={cat}
            className={`rfp-palette__tab ${activeCategory === cat ? 'rfp-palette__tab--active' : ''}`}
            onClick={() => setActiveCategory(cat)}
          >
            {cat}
          </div>
        ))}
      </div>

      <div className="rfp-palette__inner">
        {activeNodes.map((node) => (
          <div
            key={node.type}
            className="rf-chip"
            draggable
            title={`Drag ${node.label} to canvas`}
            onDragStart={(e) => {
              e.dataTransfer.setData("application/rf-node", node.type);
              e.dataTransfer.effectAllowed = "move";
            }}
          >
            {node.label}
          </div>
        ))}
      </div>
    </div>
  );
}
