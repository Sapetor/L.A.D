# Remaining ROSFlow UI/UX Improvements

## Completed So Far
- ✓ Handle animations removed (static now)
- ✓ Handle labels only show on hover
- ✓ Field labels stay visible when inputs collapse
- ✓ Proper spacing between handles and content
- ✓ CreatePackage and RosRun accept external connections
- ✓ UrdfVisualNode updated with HandleWithLabel and accepts Geometry/Coordinates inputs

## Still TODO

### 1. Update UrdfCollisionNode
**File:** `src/components/blocks/UrdfCollisionNode.jsx`

Same pattern as UrdfVisualNode:
- Add imports: `useEffect, useStore, HandleWithLabel`
- Add input handles for `geometry` and `origin` (collapsible)
- Replace old Handle with HandleWithLabel
- Add useEffect to accept external Geometry and Coordinates nodes
- Show summary of geometry/origin when collapsed

### 2. Update UrdfInertialNode
**File:** `src/components/blocks/UrdfInertialNode.jsx`

- Add input handle for `origin` (collapsible)
- Keep mass and inertia tensor editable
- Accept Coordinates node for origin transform
- Use HandleWithLabel for all handles

### 3. Update UrdfJointNode
**File:** `src/components/blocks/UrdfJointNode.jsx`

Add input handles for:
- `origin` - accepts Coordinates node
- `axis` - accepts simple vector input (create AxisNode?)
- `parent` - accepts Text node for parent link name
- `child` - accepts Text node for child link name

Make these fields collapsible when connected, showing summaries.

### 4. Update All Flow Files to Use Categorized Palette
**Files to update:**
- `src/levels/slidesROSBasic/06-PackageCreatorFlow.jsx`
- `src/levels/slidesROSBasic/07-RunTurtlesimFlow.jsx`
- Any other flow files

**Changes:**
```jsx
// Instead of:
import { nodeTypes, paletteCreate } from "../../components/blocks";

// Use:
import { nodeTypes, paletteCategorized, CategorizedPalette } from "../../components/blocks";

// In render, replace the palette section with:
<CategorizedPalette
  categories={paletteCategorized}
  defaultCategory="ROS"
/>
```

This gives a consistent tabbed interface across ALL canvases with logical grouping:
- **Input** tab: Text, Lists, Coordinates, Geometry
- **ROS** tab: Create Package, ROS Run
- **URDF** tab: Inertial, Visual, Collision, Link, Joint, Assembly, Robot
- **Output** tab: Convert to Code, XML Preview
- **Visualization** tab: URDF Viewer

### 5. Handle Position Adjustments
For all URDF nodes, ensure handles align with their field labels:
- Measure approximate positions based on card body layout
- Adjust `top` prop on HandleWithLabel components
- Test with both collapsed and expanded states

### 6. Coordinates Node Update
**File:** `src/components/blocks/CoordinatesNode.jsx`

Currently outputs `{xyz, rpy}`. Some nodes might expect `{origin: {xyz, rpy}}`.
Update defaultDataFor to ensure compatibility or add data transformation in receiving nodes.

### 7. Create AxisNode (Optional but Recommended)
**New file:** `src/components/blocks/AxisNode.jsx`

Simple node that outputs an axis vector [x, y, z] for URDF joints.
```jsx
export default function AxisNode({ id, data }) {
  const [axis, setAxis] = useState(data.axis || [1, 0, 0]);
  // ... simple xyz inputs
  // Output: { axis: [x, y, z] }
}
```

Add to nodeTypes and paletteCategorized under "Input" category.

### 8. Testing Checklist
After implementing all changes:
- [ ] Can create full URDF robot using only node connections (no manual input)
- [ ] All handles align with labels
- [ ] Labels stay visible when fields collapse
- [ ] Handle labels appear on hover
- [ ] Tabbed palette works in all flow canvases
- [ ] All nodes follow consistent HandleWithLabel pattern
- [ ] External connections properly update node data
- [ ] Manual input still works when no connections

### 9. Documentation
Update CLAUDE.md with:
- New Coordinates and Geometry blocks
- Pattern for accepting external inputs
- How collapsible fields work
- Categorized palette usage

## Implementation Priority
1. **HIGH**: Categorized palette in all flows (best UX improvement)
2. **HIGH**: UrdfCollisionNode and UrdfInertialNode updates (completes Visual/Collision/Inertial trio)
3. **MEDIUM**: UrdfJointNode input handles (enables full modular URDF)
4. **MEDIUM**: AxisNode creation (nice to have for joints)
5. **LOW**: Fine-tuning handle positions (polish)

## Quick Reference: HandleWithLabel Props
```jsx
<HandleWithLabel
  type="target" | "source"
  position={Position.Left | Position.Right}
  id="handleId"
  label="labelText"
  color="blue" | "red" | "green" | "orange" | "purple" | "neon"
  top="percentage" // e.g., "30%"
  title="tooltip" // optional, defaults to label
/>
```

## Collapsible Field Pattern
```jsx
const isConnected = connectedHandles.includes("fieldName");

<div className={`rf-field--collapsible ${isConnected ? 'rf-field--collapsed' : ''}`}>
  <span className="rf-field__label">Field Name</span>
  <div className="rf-field__input-wrapper">
    {/* inputs go here - will collapse when connected */}
  </div>
</div>
```
