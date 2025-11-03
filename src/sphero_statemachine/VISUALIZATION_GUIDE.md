# State Machine Visualization Guide

The Sphero State Machine web interface now includes **real-time visual diagram preview** powered by Mermaid.js!

## Features

### 🎨 Visual State Diagram Preview
- See your state machine configuration as an interactive diagram
- Automatically updates as you edit the JSON configuration
- Shows states, transitions, entry conditions, and tasks

### 🔄 Auto-Preview Mode
- **Auto-Preview ON** (default): Diagram updates automatically as you type (with 500ms debounce)
- **Auto-Preview OFF**: Manually click "Preview Diagram" to update

### 📊 Diagram Information
Each state in the diagram shows:
- **State name** and description
- **Entry condition** type (always, timer, topic_value, etc.)
- **Task type** to be executed
- **Transitions** with trigger labels

## How to Use

### 1. Access the Web Interface
```bash
ros2 run sphero_web_interface web_server_node
```
Open browser to: `http://localhost:5000/state_machine`

### 2. Load a Template or Enter Configuration
- Select a template from the dropdown (Simple, Patrol, Timed, or Sensor-Based)
- Or manually enter your JSON configuration in the text editor
- The diagram will automatically appear in the "Visual Preview" panel

### 3. Toggle Auto-Preview
- Click the "Auto-Preview: ON/OFF" button to toggle automatic updates
- When OFF, click "Preview Diagram" to manually refresh

### 4. Understand the Diagram

**Example Diagram Elements:**

```
┌─────────────────┐
│  [*] (start)    │  ← Initial state marker
└────────┬────────┘
         │
         ▼
   ┌─────────┐
   │  idle   │        ← State name
   │  Blue   │        ← Description
   └─────────┘
   Entry: timer     ← Entry condition
   Task: set_led    ← Task to execute
         │
         │ auto      ← Transition trigger
         ▼
   ┌─────────┐
   │ active  │
   └─────────┘
```

## Example Configurations

### Simple Two-State Machine
```json
{
  "name": "Simple Two-State Machine",
  "initial_state": "idle",
  "states": [
    {
      "name": "idle",
      "description": "Robot is idle with blue LED",
      "entry_condition": {
        "type": "timer",
        "params": {"duration": 3.0}
      },
      "task": {
        "type": "set_led",
        "params": {"color": "blue"}
      }
    },
    {
      "name": "active",
      "description": "Robot is active with green LED",
      "entry_condition": {
        "type": "timer",
        "params": {"duration": 3.0}
      },
      "task": {
        "type": "set_led",
        "params": {"color": "green"}
      }
    }
  ],
  "transitions": [
    {
      "source": "idle",
      "destination": "active",
      "trigger": "auto"
    },
    {
      "source": "active",
      "destination": "idle",
      "trigger": "auto"
    }
  ]
}
```

This will generate a diagram showing:
- Initial marker pointing to "idle"
- Two states with their descriptions
- Entry conditions (timer: 3.0s for both)
- Tasks (LED colors)
- Bidirectional transitions

### Patrol Pattern
The patrol pattern creates a circular diagram showing the robot moving through 4 cardinal directions (North, East, South, West) in sequence.

## Diagram Interpretation

### State Nodes
- **Rectangle**: Represents a state
- **Text inside**: State name and description
- **Note attached**: Entry condition and task type

### Transitions
- **Arrow**: Shows possible state transitions
- **Label on arrow**: Transition trigger (usually "auto" for automatic transitions)

### Initial State
- **[*] marker**: Shows which state the machine starts in

## Tips

### For Complex State Machines
- Use clear, descriptive state names
- Keep descriptions concise (they appear in the diagram)
- Use meaningful trigger names for transitions

### Troubleshooting
- **Diagram not appearing?** Check that your JSON is valid (use "Validate JSON" button)
- **Diagram looks cluttered?** Consider simplifying state descriptions
- **Changes not showing?** Ensure Auto-Preview is ON, or click "Preview Diagram"

## Technical Details

### Mermaid.js Integration
- Uses Mermaid.js v10 via CDN
- Generates `stateDiagram-v2` syntax
- Renders as SVG for scalability
- Supports zoom and pan in browser

### Conversion Process
1. Parse JSON configuration
2. Extract states, transitions, and metadata
3. Generate Mermaid state diagram syntax
4. Render to SVG diagram
5. Display in preview panel

### Auto-Update Mechanism
- Debounced input handler (500ms delay)
- Prevents excessive re-rendering while typing
- Validates JSON before attempting to render

## Benefits

✅ **Visual Validation** - Quickly spot configuration errors
✅ **Better Understanding** - See the flow of your state machine
✅ **Documentation** - Diagrams serve as visual documentation
✅ **Debugging** - Easier to identify logic issues
✅ **Sharing** - Clear visual representation for team communication

## Future Enhancements

Potential future features:
- [ ] Export diagram as PNG/SVG
- [ ] Highlight current state in real-time
- [ ] Drag-and-drop visual editor
- [ ] State timing annotations
- [ ] Transition condition details
- [ ] Zoom controls

---

**🤖 Generated with Claude Code**
