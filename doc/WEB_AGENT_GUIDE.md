# Web Application Expert Agent Guide

This workspace includes a persistent web application expert agent that handles all web development tasks using a plan-first approach, with special focus on Flask, Socket.IO, real-time dashboards, and ROS2 integration.

## Quick Start

To use the web application expert agent, invoke it with:

```
/agent web_expert
```

Then describe your web development task, for example:
- "Add a real-time battery graph to the Sphero web interface"
- "Create a new dashboard for monitoring multiple robots"
- "Implement a form for configuring state machine parameters"
- "Add WebSocket reconnection handling to the multi-robot interface"
- "Create a mobile-responsive control panel for Sphero movement"

## What the Agent Does

The web application expert agent follows a strict workflow:

### 1. **Analyzes** your request
- Evaluates UI/UX requirements
- Considers backend integration needs
- Assesses real-time communication requirements
- Reviews accessibility and responsiveness

### 2. **Creates a detailed plan**
- UI/UX wireframes or descriptions
- API endpoints and WebSocket events
- Data flow diagrams
- HTML/CSS/JavaScript changes
- Security and performance considerations

### 3. **Saves the plan**
- Stores in `plans/` directory
- Includes mockups and API specifications
- Format: `plans/<task-name>-<timestamp>.md`

### 4. **Asks for your approval**
- Presents plan with UI mockups
- Highlights integration points
- Notes browser compatibility
- **Does NOT execute until you approve**

### 5. **Executes the plan** (only after approval)
- Creates/modifies HTML, CSS, JavaScript
- Implements Flask routes and WebSocket handlers
- Adds ROS2 integration
- Includes error handling and validation

### 6. **Reports results**
- Provides testing instructions
- Notes browser compatibility
- Includes screenshots or descriptions
- Lists endpoints to test

## Example Interaction

```
You: /agent web_expert

You: Add a real-time position map showing Sphero location on a 2D canvas

Agent: I'll create a plan for a real-time position visualization...
[Agent analyzes requirements]

Agent: I've created a detailed plan saved at:
plans/add-position-map-canvas-2026-03-07T16-45-00.md

The plan includes:
- HTML5 Canvas element (600x400px, scaled to field dimensions)
- JavaScript to render robot position and trail
- WebSocket event 'position_update' for real-time data
- Subscribe to /aruco_slam/<robot>/position ROS2 topic
- Grid overlay with coordinate labels
- Color-coded robot markers
- Performance: 30 FPS max, debounced updates

UI placement: New "Map" tab in web interface

Browser compatibility: Chrome 90+, Firefox 88+, Safari 14+

Shall I proceed with this plan?

You: Yes, proceed

Agent: [Executes the plan step by step...]
```

## Agent Expertise

The web application expert agent specializes in:

### Frontend
- ✅ **HTML5** - Semantic markup, Canvas, SVG
- ✅ **CSS3** - Flexbox, Grid, animations, responsive design
- ✅ **JavaScript** - ES6+, async/await, event handling
- ✅ **WebSocket/Socket.IO** - Real-time communication
- ✅ **Chart.js/D3.js** - Data visualization
- ✅ **Bootstrap/Tailwind** - CSS frameworks

### Backend
- ✅ **Flask** - Python web framework
- ✅ **Flask-SocketIO** - WebSocket integration
- ✅ **RESTful APIs** - Endpoint design
- ✅ **Jinja2 templates** - Server-side rendering
- ✅ **Session management** - Authentication

### Integration
- ✅ **ROS2 integration** - Web interfaces for ROS2 nodes
- ✅ **Real-time dashboards** - Live data visualization
- ✅ **Multi-client architecture** - Concurrent users
- ✅ **State synchronization** - Client-server sync
- ✅ **Error handling** - User-friendly messages

### Best Practices
- ✅ **Security** - XSS prevention, CSRF protection, input validation
- ✅ **Accessibility** - WCAG 2.1 AA, keyboard navigation, ARIA
- ✅ **Performance** - Lazy loading, caching, optimization
- ✅ **Responsive design** - Mobile-first, media queries
- ✅ **Browser compatibility** - Cross-browser testing

## Workspace-Specific Knowledge

The agent understands the existing web interfaces:

### sphero_web_interface
- **Port:** 5000
- **Framework:** Flask + Socket.IO (as ROS2 node)
- **Structure:**
  - `web_server_node.py` - Flask app + ROS2 integration
  - `templates/index.html` - Tabbed interface
  - `static/css/style.css` - Styling
  - `static/js/app.js` - WebSocket handling
- **Features:** Connection, state, sensors, matrix, motion tabs

### multirobot_webserver
- **Port:** 5000 (main), 5001+ (per-robot WebSocket)
- **Framework:** Standalone Flask + multiple WebSocket servers
- **Architecture:** Central dashboard managing multiple instances
- **Pattern:** Dynamic robot addition/removal

## Plan Storage

All plans saved in `plans/` directory:

```
plans/
├── README.md
├── add-battery-graph-2026-03-07T16-30-00.md
├── create-multi-robot-dashboard-2026-03-07T17-00-00.md
└── implement-state-machine-editor-2026-03-08T10-00-00.md
```

Each plan includes:
- UI wireframes/mockups
- API endpoint specifications
- WebSocket event definitions
- Data flow diagrams
- Security considerations
- Browser compatibility notes

## Tips for Best Results

### Be Specific About UI
❌ "Add a graph"
✅ "Add a line graph showing battery percentage over the last 5 minutes, updating every 2 seconds via WebSocket"

### Include Integration Details
❌ "Show robot state"
✅ "Display robot state from /sphero/state topic in a card layout with color-coded status indicators"

### Specify User Interaction
❌ "Add controls"
✅ "Add a directional pad (up/down/left/right buttons) that sends roll commands via WebSocket, with visual feedback on button press"

### Mention Responsiveness
❌ "Create an interface"
✅ "Create a mobile-responsive interface with collapsible sidebar on screens < 768px"

## When to Use the Agent

Use the web application expert agent for:

- ✅ Creating new web interfaces
- ✅ Adding features to existing web apps
- ✅ Implementing real-time visualizations
- ✅ WebSocket/Socket.IO communication
- ✅ RESTful API endpoints
- ✅ Forms and user input handling
- ✅ Responsive layout design
- ✅ Dashboard creation
- ✅ State management (client/server)
- ✅ Error handling and UX improvements
- ✅ ROS2-web integration
- ✅ Accessibility enhancements
- ✅ Performance optimization

## Reviewing Plans

Before approving a web development plan, check:

1. **UI/UX** - Is the interface intuitive and user-friendly?
2. **Security** - Are inputs validated and outputs sanitized?
3. **Performance** - Will it handle expected load? Any bottlenecks?
4. **Accessibility** - Keyboard navigation? Screen reader support?
5. **Responsiveness** - Works on mobile/tablet/desktop?
6. **Browser compatibility** - Supported browsers clear?
7. **Integration** - ROS2 topics/WebSocket events correct?
8. **Error handling** - User-friendly error messages?

You can ask for revisions:
- "Can you add dark mode support?"
- "Make the graph more interactive with zoom/pan"
- "Add more detailed error messages"
- "Include a loading indicator"

## Common Web Development Patterns

### Flask + ROS2 Integration

**Pattern A: Flask as ROS2 Node**
```python
class WebServerNode(Node):
    def __init__(self):
        super().__init__('web_server')
        # ROS2 subscribers/publishers

        # Flask app
        self.app = Flask(__name__)
        self.socketio = SocketIO(self.app)

        # Background thread for ROS2 spinning
        threading.Thread(target=self.ros_spin).start()
```

**Pattern B: Standalone Flask + ROS2 Bridge**
- Separate processes
- Flask publishes to ROS2 topics via subprocess or socket

### WebSocket Real-Time Updates

**Server-side:**
```python
@app.route('/api/status')
def get_status():
    return jsonify({'status': current_status})

@socketio.on('send_command')
def handle_command(data):
    # Publish to ROS2
    publish_to_ros2(data)

# ROS2 callback
def ros_callback(msg):
    socketio.emit('state_update', msg.data)
```

**Client-side:**
```javascript
const socket = io();

socket.on('connect', function() {
    console.log('Connected');
});

socket.on('state_update', function(data) {
    updateUI(data);
});

function sendCommand(cmd) {
    socket.emit('send_command', {command: cmd});
}
```

### Responsive Design Pattern

```css
/* Mobile-first approach */
.container {
    padding: 1rem;
}

/* Tablet and up */
@media (min-width: 768px) {
    .container {
        padding: 2rem;
        display: grid;
        grid-template-columns: 250px 1fr;
    }
}

/* Desktop and up */
@media (min-width: 1024px) {
    .container {
        max-width: 1200px;
        margin: 0 auto;
    }
}
```

## Integration with ROS2 Workspace

Web interfaces connect to:
- **sphero_package** - Single robot control
- **sphero_instance_controller** - Multi-robot with namespaced topics
- **sphero_statemachine** - State machine configuration
- **aruco_slam** - Position data visualization
- **multirobot_webserver** - Central coordination

The agent understands these integration points and can plan accordingly.

## Security Best Practices

The agent ensures:
- ✅ Input validation (server-side)
- ✅ Output sanitization (prevent XSS)
- ✅ CSRF tokens for state-changing operations
- ✅ Rate limiting on API endpoints
- ✅ Secure WebSocket connections (wss:// in production)
- ✅ No sensitive data in client-side code
- ✅ Proper CORS configuration

## Performance Best Practices

The agent implements:
- ✅ Asset minification and compression
- ✅ Lazy loading of images/scripts
- ✅ Debouncing/throttling of frequent events
- ✅ Efficient DOM manipulation
- ✅ WebSocket message optimization
- ✅ Caching strategies
- ✅ CDN usage for libraries

## Accessibility Best Practices

The agent ensures:
- ✅ Semantic HTML elements
- ✅ ARIA labels and roles
- ✅ Keyboard navigation support
- ✅ Color contrast compliance (WCAG AA)
- ✅ Focus indicators
- ✅ Alt text for images
- ✅ Form labels and error messages
- ✅ Screen reader compatibility

## File Structure

```
.claude/
└── .agents/
    ├── ros2_expert.md           # ROS2 agent
    ├── arduino_expert.md        # Arduino agent
    └── web_expert.md            # Web agent (this one)

plans/
├── README.md
└── [task]-[timestamp].md        # All agent plans

ROS2_AGENT_GUIDE.md
ARDUINO_AGENT_GUIDE.md
WEB_AGENT_GUIDE.md              # This file
AGENTS.md                        # Master agent list
```

## Workflow Benefits

This plan-first approach ensures:

- 🎨 **Better UX** - Thoughtful design before implementation
- 🔒 **Security** - Security review before coding
- ⚡ **Performance** - Optimization planned upfront
- ♿ **Accessibility** - Inclusive design from start
- 📱 **Responsiveness** - Mobile-first approach
- 🌐 **Compatibility** - Cross-browser testing planned
- 📚 **Documentation** - API specs and mockups preserved
- 🐛 **Fewer bugs** - Edge cases considered early

---

**Ready to get started?** Just type `/agent web_expert` and describe your web development task!
