# Persistent Expert Agents

This workspace includes specialized persistent agents that follow a plan-first approach for their respective domains.

## Available Agents

### 1. ROS2 Expert Agent
**Invoke with:** `/agent ros2_expert`

**Specialization:**
- ROS2 node development (Python & C++)
- Topic/service/action communication
- Message and service definitions
- colcon build system
- Multi-robot coordination
- State machines and task executors
- Launch files
- ROS2 debugging

**Best for:**
- Creating/modifying ROS2 packages
- Adding nodes to existing packages
- Implementing publishers/subscribers
- Multi-robot coordination features
- State machine configurations
- Integration with web interfaces

**Documentation:** See [ROS2_AGENT_GUIDE.md](ROS2_AGENT_GUIDE.md)

---

### 2. Arduino Expert Agent
**Invoke with:** `/agent arduino_expert`

**Specialization:**
- Arduino Portenta C33
- Portenta UWB Shield
- Arduino Stella boards
- FreeRTOS and task schedulers
- Real-time embedded systems
- UWB positioning systems
- Sensor integration
- Motor control

**Best for:**
- FreeRTOS task implementations
- UWB ranging and positioning
- Sensor integration (I2C, SPI, UART)
- Low-power implementations
- Real-time control systems
- Arduino-ROS2 integration
- Hardware debugging

**Documentation:** See [ARDUINO_AGENT_GUIDE.md](ARDUINO_AGENT_GUIDE.md)

---

### 3. Web Application Expert Agent
**Invoke with:** `/agent web_expert`

**Specialization:**
- Flask web framework
- WebSocket/Socket.IO
- HTML5, CSS3, JavaScript (ES6+)
- Real-time dashboards
- RESTful APIs
- Responsive design
- ROS2-web integration
- Data visualization

**Best for:**
- Creating/modifying web interfaces
- Real-time data visualization
- WebSocket communication
- Dashboard development
- API endpoint implementation
- Mobile-responsive design
- Accessibility improvements
- Security enhancements

**Documentation:** See [WEB_AGENT_GUIDE.md](WEB_AGENT_GUIDE.md)

---

## Common Workflow (All Agents)

All expert agents follow the same rigorous workflow:

1. **Analyze** - Understand requirements and constraints
2. **Plan** - Create detailed step-by-step plan
3. **Save** - Store plan in `plans/<task-name>-<timestamp>.md`
4. **Approve** - Ask for user confirmation
5. **Execute** - Implement only after approval
6. **Report** - Summarize results and testing

## Why Use These Agents?

### Quality Assurance
- Thoughtful planning reduces bugs
- Safety checks before execution
- Proper resource allocation

### Visibility
- See what will change before it happens
- Review plans before approval
- Track implementation decisions

### Documentation
- Plans serve as implementation docs
- Timestamped history of changes
- Rationale for design decisions

### Learning
- Understand expert reasoning
- Review past solutions
- Build domain knowledge

## Plans Directory

All plans from all agents are stored in:
```
plans/
├── README.md
├── [ros2-task]-[timestamp].md
├── [arduino-task]-[timestamp].md
└── ...
```

Each plan includes:
- Task description and analysis
- Detailed step-by-step implementation
- Expected outcomes
- Testing procedures
- Approval status

## Quick Reference

| Task Type | Agent | Command |
|-----------|-------|---------|
| Add ROS2 node | ROS2 Expert | `/agent ros2_expert` |
| Create service definition | ROS2 Expert | `/agent ros2_expert` |
| Multi-robot coordination | ROS2 Expert | `/agent ros2_expert` |
| Launch files | ROS2 Expert | `/agent ros2_expert` |
| FreeRTOS task | Arduino Expert | `/agent arduino_expert` |
| UWB ranging | Arduino Expert | `/agent arduino_expert` |
| Sensor integration | Arduino Expert | `/agent arduino_expert` |
| Motor control | Arduino Expert | `/agent arduino_expert` |
| Web dashboard | Web Expert | `/agent web_expert` |
| Real-time visualization | Web Expert | `/agent web_expert` |
| WebSocket/API | Web Expert | `/agent web_expert` |
| Responsive UI | Web Expert | `/agent web_expert` |

## Example Usage

### ROS2 Task
```
/agent ros2_expert
Add a service to get the current battery percentage from a Sphero
```

### Arduino Task
```
/agent arduino_expert
Implement UWB two-way ranging between two Portenta C33 boards with FreeRTOS
```

### Web Application Task
```
/agent web_expert
Add a real-time battery graph to the Sphero web interface that updates via WebSocket
```

## Agent Interaction

You can ask agents to:
- **Revise plans** - "Can you use less RAM?"
- **Explain decisions** - "Why did you choose priority 3?"
- **Add details** - "Include more debugging steps"
- **Split tasks** - "Separate this into hardware test and full implementation"
- **Reject plans** - "Let's try a different approach"

Agents will update the plan and ask for approval again.

## Best Practices

### Be Specific
✅ "Add a FreeRTOS task to read IMU at 100Hz on Portenta C33"
❌ "Read a sensor"

### Provide Context
✅ "Implement UWB ranging for integration with ArUco SLAM system"
❌ "Add UWB"

### Include Constraints
✅ "Add battery monitoring to multi-robot package with namespaced topics"
❌ "Monitor battery"

### Mention Integration Points
✅ "Create Arduino code that publishes position to ROS2 via serial"
❌ "Get position from Arduino"

## File Structure

```
.claude/
└── .agents/
    ├── ros2_expert.md           # ROS2 expert definition
    ├── arduino_expert.md        # Arduino expert definition
    └── web_expert.md            # Web application expert definition

plans/
├── README.md                    # Plans directory docs
└── [task]-[timestamp].md        # Saved execution plans

AGENTS.md                        # This file
ROS2_AGENT_GUIDE.md             # Detailed ROS2 agent guide
ARDUINO_AGENT_GUIDE.md          # Detailed Arduino agent guide
WEB_AGENT_GUIDE.md              # Detailed web application agent guide
```

## When NOT to Use Agents

For simple queries that don't require code changes:
- ❌ "What does this function do?"
- ❌ "How do I compile this?"
- ❌ "What pin is the LED on?"

For these, just ask directly without invoking an agent.

Use agents for **implementation tasks** that will modify or create code/configurations.

---

**Get started:** Choose an agent above and describe your task!
