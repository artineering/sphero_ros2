# ROS2 Expert Agent Guide

This workspace includes a persistent ROS2 expert agent that handles all ROS2-related tasks using a plan-first approach.

## Quick Start

To use the ROS2 expert agent, simply invoke it with:

```
/agent ros2_expert
```

Then describe your ROS2 task, for example:
- "Add a new node for obstacle avoidance"
- "Create a service to reset the Sphero's position"
- "Implement a launch file for multi-robot soccer game"
- "Debug why topics aren't publishing"
- "Add support for external localization integration"

## What the Agent Does

The ROS2 expert agent follows a strict workflow:

### 1. **Analyzes** your request
- Understands the task requirements
- Reviews existing code and architecture
- Identifies which packages are affected
- Considers single vs multi-robot implications

### 2. **Creates a detailed plan**
- Breaks down the task into clear steps
- Lists all files that will be created/modified
- Specifies commands to run
- Identifies potential risks
- Includes testing procedures

### 3. **Saves the plan**
- Stores in `plans/` directory
- Uses descriptive filename with timestamp
- Format: `plans/<task-name>-<timestamp>.md`

### 4. **Asks for your approval**
- Presents a summary of the plan
- Waits for your confirmation
- **Does NOT execute until you approve**

### 5. **Executes the plan** (only after approval)
- Follows the plan step-by-step
- Reports progress
- Handles issues that arise

### 6. **Reports results**
- Summarizes what was done
- Notes any deviations from the plan
- Provides testing commands

## Example Interaction

```
You: /agent ros2_expert

You: Add a service to check if a Sphero is currently moving

Agent: I'll create a plan for adding a "is_moving" service...
[Agent analyzes the request]

Agent: I've created a detailed plan saved at:
plans/add-is-moving-service-2026-03-07T14-30-00.md

The plan includes:
- Creating IsMoving.srv message definition
- Implementing service server in sphero_instance_device_controller_node.py
- Updating package.xml and CMakeLists.txt
- Build and test procedures

Shall I proceed with this plan?

You: Yes, proceed

Agent: [Executes the plan step by step...]
```

## Agent Expertise

The ROS2 expert agent specializes in:

- ✅ ROS2 node development (Python & C++)
- ✅ Topic/service/action communication
- ✅ Message and service definitions
- ✅ colcon build system
- ✅ Package creation and configuration
- ✅ Launch files
- ✅ Multi-robot coordination
- ✅ State machines and task executors
- ✅ Debugging ROS2 communication
- ✅ Integration with this workspace's architecture

## Workspace-Specific Knowledge

The agent understands:

- **Single vs Multi-robot packages** - Knows when to use `sphero_package` vs `sphero_instance_controller`
- **Namespacing conventions** - Flat (`/sphero/*`) vs per-instance (`sphero/<name>/*`)
- **Core architecture** - The `Sphero` class, task executors, state machines
- **Build system** - ament_python with colcon
- **Testing approaches** - Manual testing with ros2 commands
- **ArUco integration** - How localization fits into the system
- **Web interfaces** - How nodes integrate with web servers

## Plan Storage

All plans are saved in the `plans/` directory:

```
plans/
├── README.md
├── add-battery-service-2026-03-07T14-30-00.md
├── fix-namespace-issue-2026-03-07T15-20-00.md
└── implement-formation-control-2026-03-08T09-00-00.md
```

Benefits:
- **Traceability** - Complete history of changes
- **Review** - Catch issues before implementation
- **Documentation** - Plans document implementation decisions
- **Learning** - Reference past solutions

## Tips for Best Results

### Be Specific
❌ "Fix the robot"
✅ "Debug why SB-3660 is not responding to roll commands"

### Provide Context
❌ "Add a new feature"
✅ "Add collision avoidance using the existing collision detection callback"

### Specify Single vs Multi-robot
❌ "Add battery monitoring"
✅ "Add battery monitoring to the multi-robot package so each instance reports separately"

### Include Your Constraints
"Add a service for manual heading calibration, but don't modify the core Sphero class - keep it in the node"

## When to Use the Agent

Use the ROS2 expert agent for:

- ✅ Creating new ROS2 packages or nodes
- ✅ Modifying existing ROS2 nodes
- ✅ Adding message/service definitions
- ✅ Creating or modifying launch files
- ✅ Debugging topic/service communication
- ✅ Implementing new behaviors or state machines
- ✅ Multi-robot coordination features
- ✅ Integration work (ArUco, web interfaces, etc.)

**The agent handles the plan-first workflow automatically - you just describe what you need!**

## Reviewing Plans

Before approving a plan, check:

1. **Affected files** - Are the right files being modified?
2. **Dependencies** - Are new dependencies needed?
3. **Testing** - Is there a clear way to test the changes?
4. **Scope** - Is the plan doing too much or too little?
5. **Architecture** - Does it fit the existing patterns?

You can ask the agent to revise the plan before approving:
- "Can you avoid modifying the core Sphero class?"
- "Add more comprehensive testing steps"
- "Split this into two separate tasks"

## File Structure

```
.claude/
└── .agents/
    └── ros2_expert.md          # Agent definition

plans/
├── README.md                    # Plans directory documentation
└── [timestamped-plans].md      # Saved execution plans

ROS2_AGENT_GUIDE.md             # This file
CLAUDE.md                        # Workspace overview for AI
```

## Workflow Benefits

This plan-first approach ensures:

- 🎯 **Quality** - Thoughtful planning reduces bugs
- 🔍 **Visibility** - You see what will change before it happens
- 📚 **Documentation** - Plans document why changes were made
- 🛡️ **Safety** - Catch issues before code is modified
- 🎓 **Learning** - Understand the reasoning behind implementations

---

**Ready to get started?** Just type `/agent ros2_expert` and describe your ROS2 task!
