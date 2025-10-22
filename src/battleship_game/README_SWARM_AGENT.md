# Sphero Agent Node - OpenAI Swarm

## Overview

A ROS2 node that uses the OpenAI Swarm framework to control a Sphero robot via voice or text commands. The agent understands natural language and translates commands into Sphero control actions.

## Architecture

```
Voice/Text Input → Speech Recognition → OpenAI Swarm Agent → ROS2 Topics → Sphero Controller → Sphero
```

## Features

- **Natural Language Understanding**: Uses OpenAI Swarm for intelligent command interpretation
- **Voice Input**: Uses Google Speech Recognition (when microphone is available)
- **Text Fallback**: Falls back to text input if no microphone is detected
- **Function Calling**: Agent can call predefined Sphero control functions
- **Real-time Control**: Direct publishing to Sphero controller topics

## Prerequisites

### 1. OpenAI API Key

```bash
export OPENAI_API_KEY='your-openai-api-key'
```

Add to `~/.bashrc` for persistence:
```bash
echo 'export OPENAI_API_KEY="your-key-here"' >> ~/.bashrc
source ~/.bashrc
```

### 2. Dependencies

Already installed if you followed the setup:
- `openai` - OpenAI Python SDK
- `openai-swarm` - Swarm agentic framework
- `speechrecognition` - Google Speech Recognition

### 3. Optional: Microphone Setup

For voice input (optional):
```bash
# Install PortAudio system dependencies
sudo apt-get install portaudio19-dev

# Install PyAudio
pip install --break-system-packages pyaudio
```

**Note**: Voice input is optional. The node works with text input if no microphone is available.

## Usage

### Running the System

**Terminal 1: Sphero Controller**
```bash
source ~/ros2_ws_2/install/setup.bash
ros2 run battleship_game sphero_controller_node
```

**Terminal 2: Sphero Agent**
```bash
source ~/ros2_ws_2/install/setup.bash
export OPENAI_API_KEY='your-key-here'
ros2 run battleship_game sphero_agent_node
```

### Example Commands

The agent understands natural language:

**LED Control:**
- "Turn the LED red"
- "Make the light blue"
- "Set LED to green"
- "Change color to purple" (agent will figure out RGB values)

**Movement:**
- "Move forward"
- "Roll forward at speed 100"
- "Go in direction 90 degrees"
- "Move forward slowly"

**Rotation:**
- "Spin around"
- "Spin 360 degrees"
- "Turn clockwise"
- "Rotate counterclockwise"

**Stop:**
- "Stop"
- "Stop moving"
- "Halt"

**Complex Commands:**
- "Turn the LED blue and move forward"
- "Spin twice then stop"

## Available Functions

The Swarm agent has access to these functions:

### 1. `set_led_color(red, green, blue)`
Control Sphero's main LED.

### 2. `roll_sphero(heading, speed, duration)`
Roll in a specific direction.
- `heading`: 0-359 degrees
- `speed`: 0-255
- `duration`: seconds (0 = continuous)

### 3. `spin_sphero(angle, duration)`
Spin by a specific angle.
- `angle`: degrees (positive = clockwise)
- `duration`: seconds

### 4. `set_heading(heading)`
Set movement direction.
- `heading`: 0-359 degrees

### 5. `set_speed(speed, duration)`
Set movement speed.
- `speed`: 0-255
- `duration`: seconds (0 = continuous)

### 6. `stop_sphero()`
Stop all movement.

### 7. `display_matrix_pattern(pattern, red, green, blue, duration)`
Display LED matrix pattern (BOLT only).
- `pattern`: smile, cross, arrow_up
- RGB values: 0-255
- `duration`: seconds

## How It Works

### 1. Input Processing
- **With Microphone**: Captures audio and uses Google Speech Recognition
- **Without Microphone**: Uses text input from terminal

### 2. Agent Processing
- Command sent to OpenAI Swarm agent
- Agent analyzes the natural language
- Agent decides which function(s) to call
- Agent executes function calls

### 3. Sphero Control
- Functions publish JSON messages to ROS2 topics
- sphero_controller_node receives and executes commands
- Sphero performs the action

## Configuration

The agent is configured in the code:

```python
agent = Agent(
    name="Sphero Controller",
    instructions="You are a helpful assistant that controls a Sphero robot...",
    functions=[
        set_led_color,
        roll_sphero,
        spin_sphero,
        # ... more functions
    ]
)
```

## Troubleshooting

### "OPENAI_API_KEY not set"
```bash
export OPENAI_API_KEY='your-key'
echo $OPENAI_API_KEY  # Verify it's set
```

### "Audio not available"
This is expected if no microphone is detected. The agent will use text input instead. This is normal and the agent will work fine.

### "Speech recognition error"
Make sure you have internet connection (Google Speech Recognition is online).

### Agent not responding
- Check OpenAI API key is valid
- Verify you have API credits/quota
- Check internet connection

### Sphero not responding
- Verify sphero_controller_node is running
- Check Sphero is connected (see controller logs)
- Try manual commands first to test connection

## Example Session

```
$ ros2 run battleship_game sphero_agent_node

[INFO] Sphero Agent Node initialized
[INFO] Audio input available: False
[INFO] Agent ready to accept commands
[INFO] Starting interactive mode
[INFO] Say or type commands like:
[INFO]   - "Turn the LED red"
[INFO]   - "Move forward at speed 100"
[INFO]   - "Spin around"
[INFO]   - "Stop"
[INFO] Type "quit" to exit

Enter command: turn the LED green
[INFO] Processing: turn the LED green
[INFO] LED set to RGB(0, 255, 0)
[INFO] Agent: LED set to red=0, green=255, blue=0

Enter command: move forward slowly
[INFO] Processing: move forward slowly
[INFO] Rolling at 0deg, speed 50
[INFO] Agent: Rolling at 0 degrees with speed 50

Enter command: stop
[INFO] Processing: stop
[INFO] Sphero stopped
[INFO] Agent: Sphero stopped

Enter command: quit
[INFO] Exiting...
```

## ROS2 Topics

### Published

- `sphero/led` (std_msgs/String) - LED commands
- `sphero/roll` (std_msgs/String) - Roll commands
- `sphero/spin` (std_msgs/String) - Spin commands
- `sphero/heading` (std_msgs/String) - Heading commands
- `sphero/speed` (std_msgs/String) - Speed commands
- `sphero/stop` (std_msgs/String) - Stop commands
- `sphero/matrix` (std_msgs/String) - Matrix display commands
- `sphero_agent/response` (std_msgs/String) - Agent responses
- `sphero_agent/status` (std_msgs/String) - Agent status

## Architecture Details

### OpenAI Swarm

Swarm is OpenAI's lightweight multi-agent orchestration framework. Key concepts:

- **Agent**: An entity with instructions and access to functions
- **Functions**: Python functions the agent can call
- **Client**: Manages agent execution and OpenAI API calls

### Advantages

- **Simple**: Minimal code, clear function definitions
- **Flexible**: Easy to add new capabilities by defining new functions
- **Powerful**: Leverages GPT-4 for natural language understanding
- **Composable**: Can coordinate multiple agents (future enhancement)

## Future Enhancements

- [ ] Multi-agent coordination (e.g., game strategy agent + movement agent)
- [ ] Context retention across commands
- [ ] Voice output (text-to-speech responses)
- [ ] Custom wake word detection
- [ ] Integration with game controller for autonomous gameplay
- [ ] Batch command execution
- [ ] Macro/routine creation via conversation

## License

TODO: License declaration
