# Arduino Expert Agent Guide

This workspace includes a persistent Arduino expert agent that handles all Arduino/embedded systems tasks using a plan-first approach, with special focus on Arduino Portenta C33, Portenta UWB Shield, Arduino Stella, and FreeRTOS development.

## Quick Start

To use the Arduino expert agent, invoke it with:

```
/agent arduino_expert
```

Then describe your Arduino/embedded task, for example:
- "Implement UWB ranging between two Portenta C33 boards"
- "Create a FreeRTOS task to read IMU data at 100Hz"
- "Setup SPI communication with a custom sensor"
- "Implement low-power mode with periodic wake-ups"
- "Create a motor control system with PWM on Portenta C33"

## What the Agent Does

The Arduino expert agent follows a strict workflow:

### 1. **Analyzes** your request
- Evaluates hardware requirements and constraints
- Considers real-time timing requirements
- Checks pin availability and conflicts
- Assesses power and resource requirements

### 2. **Creates a detailed plan**
- Hardware setup with pin assignments
- FreeRTOS task structure with priorities
- Timing diagrams and state machines
- Resource allocation (RAM, flash, stack sizes)
- Safety and error handling

### 3. **Saves the plan**
- Stores in `plans/` directory
- Includes hardware diagrams and specifications
- Format: `plans/<task-name>-<timestamp>.md`

### 4. **Asks for your approval**
- Presents plan with pin assignments
- Highlights timing constraints
- Notes power/resource usage
- **Does NOT execute until you approve**

### 5. **Executes the plan** (only after approval)
- Creates/modifies Arduino sketches
- Implements FreeRTOS tasks
- Configures hardware peripherals
- Adds debug outputs

### 6. **Reports results**
- Provides compilation and upload instructions
- Includes testing procedures
- Notes expected behavior and debug outputs

## Example Interaction

```
You: /agent arduino_expert

You: Implement a FreeRTOS-based UWB ranging system on Portenta C33 with UWB shield

Agent: I'll create a plan for UWB ranging with FreeRTOS...
[Agent analyzes hardware and timing requirements]

Agent: I've created a detailed plan saved at:
plans/uwb-ranging-freertos-2026-03-07T15-30-00.md

The plan includes:
- Pin assignments for UWB shield SPI connection
- 4 FreeRTOS tasks with priorities (ranging, calculation, display, logging)
- DW3000 initialization and configuration
- Two-Way Ranging (TWR) implementation
- Queue-based inter-task communication
- Stack sizes: 512 words per task (validated)
- Expected ranging accuracy: ±10cm

Hardware requirements:
- Portenta C33 + UWB Shield
- SPI pins: COPI/CIPO/SCK on standard header
- Interrupt pin: D2
- Power: 3.3V, ~150mA during ranging

Shall I proceed with this plan?

You: Yes, proceed

Agent: [Executes the plan step by step...]
```

## Agent Expertise

The Arduino expert agent specializes in:

### Hardware Platforms
- ✅ **Arduino Portenta C33** - Renesas RA6M5, 200MHz Cortex-M33
- ✅ **Portenta UWB Shield** - DW3000 ultra-wideband positioning
- ✅ **Arduino Stella** - STM32H7 dual-core, edge AI
- ✅ General Arduino boards (Uno, Mega, Due, etc.)

### Software/RTOS
- ✅ **FreeRTOS** - Tasks, queues, semaphores, mutexes, timers
- ✅ **Mbed OS** - RTOS for Cortex-M
- ✅ **CMSIS-RTOS** - ARM RTOS standard
- ✅ Bare-metal programming
- ✅ Arduino framework and libraries

### Protocols & Interfaces
- ✅ **UWB** - Two-Way Ranging, TDoA, positioning
- ✅ **WiFi/Bluetooth** - Wireless communication
- ✅ **I2C/SPI/UART** - Serial protocols
- ✅ **CAN bus** - Industrial/automotive
- ✅ **USB** - Device and host modes
- ✅ **ADC/DAC/PWM** - Analog I/O

### Applications
- ✅ Sensor fusion and data acquisition
- ✅ Motor control and robotics
- ✅ Indoor positioning systems
- ✅ Low-power IoT devices
- ✅ Real-time control systems
- ✅ Machine learning on edge devices

## Hardware-Specific Knowledge

### Portenta C33 Specs
- **MCU:** Renesas RA6M5 (Cortex-M33, 200 MHz)
- **RAM:** 512 KB SRAM
- **Flash:** 2 MB
- **Connectivity:** WiFi 802.11b/g/n, Bluetooth 5.0
- **Security:** Hardware crypto (AES, SHA, TRNG)
- **Peripherals:** Rich set of timers, ADC, DAC, SPI, I2C, UART
- **Power modes:** Multiple low-power modes

### Portenta UWB Shield
- **Chip:** DW3000 UWB transceiver
- **Interface:** SPI
- **Ranging accuracy:** ±10cm typical
- **Channels:** 5 and 9 (6.5 GHz, 8 GHz)
- **Range:** 30-50m typical (environment dependent)
- **Antenna delay:** Requires calibration

### Arduino Stella
- **MCU:** STM32H747 (Cortex-M7 480MHz + Cortex-M4 240MHz)
- **RAM:** 1 MB
- **Flash:** 2 MB
- **AI acceleration:** Chrom-ART for ML
- **Motor drivers:** Built-in
- **CAN:** For industrial applications

## Plan Storage

All plans saved in `plans/` directory with Arduino-specific information:

```
plans/
├── README.md
├── uwb-ranging-freertos-2026-03-07T15-30-00.md
├── imu-sensor-fusion-2026-03-07T16-15-00.md
└── motor-control-pid-2026-03-08T10-00-00.md
```

Each plan includes:
- Pin assignment tables
- Task priority diagrams
- Timing analysis
- Power consumption estimates
- Stack size calculations

## Tips for Best Results

### Be Specific About Hardware
❌ "Read a sensor"
✅ "Read MPU6050 IMU via I2C on Portenta C33 at 100Hz using FreeRTOS task"

### Include Real-Time Requirements
❌ "Create a task"
✅ "Create a FreeRTOS task with priority 3 that samples ADC every 10ms with <1ms jitter"

### Specify Hardware Constraints
❌ "Add UWB ranging"
✅ "Add UWB ranging using DW3000 on UWB shield, SPI at 10MHz, interrupt-driven, with power-saving between ranges"

### Include Integration Context
"Implement UWB positioning that publishes to ROS2 via serial bridge at 10Hz"

## When to Use the Agent

Use the Arduino expert agent for:

- ✅ FreeRTOS task implementations
- ✅ UWB ranging and positioning systems
- ✅ Sensor integration (I2C, SPI, UART)
- ✅ Motor control and PWM
- ✅ Low-power implementations
- ✅ Real-time control systems
- ✅ Wireless communication (WiFi, BLE, UWB)
- ✅ Interrupt-driven architectures
- ✅ DMA configurations
- ✅ State machine implementations
- ✅ Arduino-ROS2 integration
- ✅ Hardware debugging strategies

## Reviewing Plans

Before approving an Arduino plan, check:

1. **Pin assignments** - No conflicts, correct voltage levels
2. **Stack sizes** - Adequate for task requirements (with margin)
3. **Task priorities** - Logical priority assignment
4. **Timing** - Real-time constraints met
5. **Power** - Current draw within limits
6. **Safety** - Watchdog, error handling, safe states
7. **Testing** - Clear validation procedures
8. **Debug outputs** - Serial or LED indicators included

You can ask for revisions:
- "Can you reduce RAM usage?"
- "Add more detailed timing analysis"
- "Include oscilloscope checkpoints for debugging"
- "Split this into hardware test first, then full implementation"

## FreeRTOS Development Focus

The agent excels at FreeRTOS patterns:

### Task Management
```cpp
// High-priority sensor task
xTaskCreate(sensorTask, "Sensor", 512, NULL, 3, &sensorHandle);

// Medium-priority processing task
xTaskCreate(processingTask, "Process", 1024, NULL, 2, &processHandle);

// Low-priority logging task
xTaskCreate(logTask, "Log", 256, NULL, 1, &logHandle);
```

### Inter-Task Communication
- Queues for data passing
- Semaphores for synchronization
- Mutexes for resource sharing
- Event groups for multiple conditions

### Timing Precision
- `vTaskDelayUntil()` for periodic tasks
- Software timers for callbacks
- Hardware timers for critical timing

## Integration with ROS2

Arduino code can integrate with this ROS2 workspace via:
- **micro-ROS** - Full ROS2 on microcontroller
- **Serial bridge** - Custom protocol over UART
- **WiFi/Ethernet** - Network communication
- **External localization** - Arduino provides position data to ROS2

The agent understands these integration patterns and can plan accordingly.

## File Structure

```
.claude/
└── .agents/
    ├── ros2_expert.md           # ROS2 agent
    └── arduino_expert.md        # Arduino agent (this one)

plans/
├── README.md
└── [timestamped-plans].md      # Both ROS2 and Arduino plans

ROS2_AGENT_GUIDE.md             # ROS2 agent guide
ARDUINO_AGENT_GUIDE.md          # This file
```

## Workflow Benefits

This plan-first approach ensures:

- 🔒 **Safety** - Hardware checks before power-on
- ⏱️ **Timing** - Real-time constraints validated upfront
- 📐 **Resource planning** - RAM/flash usage calculated
- 🐛 **Debuggability** - Debug strategy included in plan
- ⚡ **Power efficiency** - Power budget considered
- 📚 **Documentation** - Pin assignments and timing documented
- 🎯 **Testability** - Clear validation procedures

---

**Ready to get started?** Just type `/agent arduino_expert` and describe your embedded systems task!
