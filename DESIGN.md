# Battleship Game - Design Document

## Project Overview

**Project Name:** Battleship Game with Sphero Robots
**Version:** 0.0.0
**Platform:** ROS2 (Robot Operating System 2)
**Language:** Python
**Maintainer:** Siddharth (siddharth.vaghela@tufts.edu)

### Description
A ROS2-based implementation of the classic Battleship game that enables gameplay between human players and Sphero robots. The system uses a distributed node architecture to manage game logic, human player interactions, and Sphero robot control.

---

## System Architecture

### High-Level Architecture
The system follows a distributed node-based architecture pattern common in ROS2 applications, with three primary components communicating via ROS2 topics and services.

```
┌─────────────────────┐
│  Game Controller    │
│      Node           │
│  (Central Logic)    │
└──────────┬──────────┘
           │
    ┌──────┴──────┐
    │             │
    ▼             ▼
┌───────────┐ ┌──────────────┐
│  Human    │ │   Sphero     │
│Controller │ │  Controller  │
│   Node    │ │     Node     │
└───────────┘ └──────────────┘
```

### Component Overview

#### 1. Game Controller Node
**File:** [battleship_game/game_controller_node.py](src/battleship_game/battleship_game/game_controller_node.py)

**Responsibilities:**
- Maintains the game state (boards, ship placements, hit/miss tracking)
- Implements core battleship game logic
- Validates moves and determines game outcomes
- Coordinates turn-based gameplay
- Publishes game state updates
- Manages win/loss conditions

**Key Features:**
- Centralized game state management
- Rule enforcement
- Turn management
- Score tracking
- Game session lifecycle management

#### 2. Human Controller Node
**File:** [battleship_game/human_controller_node.py](src/battleship_game/battleship_game/human_controller_node.py)

**Responsibilities:**
- Provides human player interface (CLI or GUI)
- Accepts user input for ship placement and attack coordinates
- Displays game board state to the human player
- Communicates player actions to the game controller
- Receives and displays game state updates

**Key Features:**
- User input validation
- Board visualization
- Move submission
- Feedback display (hits, misses, ships sunk)

#### 3. Sphero Controller Node
**File:** [battleship_game/sphero_controller_node.py](src/battleship_game/battleship_game/sphero_controller_node.py)

**Responsibilities:**
- Interfaces with Sphero robots via spherov2 library
- Implements AI or automated gameplay logic
- Sends move commands to game controller
- Receives game state and provides visual/physical feedback via Sphero
- Controls Sphero movement, lights, and sensors

**Key Features:**
- Sphero robot communication
- AI decision-making for ship placement and attacks
- Visual feedback (LED colors for hits/misses)
- Motion feedback for game events
- Bluetooth connection management

---

## Technology Stack

### Core Dependencies
- **ROS2:** Framework for distributed node communication
- **rclpy:** ROS2 Python client library
- **spherov2:** Python library for Sphero robot control
- **Python 3:** Primary programming language

### Development Dependencies
- **pytest:** Unit testing framework
- **ament_copyright:** Copyright compliance checking
- **ament_flake8:** Python linting
- **ament_pep257:** Docstring compliance
- **ament_xmllint:** XML validation

### Build System
- **ament_python:** ROS2 Python package build system
- **setuptools:** Python package management

---

## Game Logic Design

### Board Representation
```python
# 10x10 grid (standard battleship)
# Coordinates: A-J (rows), 1-10 (columns)
board = {
    'ships': [],      # Ship positions
    'attacks': [],    # Attack history
    'hits': [],       # Successful hits
    'misses': []      # Missed attacks
}
```

### Ship Types
Standard battleship ship configuration:
- Carrier (5 cells)
- Battleship (4 cells)
- Cruiser (3 cells)
- Submarine (3 cells)
- Destroyer (2 cells)

### Game States
1. **SETUP:** Players place ships on their boards
2. **PLAYING:** Turn-based attack phase
3. **FINISHED:** Game over (all ships of one player sunk)

### Turn Sequence
1. Player/Sphero selects attack coordinate
2. Game controller validates the move
3. Game controller checks for hit/miss
4. Game controller updates board state
5. Game controller broadcasts result
6. Check for win condition
7. Switch turns

---

## Communication Protocol

### ROS2 Topics

#### Published Topics
- `/game/state` - Current game state broadcasts
- `/game/player1/board` - Player 1 board state
- `/game/player2/board` - Player 2 board state
- `/game/events` - Game events (hits, misses, ships sunk)
- `/game/turn` - Current turn indicator

#### Subscribed Topics
- `/player/human/move` - Human player move commands
- `/player/sphero/move` - Sphero player move commands
- `/player/human/setup` - Human ship placement
- `/player/sphero/setup` - Sphero ship placement

### ROS2 Services
- `/game/start` - Initialize new game
- `/game/reset` - Reset current game
- `/game/validate_move` - Validate a proposed move
- `/game/get_state` - Query current game state

### Message Types (Custom)
```python
# MoveCommand.msg
int8 row
int8 col
string player_id

# GameState.msg
string state  # SETUP, PLAYING, FINISHED
string current_player
bool game_over
string winner

# AttackResult.msg
int8 row
int8 col
bool is_hit
bool ship_sunk
string ship_type
```

---

## Sphero Integration

### Sphero Features Utilized

#### Visual Feedback
- **Green LED:** Hit on opponent's ship
- **Red LED:** Opponent hit your ship
- **Blue LED:** Miss
- **Yellow LED:** Your turn
- **Flashing:** Ship sunk

#### Motion Feedback
- **Spin:** Successful hit
- **Back and forth:** Ship sunk
- **Vibration:** Opponent's turn

#### Input Methods
- **Collision detection:** Alternative move selection
- **Orientation:** Direction-based grid navigation
- **Button press:** Confirm move selection

### Bluetooth Connection Management
- Auto-discovery of nearby Sphero devices
- Connection retry logic
- Graceful disconnection handling
- Multiple Sphero support for multi-game scenarios

---

## AI Strategy (Sphero)

### Ship Placement Algorithm
- **Random placement:** Basic implementation
- **Strategic placement:** Avoid edges and corners (future enhancement)

### Attack Strategy
1. **Hunt Mode:** Random attacks to find ships
2. **Target Mode:** Adjacent attacks after hit
3. **Probability-based:** Attack high-probability cells (future enhancement)

### Decision Tree
```
Is there a known hit without sunk ship?
├─ Yes → Target adjacent cells
└─ No → Hunt mode (random or probability-based)
```

---

## Data Flow Diagram

```
Human Player Input
       │
       ▼
┌──────────────┐
│   Human      │
│  Controller  │
└──────┬───────┘
       │ /player/human/move
       ▼
┌──────────────────┐     ┌──────────────┐
│      Game        │◄────┤   Sphero     │
│    Controller    │     │  Controller  │
└────────┬─────────┘     └──────▲───────┘
         │                      │
         │ /game/events         │ /player/sphero/move
         │                      │
         └──────────────────────┘
```

---

## Directory Structure

```
ros2_ws_2/
└── src/
    └── battleship_game/
        ├── battleship_game/
        │   ├── __init__.py
        │   ├── game_controller_node.py      # Main game logic
        │   ├── human_controller_node.py     # Human interface
        │   └── sphero_controller_node.py    # Sphero control
        ├── test/
        │   ├── test_copyright.py
        │   ├── test_flake8.py
        │   ├── test_pep257.py
        │   └── test_xmllint.py
        ├── resource/
        │   └── battleship_game              # Package marker
        ├── package.xml                       # Package metadata
        ├── setup.py                          # Build configuration
        └── setup.cfg                         # Build settings
```

---

## Implementation Phases

### Phase 1: Core Game Logic (Current)
- [ ] Implement basic board representation
- [ ] Create game state management
- [ ] Implement move validation
- [ ] Create win condition checking
- [ ] Set up ROS2 node structure

### Phase 2: Human Interface
- [ ] Create CLI board display
- [ ] Implement input parsing (coordinate entry)
- [ ] Add ship placement interface
- [ ] Create turn-based interaction flow
- [ ] Add game status display

### Phase 3: Sphero Integration
- [ ] Implement Sphero connection logic
- [ ] Create LED feedback system
- [ ] Add motion feedback
- [ ] Implement basic AI (random moves)
- [ ] Test Sphero-human gameplay

### Phase 4: Advanced Features
- [ ] Implement advanced AI strategy
- [ ] Add GUI option (optional)
- [ ] Support multiple game modes
- [ ] Add game statistics and logging
- [ ] Implement replay functionality

### Phase 5: Testing & Polish
- [ ] Unit tests for game logic
- [ ] Integration tests for ROS2 communication
- [ ] Sphero hardware testing
- [ ] Performance optimization
- [ ] Documentation completion

---

## Testing Strategy

### Unit Tests
- Game logic validation (hit detection, win conditions)
- Board state management
- Move validation
- AI decision-making

### Integration Tests
- ROS2 node communication
- Sphero connection and control
- End-to-end gameplay scenarios

### Hardware Tests
- Sphero Bluetooth connectivity
- LED and motion feedback
- Multi-Sphero scenarios

---

## Error Handling

### Game Controller
- Invalid move detection
- Out-of-bounds coordinate handling
- Duplicate attack prevention
- State synchronization errors

### Sphero Controller
- Bluetooth connection failures
- Device discovery timeout
- Command transmission errors
- Battery level monitoring

### Human Controller
- Invalid input format handling
- Connection loss recovery
- Timeout handling

---

## Configuration

### Game Settings
```python
GAME_CONFIG = {
    'board_size': (10, 10),
    'ships': {
        'carrier': 5,
        'battleship': 4,
        'cruiser': 3,
        'submarine': 3,
        'destroyer': 2
    },
    'turn_timeout': 30,  # seconds
    'sphero_feedback': True,
    'ai_difficulty': 'medium'
}
```

### Sphero Settings
```python
SPHERO_CONFIG = {
    'connection_timeout': 10,
    'retry_attempts': 3,
    'led_brightness': 255,
    'motion_speed': 128,
    'battery_warning_threshold': 20  # percent
}
```

---

## Future Enhancements

### Planned Features
1. **Multiplayer Support:** Multiple Spheros vs multiple humans
2. **Network Play:** Remote gameplay over network
3. **Advanced AI:** Machine learning-based strategy
4. **Tournament Mode:** Best of N games tracking
5. **Custom Board Sizes:** Variable grid dimensions
6. **Power-ups:** Special abilities (scan, multi-shot, etc.)
7. **Voice Control:** Verbal command input
8. **Web Interface:** Browser-based game board
9. **Statistics Dashboard:** Win rates, accuracy, etc.
10. **Game Replay System:** Record and playback games

### Performance Optimizations
- Message batching for reduced network overhead
- Efficient board state compression
- Caching for repeated queries
- Asynchronous Sphero command execution

---

## Security & Privacy

### Considerations
- No sensitive data stored
- Local network communication only
- Bluetooth pairing security for Sphero
- Input validation to prevent injection

---

## Deployment

### Prerequisites
- ROS2 (Humble, Iron, or later)
- Python 3.8+
- Sphero robot (BOLT, SPRK+, or compatible model)
- Bluetooth adapter (if not built-in)

### Installation Steps
```bash
# Clone repository
cd ~/ros2_ws_2/src
git clone <repository_url> battleship_game

# Install dependencies
cd ~/ros2_ws_2
rosdep install --from-paths src --ignore-src -r -y

# Build package
colcon build --packages-select battleship_game

# Source workspace
source install/setup.bash
```

### Running the Game
```bash
# Terminal 1: Start game controller
ros2 run battleship_game game_controller_node

# Terminal 2: Start human controller
ros2 run battleship_game human_controller_node

# Terminal 3: Start Sphero controller
ros2 run battleship_game sphero_controller_node
```

---

## Troubleshooting

### Common Issues

**Sphero won't connect:**
- Ensure Bluetooth is enabled
- Check battery level
- Reset Sphero by placing in charging cradle
- Verify spherov2 library installation

**Nodes can't communicate:**
- Check ROS2 daemon is running: `ros2 daemon status`
- Verify nodes are on same ROS domain: `echo $ROS_DOMAIN_ID`
- Check topic list: `ros2 topic list`

**Game state desynchronization:**
- Restart all nodes
- Check network latency
- Review logs: `ros2 log`

---

## References

- [ROS2 Documentation](https://docs.ros.org/)
- [Spherov2 Library](https://github.com/artificial-intelligence-class/spherov2.py)
- [Battleship Game Rules](https://en.wikipedia.org/wiki/Battleship_(game))

---

## Changelog

### Version 0.0.0 (Current)
- Initial project structure
- Package configuration
- Design document created
- Node placeholders established

---

## Contributors

- Siddharth Vaghela (Project Lead & Developer)

---

## License

TODO: License declaration pending
