# Topic Naming Convention - Important Note

## ROS2 Topic Naming Rules

ROS2 topic names cannot contain hyphens (`-`). They can only contain:
- Alphanumeric characters
- Underscores (`_`)
- Tildes (`~`)
- Curly braces (`{`, `}`)

## Automatic Name Sanitization

All Sphero controllers **automatically sanitize** the Sphero name for use in topic names by replacing hyphens with underscores.

### Example

If you add a Sphero named `SB-3660`:

**Sphero Name (for hardware connection):**
```
SB-3660
```

**Topic Names (automatically sanitized):**
```
/sphero/SB_3660/led
/sphero/SB_3660/roll
/sphero/SB_3660/state
/sphero/SB_3660/battery
/sphero/SB_3660/task
/sphero/SB_3660/state_machine/config
...
```

Notice the hyphen `-` became an underscore `_` in the topic names.

## Where Sanitization Happens

The sanitization occurs in all four controller nodes:

1. **Device Controller** (`sphero_instance_device_controller_node.py`)
   ```python
   topic_name_safe = sphero_name.replace("-", "_")
   self.topic_prefix = f'sphero/{topic_name_safe}'
   ```

2. **Task Controller** (`sphero_instance_task_controller_node.py`)
   ```python
   topic_name_safe = sphero_name.replace("-", "_")
   self.topic_prefix = f'sphero/{topic_name_safe}'
   ```

3. **State Machine Controller** (`sphero_instance_statemachine_controller_node.py`)
   ```python
   topic_name_safe = sphero_name.replace("-", "_")
   self.topic_prefix = f'sphero/{topic_name_safe}'
   ```

4. **WebSocket Server** (`sphero_instance_websocket_server.py`)
   ```python
   self.topic_name_safe = sphero_name.replace("-", "_")
   self.topic_prefix = f'sphero/{self.topic_name_safe}'
   ```

## Important Notes

1. **Sphero Name Unchanged**: The actual Sphero name (`SB-3660`) is preserved for connecting to the hardware via Bluetooth.

2. **Topic Names Sanitized**: Only the ROS2 topic names are modified to comply with ROS2 naming rules.

3. **Transparent to User**: This happens automatically - you don't need to do anything special when adding Spheros.

4. **Dashboard Display**: The dashboard will show the original Sphero name (`SB-3660`), not the sanitized version.

## Examples

| Sphero Name | Topics Use          | Display Shows |
|-------------|---------------------|---------------|
| SB-3660     | sphero/SB_3660/*    | SB-3660       |
| SB-1234     | sphero/SB_1234/*    | SB-1234       |
| MY-SPHERO   | sphero/MY_SPHERO/*  | MY-SPHERO     |
| BOLT-01     | sphero/BOLT_01/*    | BOLT-01       |

## Checking Topics

To see the actual topic names being used:

```bash
# List all Sphero topics
ros2 topic list | grep sphero

# Example output for SB-3660:
/sphero/SB_3660/battery
/sphero/SB_3660/led
/sphero/SB_3660/roll
/sphero/SB_3660/state
/sphero/SB_3660/task
...
```

## Why This Matters

Without sanitization, you would get errors like:

```
rclpy.exceptions.InvalidTopicNameException: Invalid topic name:
topic name must not contain characters other than alphanumerics,
'_', '~', '{', or '}'
```

The controllers now handle this automatically, so your Spheros can have any name (like `SB-3660`, `BOLT-A1`, etc.) and they will work correctly.

## Best Practices

### Recommended Sphero Names
- ✅ `SB-3660` → Works (becomes `SB_3660`)
- ✅ `BOLT-01` → Works (becomes `BOLT_01`)
- ✅ `MY-ROBOT` → Works (becomes `MY_ROBOT`)
- ✅ `SPHERO_A` → Works (no change needed)

### Names to Avoid
- ❌ Names with spaces: `MY SPHERO` (will cause issues)
- ❌ Special characters: `SB@3660`, `BOLT#1` (will cause issues)
- ❌ Starting with numbers: `123-SPHERO` (may cause issues)

### Recommendation
Stick to alphanumeric characters, hyphens, and underscores for Sphero names. The system will handle hyphens automatically.

---

This automatic sanitization ensures smooth operation while allowing you to use standard Sphero naming conventions! 🤖
