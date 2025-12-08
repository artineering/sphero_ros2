#!/usr/bin/env python3
"""
Test script for Sphero multi-robot control.

This script:
1. Starts device and task controller nodes for multiple Spheros
2. Sends reset_aim commands to each Sphero
3. Sends concurrent roll commands with specific parameters
"""

import subprocess
import time
import json
import sys
from typing import List, Dict
import signal
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Sphero configuration
SPHEROS = ['SB-3660', 'SB-3716', 'SB-74FB', 'SB-D2F0']


STRATEGIES = {
    'STRATEGY_1_NOT_COLOCATED_FORMATION': {
        'ZONE 1': {
            'SB-3660': {'heading': 45, 'speed': 150, 'duration': 0.6},
            'SB-3716': {'heading': 65, 'speed': 150, 'duration': 1.58},
            'SB-D2F0': {'heading': 295, 'speed': 150, 'duration': 1.59},
            'SB-74FB': {'heading': 290, 'speed': 150, 'duration': 0.5}
        },
        'ZONE 2': {
            'SB-3660': {'heading': 70, 'speed': 150, 'duration': 1},
            'SB-3716': {'heading': 55, 'speed': 150, 'duration': 1},
            'SB-D2F0': {'heading': 305, 'speed': 150, 'duration': 1},
            'SB-74FB': {'heading': 290, 'speed': 150, 'duration': 1}
        },
        'ZONE 3': {
            'SB-3660': {'heading': 84, 'speed': 150, 'duration': 1.5},
            'SB-3716': {'heading': 75, 'speed': 150, 'duration': 0.5},
            'SB-D2F0': {'heading': 280, 'speed': 150, 'duration': 0.6},
            'SB-74FB': {'heading': 276, 'speed': 150, 'duration': 1.32}
        }
    },
    'STRATEGY_2_COLOCATED_RANDOM': {
        'ZONE 1': {
            'SB-3660': {'heading': 60, 'speed': 150, 'duration': 1.02},
            'SB-3716': {'heading': 45, 'speed': 150, 'duration': 1.58},
            'SB-D2F0': {'heading': 215, 'speed': 150, 'duration': 0.59},
            'SB-74FB': {'heading': 190, 'speed': 150, 'duration': 1.2}
        },
        'ZONE 2': {
            'SB-3660': {'heading': 70, 'speed': 150, 'duration': 1.12},
            'SB-3716': {'heading': 85, 'speed': 150, 'duration': 1.58},
            'SB-D2F0': {'heading': 195, 'speed': 150, 'duration': 1.59},
            'SB-74FB': {'heading': 90, 'speed': 150, 'duration': 0.32}
        },
        'ZONE 3': {
            'SB-3660': {'heading': 80, 'speed': 150, 'duration': 0.32},
            'SB-3716': {'heading': 65, 'speed': 150, 'duration': 0.58},
            'SB-D2F0': {'heading': 295, 'speed': 150, 'duration': 1.19},
            'SB-74FB': {'heading': 290, 'speed': 150, 'duration': 1.32}
        }
    },
    'STRATEGY_3_COLOCATED_FORMATION': [
        {
            'ZONE 3': {
                'SB-3660': {'heading': 0, 'speed': 150, 'duration': 1.11},
                'SB-3716': {'heading': 0, 'speed': 150, 'duration': 0.95},
                'SB-D2F0': {'heading': 0, 'speed': 150, 'duration': 0.89},
                'SB-74FB': {'heading': 0, 'speed': 150, 'duration': 0.85}
            },
            'ZONE 2': {
                'SB-3660': {'heading': 0, 'speed': 150, 'duration': 1.11},
                'SB-3716': {'heading': 0, 'speed': 150, 'duration': 0.95},
                'SB-D2F0': {'heading': 0, 'speed': 150, 'duration': 0.89},
                'SB-74FB': {'heading': 0, 'speed': 150, 'duration': 0.85}
            },
            'ZONE 1': {
                'SB-3660': {'heading': 0, 'speed': 150, 'duration': 1.11},
                'SB-3716': {'heading': 0, 'speed': 150, 'duration': 0.95},
                'SB-D2F0': {'heading': 0, 'speed': 150, 'duration': 0.89},
                'SB-74FB': {'heading': 0, 'speed': 150, 'duration': 0.85}
            }
        },
        {
            'ZONE 3': {
                'SB-3660': {'heading': 0, 'speed': 0, 'duration': 0.1},
                'SB-3716': {'heading': 0, 'speed': 0, 'duration': 0.1},
                'SB-D2F0': {'heading': 0, 'speed': 0, 'duration': 0.1},
                'SB-74FB': {'heading': 0, 'speed': 0, 'duration': 0.1},
            },
            'ZONE 2': {
                'SB-3660': {'heading': 0, 'speed': 0, 'duration': 0.1},
                'SB-3716': {'heading': 0, 'speed': 0, 'duration': 0.1},
                'SB-D2F0': {'heading': 0, 'speed': 0, 'duration': 0.1},
                'SB-74FB': {'heading': 0, 'speed': 0, 'duration': 0.1},
            },
            'ZONE 1': {
                'SB-3660': {'heading': 0, 'speed': 0, 'duration': 0.1},
                'SB-3716': {'heading': 0, 'speed': 0, 'duration': 0.1},
                'SB-D2F0': {'heading': 0, 'speed': 0, 'duration': 0.1},
                'SB-74FB': {'heading': 0, 'speed': 0, 'duration': 0.1},
            }
        },
        {
            'ZONE 3': {
                'SB-3660': {'heading': 270, 'speed': 150, 'duration': 0.3},
                'SB-3716': {'heading': 270, 'speed': 150, 'duration': 0.2},
                'SB-D2F0': {'heading': 270, 'speed': 150, 'duration': 0.1},
                'SB-74FB': {'heading': 270, 'speed': 150, 'duration': 0.05}
            },
            'ZONE 2': {
                'SB-3660': {'heading': 270, 'speed': 150, 'duration': 0.3},
                'SB-3716': {'heading': 270, 'speed': 150, 'duration': 0.2},
                'SB-D2F0': {'heading': 270, 'speed': 150, 'duration': 0.1},
                'SB-74FB': {'heading': 270, 'speed': 150, 'duration': 0.05}
            },
            'ZONE 1': {
                'SB-3660': {'heading': 270, 'speed': 150, 'duration': 0.3},
                'SB-3716': {'heading': 270, 'speed': 150, 'duration': 0.2},
                'SB-D2F0': {'heading': 270, 'speed': 150, 'duration': 0.1},
                'SB-74FB': {'heading': 270, 'speed': 150, 'duration': 0.05}
            }
        }
    ],

    'STRATEGY_4_NOT_COLOCATED_RANDOM': {
        'ZONE 1': {
            'SB-3660': {'heading': 30, 'speed': 150, 'duration': 1.32},
            'SB-3716': {'heading': 85, 'speed': 150, 'duration': 1.58},
            'SB-D2F0': {'heading': 195, 'speed': 150, 'duration': 1.59},
            'SB-74FB': {'heading': 210, 'speed': 150, 'duration': 1.32}
        },
        'ZONE 2': {
            'SB-3660': {'heading': 60, 'speed': 150, 'duration': 1.32},
            'SB-3716': {'heading': 45, 'speed': 150, 'duration': 1.58},
            'SB-D2F0': {'heading': 215, 'speed': 150, 'duration': 1.59},
            'SB-74FB': {'heading': 190, 'speed': 150, 'duration': 1.32}
        },
        'ZONE 3': {
            'SB-3660': {'heading': 80, 'speed': 150, 'duration': 1.32},
            'SB-3716': {'heading': 45, 'speed': 150, 'duration': 1.58},
            'SB-D2F0': {'heading': 235, 'speed': 150, 'duration': 1.59},
            'SB-74FB': {'heading': 220, 'speed': 150, 'duration': 1.32}
        }
    }
}

# Global list to track all spawned processes
processes = []

# ROS2 node and publishers
ros_node = None
roll_publishers: Dict[str, any] = {}
led_publishers: Dict[str, any] = {}

def get_key() -> str:
    """Read a single keypress from stdin.

    Returns:
        The key pressed as a string. ESC returns '\x1b'.
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def select_strategy() -> str | None:
    """Prompt user to select a strategy from available options.

    Returns:
        Strategy name, or None if user wants to quit (ESC or 'q').
    """
    strategies = list(STRATEGIES.keys())

    print("\nAvailable Strategies:")
    print("-" * 40)
    for i, strategy in enumerate(strategies, 1):
        print(f"  {i}. {strategy}")
    print()
    print("Press 1-{} to select, ESC or 'q' to quit".format(len(strategies)))

    while True:
        key = get_key()
        # ESC key
        if key == '\x1b':
            return None
        # 'q' or 'Q' to quit
        if key.lower() == 'q':
            return None
        # Number selection
        if key.isdigit():
            idx = int(key) - 1
            if 0 <= idx < len(strategies):
                print(f"  Selected: {strategies[idx]}")
                return strategies[idx]

def select_zone(strategy_name: str) -> str | None:
    """Prompt user to select a zone from the given strategy.

    Returns:
        Zone name, or None if user wants to quit (ESC or 'q').
    """
    strategy = STRATEGIES[strategy_name]
    if isinstance(strategy, list):
        zones = list(strategy[0].keys())
    else:
        zones = list(strategy.keys())

    print(f"\nAvailable Zones for {strategy_name}:")
    print("-" * 40)
    for i, zone in enumerate(zones, 1):
        print(f"  {i}. {zone}")
    print()
    print("Press 1-{} to select, ESC or 'q' to quit".format(len(zones)))

    while True:
        key = get_key()
        # ESC key
        if key == '\x1b':
            return None
        # 'q' or 'Q' to quit
        if key.lower() == 'q':
            return None
        # Number selection
        if key.isdigit():
            idx = int(key) - 1
            if 0 <= idx < len(zones):
                print(f"  Selected: {zones[idx]}")
                return zones[idx]

def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully by terminating all processes."""
    print("\n\n[INFO] Caught interrupt signal, shutting down all nodes...")
    cleanup_processes()
    cleanup_ros()
    sys.exit(0)


def cleanup_ros():
    """Shutdown ROS2 node and context."""
    global ros_node
    if ros_node is not None:
        ros_node.destroy_node()
        ros_node = None
    if rclpy.ok():
        rclpy.shutdown()


def init_ros_publishers(sphero_names: List[str]):
    """
    Initialize ROS2 node and create roll and LED publishers for each Sphero.

    Args:
        sphero_names: List of Sphero names to create publishers for
    """
    global ros_node, roll_publishers, led_publishers

    rclpy.init()
    ros_node = rclpy.create_node('sphero_roll_test')

    for sphero_name in sphero_names:
        name_safe = sphero_name.replace("-", "_")

        # Create roll publisher
        roll_topic = f'/sphero/{name_safe}/roll'
        roll_publishers[sphero_name] = ros_node.create_publisher(String, roll_topic, 10)
        print(f"  [✓] Created publisher for {roll_topic}")


    print(f"[SUCCESS] Created {len(roll_publishers)} roll publishers")


def cleanup_processes():
    """Terminate all spawned processes."""
    for proc in processes:
        if proc.poll() is None:  # Process is still running
            print(f"[INFO] Terminating process PID {proc.pid}...")
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                print(f"[WARNING] Force killing process PID {proc.pid}")
                proc.kill()
    processes.clear()


def start_node(node_type: str, sphero_name: str, external_localization: bool = False) -> subprocess.Popen:
    """
    Start a Sphero node (device_controller or task_controller).

    Args:
        node_type: Either 'device' or 'task'
        sphero_name: Name of the Sphero (e.g., 'SB-3660')
        external_localization: Enable external localization (device controller only)

    Returns:
        The subprocess.Popen object
    """
    if node_type == 'device':
        executable = 'sphero_instance_device_controller_node.py'
        if external_localization:
            cmd = [
                'ros2', 'run', 'sphero_instance_controller', executable,
                '--ros-args',
                '-p', f'sphero_name:={sphero_name}',
                '-p', 'external_localization:=true'
            ]
        else:
            cmd = [
                'ros2', 'run', 'sphero_instance_controller', executable,
                '--ros-args',
                '-p', f'sphero_name:={sphero_name}',
                '-p', 'external_localization:=false'
            ]
    elif node_type == 'task':
        executable = 'sphero_instance_task_controller_node.py'
        cmd = [
            'ros2', 'run', 'sphero_instance_controller', executable,
            '--ros-args',
            '-p', f'sphero_name:={sphero_name}'
        ]
    else:
        raise ValueError(f"Unknown node_type: {node_type}")

    print(f"[INFO] Starting {node_type} controller for {sphero_name}...")
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=None
    )
    processes.append(proc)
    return proc


def send_roll_command(sphero_name: str, heading: int, speed: int, duration: float):
    """
    Send roll command to a Sphero using ROS2 publisher.

    Args:
        sphero_name: Name of the Sphero
        heading: Direction in degrees (0-359)
        speed: Speed (0-255)
        duration: Duration in seconds
    """
    if sphero_name not in roll_publishers:
        print(f"[ERROR] No publisher found for {sphero_name}")
        return

    roll_data = {
        'heading': heading,
        'speed': speed,
        'duration': duration
    }
    json_str = json.dumps(roll_data)

    msg = String()
    msg.data = json_str

    print(f"[INFO] Sending roll command to {sphero_name}: heading={heading}, speed={speed}, duration={duration}")
    roll_publishers[sphero_name].publish(msg)


def main():
    """Main execution function."""
    # Set up signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)

    print("=" * 80)
    print("Sphero Multi-Robot Roll Test Script")
    print("=" * 80)
    print(f"Spheros to control: {', '.join(SPHEROS)}")
    print(f"External localization: False")
    print("=" * 80)
    print()

    # Step 1: Start all device and task controller nodes
    print("[STEP 1] Starting device and task controller nodes for all Spheros...")
    print("-" * 80)

    for sphero_name in SPHEROS:
        # Start device controller
        start_node('device', sphero_name, external_localization=False)
        time.sleep(20)  # Small delay between node starts


    # Check if any processes have died
    dead_processes = [proc for proc in processes if proc.poll() is not None]
    if dead_processes:
        print(f"[WARNING] {len(dead_processes)} process(es) died during initialization!")
        for proc in dead_processes:
            stderr_output = proc.stderr.read() if proc.stderr else "N/A"
            print(f"[ERROR] Process PID {proc.pid} exited with code {proc.returncode}")
            if stderr_output:
                print(f"  Error output: {stderr_output[:500]}")

    print(f"[SUCCESS] All nodes started ({len(processes)} processes running)")
    print()

    init_ros_publishers(SPHEROS)

    try:

        # Run the main experiment loop
        experiment_num = 1
        stop_experiment = False
        while not stop_experiment:
            print(f"\n{'=' * 80}")
            print(f"TRIAL #{experiment_num}")
            print("=" * 80)

            # Step 3: Select strategy and zone
            print("Select strategy and zone for roll commands...")
            print("-" * 80)

            selected_strategy = select_strategy()
            if selected_strategy is None:
                print("[INFO] User requested quit.")
                break

            selected_zone = select_zone(selected_strategy)
            if selected_zone is None:
                print("[INFO] User requested quit.")
                break

            print(f"\n[INFO] Selected: {selected_strategy} -> {selected_zone}")
            print("[READY] Press ENTER to send roll commands, ESC or 'q' to quit...")
            key = get_key()
            if key == '\x1b' or key.lower() == 'q':
                print("\n[INFO] User requested quit.")
                break
            print()

            # Get the selected strategy and check if it contains multiple steps
            strategy = STRATEGIES[selected_strategy]

            if isinstance(strategy, list) :
                for step in strategy:
                    roll_commands = step[selected_zone]
                    for sphero, cmd in roll_commands.items():
                        print(f"  {sphero}: heading={cmd['heading']}, speed={cmd['speed']}, duration={cmd['duration']}")
                        send_roll_command(
                            sphero,
                            cmd['heading'],
                            cmd['speed'],
                            cmd['duration']
                        )
            else:
                roll_commands = STRATEGIES[selected_strategy][selected_zone]
                for sphero, cmd in roll_commands.items():
                    print(f"  {sphero}: heading={cmd['heading']}, speed={cmd['speed']}, duration={cmd['duration']}")
                print()
                for sphero_name in SPHEROS:
                    if sphero_name in roll_commands:
                        command = roll_commands[sphero_name]
                        send_roll_command(
                            sphero_name,
                            command['heading'],
                            command['speed'],
                            command['duration']
                        )

            print(f"\n[SUCCESS] TRIAL #{experiment_num} complete!")
            experiment_num += 1
    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

    finally:
        # Cleanup on exit
        print("\n[INFO] Cleaning up...")
        cleanup_processes()
        cleanup_ros()
        print("[INFO] All processes terminated. Exiting.")

    


if __name__ == '__main__':
    main()
