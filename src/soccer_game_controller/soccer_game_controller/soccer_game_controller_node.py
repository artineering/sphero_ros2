#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Soccer Game Controller Node.

This node orchestrates a multi-robot soccer game with the following workflow:
1. Field calibration using ArUco SLAM
2. Sphero detection and registration
3. Interactive sphero calibration (heading and position)
4. Final positioning and dance routine

Author: Siddharth Vaghela
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import json
import time
import requests
import sys
import threading
from typing import Dict, List, Optional, Tuple
from enum import Enum


class GameState(Enum):
    """States of the soccer game controller."""
    INIT = "init"
    ASK_FIELD_DIMENSIONS = "ask_field_dimensions"
    START_ARUCO_SLAM = "start_aruco_slam"
    FIELD_CALIBRATION = "field_calibration"
    WAIT_FOR_SPHERO_PLACEMENT = "wait_for_sphero_placement"
    SPHERO_DETECTION = "sphero_detection"
    SPHERO_ACTIVATION = "sphero_activation"
    SPHERO_CALIBRATION = "sphero_calibration"
    MOVE_TO_POSITION = "move_to_position"
    ALL_CALIBRATED = "all_calibrated"
    DANCE = "dance"
    DONE = "done"


class SpheroInfo:
    """Information about a Sphero robot."""

    def __init__(self, marker_id: int, sphero_name: str):
        """Initialize sphero info."""
        self.marker_id = marker_id
        self.sphero_name = sphero_name
        self.port: Optional[int] = None
        self.detected = False
        self.activated = False
        self.calibrated = False
        self.last_position: Optional[Tuple[float, float]] = None  # (x, y) in meters


class SoccerGameController(Node):
    """
    ROS2 node that orchestrates a multi-robot soccer game.

    Workflow:
    1. Ask user for field dimensions
    2. Start ArUco SLAM with field dimensions
    3. Calibrate field
    4. For each Sphero (4 total):
       a. Prompt user to place Sphero on field marker
       b. Detect Sphero via ArUco marker
       c. Activate Sphero controllers
       d. Calibrate heading (0° = left-top, 90° = right-bottom)
       e. Calibrate position to (0, 0)
       f. Move to center field position
    5. Make all Spheros dance
    """

    def __init__(self):
        """Initialize the soccer game controller."""
        super().__init__('soccer_game_controller',
                        enable_rosout=False,  # Disable ROS logging output
                        parameter_overrides=[])

        # Disable ROS2 logger output to terminal
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.FATAL)

        # Parameters (will be set after user input)
        self.field_width = None
        self.field_height = None
        self.corner_marker_ids = [4, 5, 6, 7]
        self.sphero_marker_ids = [3, 1, 2, 20]
        self.sphero_names = ['SB-3660', 'SB-74FB', 'SB-3716', 'SB-58EF']
        self.webserver_url = 'http://localhost:5000'

        # Initialize Sphero info
        self.spheros: List[SpheroInfo] = []
        for i, (marker_id, name) in enumerate(zip(self.sphero_marker_ids, self.sphero_names)):
            self.spheros.append(SpheroInfo(marker_id, name))

        self.current_sphero_index = 0

        # State machine
        self.state = GameState.INIT
        self.field_calibrated = False
        self.aruco_slam_process = None
        self.input_ready = False
        self.user_input = None

        # Subscribers (created after ArUco SLAM starts)
        self.calibration_sub = None
        self.markers_sub = None
        self.position_subscribers: Dict[int, rclpy.subscription.Subscription] = {}

        # Timer for state machine
        self.timer = self.create_timer(0.5, self.state_machine_callback)

        # Input thread
        self.input_thread = None

    def print(self, message: str = ""):
        """Print message to terminal (bypassing ROS logger)."""
        print(message, flush=True)

    def get_user_input(self, prompt: str) -> str:
        """Get user input directly (blocking)."""
        return input(prompt)

    def start_aruco_slam(self):
        """Start ArUco SLAM node as a subprocess with user-specified field dimensions."""
        import subprocess

        self.print("\nStarting ArUco SLAM node...")

        # Launch ArUco SLAM with field dimensions
        self.aruco_slam_process = subprocess.Popen([
            'ros2', 'run', 'aruco_slam', 'aruco_slam_node',
            '--ros-args',
            '-p', f'field_width_cm:={self.field_width}',
            '-p', f'field_height_cm:={self.field_height}',
            '-p', f'corner_marker_ids:={self.corner_marker_ids}',
            '-p', f'sphero_marker_ids:={self.sphero_marker_ids}',
            '-p', f'sphero_names:={self.sphero_names}'
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        # Wait for ArUco SLAM to initialize
        time.sleep(3)

        # Create subscribers now that ArUco SLAM is running
        self.calibration_sub = self.create_subscription(
            String,
            '/aruco_slam/calibration_status',
            self.calibration_callback,
            10
        )

        self.markers_sub = self.create_subscription(
            String,
            '/aruco_slam/all_markers',
            self.markers_callback,
            10
        )

        self.print("✓ ArUco SLAM started\n")

    def calibration_callback(self, msg: String):
        """Handle ArUco SLAM calibration status updates."""
        status_text = msg.data
        if "CALIBRATED" in status_text and not self.field_calibrated:
            self.field_calibrated = True

    def markers_callback(self, msg: String):
        """Handle detected ArUco markers."""
        try:
            data = json.loads(msg.data)
            markers = data.get('markers', {})

            # Check if current Sphero is detected
            if self.state == GameState.SPHERO_DETECTION:
                current_sphero = self.spheros[self.current_sphero_index]
                marker_id_str = str(current_sphero.marker_id)

                if marker_id_str in markers:
                    marker_data = markers[marker_id_str]
                    if 'field_x' in marker_data and 'field_y' in marker_data:
                        if not current_sphero.detected:
                            current_sphero.detected = True

        except json.JSONDecodeError:
            pass

    def position_callback(self, msg: PoseStamped, sphero_index: int):
        """Handle position updates for a specific Sphero."""
        sphero = self.spheros[sphero_index]
        sphero.last_position = (msg.pose.position.x, msg.pose.position.y)

    def state_machine_callback(self):
        """Main state machine loop."""
        if self.state == GameState.INIT:
            # Skip - handled in main()
            self.state = GameState.START_ARUCO_SLAM

        elif self.state == GameState.ASK_FIELD_DIMENSIONS:
            # Skip - handled in main()
            self.state = GameState.START_ARUCO_SLAM

        elif self.state == GameState.START_ARUCO_SLAM:
            self.start_aruco_slam()
            self.state = GameState.FIELD_CALIBRATION
            self.print("Field Calibration")
            self.print("-" * 70)
            self.print("")
            self.print("Place ArUco markers at the 4 corners of the soccer field:")
            self.print(f"  • Top-Left corner:     Marker {self.corner_marker_ids[0]}")
            self.print(f"  • Top-Right corner:    Marker {self.corner_marker_ids[1]}")
            self.print(f"  • Bottom-Right corner: Marker {self.corner_marker_ids[2]}")
            self.print(f"  • Bottom-Left corner:  Marker {self.corner_marker_ids[3]}")
            self.print("")
            self.print("Waiting for field calibration...")

        elif self.state == GameState.FIELD_CALIBRATION:
            if self.field_calibrated:
                self.print("✓ Field calibrated successfully!")
                self.print("")
                self.state = GameState.WAIT_FOR_SPHERO_PLACEMENT
                self.start_sphero_workflow()

        elif self.state == GameState.WAIT_FOR_SPHERO_PLACEMENT:
            # Waiting for user input (handled in start_sphero_workflow)
            pass

        elif self.state == GameState.SPHERO_DETECTION:
            current_sphero = self.spheros[self.current_sphero_index]
            if current_sphero.detected:
                self.print("✓ Sphero detected!")
                self.print("")
                self.state = GameState.SPHERO_ACTIVATION
                self.activate_sphero_controllers()

        elif self.state == GameState.SPHERO_ACTIVATION:
            # Waiting for controllers to activate
            pass

        elif self.state == GameState.SPHERO_CALIBRATION:
            # Interactive calibration handled separately
            pass

        elif self.state == GameState.MOVE_TO_POSITION:
            # Moving to center field position
            pass

        elif self.state == GameState.ALL_CALIBRATED:
            self.print("")
            self.print("="*70)
            self.print("✓ ALL SPHEROS CALIBRATED AND POSITIONED!")
            self.print("="*70)
            self.print("")
            self.print("Starting dance routine...")
            self.print("")
            self.state = GameState.DANCE
            self.start_dance_routine()

        elif self.state == GameState.DANCE:
            # Dancing
            pass

        elif self.state == GameState.DONE:
            self.print("")
            self.print("="*70)
            self.print("✓ SOCCER GAME SETUP COMPLETE!")
            self.print("="*70)
            self.print("")
            # Stop timer
            self.timer.cancel()

    def start_sphero_workflow(self):
        """Start the workflow for calibrating the current Sphero."""
        self.timer.cancel()  # Stop timer during input

        current_sphero = self.spheros[self.current_sphero_index]

        self.print("")
        self.print("="*70)
        self.print(f"SPHERO {self.current_sphero_index + 1} of {len(self.spheros)}: {current_sphero.sphero_name}")
        self.print("="*70)
        self.print("")
        self.print("Setup Instructions:")
        self.print(f"  1. Attach ArUco marker {current_sphero.marker_id} to {current_sphero.sphero_name}")
        self.print(f"  2. Place {current_sphero.sphero_name} on the left-bottom field marker")
        self.print(f"  3. Ensure marker is visible to camera")
        self.print("")

        input("Press Enter when ready...")

        self.print("")
        self.print(f"Detecting {current_sphero.sphero_name}...")

        self.state = GameState.SPHERO_DETECTION
        # Restart timer
        self.timer = self.create_timer(0.5, self.state_machine_callback)

    def activate_sphero_controllers(self):
        """Activate controllers for the current Sphero via multi-robot webserver."""
        current_sphero = self.spheros[self.current_sphero_index]

        self.print(f"Activating controllers for {current_sphero.sphero_name}...")

        try:
            response = requests.post(
                f'{self.webserver_url}/api/spheros',
                json={'sphero_name': current_sphero.sphero_name},
                timeout=30
            )

            if response.status_code in [200, 201]:
                result = response.json()
                if result.get('success'):
                    instance = result.get('instance', {})
                    current_sphero.port = instance.get('port')
                    current_sphero.activated = True

                    self.print(f"✓ Controllers activated (Port: {current_sphero.port})")
                    self.print("")

                    # Subscribe to position updates
                    topic_safe_name = current_sphero.sphero_name.replace('-', '_')
                    topic_name = f'/aruco_slam/{topic_safe_name}/position'

                    sub = self.create_subscription(
                        PoseStamped,
                        topic_name,
                        lambda msg, idx=self.current_sphero_index: self.position_callback(msg, idx),
                        10
                    )
                    self.position_subscribers[self.current_sphero_index] = sub

                    # Wait for controllers to initialize
                    self.print("Initializing controllers...")
                    time.sleep(5)
                    self.print("✓ Controllers ready")
                    self.print("")

                    # Start calibration
                    self.state = GameState.SPHERO_CALIBRATION
                    self.start_sphero_calibration()
                else:
                    self.print(f"✗ Failed to activate {current_sphero.sphero_name}: {result.get('message')}")
                    self.state = GameState.DONE

            else:
                self.print(f"✗ Failed to activate {current_sphero.sphero_name}: HTTP {response.status_code}")
                self.state = GameState.DONE

        except Exception as e:
            self.print(f"✗ Error activating {current_sphero.sphero_name}: {e}")
            self.state = GameState.DONE

    def start_sphero_calibration(self):
        """Start interactive calibration for the current Sphero."""
        self.timer.cancel()  # Stop timer during calibration

        current_sphero = self.spheros[self.current_sphero_index]

        self.print("Calibration")
        self.print("-" * 70)
        self.print("")
        self.print("Follow these steps:")
        self.print(f"  1. Open web interface: http://localhost:{current_sphero.port}")
        self.print("  2. Go to Motion tab")
        self.print("  3. Adjust heading so:")
        self.print("     • 0° points toward TOP-LEFT field marker")
        self.print("     • 90° points toward BOTTOM-RIGHT field marker")
        self.print("  4. Click 'Reset Aim' button")
        self.print("  5. Click 'Reset to Origin' button")
        self.print("")

        input("Press Enter when calibration is complete...")

        self.confirm_sphero_calibration()

    def confirm_sphero_calibration(self):
        """Confirm calibration and move to next step."""
        current_sphero = self.spheros[self.current_sphero_index]
        current_sphero.calibrated = True

        self.print("")
        self.print(f"✓ {current_sphero.sphero_name} calibrated!")
        self.print("")

        # Move Sphero to center field position
        self.state = GameState.MOVE_TO_POSITION
        self.move_sphero_to_position()

    def move_sphero_to_position(self):
        """Move current Sphero to its designated position near center field."""
        current_sphero = self.spheros[self.current_sphero_index]

        # Calculate position based on Sphero index
        center_x = self.field_width / 2.0  # cm
        center_y = self.field_height / 2.0  # cm
        offset = 30.0  # cm from center

        positions = [
            (center_x - offset, center_y - offset),  # Sphero 0: left-top of center
            (center_x + offset, center_y - offset),  # Sphero 1: right-top of center
            (center_x + offset, center_y + offset),  # Sphero 2: right-bottom of center
            (center_x - offset, center_y + offset),  # Sphero 3: left-bottom of center
        ]

        target_x, target_y = positions[self.current_sphero_index]

        self.print(f"Moving {current_sphero.sphero_name} to position ({target_x:.0f}, {target_y:.0f}) cm...")

        try:
            response = requests.post(
                f'http://localhost:{current_sphero.port}/api/task',
                json={
                    'task_type': 'move_to',
                    'parameters': {
                        'x': target_x,
                        'y': target_y,
                        'speed': 100
                    }
                },
                timeout=5
            )

            if response.status_code == 200:
                self.print(f"✓ Move command sent")
                self.print("")

                # Wait for movement to complete
                self.print("Moving...")
                time.sleep(8)
                self.print("✓ Position reached")
                self.print("")

                # Move to next Sphero or finish
                if self.current_sphero_index < len(self.spheros) - 1:
                    self.current_sphero_index += 1
                    self.state = GameState.WAIT_FOR_SPHERO_PLACEMENT
                    # Restart timer
                    self.timer = self.create_timer(0.5, self.state_machine_callback)
                    self.start_sphero_workflow()
                else:
                    self.state = GameState.ALL_CALIBRATED
                    # Restart timer
                    self.timer = self.create_timer(0.5, self.state_machine_callback)

        except Exception as e:
            self.print(f"✗ Error sending move command: {e}")
            self.state = GameState.DONE

    def start_dance_routine(self):
        """Make all Spheros dance in place."""
        for sphero in self.spheros:
            if sphero.activated and sphero.port:
                self.send_dance_task(sphero)

        # Wait for dance to complete
        self.print("Dancing...")
        time.sleep(15)
        self.print("✓ Dance complete!")
        self.state = GameState.DONE

    def send_dance_task(self, sphero: SpheroInfo):
        """Send a dance task to a Sphero (spin + LED changes)."""
        try:
            # Send spin task
            requests.post(
                f'http://localhost:{sphero.port}/api/task',
                json={
                    'task_type': 'spin',
                    'parameters': {
                        'rotations': 5,
                        'speed': 150
                    }
                },
                timeout=5
            )

            # Cycle LED colors while spinning
            colors = [
                (255, 0, 0),    # Red
                (0, 255, 0),    # Green
                (0, 0, 255),    # Blue
                (255, 255, 0),  # Yellow
                (255, 0, 255),  # Magenta
                (0, 255, 255),  # Cyan
            ]

            for i, (r, g, b) in enumerate(colors):
                time.sleep(2)
                requests.post(
                    f'http://localhost:{sphero.port}/api/led',
                    json={'red': r, 'green': g, 'blue': b, 'type': 'main'},
                    timeout=2
                )

        except Exception:
            pass

    def destroy_node(self):
        """Cleanup on shutdown."""
        # Stop ArUco SLAM process
        if self.aruco_slam_process:
            self.aruco_slam_process.terminate()
            self.aruco_slam_process.wait()
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    # Print header before ROS initialization
    print("\n" + "="*70)
    print("         SOCCER GAME CONTROLLER")
    print("="*70)
    print("")
    print("Field Configuration")
    print("-" * 70)
    print("")

    # Get field dimensions before initializing ROS
    while True:
        width_input = input("Enter field width in cm (e.g., 300): ")
        try:
            field_width = float(width_input)
            if field_width > 0:
                break
            else:
                print("⚠ Width must be positive. Try again.")
        except ValueError:
            print("⚠ Invalid input. Please enter a number.")

    while True:
        height_input = input("Enter field height in cm (e.g., 200): ")
        try:
            field_height = float(height_input)
            if field_height > 0:
                break
            else:
                print("⚠ Height must be positive. Try again.")
        except ValueError:
            print("⚠ Invalid input. Please enter a number.")

    print("")
    print(f"✓ Field dimensions set: {field_width} x {field_height} cm")
    print("")

    # Now initialize ROS with the dimensions
    rclpy.init(args=args)

    try:
        node = SoccerGameController()
        # Set the dimensions that were input
        node.field_width = field_width
        node.field_height = field_height
        # Skip the input state and go straight to starting ArUco SLAM
        node.state = GameState.START_ARUCO_SLAM

        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
