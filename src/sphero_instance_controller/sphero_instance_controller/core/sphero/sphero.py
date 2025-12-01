#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Sphero Core Class.

This module provides a high-level interface for controlling Sphero robots,
encapsulating all hardware interaction, state management, and command execution.
"""

import time
import json
from typing import Optional, Callable, Dict, Any

from spherov2.toy.sphero import Sphero as SpheroToy
from spherov2.sphero_edu import SpheroEduAPI, EventType
from spherov2.types import Color
from spherov2.commands.sensor import Sensor, CollisionDetectionMethods
from spherov2.commands.sphero import RawMotorModes

from .state import SpheroState, SpheroConnectionState
from .matrix_patterns import get_pattern


class Sphero:
    """
    High-level interface for controlling a Sphero robot.

    This class encapsulates all Sphero hardware interactions, providing
    clean methods for LED control, movement, collision detection, sensor
    queries, and more.

    Attributes:
        name: The name/identifier of this Sphero
        robot: The raw robot object from spherov2 scanner
        api: The SpheroEduAPI object for controlling the robot
        state: SpheroState object tracking current robot state
    """

    def __init__(self, robot: SpheroToy, api: SpheroEduAPI, name: str):
        """
        Initialize the Sphero controller.

        Args:
            robot: The Sphero robot object from scanner
            api: The SpheroEduAPI object
            name: Name/identifier for this Sphero
        """
        self.name = name
        self.robot = robot
        self.api = api

        # Initialize state tracking
        self.state = SpheroState(
            toy_name=name,
            connection_state=SpheroConnectionState.CONNECTED
        )
        self.state.set_api(api)
        self.state.set_toy(robot)

        # Internal state
        self._current_heading = 0
        self._current_speed = 0
        self._is_moving = False
        self._collision_mode = None
        self._collision_event_handler = None

        # Callback handlers (set by external code like ROS node)
        self._tap_callback: Optional[Callable] = None
        self._obstacle_callback: Optional[Callable] = None

    # ===== LED Control =====

    def set_led(self, red: int, green: int, blue: int, led_type: str = 'main') -> bool:
        """
        Set LED color.

        Args:
            red: Red value (0-255)
            green: Green value (0-255)
            blue: Blue value (0-255)
            led_type: Type of LED ('main', 'front', 'back')

        Returns:
            True if successful, False otherwise
        """
        try:
            # Validate and clamp RGB values
            red = max(0, min(255, int(red)))
            green = max(0, min(255, int(green)))
            blue = max(0, min(255, int(blue)))

            color = Color(r=red, g=green, b=blue)

            # Set LED based on type
            if led_type == 'main':
                self.api.set_main_led(color)
                return True
            elif led_type == 'front':
                if hasattr(self.api, 'set_front_led'):
                    self.api.set_front_led(color)
                    return True
                return False
            elif led_type == 'back':
                if hasattr(self.api, 'set_back_led'):
                    self.api.set_back_led(color)
                    return True
                return False
            else:
                # Default to main LED for unknown types
                self.api.set_main_led(color)
                return True

        except Exception as e:
            print(f"Error setting LED: {e}")
            return False

    def flash_led(self, red: int, green: int, blue: int, duration: float = 1.5,
                  restore_red: int = 0, restore_green: int = 255, restore_blue: int = 0):
        """
        Flash the LED to a color and restore it.

        Args:
            red: Flash color red (0-255)
            green: Flash color green (0-255)
            blue: Flash color blue (0-255)
            duration: How long to show the flash color (seconds)
            restore_red: Color to restore red (0-255)
            restore_green: Color to restore green (0-255)
            restore_blue: Color to restore blue (0-255)
        """
        try:
            self.set_led(red, green, blue)
            time.sleep(duration)
            self.set_led(restore_red, restore_green, restore_blue)
        except Exception as e:
            print(f"Error flashing LED: {e}")

    # ===== Movement Control =====

    def roll(self, heading: int, speed: int, duration: float = 0) -> bool:
        """
        Roll the Sphero in a direction.

        Args:
            heading: Direction in degrees (0-359)
            speed: Speed (0-255)
            duration: Duration in seconds (0 = indefinite)

        Returns:
            True if successful, False otherwise
        """
        try:
            # Normalize heading and clamp speed
            heading = heading % 360
            speed = max(0, min(255, speed))

            self._current_heading = heading
            self._current_speed = speed
            self._is_moving = speed > 0

            if duration > 0:
                # Duration-based rolling
                self.api.roll(heading, speed, duration)
            else:
                # Indefinite rolling
                self.api.set_heading(heading)
                self.api.set_speed(speed)

            return True

        except Exception as e:
            print(f"Error rolling: {e}")
            return False

    def spin(self, angle: int, duration: float = 1.0) -> bool:
        """
        Spin the Sphero by a given angle.

        Args:
            angle: Angle to spin in degrees
            duration: Duration of spin in seconds

        Returns:
            True if successful, False otherwise
        """
        try:
            initial_heading = self._current_heading
            target_heading = (initial_heading + angle) % 360

            self.api.spin(angle, duration)
            self._current_heading = target_heading

            return True

        except Exception as e:
            print(f"Error spinning: {e}")
            return False

    def set_heading(self, heading: int) -> bool:
        """
        Set the Sphero's heading.

        Args:
            heading: Direction in degrees (0-359)

        Returns:
            True if successful, False otherwise
        """
        try:
            heading = heading % 360
            self.api.set_heading(heading)
            self._current_heading = heading
            return True

        except Exception as e:
            print(f"Error setting heading: {e}")
            return False
        
    def set_raw_motor_speed(self, leftMode: RawMotorModes, leftSpeed, rightMode: RawMotorModes, rightSpeed, proc = None) -> bool:
        """
        Set the raw motor speeds.
        
        Args:
            leftMode: Motor mode (Fwd, Rev, Brake, Off, Ignore)
            leftSpeed: Raw motor speed 0-255
            rightMode: Motor mode (Fwd, Rev, Brake, Off, Ignore)
            rightSpeed: Raw motor speed 0-255

        Returns:
            True if successful, False otherwise

        """
        try:
            self.robot.set_raw_motors(leftMode, leftSpeed, rightMode, rightSpeed, proc)
            return True
        
        except Exception as e:
            print(f"Error setting raw motor speeds: {e}")
            return False


    def set_speed(self, speed: int, duration: float = 0) -> bool:
        """
        Set the Sphero's speed.

        Args:
            speed: Speed (0-255)
            duration: Duration in seconds (0 = indefinite)

        Returns:
            True if successful, False otherwise
        """
        try:
            speed = max(0, min(255, speed))

            self.api.set_speed(speed)
            self._current_speed = speed
            self._is_moving = speed > 0

            # If duration specified, stop after duration
            if duration > 0:
                time.sleep(duration)
                self.api.set_speed(0)
                self._current_speed = 0
                self._is_moving = False

            return True

        except Exception as e:
            print(f"Error setting speed: {e}")
            return False

    def stop(self) -> bool:
        """
        Stop the Sphero's movement.

        Returns:
            True if successful, False otherwise
        """
        try:
            self.api.set_speed(0)
            self._current_speed = 0
            self._is_moving = False
            return True

        except Exception as e:
            print(f"Error stopping: {e}")
            return False

    def reset_aim(self) -> bool:
        """
        Reset the Sphero's aim (heading and position to origin).

        Returns:
            True if successful, False otherwise
        """
        try:
            # Query current location before reset
            current_location = self.api.get_location()
            device_x = current_location.get('x', 0.0)
            device_y = current_location.get('y', 0.0)

            # Reset physical heading
            self.api.reset_aim()
            self._current_heading = 0

            # Set position offset in state
            self.state._position_offset_x = device_x
            self.state._position_offset_y = device_y

            # Reset tracked position
            self.state.position.x = 0.0
            self.state.position.y = 0.0
            self.state._last_published_position = (0.0, 0.0)

            return True

        except Exception as e:
            print(f"Error resetting aim: {e}")
            return False

    def set_stabilization(self, enable: bool) -> bool:
        """
        Enable or disable stabilization.

        Args:
            enable: True to enable, False to disable

        Returns:
            True if successful, False otherwise
        """
        try:
            self.api.set_stabilization(enable)
            return True

        except Exception as e:
            print(f"Error setting stabilization: {e}")
            return False

    # ===== Matrix Display (BOLT) =====

    def set_matrix(self, pattern: str = None, custom_matrix: list = None,
                   red: int = 255, green: int = 255, blue: int = 255,
                   duration: float = 0) -> bool:
        """
        Set LED matrix display (Sphero BOLT only).

        Args:
            pattern: Predefined pattern name (e.g., 'smile', 'cross')
            custom_matrix: Custom 64-element matrix pattern
            red: Color red value (0-255)
            green: Color green value (0-255)
            blue: Color blue value (0-255)
            duration: Display duration in seconds (0 = permanent)

        Returns:
            True if successful, False otherwise
        """
        try:
            # Check if matrix is supported
            if not hasattr(self.api, 'set_matrix_pixel'):
                return False

            # Get pattern data
            matrix_data = None
            if pattern:
                matrix_data = get_pattern(pattern)
            elif custom_matrix and len(custom_matrix) == 64:
                matrix_data = custom_matrix

            if matrix_data is None:
                return False

            # Set matrix pixels
            for i, brightness in enumerate(matrix_data):
                if brightness > 0:
                    row = i // 8
                    col = i % 8
                    self.api.set_matrix_pixel(row, col, Color(r=red, g=green, b=blue))

            # Clear after duration if specified
            if duration > 0:
                time.sleep(duration)
                self.api.clear_matrix()

            return True

        except Exception as e:
            print(f"Error setting matrix: {e}")
            return False

    def clear_matrix(self) -> bool:
        """
        Clear the LED matrix display.

        Returns:
            True if successful, False otherwise
        """
        try:
            if hasattr(self.api, 'clear_matrix'):
                self.api.clear_matrix()
                return True
            return False

        except Exception as e:
            print(f"Error clearing matrix: {e}")
            return False

    # ===== Collision Detection =====

    def start_collision_detection(self, mode: str = 'obstacle',
                                  sensitivity: str = 'HIGH',
                                  tap_callback: Callable = None,
                                  obstacle_callback: Callable = None) -> bool:
        """
        Start collision detection.

        Args:
            mode: 'tap' or 'obstacle'
            sensitivity: Sensitivity level ('SUPER_HIGH', 'VERY_HIGH', 'HIGH', 'MEDIUM', 'LOW', 'VERY_LOW')
            tap_callback: Callback function for tap events (called with timestamp)
            obstacle_callback: Callback function for obstacle events (called with timestamp)

        Returns:
            True if successful, False otherwise
        """
        try:
            self._collision_mode = mode
            self._tap_callback = tap_callback
            self._obstacle_callback = obstacle_callback

            # Map sensitivity to threshold
            threshold_map = {
                'SUPER_HIGH': 50,
                'VERY_HIGH': 80,
                'HIGH': 100,
                'MEDIUM': 130,
                'LOW': 160,
                'VERY_LOW': 200
            }
            threshold = threshold_map.get(sensitivity.upper(), 100)

            # Configure collision detection
            self.robot.configure_collision_detection(
                CollisionDetectionMethods.ACCELEROMETER_BASED_DETECTION,
                x_threshold=threshold,
                y_threshold=threshold,
                x_speed=100,
                y_speed=100,
                dead_time=10
            )

            # Enable notifications
            self.robot.add_collision_detected_notify_listener(self._hardware_collision_callback)

            # Register event handler
            self._collision_event_handler = self._on_collision_detected
            self.api.register_event(EventType.on_collision, self._collision_event_handler)

            return True

        except Exception as e:
            print(f"Error starting collision detection: {e}")
            return False

    def stop_collision_detection(self) -> bool:
        """
        Stop collision detection.

        Returns:
            True if successful, False otherwise
        """
        try:
            # Remove listener
            if self._collision_mode is not None:
                try:
                    self.robot.remove_collision_detected_notify_listener(self._hardware_collision_callback)
                except Exception:
                    pass

                # Unregister event
                try:
                    self.api.register_event(EventType.on_collision, None)
                    self._collision_event_handler = None
                except Exception:
                    pass

                self._collision_mode = None

            return True

        except Exception as e:
            print(f"Error stopping collision detection: {e}")
            return False

    def _hardware_collision_callback(self, collision_data):
        """Hardware-level collision callback."""
        try:
            if hasattr(self, '_collision_event_handler') and self._collision_event_handler:
                self._collision_event_handler(self.api)
        except Exception as e:
            print(f"Error in hardware collision callback: {e}")

    def _on_collision_detected(self, api):
        """Internal collision event handler."""
        if self._collision_mode is None:
            return

        try:
            # Determine if we're moving
            is_moving = self.state.query_property('motion', 'is_moving')
            if is_moving is None:
                is_moving = self._is_moving

            # Tap mode: stationary impacts
            if self._collision_mode == 'tap' and not is_moving:
                self.flash_led(255, 0, 0, 1.5, 0, 255, 0)
                if self._tap_callback:
                    self._tap_callback(time.time())

            # Obstacle mode: collisions during motion
            elif self._collision_mode == 'obstacle':
                self.flash_led(255, 0, 0, 1.5, 0, 255, 0)
                if self._obstacle_callback:
                    self._obstacle_callback(time.time())

        except Exception as e:
            print(f"Error in collision handler: {e}")

    # ===== Sensor Data =====

    def update_sensors(self) -> bool:
        """
        Update sensor data from the device.

        Returns:
            True if successful, False otherwise
        """
        try:
            self.state.update_from_device()

            # Update internal state tracking
            self._current_heading = self.state.get('motion', 'heading', 0)
            self._current_speed = self.state.get('motion', 'speed', 0)
            self._is_moving = self.state.get('motion', 'is_moving', False)

            return True

        except Exception as e:
            print(f"Error updating sensors: {e}")
            return False

    def get_state_dict(self) -> Dict[str, Any]:
        """
        Get complete state as dictionary.

        Returns:
            Dictionary containing all state information
        """
        return self.state.to_dict()

    def get_sensor_msg(self):
        """
        Get sensor data as SpheroSensor message.

        Returns:
            SpheroSensor message or None
        """
        return self.state.to_sphero_sensor_msg()

    def get_battery_msg(self):
        """
        Get battery data as BatteryState message.

        Returns:
            BatteryState message or None
        """
        return self.state.battery.to_battery_state_msg()

    def get_battery_percentage(self) -> float:
        """
        Get battery percentage.

        Returns:
            Battery percentage (0-100)
        """
        return self.state.battery.percentage

    def is_healthy(self) -> bool:
        """
        Check if Sphero is healthy.

        Returns:
            True if healthy, False otherwise
        """
        return self.state.is_healthy()

    # ===== Properties =====

    @property
    def heading(self) -> int:
        """Current heading in degrees."""
        return self._current_heading

    @property
    def speed(self) -> int:
        """Current speed (0-255)."""
        return self._current_speed

    @property
    def is_moving(self) -> bool:
        """Whether the Sphero is currently moving."""
        return self._is_moving

    @property
    def collision_mode(self) -> Optional[str]:
        """Current collision detection mode ('tap', 'obstacle', or None)."""
        return self._collision_mode

    # ===== Cleanup =====

    def cleanup(self):
        """Clean up resources before shutdown."""
        try:
            # Stop collision detection
            self.stop_collision_detection()

            # Stop movement
            self.stop()

            # Turn off LED
            self.set_led(0, 0, 0)

            # Clear matrix
            self.clear_matrix()

        except Exception as e:
            print(f"Error during cleanup: {e}")
