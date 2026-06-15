#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Topic-based Task Executor for Sphero.

This module provides a task executor that publishes commands via ROS topics.
Used by the task controller node which does not have direct Sphero access,
but instead publishes commands to the device controller via ROS topics.
"""

from typing import Dict, Any, Optional, Callable

from .sphero_task_executor import SpheroTaskExecutorBase


class TopicTaskExecutor(SpheroTaskExecutorBase):
    """
    Task executor that publishes commands via ROS topics.

    Used by the task controller node which does not have direct Sphero access,
    but instead publishes commands to the device controller via ROS topics.
    """

    def __init__(self,
                 command_publisher: Callable[[str, Dict[str, Any]], None],
                 position_callback: Optional[Callable[[], Dict[str, float]]] = None,
                 heading_callback: Optional[Callable[[], int]] = None):
        """
        Initialize topic-based task executor.

        Args:
            command_publisher: Callback to publish commands (topic_name, params)
            position_callback: Optional callback to get current position
            heading_callback: Optional callback to get current heading
        """
        super().__init__(position_callback, heading_callback)
        self.command_publisher = command_publisher

    def _send_raw_motor_command(self, left_mode: str, left_speed: int, right_mode: str, right_speed: int):
        """Publish raw motor command to ROS topic."""
        self.command_publisher('raw_motor', {
            'left_mode': left_mode,
            'left_speed': left_speed,
            'right_mode': right_mode,
            'right_speed': right_speed
        })

    def _send_roll_command(self, heading: int, speed: int, duration: float = 0):
        """Publish roll command to ROS topic."""
        self.command_publisher('motion', {
            'action': 'roll',
            'heading': heading,
            'speed': speed,
            'duration': duration
        })

    def _send_stop_command(self):
        """Publish stop command to ROS topic."""
        self.command_publisher('motion', {
            'action': 'stop'
        })

    def _send_led_command(self, red: int, green: int, blue: int, led_type: str = 'main'):
        """Publish LED command to ROS topic."""
        self.command_publisher('led', {
            'red': red,
            'green': green,
            'blue': blue,
            'type': led_type
        })

    def _send_heading_command(self, heading: int):
        """Publish heading command to ROS topic."""
        self.command_publisher('heading', {
            'heading': heading
        })

    def _send_speed_command(self, speed: int):
        """Publish speed command to ROS topic."""
        self.command_publisher('speed', {
            'speed': speed
        })

    def _send_spin_command(self, angle: int, duration: float = 1.0):
        """Publish spin command to ROS topic."""
        self.command_publisher('spin', {
            'angle': angle,
            'duration': duration
        })

    def _send_calibrate_compass_command(self):
        """Publish compass calibration command (empty payload) to ROS topic."""
        self.command_publisher('calibrate_compass', {})

    def _send_matrix_command(self, pattern: str = None, red: int = 255, green: int = 255, blue: int = 255):
        """Publish matrix command to ROS topic."""
        self.command_publisher('matrix', {
            'pattern': pattern,
            'red': red,
            'green': green,
            'blue': blue
        })

    def _send_stabilization_command(self, enable: bool):
        """Publish stabilization command to ROS topic."""
        self.command_publisher('stabilization', {
            'enable': enable
        })

    def _send_collision_detection_command(self, action: str, mode: str = 'obstacle', sensitivity: str = 'HIGH'):
        """Publish collision detection command to ROS topic."""
        self.command_publisher('collision', {
            'action': action,
            'mode': mode,
            'sensitivity': sensitivity
        })

    # All six IR commands multiplex onto one 'ir' topic via an `action` field,
    # mirroring how `motion` / `collision` carry start/stop on a single topic.
    def _send_ir_broadcast_command(self, near: int, far: int):
        """Publish IR broadcast-start command to ROS topic."""
        self.command_publisher('ir', {'action': 'broadcast', 'near': near, 'far': far})

    def _send_ir_follow_command(self, near: int, far: int):
        """Publish IR follow-start command to ROS topic."""
        self.command_publisher('ir', {'action': 'follow', 'near': near, 'far': far})

    def _send_ir_evade_command(self, near: int, far: int):
        """Publish IR evade-start command to ROS topic."""
        self.command_publisher('ir', {'action': 'evade', 'near': near, 'far': far})

    def _send_ir_broadcast_stop_command(self):
        """Publish IR broadcast-stop command to ROS topic."""
        self.command_publisher('ir', {'action': 'broadcast_stop'})

    def _send_ir_follow_stop_command(self):
        """Publish IR follow-stop command to ROS topic."""
        self.command_publisher('ir', {'action': 'follow_stop'})

    def _send_ir_evade_stop_command(self):
        """Publish IR evade-stop command to ROS topic."""
        self.command_publisher('ir', {'action': 'evade_stop'})
