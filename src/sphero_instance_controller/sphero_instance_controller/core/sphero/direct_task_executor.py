#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Direct Task Executor for Sphero.

This module provides a task executor that directly controls Sphero hardware.
Used by the device controller node which has direct access to the Sphero instance.
"""

from typing import Dict, Any, Optional, Callable
from spherov2.commands.sphero import RawMotorModes

from .sphero_task_executor import SpheroTaskExecutorBase
from .sphero import Sphero


class DirectTaskExecutor(SpheroTaskExecutorBase):
    """
    Task executor that directly controls Sphero hardware.

    Used by the device controller node which has direct access to the Sphero instance.
    """

    def __init__(self, sphero: Sphero,
                 position_callback: Optional[Callable[[], Dict[str, float]]] = None,
                 heading_callback: Optional[Callable[[], int]] = None):
        """
        Initialize direct task executor.

        Args:
            sphero: Sphero instance to control
            position_callback: Optional callback to get current position
            heading_callback: Optional callback to get current heading
        """
        super().__init__(position_callback, heading_callback)
        self.sphero = sphero

    def _send_raw_motor_command(self, left_mode: str, left_speed: int, right_mode: str, right_speed: int):
        """Send raw motor command directly to Sphero."""
        mode_map = {
            'forward': RawMotorModes.FORWARD,
            'reverse': RawMotorModes.REVERSE,
            'brake': RawMotorModes.BRAKE,
            'off': RawMotorModes.OFF,
            'ignore': RawMotorModes.IGNORE
        }
        left_mode_enum = mode_map.get(left_mode.lower(), RawMotorModes.FORWARD)
        right_mode_enum = mode_map.get(right_mode.lower(), RawMotorModes.FORWARD)
        self.sphero.set_raw_motor_speed(left_mode_enum, left_speed, right_mode_enum, right_speed)

    def _send_roll_command(self, heading: int, speed: int, duration: float = 0):
        """Send roll command directly to Sphero."""
        self.sphero.roll(heading, speed, duration)

    def _send_stop_command(self):
        """Send stop command directly to Sphero."""
        self.sphero.stop()

    def _send_led_command(self, red: int, green: int, blue: int, led_type: str = 'main'):
        """Send LED command directly to Sphero."""
        self.sphero.set_led(red, green, blue, led_type)

    def _send_heading_command(self, heading: int):
        """Send heading command directly to Sphero."""
        self.sphero.set_heading(heading)

    def _send_speed_command(self, speed: int):
        """Send speed command directly to Sphero."""
        self.sphero.set_speed(speed)

    def _send_spin_command(self, angle: int, duration: float = 1.0):
        """Send spin command directly to Sphero."""
        self.sphero.spin(angle, duration)

    def _send_calibrate_compass_command(self):
        """Trigger compass calibration directly on the Sphero (BOLT only, blocks)."""
        self.sphero.calibrate_compass()

    def _send_matrix_command(self, pattern: str = None, red: int = 255, green: int = 255, blue: int = 255):
        """Send matrix command directly to Sphero."""
        self.sphero.set_matrix(pattern=pattern, red=red, green=green, blue=blue)

    def _send_stabilization_command(self, enable: bool):
        """Send stabilization command directly to Sphero."""
        self.sphero.set_stabilization(enable)

    def _send_collision_detection_command(self, action: str, mode: str = 'obstacle', sensitivity: str = 'HIGH'):
        """Send collision detection command directly to Sphero."""
        if action == 'start':
            self.sphero.start_collision_detection(mode=mode, sensitivity=sensitivity)
        else:
            self.sphero.stop_collision_detection()

    def _send_ir_broadcast_command(self, near: int, far: int):
        """Start IR broadcasting directly on the Sphero."""
        self.sphero.start_ir_broadcast(near, far)

    def _send_ir_follow_command(self, near: int, far: int):
        """Start IR following directly on the Sphero."""
        self.sphero.start_ir_follow(near, far)

    def _send_ir_evade_command(self, near: int, far: int):
        """Start IR evading directly on the Sphero."""
        self.sphero.start_ir_evade(near, far)

    def _send_ir_broadcast_stop_command(self):
        """Stop IR broadcasting directly on the Sphero."""
        self.sphero.stop_ir_broadcast()

    def _send_ir_follow_stop_command(self):
        """Stop IR following directly on the Sphero."""
        self.sphero.stop_ir_follow()

    def _send_ir_evade_stop_command(self):
        """Stop IR evading directly on the Sphero."""
        self.sphero.stop_ir_evade()
