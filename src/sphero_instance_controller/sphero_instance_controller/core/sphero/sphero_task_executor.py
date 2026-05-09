#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Sphero-specific task executor base class.

Extends the robot-agnostic ``TaskExecutorBase`` with:
  * 2D position + heading callbacks (Sphero kinematics).
  * Speed/tolerance config knobs used by the Sphero handlers.
  * Abstract ``_send_*`` command-emission methods that concrete subclasses
    (``DirectTaskExecutor``, ``TopicTaskExecutor``) implement.
  * Default registration of all Sphero handlers from
    ``sphero_task_handlers``.
"""

from typing import Callable, Dict, Optional

from sphero_instance_controller.core.common.task import TaskExecutorBase

from . import sphero_task_handlers as h


class SpheroTaskExecutorBase(TaskExecutorBase):
    """Sphero-shaped task executor. Extend, then implement the _send_* methods."""

    cancel_task_type = 'stop'

    def __init__(self,
                 position_callback: Optional[Callable[[], Dict[str, float]]] = None,
                 heading_callback: Optional[Callable[[], int]] = None):
        self.position_callback = position_callback
        self.heading_callback = heading_callback

        self.default_speed = 100
        self.position_tolerance = 10.0  # cm
        self.heading_tolerance = 5  # degrees

        # super().__init__ invokes _register_default_handlers, so all Sphero
        # config attributes need to be set before this call.
        super().__init__()

    def _register_default_handlers(self) -> None:
        self.register_handler('move_to', h.execute_move_to)
        self.register_handler('patrol', h.execute_patrol)
        self.register_handler('circle', h.execute_circle)
        self.register_handler('square', h.execute_square)
        self.register_handler('led_sequence', h.execute_led_sequence)
        self.register_handler('matrix_sequence', h.execute_matrix_sequence)
        self.register_handler('spin', h.execute_spin)
        self.register_handler('stop', h.execute_stop)
        self.register_handler('custom', h.execute_custom)
        self.register_handler('set_led', h.execute_set_led)
        self.register_handler('roll', h.execute_roll)
        self.register_handler('heading', h.execute_heading)
        self.register_handler('speed', h.execute_speed)
        self.register_handler('matrix', h.execute_matrix)
        self.register_handler('collision', h.execute_collision)
        self.register_handler('reflect', h.execute_reflect)
        self.register_handler('jumping_bean', h.execute_jumping_bean)

    def get_current_position(self) -> Dict[str, float]:
        if self.position_callback:
            return self.position_callback()
        return {'x': 0.0, 'y': 0.0}

    def get_current_heading(self) -> int:
        if self.heading_callback:
            return self.heading_callback()
        return 0

    # ----- Abstract command-emission interface (subclass implements) -----

    def _send_raw_motor_command(self, left_mode: str, left_speed: int,
                                 right_mode: str, right_speed: int):
        raise NotImplementedError("Subclass must implement _send_raw_motor_command")

    def _send_roll_command(self, heading: int, speed: int, duration: float = 0):
        raise NotImplementedError("Subclass must implement _send_roll_command")

    def _send_stop_command(self):
        raise NotImplementedError("Subclass must implement _send_stop_command")

    def _send_led_command(self, red: int, green: int, blue: int, led_type: str = 'main'):
        raise NotImplementedError("Subclass must implement _send_led_command")

    def _send_heading_command(self, heading: int):
        raise NotImplementedError("Subclass must implement _send_heading_command")

    def _send_speed_command(self, speed: int):
        raise NotImplementedError("Subclass must implement _send_speed_command")

    def _send_spin_command(self, angle: int, duration: float = 1.0):
        raise NotImplementedError("Subclass must implement _send_spin_command")

    def _send_matrix_command(self, pattern: str = None,
                              red: int = 255, green: int = 255, blue: int = 255):
        raise NotImplementedError("Subclass must implement _send_matrix_command")

    def _send_stabilization_command(self, enable: bool):
        raise NotImplementedError("Subclass must implement _send_stabilization_command")

    def _send_collision_detection_command(self, action: str,
                                            mode: str = 'obstacle',
                                            sensitivity: str = 'HIGH'):
        raise NotImplementedError("Subclass must implement _send_collision_detection_command")
