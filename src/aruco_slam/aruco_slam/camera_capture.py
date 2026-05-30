#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Shared camera capture with tunable locks.

Wraps cv2.VideoCapture at 1920x1080 and applies camera locks tuned for
emissive (LED) markers and blue-tape arena boundary: white-balance lock +
fixed temperature, manual exposure with a MODERATE default that renders both
the tape boundary and the (dimmed) LED markers without washing out hue, and
optional autofocus lock.

Used for CAPTURE ONLY by both the ArUco detector and the matrix detector;
no detection logic lives here.
"""

import cv2
import numpy as np
from typing import Optional


class CameraCapture:
    """Camera frame source with configurable WB / exposure / focus locks."""

    def __init__(
        self,
        camera_id: int = 0,
        width: int = 1920,
        height: int = 1080,
        lock_white_balance: bool = True,
        wb_temperature: int = 4600,
        lock_exposure: bool = True,
        exposure: float = 1000.0,
        lock_autofocus: bool = True,
    ):
        """
        Args:
            camera_id: V4L camera device id.
            width, height: capture resolution.
            lock_white_balance: disable auto-WB and set a fixed temperature.
            wb_temperature: fixed white-balance temperature (Kelvin) when locked.
            lock_exposure: switch to manual exposure with a fixed value.
            exposure: fixed exposure value when locked (moderate: renders the
                tape boundary while dimmed LED markers preserve hue).
            lock_autofocus: disable autofocus.
        """
        self.camera_id = camera_id
        self.width = width
        self.height = height
        self.lock_white_balance = lock_white_balance
        self.wb_temperature = wb_temperature
        self.lock_exposure = lock_exposure
        self.exposure = exposure
        self.lock_autofocus = lock_autofocus

        self.cap: Optional[cv2.VideoCapture] = None

    def open(self) -> None:
        """Open the camera and apply resolution + locks. Raises on failure."""
        if self.cap is not None:
            return

        cap = cv2.VideoCapture(self.camera_id, cv2.CAP_V4L)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        if not cap.isOpened():
            raise RuntimeError(f"Cannot open camera {self.camera_id}")

        if self.lock_white_balance:
            cap.set(cv2.CAP_PROP_AUTO_WB, 0)
            cap.set(cv2.CAP_PROP_WB_TEMPERATURE, self.wb_temperature)

        if self.lock_exposure:
            # 1 = manual exposure mode for V4L backend.
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
            cap.set(cv2.CAP_PROP_EXPOSURE, self.exposure)

        if self.lock_autofocus:
            cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)

        self.cap = cap

    def read(self) -> Optional[np.ndarray]:
        """Return the latest frame (BGR), or None if capture failed/closed."""
        if self.cap is None:
            return None
        ret, frame = self.cap.read()
        if not ret:
            return None
        return frame

    def is_open(self) -> bool:
        return self.cap is not None and self.cap.isOpened()

    def release(self) -> None:
        """Release the camera."""
        if self.cap is not None:
            self.cap.release()
            self.cap = None
