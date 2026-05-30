#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Blue-tape arena boundary detector.

Replaces ArUco corner markers for the matrix positioning source. Detects a
large blue-tape rectangle outlining the play area and returns its 4 corners,
ordered [TL, TR, BR, BL], for use as the homography source quad.

Detection only; the FieldMapper consumes the ordered corners.
"""

from typing import List, Optional

import cv2
import numpy as np


def order_corners(pts: np.ndarray) -> np.ndarray:
    """Order 4 (x, y) points as [TL, TR, BR, BL] via sum/diff.

    Top-Left has the smallest x+y, Bottom-Right the largest. Top-Right has the
    smallest (y - x) ... here we use the standard diff = x - y: Top-Right has
    the largest x - y, Bottom-Left the smallest.

    Args:
        pts: array shape (4, 2).

    Returns:
        float32 array shape (4, 2) ordered TL, TR, BR, BL.
    """
    pts = np.asarray(pts, dtype=np.float32).reshape(4, 2)
    ordered = np.zeros((4, 2), dtype=np.float32)

    s = pts.sum(axis=1)
    ordered[0] = pts[np.argmin(s)]   # TL: smallest x+y
    ordered[2] = pts[np.argmax(s)]   # BR: largest x+y

    d = pts[:, 0] - pts[:, 1]
    ordered[1] = pts[np.argmax(d)]   # TR: largest x-y
    ordered[3] = pts[np.argmin(d)]   # BL: smallest x-y

    return ordered


class BoundaryDetector:
    """Detects a blue-tape rectangle and returns ordered corner pixels."""

    def __init__(
        self,
        blue_lower_hsv: List[int] = [100, 80, 40],
        blue_upper_hsv: List[int] = [130, 255, 255],
        min_area_frac: float = 0.1,
    ):
        """
        Args:
            blue_lower_hsv: lower HSV bound for the blue tape.
            blue_upper_hsv: upper HSV bound for the blue tape.
            min_area_frac: reject boundary contours smaller than this fraction
                           of the full frame area.
        """
        self.lower = np.array(blue_lower_hsv, dtype=np.uint8)
        self.upper = np.array(blue_upper_hsv, dtype=np.uint8)
        self.min_area_frac = float(min_area_frac)

    def detect(self, frame: np.ndarray) -> Optional[np.ndarray]:
        """Detect the blue-tape arena boundary.

        Args:
            frame: BGR camera frame.

        Returns:
            float32 array shape (4, 2) ordered [TL, TR, BR, BL] if a 4-vertex
            blue rectangle large enough is found, else None.
        """
        if frame is None:
            return None

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower, self.upper)

        # Close gaps so the tape outline forms a connected band.
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        frame_area = float(frame.shape[0] * frame.shape[1])
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < self.min_area_frac * frame_area:
            return None

        peri = cv2.arcLength(largest, True)
        approx = cv2.approxPolyDP(largest, 0.02 * peri, True)
        if len(approx) != 4:
            return None

        return order_corners(approx.reshape(4, 2))
