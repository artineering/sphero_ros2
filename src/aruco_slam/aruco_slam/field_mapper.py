#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Soccer Field Mapper.

This module handles calibration of the virtual soccer field using corner markers
and performs perspective transformation to map camera coordinates to field coordinates.
"""

import cv2
import numpy as np
from typing import Dict, Tuple, Optional, List


class FieldMapper:
    """
    Maps camera coordinates to virtual soccer field coordinates.

    Uses 4 ArUco markers placed at the corners of a physical field area to establish
    a perspective transformation. The field is defined as a half soccer field with
    configurable dimensions.
    """

    def __init__(
        self,
        field_width_cm: float = 600.0,  # 6 meters
        field_height_cm: float = 400.0,  # 4 meters
        corner_marker_ids: List[int] = [0, 1, 2, 3]
    ):
        """
        Initialize field mapper.

        Args:
            field_width_cm: Virtual field width in centimeters
            field_height_cm: Virtual field height in centimeters
            corner_marker_ids: List of 4 marker IDs for corners [TL, TR, BR, BL]
                              Order: Top-Left, Top-Right, Bottom-Right, Bottom-Left
        """
        self.field_width = field_width_cm
        self.field_height = field_height_cm
        self.corner_marker_ids = corner_marker_ids

        # Transformation matrix (computed during calibration)
        self.transform_matrix: Optional[np.ndarray] = None
        self.inverse_transform_matrix: Optional[np.ndarray] = None

        # Calibration status
        self.calibrated = False

        # Define virtual field corners (in cm)
        # Origin at top-left, +x right, +y down
        self.field_corners = np.float32([
            [0, 0],                                    # Top-left
            [self.field_width, 0],                     # Top-right
            [self.field_width, self.field_height],     # Bottom-right
            [0, self.field_height]                     # Bottom-left
        ])

    def calibrate(self, marker_centers: Dict[int, np.ndarray]) -> bool:
        """
        Calibrate field mapping using detected corner markers.

        Args:
            marker_centers: Dictionary mapping marker IDs to their center positions (pixels)

        Returns:
            True if calibration successful, False otherwise
        """
        # Check if all corner markers are detected
        detected_corners = []
        for marker_id in self.corner_marker_ids:
            if marker_id not in marker_centers:
                print(f"Warning: Corner marker {marker_id} not detected")
                return False
            detected_corners.append(marker_centers[marker_id])

        # Convert to numpy array
        camera_corners = np.float32(detected_corners)

        # Compute perspective transformation matrix
        # Maps from camera coordinates to field coordinates
        self.transform_matrix = cv2.getPerspectiveTransform(
            camera_corners,
            self.field_corners
        )

        # Compute inverse transformation (field -> camera)
        self.inverse_transform_matrix = cv2.getPerspectiveTransform(
            self.field_corners,
            camera_corners
        )

        self.calibrated = True
        print("Field calibration successful!")
        print(f"Field dimensions: {self.field_width}cm x {self.field_height}cm")
        return True

    def camera_to_field(self, camera_point: np.ndarray) -> Optional[np.ndarray]:
        """
        Transform camera coordinates to field coordinates.

        Args:
            camera_point: Point in camera coordinates [x, y] (pixels)

        Returns:
            Point in field coordinates [x, y] (cm), or None if not calibrated
        """
        if not self.calibrated or self.transform_matrix is None:
            return None

        # Reshape point for perspective transform
        point = np.array([[camera_point]], dtype=np.float32)

        # Apply perspective transformation
        field_point = cv2.perspectiveTransform(point, self.transform_matrix)

        return field_point[0][0]

    def field_to_camera(self, field_point: np.ndarray) -> Optional[np.ndarray]:
        """
        Transform field coordinates to camera coordinates.

        Args:
            field_point: Point in field coordinates [x, y] (cm)

        Returns:
            Point in camera coordinates [x, y] (pixels), or None if not calibrated
        """
        if not self.calibrated or self.inverse_transform_matrix is None:
            return None

        # Reshape point for perspective transform
        point = np.array([[field_point]], dtype=np.float32)

        # Apply inverse transformation
        camera_point = cv2.perspectiveTransform(point, self.inverse_transform_matrix)

        return camera_point[0][0]

    def camera_to_field_batch(self, camera_points: np.ndarray) -> Optional[np.ndarray]:
        """
        Transform multiple camera coordinates to field coordinates.

        Args:
            camera_points: Array of points [[x1, y1], [x2, y2], ...] (pixels)

        Returns:
            Array of points in field coordinates (cm), or None if not calibrated
        """
        if not self.calibrated or self.transform_matrix is None:
            return None

        # Reshape for perspective transform
        points = camera_points.reshape(-1, 1, 2).astype(np.float32)

        # Apply transformation
        field_points = cv2.perspectiveTransform(points, self.transform_matrix)

        return field_points.reshape(-1, 2)

    def is_calibrated(self) -> bool:
        """Check if field mapping is calibrated."""
        return self.calibrated

    def get_field_dimensions(self) -> Tuple[float, float]:
        """Get field dimensions in cm."""
        return (self.field_width, self.field_height)

    def draw_field_overlay(self, frame: np.ndarray, color=(0, 255, 0), thickness=2) -> np.ndarray:
        """
        Draw virtual field boundaries on camera frame.

        Args:
            frame: Camera frame to draw on
            color: Line color (BGR)
            thickness: Line thickness

        Returns:
            Frame with field overlay
        """
        if not self.calibrated:
            return frame

        # Transform field corners to camera coordinates
        camera_corners = []
        for field_corner in self.field_corners:
            cam_corner = self.field_to_camera(field_corner)
            if cam_corner is not None:
                camera_corners.append(cam_corner.astype(int))

        # Draw field rectangle
        if len(camera_corners) == 4:
            pts = np.array(camera_corners, dtype=np.int32)
            cv2.polylines(frame, [pts], isClosed=True, color=color, thickness=thickness)

            # Draw center line
            top_center = (camera_corners[0] + camera_corners[1]) // 2
            bottom_center = (camera_corners[2] + camera_corners[3]) // 2
            cv2.line(frame, tuple(top_center), tuple(bottom_center), color, thickness // 2)

        return frame

    def get_calibration_status_text(self) -> str:
        """Get calibration status as text."""
        if self.calibrated:
            return f"CALIBRATED - Field: {self.field_width:.0f}x{self.field_height:.0f} cm"
        else:
            missing = []
            for marker_id in self.corner_marker_ids:
                missing.append(str(marker_id))
            return f"NOT CALIBRATED - Need corner markers: {', '.join(missing)}"
