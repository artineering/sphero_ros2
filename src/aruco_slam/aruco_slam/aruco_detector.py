#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ArUco Marker Detector.

This module provides real-time ArUco marker detection from a webcam feed.
"""

import cv2
import numpy as np
from typing import Dict, List, Tuple, Optional
from .aruco_marker import ArucoMarker


class ArucoDetector:
    """
    ArUco marker detector using OpenCV.

    Detects ArUco markers from camera feed and tracks their positions.
    """

    def __init__(self, camera_id: int = 0, dict_type=cv2.aruco.DICT_4X4_50):
        """
        Initialize ArUco detector.

        Args:
            camera_id: Camera device ID (default: 0)
            dict_type: ArUco dictionary type
        """
        # Initialize ArUco detector (OpenCV 4.12 API)
        self.dictionary = cv2.aruco.getPredefinedDictionary(dict_type)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

        # Camera
        self.camera_id = camera_id
        self.cap: Optional[cv2.VideoCapture] = None

        # Marker tracking
        self.aruco_tags: Dict[int, ArucoMarker] = {}

        # Visualization state
        self.visualizing = False

    def begin_visualization(self):
        """Start camera capture for visualization."""
        if self.cap is None:
            self.cap = cv2.VideoCapture(self.camera_id, cv2.CAP_V4L)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
            if not self.cap.isOpened():
                raise RuntimeError(f"Cannot open camera {self.camera_id}")
        self.visualizing = True

    def end_visualization(self):
        """Stop camera capture and cleanup."""
        self.visualizing = False
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        cv2.destroyAllWindows()

    def detect_markers(self, frame: np.ndarray) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], np.ndarray]:
        """
        Detect ArUco markers in a frame.

        Args:
            frame: Input image frame (BGR)

        Returns:
            Tuple of (corners, ids, rejected_candidates)
        """
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, rejected = self.detector.detectMarkers(gray)

        # Update tracked markers
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                marker_id = int(marker_id)

                # Create new marker object if first detection
                if marker_id not in self.aruco_tags:
                    self.aruco_tags[marker_id] = ArucoMarker(marker_id)

                # Update marker position
                self.aruco_tags[marker_id].update_corners(corners[i])

        return corners, ids, rejected

    def visualize(self, show_ids: bool = True, show_axes: bool = False):
        """
        Display live camera feed with detected markers.

        Args:
            show_ids: Whether to display marker IDs
            show_axes: Whether to display coordinate axes (requires camera calibration)

        Returns:
            True if frame was processed, False if camera unavailable
        """
        if self.cap is None:
            return False

        ret, frame = self.cap.read()
        if not ret:
            return False

        # Detect markers
        corners, ids, rejected = self.detect_markers(frame)

        # Draw detected markers
        if ids is not None:
            # Draw marker boundaries
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # Optionally draw IDs
            if show_ids:
                for i, marker_id in enumerate(ids.flatten()):
                    # Get center point
                    center = self.aruco_tags[int(marker_id)].get_center()
                    if center is not None:
                        cv2.putText(
                            frame,
                            f"ID: {marker_id}",
                            (int(center[0]), int(center[1]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 255, 0),
                            2
                        )

        # Display frame
        cv2.imshow('AruCo SLAM', frame)

        # Check for quit key ('q')
        if cv2.waitKey(1) & 0xFF == ord('q'):
            return False

        return True

    def get_frame(self) -> Optional[np.ndarray]:
        """
        Capture a single frame from camera.

        Returns:
            Frame as numpy array, or None if capture failed
        """
        if self.cap is None:
            self.cap = cv2.VideoCapture(self.camera_id)

        ret, frame = self.cap.read()
        if ret:
            return frame
        return None

    def get_tags(self) -> Dict[int, ArucoMarker]:
        """Get all detected marker objects."""
        return self.aruco_tags

    def get_tag(self, marker_id: int) -> Optional[ArucoMarker]:
        """Get specific marker by ID."""
        return self.aruco_tags.get(marker_id)

    def get_last_tag_corners(self) -> Dict[int, np.ndarray]:
        """Get current corners for all detected markers."""
        return {
            marker_id: marker.get_corners()
            for marker_id, marker in self.aruco_tags.items()
            if marker.get_corners() is not None
        }

    def get_last_tag_centers(self) -> Dict[int, np.ndarray]:
        """Get current centers for all detected markers."""
        return {
            marker_id: marker.get_center()
            for marker_id, marker in self.aruco_tags.items()
            if marker.get_center() is not None
        }

    def get_all_tag_corners(self) -> Dict[int, List[np.ndarray]]:
        """Get corner history for all detected markers."""
        return {
            marker_id: marker.get_corner_history()
            for marker_id, marker in self.aruco_tags.items()
        }

    def get_all_tag_centers(self) -> Dict[int, List[np.ndarray]]:
        """Get center history for all detected markers."""
        return {
            marker_id: marker.get_center_history()
            for marker_id, marker in self.aruco_tags.items()
        }

    def clear_history(self):
        """Clear all marker tracking history."""
        self.aruco_tags.clear()
