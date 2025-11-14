#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ArUco Marker Object.

This module defines the ArUco marker object that stores position data
and historical tracking information.
"""

import numpy as np
from typing import List, Tuple, Optional


class ArucoMarker:
    """
    Represents a detected ArUco marker with position tracking.

    Attributes:
        marker_id: Unique identifier for the marker
        corners: Current corner positions [[x,y], [x,y], [x,y], [x,y]]
        center: Current center position [x, y]
        past_corners: Historical corner positions
        past_centers: Historical center positions
    """

    def __init__(self, marker_id: int):
        """
        Initialize ArUco marker.

        Args:
            marker_id: Unique identifier for this marker
        """
        self.marker_id = marker_id
        self.corners: Optional[np.ndarray] = None
        self.center: Optional[np.ndarray] = None
        self.past_corners: List[np.ndarray] = []
        self.past_centers: List[np.ndarray] = []

    def update_corners(self, corners: np.ndarray):
        """
        Update marker corner positions.

        Args:
            corners: New corner positions from detector
                    Shape: [1, 4, 2] - one marker, 4 corners, (x,y) coordinates
        """
        # Save previous corners to history
        if self.corners is not None:
            self.past_corners.append(self.corners.copy())

        # Update current corners
        self.corners = corners

        # Update center point
        self.update_center()

    def update_center(self):
        """Calculate center point from current corners."""
        if self.corners is not None:
            # Save previous center to history
            if self.center is not None:
                self.past_centers.append(self.center.copy())

            # Calculate mean of all 4 corner points
            # corners shape: [1, 4, 2] -> squeeze to [4, 2] -> mean across axis 0
            self.center = np.mean(self.corners[0], axis=0)

    def get_corners(self) -> Optional[np.ndarray]:
        """Get current corner positions."""
        return self.corners

    def get_center(self) -> Optional[np.ndarray]:
        """Get current center position."""
        return self.center

    def get_corner_history(self) -> List[np.ndarray]:
        """Get historical corner positions."""
        return self.past_corners

    def get_center_history(self) -> List[np.ndarray]:
        """Get historical center positions."""
        return self.past_centers
